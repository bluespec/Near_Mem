// Copyright (c) 2016-2021 Bluespec, Inc. All Rights Reserved.
//
// This package implements the ITCM and was hived off from
// Near_Mem_TCM for maintainability reasons. Please refer to the
// introduction in Near_Mem_TCM for details.
//
// ----------------

package ITCM;

// ================================================================
// BSV lib imports

import ConfigReg        :: *;
import SpecialFIFOs     :: *;
import FIFOF            :: *;
import GetPut           :: *;
import ClientServer     :: *;
import BRAMCore         :: *;
import Connectable      :: *;

// ----------------
// Additional libs

import Cur_Cycle        :: *;
import GetPut_Aux       :: *;
import Semi_FIFOF       :: *;
import ByteLane         :: *;

// ================================================================
// Project imports

import ISA_Decls        :: *;
import TCM_Decls        :: *;
import Near_Mem_IFC     :: *;
import MMU_Cache_Common :: *;
import MMIO             :: *;

`ifdef FABRIC_AXI4
import TCM_AXI4_Adapter :: *;
`endif

import TCM_DMA_AXI4_Adapter :: *;
import AXI4_Deburster   :: *;
import Fabric_Defs      :: *;
import AXI4_Types       :: *;
import SoC_Map          :: *;


// ================================================================
// BRAM config constants

Bool config_output_register_BRAM = False; // no output register
Bool load_file_is_binary_BRAM = False;    // load file is in hex format

// ================================================================
// Local functions

function Instr fv_extract_instr (Addr pc, TCM_Word ram_out);
`ifdef NM32
   return (ram_out);
`else
   Bool lower_word = (pc[(bits_per_byte_in_tcm_word - 1)] == 1'b0);
   return (lower_word ? ram_out [31:0] : ram_out [63:32]);
`endif
endfunction

// ================================================================
// Interface Definition

interface ITCM_IFC;
   method Action  reset;

   // CPU side
   interface IMem_IFC  imem;

   // DMA server interface for back-door access to the ITCM
   interface AXI4_Slave_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User)  dma_server;
endinterface

// ================================================================
// Here begins the module
//
(* synthesize *)
module mkITCM #(Bit #(2) verbosity) (ITCM_IFC);

   // Verbosity: 0: quiet
   //            1: Requests and responses
   //            2: rule firings
   //            3: + detail
   Bit#(2) verbosity_mmio = verbosity;
   Bit#(2) verbosity_axi4 = verbosity;

   // Module state
   Reg #(Mem_State) rg_imem_state   <- mkReg (MEM_IDLE);
   Reg #(Bool)      rg_result_valid <- mkReg (False);
   Reg #(Addr)      rg_pc           <- mkRegU;
   Reg #(WordXL)    rg_instr        <- mkRegU;
   Reg #(Bool)      rg_exc          <- mkReg (False);
   Reg #(Exc_Code)  rg_exc_code     <- mkRegU;

   // Current request from the CPU
   Reg #(IMem_Req)  rg_imem_req     <- mkRegU;

   Reg #(Bool)      dw_valid        <- mkDWire (False);
   Reg #(Bool)      dw_exc          <- mkDWire (False);
   Reg #(Exc_Code)  dw_exc_code     <- mkDWire (?);
   Reg #(Instr)     dw_instr        <- mkDWire (?);

   // Requests and data to/from memory (AXI4 fabric)
   FIFOF #(Single_Req) f_mem_req    <- mkFIFOF1;
   FIFOF #(Read_Data)  f_mem_rdata  <- mkFIFOF1;

   // No WData for the IMem. Dummy interface to the AXI4 adapter
   FIFOF #(Bit #(32))  f_mem_wdata  = dummy_FIFOF;

`ifdef MICROSEMI
   // The TCM RAM - dual-ported to allow backdoor to change IMem contents
   BRAM_DUAL_PORT_BE #(ITCM_INDEX
                     , TCM_Word
                     , Bytes_per_TCM_Word) itcm <- mkBRAMCore2BE (n_words_IBRAM
                                                                , config_output_register_BRAM);
`else
   // The TCM RAM - dual-ported to allow backdoor to change IMem contents
   BRAM_DUAL_PORT_BE #(ITCM_INDEX
                     , TCM_Word
                     , Bytes_per_TCM_Word) itcm <- mkBRAMCore2BELoad (n_words_IBRAM
                                                                    , config_output_register_BRAM
                                                                    , itcmname
                                                                    , load_file_is_binary_BRAM);
`endif

   // The "front-door" to the itcm (port A)
   let irom = itcm.a;

   // The "back-door" for direct memory accesses to the itcm
   let iram = itcm.b;

   // Back-door access to the TCM RAM from the AXI4
   let dma_port <- mkTCM_DMA_AXI4_Adapter (iram, verbosity);
   AXI4_Deburster_IFC #(
      Wd_Id, Wd_Addr, Wd_Data, Wd_User) deburstr <- mkAXI4_Deburster;
   mkConnection (deburstr.to_slave, dma_port.dma_server);
   let dma_port_extnl = deburstr.from_master;

   SoC_Map_IFC soc_map <- mkSoC_Map;

   // Drive response from TCM -- loads, exceptions
   rule rl_tcm_rsp (rg_imem_state == MEM_TCM_RSP);
      // extract the instruction from the RAM data
      let instr = fv_extract_instr (rg_imem_req.pc, irom.read);

      if (verbosity > 2)
         $display ("%0d: %m.rl_tcm_rsp: pc %08h data %08h"
            , cur_cycle, rg_imem_req.pc, instr);

      // drive the outputs
      dw_valid       <= rg_result_valid;
      dw_exc         <= rg_exc;
      dw_exc_code    <= rg_exc_code;
      dw_instr       <= instr;
   endrule

   // This rule is basically the body of method ma_req; decoupling
   // through a wire affords scheduling flexibility.
   //
   // Registers an incoming request and starts the TCM/MMIO probe
   Wire #(IMem_Req) w_imem_req <- mkWire;
   (* fire_when_enabled *)
   rule rl_req;
      let imem_req = w_imem_req;
      let pc = imem_req.pc;
      let f3 = imem_req.f3;

      // Note: ignoring all VM args in this version of Near_Mem_TCM
      if (verbosity > 1)
         $display ("%0d: %m.rl_req: pc 0x%08h", cur_cycle, pc);

      rg_imem_req     <= imem_req;

      // The read to the RAM is initiated here speculatively.
      // This read is specualtive because we still don't know if the
      // address is good and is indeed meant for the TCM. Since it is a
      // read, there is no side-effect and can be safely initiated without
      // waiting for all the results to come in about the address.
      ITCM_INDEX word_addr = truncate (pc >> bits_per_byte_in_tcm_word);
      irom.put (0, truncate(word_addr), ?);

      // for all the checks relating to the soc-map
      Fabric_Addr fabric_pc = fv_Addr_to_Fabric_Addr (pc);

      // Check if f3 is legal, and if f3 and addr are compatible
      let addr_is_aligned = fn_is_aligned (f3 [1:0], pc);

      // address checks
      // regardless, we always move to the RSP state next, and will
      // have a valid result (even if it is an error response)
      rg_imem_state        <= MEM_TCM_RSP;
      rg_result_valid      <= True;

      // Legality check -- aligned address, and address should
      // belong to the ITCM
      if (!addr_is_aligned) begin
         rg_exc            <= True;
         rg_exc_code       <= exc_code_INSTR_ADDR_MISALIGNED;
      end

      // serviced by the ITCM
      else if (soc_map.m_is_itcm_addr (fabric_pc)) begin
         rg_exc            <= False;
      end

      // outside ITCM address space -- respond with an access fault
      else begin
         rg_exc            <= True;
         rg_exc_code       <= exc_code_INSTR_ACCESS_FAULT;
      end
   endrule

   // ----------------
   // INTERFACE

   method Action reset;
      rg_result_valid   <= False;
      rg_imem_state     <= MEM_IDLE;
      dma_port.reset;
      deburstr.reset;

      if (verbosity > 1)
         $display ("%0d: %m.reset", cur_cycle);
   endmethod

   // CPU side
   interface IMem_IFC imem;
      // CPU interface: request
      // NOTE: this has no flow control: CPU should only invoke it
      // when consuming prev output
      method Action  req (
           Bit #(3) f3
         , WordXL pc
`ifdef ISA_PRIV_S
           // The following  args for VM only
         , Priv_Mode  priv
         , Bit #(1) sstatus_SUM
         , Bit #(1) mstatus_MXR
         , WordXL   satp
`endif
         );    // { VM_Mode, ASID, PPN_for_page_table }

         w_imem_req <= IMem_Req {pc: pc, f3: f3};
      endmethod

      method Bool valid = dw_valid;
      method Bool is_i32_not_i16 = True;
      method WordXL pc = rg_imem_req.pc;

      method Instr instr = dw_instr;

      method Bool exc = dw_exc;
      method Exc_Code exc_code = dw_exc_code;
      method WordXL tval = rg_imem_req.pc;   // the faulting address. not always the PC
   endinterface

   // Back-door from fabric into ITCM
   interface dma_server = dma_port_extnl;

endmodule: mkITCM

endpackage
