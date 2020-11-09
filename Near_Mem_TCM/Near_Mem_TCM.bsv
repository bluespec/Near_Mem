// Copyright (c) 2016-2018 Bluespec, Inc. All Rights Reserved.

// Near_Mem_IFC is an abstraction of two alternatives: caches or TCM
// (TCM = Tightly Coupled Memory).  Both are memories that are
// 'near' the CPU (1-cycle access in common case).

// On the CPU side it directly services instruction fetches and DMem
// reads and writes.

// On the Fabric side it has two Master sub-interfaces and one Slave
// sub-interface.  The Master sub-interfaces are used for memory and
// memory-mapped I/O requests/responses from the CPU to the fabric.
// There are two Master interfaces, for concurrent IMem and DMem
// access.  The Slave sub-interface is used in the TCM variant for
// back-door access from the fabric to the TCM.

// This implementation of Near_Mem contains a TCM (Tightly Coupled Memory).
// - TCM is not a cache; it's just an SRAM/BRAM servicing a segment of
//     the address space. Accesses to other addresses (other memory, and
//     memory-mapped I/O) are still serviced by the Fabric. TCMs:
//     - have a 100% 'hit rate' for CPU access
//     - have a latency of exactly 1 cycle, and
//     - have a throughput of exactly 1 access/cycle.
//     and thus deliver best-case CPI performance (Cycles per Instruction).
//     Overall CPI can still be > 1 for reasons other than memory access
//     - Stalls due to pipeline dependencies (branches, register hazards, ...)
//     - Accesses to the Fabric (non-TCM memory and memory-mapped I/O)

// In this implementation, Instruction-Fetches are assumed always to
// be serviced by the TCM, and so the the Near_Mem_IFC sub-interface
// imem_to_fabric is unused (stubbed out).

// The sub-interface 'near_mem_slave' enables 'back-door' access of
// TCM memory by devices and debuggers.

// ----------------
// NOTE: "tohost"
// Special (fragile) ad hoc support for standard ISA tests during
// simulation: watch writes to physical addr <tohost> and stop on
// non-zero write.  This activity is done here rather than at memory
// because, in the standard ISA tests, the <tohost> addr is within the
// cacheable memory region, and therefore may never be written back to
// memory.  The actual address is supplied via the 'set_watch_tohost'
// method.  Standard ISA tests terminate by writing a non-zero value
// to the <tohost> addr. Bit [0] is always 1. Bits [n:1] specify which
// specific sub-test within the test failed.
//
// This logic is not meant to be included in the synthesizable version.
// ----------------


package Near_Mem_TCM;

// ================================================================
// BSV lib imports

import ConfigReg        :: *;
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
import TCM_AXI4_Adapter :: *;

import SoC_Map          :: *;
import Fabric_Defs      :: *;
import AXI4_Types       :: *;

// ================================================================
// Exports

export mkNear_Mem;

// ================================================================
// BRAM config constants

Bool config_output_register_BRAM = False;    // i.e., no output register
Bool load_file_is_binary_BRAM = False;       // file to be loaded is in hex format

// ================================================================
// Dummy server interfaces to stub off fence requests
function Server #(Token, Token) fv_dummy_server_stub;
   return (
      interface Server
         interface Put request;
            method Action put (Token t);
               noAction;
            endmethod
         endinterface
      interface Get response;
         method ActionValue #(Token) get;
            noAction;
            return (?);
         endmethod
      endinterface
   endinterface);
endfunction

// ================================================================
// TCM interfaces

interface ITCM_IFC;
   method Action  reset;

   // CPU side
   interface IMem_IFC  imem;

   // Fabric side -- unused for TCMs
   interface AXI4_Master_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) mem_master;

   // DMA server interface for back-door access to the ITCM
   interface AXI4_Slave_IFC #(Wd_Id_Dma, Wd_Addr_Dma, Wd_Data_Dma, Wd_User_Dma)  dma_server;

   // Inform core that DDR4 has been initialized and is ready to accept requests
   method Action ma_ddr4_ready;

   // Misc. status; 0 = running, no error
   (* always_ready *)
   method Bit #(8) mv_status;
endinterface

interface DTCM_IFC;
   method Action  reset;

   // CPU side
   interface DMem_IFC  dmem;

   // Fabric side
   // For accesses outside TCM (fabric memory, and memory-mapped I/O)
   interface Near_Mem_Fabric_IFC mem_master;

   // Inform core that DDR4 has been initialized and is ready to accept requests
   method Action ma_ddr4_ready;

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
   method Bit #(64) mv_tohost_value;
`endif

   // Misc. status; 0 = running, no error
   (* always_ready *)
   method Bit #(8) mv_status;
endinterface

// ================================================================
// Near_Mem_TCM module

(* synthesize *)
module mkNear_Mem (Near_Mem_IFC);

   // Verbosity: 0: quiet
   //            1: Requests and responses
   //            2: rule firings
   //            3: + detail
   Bit #(2) i_verbosity = 3;
   Bit #(2) d_verbosity = 3;

   FIFOF #(Token) f_reset_rsps <- mkFIFOF1;

   // ----------------
   // The TCM instantiations
   let itcm <- mkITCM (i_verbosity);
   let dtcm <- mkDTCM (d_verbosity); 

   // Fence request/response queues
   FIFOF #(Token) f_fence_req_rsp <- mkFIFOF1;

   // ================================================================
   // INTERFACE

   // ----------------
   // Reset
   interface Server server_reset;
      interface Put request;
         method Action put (Token t);
            itcm.reset;
            dtcm.reset;

            f_reset_rsps.enq (?);
         endmethod
      endinterface

      interface Get response = toGet (f_reset_rsps);
   endinterface

   // ----------------
   // IMem

   // CPU side
   interface imem = itcm.imem;

   // Fabric side
   interface imem_master = itcm.mem_master;

   // ----------------
   // DMem

   // CPU side
   interface dmem = dtcm.dmem;

   // Fabric side
   interface mem_master = dtcm.mem_master;

   // ----------------
   // XXX Fence.I, Fence -- all fences are nops, right?
   interface server_fence_i = fv_dummy_server_stub ();

   interface Server server_fence;
      interface Put request;
         method Action put (Fence_Ordering fo);
            f_fence_req_rsp.enq (?);
         endmethod
      endinterface
      interface response = toGet (f_fence_req_rsp);
   endinterface

`ifdef ISA_PRIV_S
   // ----------------
   // SFENCE_VMA: flush TLBs (no op in this module)
   method Action sfence_vma;
      noAction;
   endmethod
`endif

   // ----------------
   // Back-door from fabric into ITCM
   interface dma_server = itcm.dma_server;

   // ----------------
   // Indication that the DDR4 is ready to both AXI4 adapters
   method Action ma_ddr4_ready;
      dtcm.ma_ddr4_ready;
      itcm.ma_ddr4_ready;
   endmethod

   // ----------------
   // For ISA tests: watch memory writes to <tohost> addr

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
      dtcm.set_watch_tohost (watch_tohost, tohost_addr);
   endmethod

   method Bit #(64) mv_tohost_value = dtcm.mv_tohost_value;
`endif

   // Misc. status; 0 = running, no error. Since it reports only write errors,
   // the imem_port will never report a non-zero status
   method Bit #(8) mv_status = dtcm.mv_status;

endmodule: mkNear_Mem

// ================================================================
// Internal types and constants
typedef enum {
     MEM_IDLE                       // Reset done, ready for request
   , MEM_TCM_RSP                    // Response from TCM
   , MEM_MMIO_RSP                   // Response from MMIO
`ifdef ISA_A
   , MEM_AMO_RSP                    // Response from TCM for AMO ops
`endif
} Mem_State deriving (Bits, Eq, FShow);

function Instr fv_extract_instr (Addr pc, TCM_Word ram_out);
   Bool lower_word = (pc[(bits_per_byte_in_tcm_word - 1)] == 1'b0);
   return (lower_word ? ram_out [31:0] : ram_out [63:32]);
endfunction

// IMem_Port into the TCM
(* synthesize *)
module mkITCM #(Bit #(2) verbosity) (ITCM_IFC);

   // Verbosity: 0: quiet
   //            1: Requests and responses
   //            2: rule firings
   //            3: + detail
   Bit#(2) verbosity_mmio = 3;
   Bit#(2) verbosity_axi4 = 3;

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
   FIFOF #(Bit #(64))  f_mem_wdata  = dummy_FIFOF;

   // Access to fabric for non-TCM requests
   IMMIO_IFC        mmio            <- mkIMMIO (
      rg_imem_req, f_mem_req, f_mem_rdata, verbosity_mmio);

   TCM_AXI4_Adapter_IFC axi4_adapter<- mkTCM_AXI4_Adapter (
      verbosity_axi4, f_mem_req, f_mem_wdata, f_mem_rdata);

   // The TCM RAM - dual-ported due to backdoor to change IMem contents
`ifdef SYNTHESIS
   BRAM_DUAL_PORT_BE #(Addr, TCM_Word, Bytes_per_TCM_Word) itcm
      <- mkBRAMCore2BE (n_words_BRAM, config_output_register_BRAM);
`else
   BRAM_DUAL_PORT_BE #(Addr, TCM_Word, Bytes_per_TCM_Word) itcm
      <- mkBRAMCore2BELoad (n_words_BRAM, config_output_register_BRAM, "itcm.hex", load_file_is_binary_BRAM);
`endif

   // Back-door access to the TCM RAM
   let dma_port <- mkTCM_DMA_AXI4_Adapter (itcm.b, verbosity);

   // The "front-door" to the itcm (port A)
   let iram = itcm.a;

   // Connect MMIO's memory interface to AXI4 fabric adapter
   mkConnection (mmio.g_mem_req,       axi4_adapter.p_mem_single_req);
   mkConnection (mmio.p_mem_read_data, axi4_adapter.g_mem_single_read_data);

   SoC_Map_IFC soc_map <- mkSoC_Map;

   // Drive response from TCM -- loads, exceptions
   rule rl_tcm_rsp (rg_imem_state == MEM_TCM_RSP);
      // extract the instruction from the RAM data
      let instr = fv_extract_instr (rg_imem_req.pc, iram.read);

      if (verbosity >= 1)
         $display ("%0d: %m.rl_tcm_rsp: pc %08h data %08h"
            , cur_cycle, rg_imem_req.pc, instr);

      // drive the outputs
      dw_valid       <= rg_result_valid;
      dw_exc         <= rg_exc;
      dw_exc_code    <= rg_exc_code;
      dw_instr       <= instr;

      // Back to idle for the next request
      // rg_imem_state  <= MEM_IDLE;
   endrule

   // Drive response from MMIO
   rule rl_mmio_rsp (rg_imem_state == MEM_MMIO_RSP);
      match { .err, .ld_val } = mmio.result;
      dw_valid          <= True;
      dw_instr          <= ld_val;
      dw_exc            <= err;
      dw_exc_code       <= exc_code_INSTR_ACCESS_FAULT;
      // rg_imem_state     <= MEM_IDLE;
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

      // Assert: rg_imem_state == MEM_IDLE
      rg_imem_req     <= imem_req;

      // for all the checks relating to the soc-map
      Fabric_Addr fabric_pc = fv_Addr_to_Fabric_Addr (pc);

      // address checks
      // mis-alignment
      if (!fn_is_aligned (f3[1:0], pc)) begin
         rg_result_valid   <= True;
         rg_exc            <= True;
         rg_imem_state     <= MEM_TCM_RSP;
         rg_exc_code       <= exc_code_INSTR_ADDR_MISALIGNED;
      end

      // serviced by the TCM
      else if (soc_map.m_is_itcm_addr_1 (fabric_pc)) begin
         rg_result_valid   <= True;
         rg_exc            <= False;
         rg_imem_state     <= MEM_TCM_RSP;

         // Initiate RAM read
         Addr word_addr = fv_Fabric_Addr_to_Addr (
            (fabric_pc - soc_map.m_itcm_addr_base) >> bits_per_byte_in_tcm_word);
         iram.put (0, word_addr, ?);
      end

      // outside TCM address space -- send to MMIO
      else begin
         rg_result_valid   <= False;
         rg_exc            <= False;
         rg_imem_state     <= MEM_MMIO_RSP;
         mmio.start;
      end
   endrule

   // ----------------
   // INTERFACE

   method Action reset;
      rg_result_valid   <= False;
      rg_imem_state     <= MEM_IDLE;
      dma_port.reset;

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

      // Choose the right instruction from the Bit#(64)
      method Instr instr = dw_instr;

      method Bool exc = dw_exc;
      method Exc_Code exc_code = dw_exc_code;
      method WordXL tval = rg_imem_req.pc;   // XXX What is this?
   endinterface

   // Fabric side
   // For accesses outside TCM (fabric memory)
   interface mem_master = axi4_adapter.mem_master;

   // Back-door from fabric into ITCM
   interface dma_server = dma_port.dma_server;

   // ----------------------------------------------------------------
   // Misc. control and status
   method Action ma_ddr4_ready = axi4_adapter.ma_ddr4_ready;

   // Misc. status; 0 = running, no error
   method Bit#(8) mv_status = axi4_adapter.mv_status;
endmodule: mkITCM

// ================================================================
// DMem

// DMem_Port into the TCM
(* synthesize *)
module mkDTCM #(Bit #(2) verbosity) (DTCM_IFC);

   // Verbosity: 0: quiet
   //            1: Requests and responses
   //            2: rule firings
   //            3: + detail
   Bit#(2) verbosity_mmio = 0;
   Bit#(2) verbosity_axi4 = 0;

   // Module state
   Reg #(Mem_State)           rg_dmem_state     <- mkReg (MEM_IDLE);
   Reg #(Bool)                rg_result_valid   <- mkReg (False);
   Reg #(Bool)                rg_exc            <- mkReg (False);
   Reg #(Exc_Code)            rg_exc_code       <- mkRegU;

   SoC_Map_IFC soc_map <- mkSoC_Map;

   // ----------------
   // Reservation regs for AMO LR/SC (Load-Reserved/Store-Conditional)

`ifdef ISA_A
   Reg #(Bool)                rg_lrsc_valid     <- mkReg (False);
   Reg #(PA)                  rg_lrsc_pa        <- mkRegU; // PA for an active LR
   Reg #(MemReqSize)          rg_lrsc_size      <- mkRegU;
   Reg #(Maybe #(Bit #(1)))   rg_lrsc_word64    <- mkReg (tagged Invalid);
`endif

   // Current request from the CPU
   Reg #(MMU_Cache_Req) rg_req <- mkRegU;

   Reg #(Bool)                dw_valid          <- mkDWire (False);
   Reg #(Bool)                dw_exc            <- mkDWire (False);
   Reg #(Exc_Code)            dw_exc_code       <- mkDWire (?);
   Reg #(Bit #(64))           dw_word64         <- mkDWire (?);
   Reg #(Bit #(64))           dw_final_st_val   <- mkDWire (?);

`ifdef WATCH_TOHOST
   // See NOTE: "tohost" above.
   // "tohost" addr on which to monitor writes, for standard ISA tests.
   // These are set by the 'set_watch_tohost' method but are otherwise read-only.
   Reg #(Bool)                rg_watch_tohost   <- mkReg (False);
   Reg #(Bit #(64))           rg_tohost_addr    <- mkReg ('h_8000_1000);
   Reg #(Bit #(64))           rg_tohost_value   <- mkReg (0);
`endif

   // Requests and data to/from memory (AXI4 fabric)
   FIFOF #(Single_Req)        f_mem_req         <- mkFIFOF1;
   FIFOF #(Bit #(64))         f_mem_wdata       <- mkFIFOF1;
   FIFOF #(Read_Data)         f_mem_rdata       <- mkFIFOF1;

   // Access to fabric for non-TCM requests
   DMMIO_IFC                  mmio              <- mkDMMIO (
      rg_req, f_mem_req, f_mem_wdata, f_mem_rdata, verbosity_mmio);

   TCM_AXI4_Adapter_IFC axi4_adapter<- mkTCM_AXI4_Adapter (
      verbosity_axi4, f_mem_req, f_mem_wdata, f_mem_rdata);

   // The TCM RAM - dual-ported due to simultaneous loads and stores when
   // integrated with pipelined processors. When integrated with cores like
   // Magritte, this is strictly not possible. However, the rules rl_tcm_rsp and
   // rl_req have not been written to be mutually exclusive. For a non-pipelined
   // processor, it is possible to work with a single-ported BRAM while sacrificing
   // concurrency between the response and request phases.
`ifdef SYNTHESIS
   BRAM_DUAL_PORT_BE #(Addr, TCM_Word, Bytes_per_TCM_Word) dtcm <- mkBRAMCore2BE (
      n_words_BRAM, config_output_register_BRAM);
`else
   BRAM_DUAL_PORT_BE #(Addr, TCM_Word, Bytes_per_TCM_Word) dtcm <- mkBRAMCore2BELoad (
      n_words_BRAM, config_output_register_BRAM, "dtcm.hex", load_file_is_binary_BRAM);
`endif

   // Connect MMIO's memory interface to AXI4 fabric adapter
   mkConnection (mmio.g_mem_req,       axi4_adapter.p_mem_single_req);
   mkConnection (mmio.g_write_data,    axi4_adapter.p_mem_single_write_data);
   mkConnection (mmio.p_mem_read_data, axi4_adapter.g_mem_single_read_data);

   let dtcm_rd_port = dtcm.a;
   let dtcm_wr_port = dtcm.b;

   // Continuous DTCM output
   let ram_out  = fn_extract_and_extend_bytes (rg_req.f3, rg_req.va, pack (dtcm_rd_port.read));

   // ----------------------------------------------------------------
   // For debugging/tracing: format the CPU request

   function Fmt show_CPU_req (CacheOp op, Bit #(3) f3, Addr addr, Bit#(64) st_value);
      return $format ("Req (op ", fshow (op), ", f3 0x%0h, addr %0h, st_value 0x%0h)", f3, addr, st_value);
   endfunction

   // ----------------------------------------------------------------
   // BEHAVIOR
   // This function writes to the TCM RAM
   function Action fa_write_to_ram (Addr tcm_byte_addr, Bit #(64) st_value);
      action
      let f3 = rg_req.f3;

      match {.byte_en, .ram_st_value} = fn_byte_adjust_write (f3, tcm_byte_addr, st_value);
      Addr tcm_word_addr = (tcm_byte_addr >> bits_per_byte_in_tcm_word);

      if (verbosity >= 1)
         $display ("      (RAM byte_en %08b) (RAM data %016h)", byte_en, ram_st_value);

      dtcm_wr_port.put (byte_en, tcm_word_addr, ram_st_value);

      // XXX is this even used by the CPU?
      // dw_final_st_val <= extend (ram_st_value);

`ifdef WATCH_TOHOST
      // ----------------
      // "tohost" addr on which to monitor writes, for standard ISA tests.
      // See NOTE: "tohost" above.
      if (  (rg_watch_tohost)
         && (rg_req.op == CACHE_ST)
         && (zeroExtend (rg_req.va) == rg_tohost_addr)
         && (ram_st_value != 0)) begin
         rg_tohost_value <= ram_st_value;
         let test_num = (ram_st_value >> 1);
         $display ("%0d: %m.fa_watch_tohost", cur_cycle);
         if (test_num == 0) $write ("    PASS");
         else               $write ("    FAIL <test_%0d>", test_num);
         $display ("  (<tohost>  addr %08h  data %08h)", rg_req.va, ram_st_value);
      end
`endif
      endaction
   endfunction
   

   // --------
`ifdef ISA_A
   // This function generates the store word for the TCM depending
   // on the opcode. For AMO ops might involve some computation
   // with read data from the RAM. In case of SC fail, it returns
   // a valid value for the word64 method
   function ActionValue #(Maybe #(Bit #(1))) fav_amo_write_to_ram (Bit #(64) ram_data);
      actionvalue
         Fabric_Addr fabric_va = fv_Addr_to_Fabric_Addr (rg_req.va);
         Addr tcm_byte_addr = fv_Fabric_Addr_to_Addr (fabric_va - soc_map.m_dtcm_addr_base);
         let st_value  = rg_req.st_value;
         let f3        = rg_req.f3;
         Maybe #(Bit #(1)) lrsc_word64 = tagged Invalid;
         Bool sc_fail = False;

         // AMO SC request
         if (fv_is_AMO_SC (rg_req)) begin
            if (rg_lrsc_valid && (rg_lrsc_pa == rg_req.va)) begin
               if (verbosity >= 1) begin
                  $display ("%0d: %m.fav_write_to_ram: SC success", cur_cycle);
                  $display ("      (va %08h) (data %016h)", rg_req.va, st_value);
               end
               // SC success: cancel LR/SC reservation
               rg_lrsc_valid <= False;
               lrsc_word64 = tagged Valid 1'h0;
            end
            else begin 
               if (verbosity >= 1) begin
                  $display ("%0d: %m.fav_write_to_ram: SC fail", cur_cycle);
                  $display ("      (va %08h) (data %016h)", rg_req.va, st_value);
               end
               lrsc_word64 = tagged Valid 1'h1;
               sc_fail = True;
            end
         end

         // All AMO read-modify-writes (i.e., AMO other than LR and SC)
         else if (fv_is_AMO_RMW (rg_req)) begin
            Fmt fmt_op = fshow_f5_AMO_op (rg_req.amo_funct7 [6:2]);
            if (verbosity >= 1) begin
               $display ("%0d: %m.fav_write_to_ram: AMO ", cur_cycle, fmt_op);
               $display ("      (va %08h) (rs2_val %016h) (f3 %03b)", rg_req.va, st_value, f3);
               $display ("      (load-result %016h)", ram_data);
            end

            let size_code  = f3 [1:0];
            // Do the AMO op on the loaded value and recalculate the st_value
            match {.new_ld_val, .value_after_op} = fv_amo_op (
               size_code, rg_req.amo_funct7 [6:2], ram_data, st_value);

            if (verbosity >= 1)
               $display ("      ", fmt_op, " (%016h, %016h) -> %016h", ram_data, st_value, value_after_op);

            st_value = pack (value_after_op);

            // Cancel LR/SC reservation if this store is for this addr
            if (rg_lrsc_pa == rg_req.va) rg_lrsc_valid <= False;
         end

         if (verbosity >= 1) begin
            $display ("%0d: %m.fav_write_to_ram: ST", cur_cycle);
            $display ("      (va %08h) (data %016h)", rg_req.va, st_value);
         end

         if (! sc_fail) fa_write_to_ram (tcm_byte_addr, st_value);

         return (lrsc_word64);
      endactionvalue
   endfunction 


   // --------
   // Process AMO ops
   rule rl_amo_rsp (rg_dmem_state == MEM_AMO_RSP);
      Maybe #(Bit #(1)) lrsc_word64 = tagged Invalid;

      // If the request involves a store, initiate the write In the case of RMWs, it will
      // involve the current RAM output as well.
      if (fv_is_AMO_SC (rg_req) || fv_is_AMO_RMW (rg_req)) begin
         lrsc_word64 <- fav_amo_write_to_ram (ram_out);
      end

      // For SC stores, the status (success (0), fail (1)) needs to be returned
      rg_lrsc_word64 <= lrsc_word64;

      // For LR ops, update reservation regs
      if (fv_is_AMO_LR (rg_req)) begin
         if (verbosity >= 1) $display ("%0d: %m.rl_tcm_rsp: LR-hit", cur_cycle);
         rg_lrsc_valid <= True;
         rg_lrsc_pa    <= rg_req.va;
         rg_lrsc_size  <= rg_req.f3 [1:0];
      end
   endrule
`endif


   // --------
   // Drive response from TCM -- loads, LR, exceptions
   rule rl_tcm_rsp (rg_dmem_state == MEM_TCM_RSP);

      // drive the outputs
      dw_valid       <= rg_result_valid;
      dw_exc         <= rg_exc;
      dw_exc_code    <= rg_exc_code;

      Bit #(64) word64 = ?;
`ifdef ISA_A
      if (isValid (rg_lrsc_word64))
         // For SC stores, the status (success (0), fail (1)) needs to be returned
         word64 = extend (rg_lrsc_word64.Valid);
      else
`endif
         // For CACHE_LD and LR, simply forward the RAM output
         word64 = ram_out;

      dw_word64 <= word64;

      if (verbosity >= 1)
         $display ("%0d: %m.rl_tcm_rsp: (va %08h) (word64 %016h)"
            , cur_cycle, rg_req.va, word64);
   endrule


   // --------
   // Drive response from MMIO
   rule rl_mmio_rsp (rg_dmem_state == MEM_MMIO_RSP);
      match { .err, .ld_val, .final_st_val } = mmio.result;
      dw_valid          <= True;
      dw_word64         <= ld_val;
      // XXX is this even used by the CPU?
      // dw_final_st_val   <= final_st_val;
      dw_exc            <= err;
      dw_exc_code       <= fv_exc_code_access_fault (rg_req);
      // rg_dmem_state     <= MEM_IDLE;

      if (verbosity >= 1)
         $display ("%0d: %m.rl_mmio_rsp: (word64 %016h) (final_st_val %016h)"
            , cur_cycle, ld_val, final_st_val);
   endrule

   // This rule is basically the body of method ma_req; decoupling
   // through a wire affords scheduling flexibility.
   //
   // Registers an incoming request and starts the TCM/MMIO probe 
   // The only situation when the rl_req cannot fire is when the DMEM is in the AMO write phase
   Wire #(MMU_Cache_Req) w_dmem_req <- mkWire;
   (* fire_when_enabled *)
`ifdef ISA_A
   rule rl_req (rg_dmem_state != MEM_AMO_RSP);
`else
   rule rl_req;
`endif
      let dmem_req = w_dmem_req;
      let op = dmem_req.op;
      let f3 = dmem_req.f3;
      let addr = dmem_req.va;
      let st_value = dmem_req.st_value;
`ifdef ISA_A
      let amo_funct7 = dmem_req.amo_funct7;
`endif

      // Note: ignoring all VM args for this version of Near_Mem_TCM
      if (verbosity > 1)
         $display ("%0d: %m.rl_req: ", cur_cycle, show_CPU_req (op, f3, addr, st_value));

      // Assert: rg_dmem_state == MEM_IDLE

      // register the request for the response stage
      rg_req <= dmem_req;

      // for all the checks relating to the soc-map
      Fabric_Addr fabric_addr = fv_Addr_to_Fabric_Addr (addr);
      let tcm_byte_addr = fv_Fabric_Addr_to_Addr (fabric_addr - soc_map.m_dtcm_addr_base);

      // Check if f3 is legal, and if f3 and addr are compatible
      if (! fn_is_aligned (f3 [1:0], addr)) begin
         // Misaligned accesses not supported
         rg_result_valid   <= True;
         rg_exc            <= True;
         rg_dmem_state     <= MEM_TCM_RSP;
         rg_exc_code       <= fv_exc_code_misaligned (dmem_req);
      end

      // TCM reqs
      else if (soc_map.m_is_dtcm_addr (fabric_addr)) begin
         rg_result_valid <= True;
         rg_exc          <= False;

         // The read/write to the RAM is initiated here. If it is a
         // AMO store, the actual write happens in the AMO phase
         if (op == CACHE_ST) begin
            fa_write_to_ram (tcm_byte_addr, st_value);
`ifdef ISA_A
            // Cancel LR/SC reservation if this store is for the reserved addr
            // TODO : should we cancel it on ANY store?
            if (rg_lrsc_pa == addr) rg_lrsc_valid <= False;
`endif
         end

         else begin
            let word_addr = (tcm_byte_addr >> bits_per_byte_in_tcm_word);
            dtcm_rd_port.put (0, word_addr, ?);
         end

         // The next state depends on the op. If it is a LD/ST/LR move to the response state
         // which allows the module to process the next request. If it is a RMW AMO op, move to
         // the AMO state, effectively introducing a one-cycle bubble.
`ifdef ISA_A
         if (fv_is_AMO_SC (rg_req) || fv_is_AMO_RMW (rg_req))
            rg_dmem_state <= MEM_AMO_RSP;
         else begin
            // Clear the lrsc_word64
            rg_lrsc_word64 <= tagged Invalid;

`endif
            rg_dmem_state <= MEM_TCM_RSP;
`ifdef ISA_A
         end
`endif

      end

      // non-TCM request (outside TCM addr range: could be memory or I/O on the fabric )
      else begin
         rg_result_valid   <= False;
         rg_exc            <= False;
         rg_dmem_state     <= MEM_MMIO_RSP;
         mmio.start;
      end
   endrule

   // ----------------------------------------------------------------
   // INTERFACE

   method Action reset ();
      rg_dmem_state <= MEM_IDLE;
      rg_result_valid <= False;

      if (verbosity > 1)
         $display ("%0d: %m.reset", cur_cycle);
   endmethod

   // CPU side
   interface DMem_IFC dmem;
      // CPU interface: request
      // NOTE: this has no flow control: CPU should only invoke it when consuming prev output
      method Action  req (
           CacheOp op
         , Bit #(3) f3
`ifdef ISA_A
         , Bit #(7) amo_funct7
`endif
         , WordXL addr
         , Bit#(64) st_value
`ifdef ISA_PRIV_S
         // The following  args for VM only    // { VM_Mode, ASID, PPN_for_page_table }
         , Priv_Mode  priv
         , Bit #(1)   sstatus_SUM
         , Bit #(1)   mstatus_MXR
         , WordXL     satp
`endif
         );
         w_dmem_req <= MMU_Cache_Req {
           op        : op
         , f3        : f3
         , va        : addr
         , st_value  : st_value
`ifdef ISA_A
         , amo_funct7: amo_funct7
`endif
         };

      endmethod

      method Bool       valid       = dw_valid;
      method Bool       exc         = dw_exc;
      method Exc_Code   exc_code    = dw_exc_code;
      method Bit#(64)   word64      = dw_word64;
      method Bit #(64)  st_amo_val  = dw_final_st_val;
   endinterface

   // Fabric side
   // For accesses outside TCM (fabric memory, and memory-mapped I/O)
   interface mem_master = axi4_adapter.mem_master;

   // ----------------------------------------------------------------
   // Misc. control and status

   // ----------------
   // For ISA tests: watch memory writes to <tohost> addr (see NOTE: "tohost" above)

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
      rg_watch_tohost <= watch_tohost;
      rg_tohost_addr  <= tohost_addr;
      $display ("%0d: %m.set_watch_tohost: watch %0d, addr %08h",
                cur_cycle, watch_tohost, tohost_addr);
   endmethod

   method Bit #(64) mv_tohost_value;
      return rg_tohost_value;
   endmethod
`endif

   method Action ma_ddr4_ready = axi4_adapter.ma_ddr4_ready;

   // Misc. status; 0 = running, no error
   method Bit#(8) mv_status = axi4_adapter.mv_status;
endmodule

// ================================================================

endpackage: Near_Mem_TCM
