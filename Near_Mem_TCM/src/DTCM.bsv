// Copyright (c) 2016-2021 Bluespec, Inc. All Rights Reserved.
//
// This package implements the DTCM and was hived off from
// Near_Mem_TCM for maintainability reasons. Please refer to the
// introduction in Near_Mem_TCM for details.
//
// ----------------

package DTCM;

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
import SoC_Map          :: *;
import Fabric_Defs      :: *;
import AXI4_Types       :: *;

`ifdef FABRIC_AHBL
import AHBL_Types       :: *;
import AHBL_Defs        :: *;
import TCM_AHBL_Adapter :: *;
`endif


// ================================================================
// BRAM config constants

Bool config_output_register_BRAM = False; // no output register
Bool load_file_is_binary_BRAM = False;    // load file is in hex format

// ================================================================
// Interface Definition

interface DTCM_IFC;
   method Action  reset;

   // CPU side
   interface DMem_IFC  dmem;

   // Fabric Interfaces
`ifdef FABRIC_AXI4
`ifdef DUAL_FABRIC
   // For accesses outside TCM (fabric memory, and memory-mapped I/O)
   interface AXI4_Master_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) nmio_master;
`else
   // For accesses outside TCM (fabric memory, and memory-mapped I/O)
   interface AXI4_Master_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) mem_master;
`endif
`endif

`ifdef FABRIC_AHBL
   // For accesses outside TCM (fabric memory, and memory-mapped I/O)
   interface AHBL_Master_IFC #(AHB_Wd_Data) mem_master;
`endif

   // DMA server interface for back-door access to the DTCM
   interface AXI4_Slave_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User)  dma_server;

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
   method Bit #(64) mv_tohost_value;
`endif
endinterface

// ================================================================
// Here begins the module
//
(* synthesize *)
module mkDTCM #(Bit #(2) verbosity) (DTCM_IFC);

   // Verbosity: 0: quiet
   //            1: Requests and responses
   //            2: rule firings
   //            3: + detail

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
   
   Reg #(TCM_Word)            dw_word           <- mkDWire (?);
   Reg #(TCM_Word)            dw_final_st_val   <- mkDWire (?);

`ifdef WATCH_TOHOST
   // See NOTE: "tohost" above.
   // "tohost" addr on which to monitor writes, for standard ISA tests.
   // These are set by the 'set_watch_tohost' method but are otherwise read-only.
   Reg #(Bool)                rg_watch_tohost   <- mkReg (False);
   Reg #(Fabric_Addr)         rg_tohost_addr    <- mkRegU;
   Reg #(Fabric_Data)         rg_tohost_value   <- mkReg (0);
`endif

   // Requests and data to/from non-TCM memory (external fabrics)
`ifdef DUAL_FABRIC
   FIFOF #(Single_Req)        f_nmio_req        <- mkFIFOF1;
   FIFOF #(Bool)              f_is_mem_req      <- mkFIFOF1;
`endif
   FIFOF #(Single_Req)        f_mem_req         <- mkFIFOF1;
`ifdef NM32
   FIFOF #(Bit #(32))         f_mem_wdata       <- mkFIFOF1;
`else
   FIFOF #(Bit #(64))         f_mem_wdata       <- mkFIFOF1;
`endif
   FIFOF #(Read_Data)         f_mem_rdata       <- mkFIFOF1;

   // Access to fabric for non-TCM requests
   Bit#(2) verbosity_mmio = verbosity;
   DMMIO_IFC                  mmio              <- mkDMMIO (
        rg_req
      , f_mem_req
`ifdef DUAL_FABRIC
      , f_nmio_req
      , f_is_mem_req.first
`endif
      , f_mem_wdata, f_mem_rdata, verbosity_mmio);

   // What fabric do we use -- AXI4 or AHBL. Select any one, or
   // both, but then you must enable DUAL_FABRIC too
   Bit#(2) verbosity_fabric = verbosity;
`ifdef FABRIC_AXI4
`ifdef DUAL_FABRIC
   TCM_AXI4_Adapter_IFC nmio_fabric_adapter<- mkTCM_AXI4_Adapter (
      verbosity_fabric, f_nmio_req, f_mem_wdata, f_mem_rdata);
`else
   TCM_AXI4_Adapter_IFC fabric_adapter<- mkTCM_AXI4_Adapter (
      verbosity_fabric, f_mem_req, f_mem_wdata, f_mem_rdata);
`endif
`endif

`ifdef FABRIC_AHBL
   TCM_AHBL_Adapter_IFC fabric_adapter<- mkTCM_AHBL_Adapter (
      verbosity_fabric, f_mem_req, f_mem_wdata, f_mem_rdata);
`endif

   // The TCM RAM - dual-ported due to simultaneous loads and
   // stores when integrated with pipelined processors. When
   // integrated with cores like Magritte, this is strictly not
   // possible. However, the rules rl_tcm_rsp and rl_req have not
   // been written to be mutually exclusive. For a non-pipelined
   // processor, it is possible to work with a single-ported BRAM
   // without sacrificing concurrency (there isn't any)
`ifdef MICROSEMI
   BRAM_DUAL_PORT_BE #(DTCM_INDEX
                     , TCM_Word
                     , Bytes_per_TCM_Word) dtcm <- mkBRAMCore2BE (n_words_DBRAM
                                                                , config_output_register_BRAM);
`else
   BRAM_DUAL_PORT_BE #(DTCM_INDEX
                     , TCM_Word
                     , Bytes_per_TCM_Word) dtcm <- mkBRAMCore2BELoad (n_words_DBRAM
                                                                    , config_output_register_BRAM
                                                                    , "/tmp/e342znd.hex"
                                                                    , load_file_is_binary_BRAM);
`endif

   // Port A is for CPU accesses, while Port B is for DMA accesses.
   // These sets of accesses are mutually exclusive as when the DMA
   // is accessing the DTCM (TCM loader or GDB), the CPU is
   // stalled.
   let dtcm_cpu_port = dtcm.a;
   let dtcm_dma_port = dtcm.b;

   // In addition to LD/ST, DMA/debug accesses read and write to the
   // DTCM. Back-door debug/DMA access to the DTCM shares the 'b' port
   let dma_port <- mkTCM_DMA_AXI4_Adapter (dtcm_dma_port, verbosity);
   AXI4_Deburster_IFC #(
      Wd_Id, Wd_Addr, Wd_Data, Wd_User) deburstr <- mkAXI4_Deburster;
   mkConnection (deburstr.to_slave, dma_port.dma_server);
   let dma_port_extnl = deburstr.from_master;

   // Continuous DTCM output
   let ram_out  = fn_extract_and_extend_bytes (
      rg_req.f3, rg_req.va, pack (dtcm_cpu_port.read));

   // ----------------------------------------------------------------
   // For debugging/tracing: format the CPU request

   function Fmt show_CPU_req (
`ifdef NM32
      CacheOp op, Bit #(3) f3, Addr addr, Bit#(32) st_value);
`else
      CacheOp op, Bit #(3) f3, Addr addr, Bit#(64) st_value);
`endif
      return $format ("Req (op ", fshow (op)
                    , ", f3 0x%0h, addr %0h, st_value 0x%0h)"
                    , f3, addr, st_value);
   endfunction

   // ----------------------------------------------------------------
   // BEHAVIOR
   // This function writes to the TCM RAM
   function Action fa_write_to_ram (
`ifdef NM32
      Addr byte_addr, MMU_Cache_Req req, Bit #(32) st_value);
`else
      Addr byte_addr, MMU_Cache_Req req, Bit #(64) st_value);
`endif
      action
	 match {.byte_en, .ram_st_value} = fn_byte_adjust_write (
            req.f3, byte_addr, st_value);
	 DTCM_INDEX word_addr = truncate(byte_addr >> bits_per_byte_in_tcm_word);

	 if (verbosity >= 1)
            $display ("      (RAM byte_en %08b) (RAM addr %08h) (RAM data %016h)"
               , byte_en, word_addr, ram_st_value);

	 dtcm_cpu_port.put (byte_en, word_addr, ram_st_value);

	 // XXX is this even used by the CPU?
	 // dw_final_st_val <= extend (ram_st_value);

      endaction
   endfunction


   // --------
`ifdef ISA_A
   // This function generates the store word for the TCM depending
   // on the opcode. For AMO ops might involve some computation
   // with read data from the RAM. In case of SC fail, it returns
   // a valid value for the word64 method
`ifdef NM32
   function ActionValue #(Maybe #(Bit #(1))) fav_amo_write_to_ram (Bit #(32) ram_data);
`else
   function ActionValue #(Maybe #(Bit #(1))) fav_amo_write_to_ram (Bit #(64) ram_data);
`endif
      actionvalue
         Addr byte_addr = rg_req.va;
         let st_value  = rg_req.st_value;
         let f3        = rg_req.f3;
         Maybe #(Bit #(1)) lrsc_word64 = tagged Invalid;
         Bool sc_fail = False;

         // AMO SC request
         if (fv_is_AMO_SC (rg_req)) begin
            if (rg_lrsc_valid && (rg_lrsc_pa == rg_req.va)) begin
               if (verbosity >= 1) begin
                  $display ("%0d: %m.fav_amo_write_to_ram: SC success", cur_cycle);
                  $display ("      (va %08h) (data %016h)", rg_req.va, st_value);
               end
               // SC success: cancel LR/SC reservation
               rg_lrsc_valid <= False;
               lrsc_word64 = tagged Valid 1'h0;
            end
            else begin
               if (verbosity >= 1) begin
                  $display ("%0d: %m.fav_amo_write_to_ram: SC fail", cur_cycle);
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
               $display ("%0d: %m.fav_amo_write_to_ram: AMO ", cur_cycle, fmt_op);
               $display ("      (va %08h) (rs2_val %016h) (f3 %03b)", rg_req.va, st_value, f3);
               $display ("      (load-result %016h)", ram_data);
            end

            let size_code  = f3 [1:0];
            // Do the AMO op on the loaded value and recalculate the st_value
            match {.new_ld_val, .value_after_op} = fv_amo_op (
`ifdef NM32
                          rg_req.amo_funct7 [6:2], ram_data, st_value);
`else
               size_code, rg_req.amo_funct7 [6:2], ram_data, st_value);
`endif

            if (verbosity >= 1)
               $display ("      ", fmt_op, " (%016h, %016h) -> %016h", ram_data, st_value, value_after_op);

            st_value = pack (value_after_op);

            // Cancel LR/SC reservation if this store is for this addr
            if (rg_lrsc_pa == rg_req.va) rg_lrsc_valid <= False;
         end

         if (! sc_fail) fa_write_to_ram (byte_addr, rg_req, st_value);

         return (lrsc_word64);
      endactionvalue
   endfunction

   // --------
   // Process AMO ops
   rule rl_amo_rsp (rg_dmem_state == MEM_AMO_RSP);
      Maybe #(Bit #(1)) lrsc_word64 = tagged Invalid;

      // If the request involves a store, initiate the write. For
      // RMWs, it will involve the current RAM output as well.
      if (fv_is_AMO_SC (rg_req) || fv_is_AMO_RMW (rg_req)) begin
         lrsc_word64 <- fav_amo_write_to_ram (ram_out);
      end

      // For SC stores, the status (success (0), fail (1)) needs to be returned
      rg_lrsc_word64 <= lrsc_word64;

      // For LR ops, update reservation regs
      if (fv_is_AMO_LR (rg_req)) begin
         if (verbosity >= 1) $display ("%0d: %m.rl_amo_rsp: LR-hit", cur_cycle);
         rg_lrsc_valid <= True;
         rg_lrsc_pa    <= rg_req.va;
         rg_lrsc_size  <= rg_req.f3 [1:0];
      end

      rg_dmem_state <= MEM_TCM_RSP;
   endrule
`endif


   // --------
   // Drive response from TCM -- loads, LR, exceptions
   rule rl_tcm_rsp (rg_dmem_state == MEM_TCM_RSP);
      // drive the outputs
      dw_valid       <= rg_result_valid;
      dw_exc         <= rg_exc;
      dw_exc_code    <= rg_exc_code;

      TCM_Word rsp_word = ?;
`ifdef ISA_A
      if (isValid (rg_lrsc_word64))
         // For SC stores, the status (success (0), fail (1)) needs
         // to be returned
         rsp_word = extend (rg_lrsc_word64.Valid);
      else
`endif
         // For CACHE_LD and LR, simply forward the RAM output
         rsp_word = ram_out;

      dw_word <= rsp_word;

      if (verbosity > 2)
         $display ("%0d: %m.rl_tcm_rsp: (va %08h) (rsp_word %016h)"
                 , cur_cycle, rg_req.va, rsp_word);

      if (verbosity > 2)
         $display ("     (ram_out %016h) ", ram_out
                 , "(rg_req ", fshow (rg_req), " )");
   endrule


   // --------
   // Drive response from MMIO
   rule rl_mmio_rsp (rg_dmem_state == MEM_MMIO_RSP);
      match { .err, .ld_val, .final_st_val } = mmio.result;
      dw_valid          <= True;
      dw_word           <= ld_val;
      // XXX is this even used by the CPU?
      // dw_final_st_val   <= final_st_val;
      dw_exc            <= err;
      dw_exc_code       <= fv_exc_code_access_fault (rg_req);

`ifdef DUAL_FABRIC
      f_is_mem_req.deq;
`endif

      if (verbosity >= 1)
         $display ("%0d: %m.rl_mmio_rsp: (rsp_word %016h) (final_st_val %016h)"
            , cur_cycle, ld_val, final_st_val);
   endrule

   Wire #(MMU_Cache_Req) w_dmem_req <- mkWire;

   // --------
   // Compiler directives
   // NMIO and external requests (and responses) are mutually exclusive
`ifdef DUAL_FABRIC
   (* mutually_exclusive = "nmio_fabric_adapter_rl_read_data, fabric_adapter_rl_read_response" *)
   (* mutually_exclusive = "nmio_fabric_adapter_rl_write_data, fabric_adapter_rl_write_response" *)
`endif

   // DMA accesses and CPU accesses should be mutually exclusive (especially for writes)
   (* mutually_exclusive = "dma_port_rl_dma_wr_req, rl_amo_rsp" *)
   (* mutually_exclusive = "dma_port_rl_dma_rd_req, rl_amo_rsp" *)
   (* mutually_exclusive = "dma_port_rl_dma_rd_req, rl_req" *)
   (* mutually_exclusive = "dma_port_rl_dma_wr_req, rl_req" *)

   // --------
   // This rule is the body of method ma_req; decoupling through a
   // wire affords scheduling flexibility.
   //
   // Registers an incoming request and starts the TCM/MMIO probe
   // The only situation when the rl_req cannot fire is when the
   // DMEM is in the AMO write phase
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

`ifdef WATCH_TOHOST
      // ----------------
      // "tohost" addr on which to monitor writes, for standard ISA
      // tests. See NOTE: "tohost" in Near_Mem_TCM. This check is
      // a testbench feature and is independent of the address
      // check for the request.
      if (  (rg_watch_tohost)
         && (op == CACHE_ST)
         && (zeroExtend (addr) == rg_tohost_addr)
         && (st_value != 0)) begin
`ifdef NM32
`ifdef RV32
         rg_tohost_value <= st_value;
`elsif RV64
         rg_tohost_value <= extend (st_value);
`endif
`else
`ifdef RV32
         rg_tohost_value <= truncate (st_value);
`elsif RV64
         rg_tohost_value <= st_value;
`endif
`endif
         let test_num = (st_value >> 1);
         $display ("%0d: %m.watch_tohost", cur_cycle);
         if (test_num == 0) $write ("    PASS");
         else               $write ("    FAIL <test_%0d>", test_num);
         $display ("  (<tohost>  addr %08h  data %08h)", addr, st_value);
	 $finish(0);
      end
`endif

      // Note: ignoring all VM args for this version of Near_Mem_TCM
      if (verbosity > 1)
         $display ("%0d: %m.rl_req: ", cur_cycle
                 , show_CPU_req (op, f3, addr, st_value));

      // register the request for the response stage
      rg_req <= dmem_req;

      // The read to the RAM is initiated here speculatively. This
      // read is specualtive because we still don't know if the
      // address is good and is indeed meant for the TCM. Since it
      // is a read, there is no side-effect and can be safely
      // initiated without waiting for all the results to come in
      // about the address. If it is a CACHE_ST or AMO store, the
      // actual write happens in the response phase or AMO phase
      if (op != CACHE_ST) begin
         let word_addr = (addr >> bits_per_byte_in_tcm_word);
         dtcm_cpu_port.put (0, truncate(word_addr), ?);
         if (verbosity >= 2)
            $display ("   dtcm_cpu_port.put (word_addr %08h)", word_addr);
      end


      // for all the checks relating to the soc-map
      Fabric_Addr fabric_addr = fv_Addr_to_Fabric_Addr (addr);

      // Check if f3 is legal, and if f3 and addr are compatible
      let addr_is_aligned = fn_is_aligned (f3 [1:0], addr);

`ifdef DUAL_FABRIC
      // Check if addr belongs to the NMIO or Mem (dual fabrics)
      let is_mem_req = !soc_map.m_is_nmio_addr (fabric_addr);
`endif

      // Check if addr belongs to the ITCM 
      let is_itcm_req = soc_map.m_is_itcm_addr (fabric_addr);

      // Check if addr belongs to the DTCM 
      let is_dtcm_req = soc_map.m_is_dtcm_addr (fabric_addr);

      // Legality check -- aligned address, and address should not
      // belong to the ITCM
      let is_bad_addr = is_itcm_req || !addr_is_aligned;
      if (is_bad_addr) begin
         // Misaligned accesses not supported
         rg_result_valid   <= True;
         rg_exc            <= True;
         rg_dmem_state     <= MEM_TCM_RSP;
         rg_exc_code       <= addr_is_aligned ? fv_exc_code_access_fault (dmem_req)
                                              : fv_exc_code_misaligned (dmem_req);
         if (verbosity >= 2) $display ("   bad addr");
      end

      // TCM reqs
      else if (is_dtcm_req) begin
         rg_result_valid <= True;
         rg_exc          <= False;

         // The write to the RAM is initiated here (can't be
         // speculative). If it is a AMO store, the actual write
         // happens in the AMO phase
         if (op == CACHE_ST) begin
            fa_write_to_ram (addr, dmem_req, st_value);
`ifdef ISA_A
            // Cancel LR/SC reservation if this store is for the reserved addr
            // TODO : should we cancel it on ANY store?
            if (rg_lrsc_pa == addr) rg_lrsc_valid <= False;
`endif
         end

         // The next state depends on the op. If it is a LD/ST move
         // to the response state which allows the module to
         // process the next request. If it is a AMO op, move to
         // the AMO state, effectively introducing a one-cycle
         // bubble.
`ifdef ISA_A
         if (   fv_is_AMO_SC (dmem_req)
             || fv_is_AMO_RMW (dmem_req)
             || fv_is_AMO_LR (dmem_req)
            ) rg_dmem_state <= MEM_AMO_RSP;
         else begin
            // Clear the lrsc_word64
            rg_lrsc_word64 <= tagged Invalid;

`endif
            rg_dmem_state <= MEM_TCM_RSP;
`ifdef ISA_A
         end
`endif
         if (verbosity >= 2) $display ("   valid DTCM addr");

      end

      // non-TCM request (outside TCM addr range: could be memory
      // or NMIO on the fabric)
      else begin
         rg_result_valid   <= False;
         rg_exc            <= False;
         rg_dmem_state     <= MEM_MMIO_RSP;

`ifdef DUAL_FABRIC
         // When two fabrics are present further decode is
         // necessary to decide if the non TCM address is meant for
         // the internal (nmio) fabric or the external fabric.
         // Remembering this for the response.
         f_is_mem_req.enq (is_mem_req);
`endif
         mmio.start;

         if (verbosity >= 2) $display ("   MMIO addr");

      end
   endrule

   // ----------------------------------------------------------------
   // INTERFACE

   method Action reset ();
      rg_result_valid <= False;
      rg_dmem_state <= MEM_IDLE;
      dma_port.reset;
      deburstr.reset;
`ifdef WATCH_TOHOST
      rg_tohost_value <= 0;
`endif

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
`ifdef NM32
         , Bit #(32) st_value
`else
         , Bit #(64) st_value
`endif
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
`ifdef NM32
      method Bit #(32)  word32      = dw_word;
      method Bit #(32)  st_amo_val  = dw_final_st_val;
`else
      method Bit #(64)  word64      = dw_word;
      method Bit #(64)  st_amo_val  = dw_final_st_val;
`endif
   endinterface

   // Fabric Interfaces
`ifdef DUAL_FABRIC
   // A separate nmio interface when we have a dual-fabric setup
   interface nmio_master = nmio_fabric_adapter.mem_master;
`endif
   // For accesses outside TCM (fabric memory, and memory-mapped I/O)
   interface mem_master = fabric_adapter.mem_master;

   // Back-door from fabric into DTCM
   interface dma_server = dma_port_extnl;

   // ----------------------------------------------------------------
   // Misc. control and status

   // ----------------
   // For ISA tests: watch memory writes to <tohost> addr (see NOTE: "tohost" above)

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
      rg_watch_tohost <= watch_tohost;
`ifdef FABRIC32
      rg_tohost_addr  <= truncate (tohost_addr);
`else
      rg_tohost_addr  <= tohost_addr;
`endif
      if (verbosity > 0)
	 $display ("%0d: %m.set_watch_tohost: watch %0d, addr %08h",
		   cur_cycle, watch_tohost, tohost_addr);
   endmethod

   method Bit #(64) mv_tohost_value;
      return extend (rg_tohost_value);
   endmethod
`endif

endmodule: mkDTCM

endpackage
