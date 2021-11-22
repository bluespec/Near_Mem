// Copyright (c) 2016-2020 Bluespec, Inc. All Rights Reserved.

package MMIO;

// ================================================================
// This module handles all load, store, and AMO ops for MMIO.
// - Loads and Stores go directly to mem
// - LR/SC are not supported: LR is treated like Load; SC always fails
// - AMO ops do a read-modify-write across the fabric
//    (CAVEAT: there is no 'locking' of the location at memory during
//     the operation, so it may not really be atomic.)

// ================================================================
// BSV lib imports

import Vector       :: *;
import FIFOF        :: *;
import GetPut       :: *;
import ClientServer :: *;

// ----------------
// BSV additional libs

import Cur_Cycle     :: *;
import GetPut_Aux    :: *;

// ================================================================
// Project imports

import ISA_Decls        :: *;
import MMU_Cache_Common :: *;
import Near_Mem_IFC     :: *;

// ================================================================
// MODULE INTERFACE

interface IMMIO_IFC;
   method Action start;

   method Tuple2 #(Bool, Instr) result;

   interface Get #(Single_Req)  g_mem_req;
   interface Put #(Read_Data)   p_mem_read_data;
endinterface

interface DMMIO_IFC;
   method Action start;

`ifdef NM32
   method Tuple3 #(Bool, Bit #(32), Bit #(32)) result;
`else
   method Tuple3 #(Bool, Bit #(64), Bit #(64)) result;
`endif
endinterface

// ================================================================

typedef enum {MMIO_IDLE,
	      MMIO_START,
	      MMIO_READ_RSP} MMIO_State
deriving (Bits, Eq, FShow);

// ================================================================
// MODULE IMPLEMENTATION -- non-TCM memory access support for isntr
// fetches (loads)

module mkIMMIO #(
     IMem_Req req
   , FIFOF #(Single_Req) f_single_reqs
   , FIFOF #(Read_Data)  f_read_data
   , Bit#(2) verbosity
) (IMMIO_IFC);

   Reg #(MMIO_State) rg_mmio_state <- mkReg (MMIO_IDLE);

   // Non-VM MMIO: PA = VA
   let req_pa  = fn_WordXL_to_PA (req.pc);

   // Results
   Reg #(Bool)          rg_err          <- mkReg (False);
   Reg #(Instr)         rg_ld_val       <- mkRegU;

   // ----------------------------------------------------------------
   // Issue read request to mem for load, LR, and AMO Read-Modify-Write
   // (all ops other than store and SC)

   rule rl_read_req (rg_mmio_state == MMIO_START);
      if (verbosity >= 1)
	 $display ("%0d: %m.rl_read_req: f3 %0h vaddr %0h  paddr %0h",
		   cur_cycle, req.f3, req.pc, req_pa);
      let r   = Single_Req {is_read:   True,
			    addr:      zeroExtend (req_pa),
			    size_code: req.f3 [1:0]};
      f_single_reqs.enq (r);
      rg_mmio_state <= MMIO_READ_RSP;
   endrule

   // ----------------------------------------------------------------
   // Receive read response from mem for Load

   rule rl_read_rsp (rg_mmio_state == MMIO_READ_RSP);
      let read_data <- pop (f_read_data);

      if (verbosity >= 1) begin
	 $display ("%0d: %m.rl_read_rsp: pc %0h  paddr %0h", cur_cycle, req.pc, req_pa);
	 $display ("    ", fshow (read_data));
      end

      // Bus error
      if (! read_data.ok) begin
	 if (verbosity >= 1)
	    $display ("    MEM_RSP_ERR");

	 rg_err       <= True;
	 rg_mmio_state <= MMIO_IDLE;
      end

      // Successful read
      else begin
	 let ld_val_bits = fv_from_byte_lanes (
            zeroExtend (req_pa), req.f3 [1:0], read_data.data);

	 // Loads and LR
	 rg_ld_val <= truncate (ld_val_bits);
	 if (verbosity >= 1)
	   $display ("    Instruction load: f3 %0h ld_val %0h", req.f3, ld_val_bits);
	 rg_mmio_state    <= MMIO_IDLE;
      end
   endrule

   // ================================================================
   // INTERFACE

   method Action start;
      rg_err <= False;
      rg_mmio_state <= MMIO_START;
   endmethod

   method result () if (rg_mmio_state == MMIO_IDLE);
      return tuple2 (rg_err, rg_ld_val);
   endmethod

   // ----------------
   // Memory interface (for refills, writebacks)

   interface Get g_mem_req       = toGet (f_single_reqs);
   interface Put p_mem_read_data = toPut (f_read_data);
endmodule

// ================================================================
// MODULE IMPLEMENTATION -- non-TCM memory access support for data
// fetches (loads, stores, AMO)

module mkDMMIO #(
     MMU_Cache_Req req
   , FIFOF #(Single_Req) f_mem_reqs
`ifdef DUAL_FABRIC
   , FIFOF #(Single_Req) f_nmio_reqs
   , FIFOF #(Bool)       f_is_mem_req
`endif
`ifdef NM32
   , FIFOF #(Bit #(32))  f_write_data
`else
   , FIFOF #(Bit #(64))  f_write_data
`endif
   , FIFOF #(Read_Data)  f_read_data
   , Bit#(2) verbosity
) (DMMIO_IFC);

   Reg #(MMIO_State) rg_mmio_state <- mkReg (MMIO_IDLE);

   // Non-VM MMIO: PA = VA
   let req_pa  = fn_WordXL_to_PA (req.va);

   // Results
   Reg #(Bool)          rg_err          <- mkReg (False);
`ifdef NM32
   Reg #(Bit #(32))     rg_ld_val       <- mkReg (0);
`else
   Reg #(Bit #(64))     rg_ld_val       <- mkReg (0);
`endif
`ifdef NM32
   Reg #(Bit #(32))     rg_final_st_val <- mkReg (0);
`else
   Reg #(Bit #(64))     rg_final_st_val <- mkReg (0);
`endif

   // ----------------
   // Memory interface


   // ----------------------------------------------------------------
   // Help-function for single-writes to mem

`ifdef NM32
   function Action fa_mem_single_write (Bit #(32) st_value);
      action
	 // Lane-align the outgoing data
	 Bit #(5)  shamt_bits = { req_pa [1:0], 3'b000 };
	 Bit #(32) data       = (st_value << shamt_bits);
`else
   function Action fa_mem_single_write (Bit #(64) st_value);
      action
	 // Lane-align the outgoing data
	 Bit #(6)  shamt_bits = { req_pa [2:0], 3'b000 };
	 Bit #(64) data       = (st_value << shamt_bits);
`endif
	 let r   = Single_Req {is_read:   False,
			       addr:      zeroExtend (req_pa),
			       size_code: req.f3 [1:0]};
`ifdef DUAL_FABRIC
      let request_fifo = f_is_mem_req.first ? f_mem_reqs : f_nmio_reqs;
      f_is_mem_req.deq;
`else
      let request_fifo = f_mem_reqs;
`endif
         request_fifo.enq (r);
	 f_write_data.enq (data);
      endaction
   endfunction

   // ----------------------------------------------------------------
   // Issue read request to mem for load, LR, and AMO Read-Modify-Write
   // (all ops other than store and SC)

   rule rl_read_req (
         (rg_mmio_state == MMIO_START)
      && (req.op != CACHE_ST)
`ifdef ISA_A
      && (! fv_is_AMO_SC (req))
`endif
   );
      if (verbosity >= 1)
	 $display ("%0d: %m.rl_read_req: f3 %0h vaddr %0h  paddr %0h",
		   cur_cycle, req.f3, req.va, req_pa);
      let r   = Single_Req {is_read:   True,
			    addr:      zeroExtend (req_pa),
			    size_code: req.f3 [1:0]};
`ifdef DUAL_FABRIC
      let request_fifo = f_is_mem_req.first ? f_mem_reqs : f_nmio_reqs;
      f_is_mem_req.deq;
`else
      let request_fifo = f_mem_reqs;
`endif
      request_fifo.enq (r);
      rg_mmio_state <= MMIO_READ_RSP;
   endrule

   // ----------------------------------------------------------------
   // Receive read response from mem for Load, LR and AMO Read-Modify-Write
   // (all ops other than store and SC)

   rule rl_read_rsp (rg_mmio_state == MMIO_READ_RSP);
      let read_data <- pop (f_read_data);

      if (verbosity >= 1) begin
	 $display ("%0d: %m.rl_read_rsp: vaddr %0h  paddr %0h"
                 , cur_cycle, req.va, req_pa);
	 $display ("    ", fshow (read_data));
      end

      // Bus error
      if (! read_data.ok) begin
	 if (verbosity >= 1)
	    $display ("    MEM_RSP_ERR");

	 rg_err       <= True;
	 rg_mmio_state <= MMIO_IDLE;
      end

      // Successful read
      else begin
	 let  ld_val_bits = fv_from_byte_lanes (
            zeroExtend (req_pa), req.f3 [1:0], read_data.data);

	 // Loads and LR
	 if ((req.op == CACHE_LD) || fv_is_AMO_LR (req)) begin
	    let ld_val = fv_extend (req.f3, ld_val_bits);
	    rg_ld_val <= ld_val;
	    if (verbosity >= 1)
	      $display ("    Load or LR: f3 %0h ld_val %0h", req.f3, ld_val);
	 end
`ifdef ISA_A
	 // AMO read-modify-write
	 else begin
            match {.final_ld_val, .final_st_val} = fv_amo_op (
`ifndef NM32
               req.f3 [1:0],
`endif
               req.amo_funct7 [6:2],
               ld_val_bits,
               req.st_value);

	    // Write back final_st_val
	    fa_mem_single_write (final_st_val);
	    if (verbosity >= 1) begin
	      $display ("    AMO: f3 %0d  f7 %0h  ld_val %0h st_val %0h",
			req.f3, req.amo_funct7, ld_val_bits, req.st_value);
	      $display ("    => final_ld_val %0h final_st_val %0h",
			final_ld_val, final_st_val);
	    end
	    rg_ld_val       <= final_ld_val;
	    rg_final_st_val <= final_st_val;
	 end
`endif
	 rg_mmio_state    <= MMIO_IDLE;
      end
   endrule

   // ----------------------------------------------------------------
   // Store requests

   rule rl_write_req (
         (rg_mmio_state == MMIO_START)
      && (req.op == CACHE_ST));
      if (verbosity >= 2)
	 $display ("%0d: %m.rl_write_req; f3 %0h  vaddr %0h  paddr %0h  word %0h",
		   cur_cycle, req.f3, req.va, req_pa, req.st_value);

      let data = fv_to_byte_lanes (
         zeroExtend (req_pa), req.f3 [1:0], req.st_value);

      fa_mem_single_write (data);

      rg_final_st_val   <= req.st_value;
      rg_mmio_state     <= MMIO_IDLE;

      if (verbosity >= 3)
	 $display ("    goto MMIO_DONE");
   endrule

`ifdef ISA_A
   // ----------------------------------------------------------------
   // Memory-mapped I/O AMO_SC requests. Always fail (and never do the write)

   rule rl_AMO_SC (
         (rg_mmio_state == MMIO_START)
      && fv_is_AMO_SC (req));

      rg_ld_val    <= 1;    // 1 is LR/SC failure value
      rg_mmio_state <= MMIO_IDLE;

      if (verbosity >= 1) begin
	 $display ("%0d: %m.rl_AMO_SC; f3 %0h  vaddr %0h  paddr %0h  st_value %0h",
		   cur_cycle, req.f3, req.va, req_pa, req.st_value);
	 $display ("    FAIL due to I/O address.");
	 $display ("    goto MMIO_DONE");
      end
   endrule
`endif

   // ================================================================
   // INTERFACE

   method Action start;
      rg_mmio_state <= MMIO_START;
      rg_err <= False;
   endmethod

   method result () if (rg_mmio_state == MMIO_IDLE);
      return tuple3 (rg_err, rg_ld_val, rg_final_st_val);
   endmethod

endmodule

// ================================================================

endpackage
