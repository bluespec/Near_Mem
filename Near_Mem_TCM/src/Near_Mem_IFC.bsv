// Copyright (c) 2016-2020 Bluespec, Inc. All Rights Reserved.

// Near_Mem_IFC encapsulates the MMU and L1 cache.
// It is 'near' the CPU (1-cycle access in common case).

// On the CPU side it directly services instruction fetches and DMem
// reads and writes.

// On the Fabric side it has two Master sub-interfaces.
// One master sub-interface is used for instruction-memory access.
// The other master sub-interface is used for data-memory and I/O access.

// It can have various implementations:
//  - As an almost empty pass-through to the fabric
//  - As a cache (unified or separate I- and D-)
//        Fabric-side Server interface is not used (no back door to caches)
//  - As a TCM (Tightly-Coupled Memory)
//        Fabric-side IMem Client is not used (all fabric traffic is data or I/O mem)

package Near_Mem_IFC;

// ================================================================
// BSV lib imports

import FIFOF        :: *;
import GetPut       :: *;
import ClientServer :: *;

// ----------------
// BSV additional libs

import Cur_Cycle :: *;

// ================================================================
// Project imports

import ISA_Decls        :: *;
import TCM_Decls        :: *;

import MMU_Cache_Common :: *;
import Fabric_Defs      :: *;

import AXI4_Types       :: *;

`ifdef FABRIC_AHBL
import AHBL_Types       :: *;
import AHBL_Defs        :: *;
`endif

`ifdef NM32
`ifdef RV64
(* doc ="ERROR: NM32-RV64 is not a supported combination.*)
`endif
`endif
// ================================================================

interface Near_Mem_IFC;
   // Reset
   interface Server #(Token, Token) server_reset;

   // ----------------
   // IMem

   // CPU side
   interface IMem_IFC  imem;

   // DMA server interface for back-door access to the ITCM
   interface AXI4_Slave_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User)  imem_dma_server;

   // ----------------
   // DMem

   // CPU side
   interface DMem_IFC  dmem;

`ifdef FABRIC_AXI4
`ifdef DUAL_FABRIC
   // Fabric side (MMIO initiator interface)
   interface AXI4_Master_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) nmio_master;
`else
   // Fabric side (MMIO initiator interface)
   interface AXI4_Master_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) dmem_master;
`endif
`endif

`ifdef FABRIC_AHBL
   // Fabric side (MMIO initiator interface)
   interface AHBL_Master_IFC #(AHB_Wd_Data) dmem_master;
`endif

   // DMA server interface for back-door access to the DTCM
   interface AXI4_Slave_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User)  dmem_dma_server;

   // ----------------------------------------------------------------
   // Optional AXI4-Lite DMem slave interface

`ifdef INCLUDE_DMEM_SLAVE
   interface AXI4_Lite_Slave_IFC #(Wd_Addr, Wd_Data, Wd_User) dmem_slave;
`endif

   // ----------------
   // Fences

   interface Server #(Token, Token) server_fence_i;

   interface Server #(Fence_Ordering, Token) server_fence;

`ifdef ISA_PRIV_S
   interface Server #(Token, Token) sfence_vma_server;
`endif

   // ----------------------------------------------------------------
   // Misc. control and status

   // ----------------
   // For ISA tests: watch memory writes to <tohost> addr

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
   method Bit #(64) mv_tohost_value;
`endif

endinterface
   
// IMem: Instruction TCM
typedef struct {
   Addr pc;
   Bit #(3) f3;
} IMem_Req deriving (Bits, Eq, FShow);

// ================================================================
// IMem interface

interface IMem_IFC;
   // CPU side: IMem request
   (* always_ready *)
   method Action  req (  Bit #(3) f3
                       , WordXL addr
`ifdef ISA_PRIV_S
                       // The following  args for VM
                       , Priv_Mode  priv
                       , Bit #(1)   sstatus_SUM
                       , Bit #(1)   mstatus_MXR
                       , WordXL     satp
`endif
                    );    // { VM_Mode, ASID, PPN_for_page_table }

   // CPU side: IMem response
   (* always_ready *)  method Bool     valid;
   (* always_ready *)  method Bool     is_i32_not_i16;
   (* always_ready *)  method WordXL   pc;
   (* always_ready *)  method Instr    instr;
   (* always_ready *)  method Bool     exc;
   (* always_ready *)  method Exc_Code exc_code;
   (* always_ready *)  method WordXL   tval;        // can be different from PC
endinterface

`ifdef ISA_C
interface IMem_C_IFC;
   interface IMem_IFC imem;
   interface Put#(Token) reset_request;
   interface Put#(Token) fence_request;
endinterface
`endif

// ================================================================
// DMem interface

interface DMem_IFC;
   // CPU side: DMem request
   (* always_ready *)
   method Action  req (  CacheOp op
                       , Bit #(3) f3
`ifdef ISA_A
                       , Bit #(7) amo_funct7
`endif
                       , WordXL addr
`ifdef NM32
                       , Bit #(32) store_value
`else
                       , Bit #(64) store_value
`endif
`ifdef ISA_PRIV_S
                       // The following  args for VM
                       , Priv_Mode  priv
                       , Bit #(1)   sstatus_SUM
                       , Bit #(1)   mstatus_MXR
                       , WordXL     satp
`endif
                    );    // { VM_Mode, ASID, PPN_for_page_table }

   // CPU side: DMem response
   (* always_ready *)  method Bool       valid;
`ifdef NM32
   (* always_ready *)  method Bit #(32)  word32;      // Load-value
   (* always_ready *)  method Bit #(32)  st_amo_val;  // Store-value: ST, SC, AMO
`else
   (* always_ready *)  method Bit #(64)  word64;      // Load-value
   (* always_ready *)  method Bit #(64)  st_amo_val;  // Store-value: ST, SC, AMO
`endif
   (* always_ready *)  method Bool       exc;
   (* always_ready *)  method Exc_Code   exc_code;
endinterface


// ================================================================
// Dummy tie-off interfaces for FIFOF

// dummy_FIFO that never accepts anything (always "full") or
// yields anything (always "empty")

FIFOF #(t) dummy_FIFOF = interface FIFOF;
   method Action enq (x) if (False);
      noAction;
   endmethod
   method notFull;
      return False;
   endmethod
   method first () if (False);
      return ?;
   endmethod
   method Action deq () if (False);
      noAction;
   endmethod
   method notEmpty;
      return False;
   endmethod
   method Action clear if (False);
      noAction;
   endmethod
endinterface;

// ================================================================
// Extract bytes from raw word read from near-mem.
// The bytes of interest are offset according to LSBs of addr.
// Arguments:
//  - a RISC-V LD/ST f3 value (encoding LB, LH, LW, LD, LBU, LHU, LWU)
//  - a byte-address
//  - a load-word (loaded from cache/mem)
// result:
//  - word with correct byte(s) shifted into LSBs and properly extended
`ifdef NM32
function Bit #(32) fn_extract_and_extend_bytes (
   Bit #(3) f3, WordXL byte_addr, Bit #(32) mem_word);
   Bit #(32) result    = 0;
   Bit #(2)  addr_lsbs = byte_addr [1:0];
`else
function Bit #(64) fn_extract_and_extend_bytes (
   Bit #(3) f3, WordXL byte_addr, Bit #(64) mem_word);
   Bit #(64) result    = 0;
   Bit #(3)  addr_lsbs = byte_addr [2:0];
`endif

   case (f3)
      f3_LB: case (addr_lsbs)
                'h0: result = signExtend (mem_word [ 7: 0]);
                'h1: result = signExtend (mem_word [15: 8]);
                'h2: result = signExtend (mem_word [23:16]);
                'h3: result = signExtend (mem_word [31:24]);
`ifndef NM32
                'h4: result = signExtend (mem_word [39:32]);
                'h5: result = signExtend (mem_word [47:40]);
                'h6: result = signExtend (mem_word [55:48]);
                'h7: result = signExtend (mem_word [63:56]);
`endif
             endcase
      f3_LBU: case (addr_lsbs)
                'h0: result = zeroExtend (mem_word [ 7: 0]);
                'h1: result = zeroExtend (mem_word [15: 8]);
                'h2: result = zeroExtend (mem_word [23:16]);
                'h3: result = zeroExtend (mem_word [31:24]);
`ifndef NM32
                'h4: result = zeroExtend (mem_word [39:32]);
                'h5: result = zeroExtend (mem_word [47:40]);
                'h6: result = zeroExtend (mem_word [55:48]);
                'h7: result = zeroExtend (mem_word [63:56]);
`endif
             endcase

      f3_LH: case (addr_lsbs)
                'h0: result = signExtend (mem_word [15: 0]);
                'h2: result = signExtend (mem_word [31:16]);
`ifndef NM32
                'h4: result = signExtend (mem_word [47:32]);
                'h6: result = signExtend (mem_word [63:48]);
`endif
             endcase
      f3_LHU: case (addr_lsbs)
                'h0: result = zeroExtend (mem_word [15: 0]);
                'h2: result = zeroExtend (mem_word [31:16]);
`ifndef NM32
                'h4: result = zeroExtend (mem_word [47:32]);
                'h6: result = zeroExtend (mem_word [63:48]);
`endif
             endcase

      f3_LW: case (addr_lsbs)
                'h0: result = signExtend (mem_word [31: 0]);
`ifndef NM32
                'h4: result = signExtend (mem_word [63:32]);
`endif
             endcase
`ifdef NM32
      // If we get here, we are handling f3 cases which require
      // special support outside the NM32 near-mem
      default: result = mem_word;
   endcase
`else
      f3_LWU: case (addr_lsbs)
                'h0: result = zeroExtend (mem_word [31: 0]);
                'h4: result = zeroExtend (mem_word [63:32]);
             endcase

      f3_LD: case (addr_lsbs)
                'h0: result = mem_word;
             endcase
   endcase
`endif
   return result;
endfunction


// ================================================================
// Adjust byte for writes to TCM
// Arguments
//  - a RISC-V LD/ST f3 value (encoding LB, LH, LW, LD, LBU, LHU, LWU)
//  - a byte-address
//  - a store-word
// result: 2-tuple containing
//  - a byte-enable (0 if any error, e.g., misaligned)
//  - adjusted word (store bits shifted into correct byte positions)

function Tuple2 #(Bit #(Bytes_per_TCM_Word), // byte-enable
                  TCM_Word)                  // adjusted word
`ifdef NM32
         fn_byte_adjust_write (Bit #(3) f3, Addr byte_addr, Bit#(32) word);
`else
         fn_byte_adjust_write (Bit #(3) f3, Addr byte_addr, Bit#(64) word);
`endif

   Bit #(Bytes_per_TCM_Word) byte_en  = 0;   // If misaligned or illegal
   TCM_Word out_word = ?;

   Byte_in_TCM_Word addr_lsbs = byte_addr [(bits_per_byte_in_tcm_word-1):0];

   case ({1'b0, f3 [1:0]})
      // Bytes
      f3_LB: case (addr_lsbs)
         'h0: begin out_word [ 7: 0] = word [7:0]; byte_en = 'h01; end
         'h1: begin out_word [15: 8] = word [7:0]; byte_en = 'h02; end
         'h2: begin out_word [23:16] = word [7:0]; byte_en = 'h04; end
         'h3: begin out_word [31:24] = word [7:0]; byte_en = 'h08; end
`ifndef NM32
         'h4: begin out_word [39:32] = word [7:0]; byte_en = 'h10; end
         'h5: begin out_word [47:40] = word [7:0]; byte_en = 'h20; end
         'h6: begin out_word [55:48] = word [7:0]; byte_en = 'h40; end
         'h7: begin out_word [63:56] = word [7:0]; byte_en = 'h80; end
`endif
      endcase

      // Halfwords (16b)
      f3_LH: case (addr_lsbs)
         'h0: begin out_word [15: 0] = word [15:0]; byte_en = 'h03; end
         'h2: begin out_word [31:16] = word [15:0]; byte_en = 'h0C; end
`ifndef NM32
         'h4: begin out_word [47:32] = word [15:0]; byte_en = 'h30; end
         'h6: begin out_word [63:48] = word [15:0]; byte_en = 'hC0; end
`endif
      endcase

      // Words (32b)
`ifdef NM32
      f3_LW: begin out_word = word; byte_en = 'hf; end

      // Doublewords (64b) -- Unsupported
      f3_LD: begin out_word = word; byte_en = 'hf; end
`else
      f3_LW: case (addr_lsbs)
         'h0: begin out_word [31: 0] = word [31:0]; byte_en = 'h0F; end
         'h4: begin out_word [63:32] = word [31:0]; byte_en = 'hF0; end
      endcase

      // Doublewords (64b)
      f3_LD: case (addr_lsbs)
         'h0: begin out_word = word; byte_en = '1; end
      endcase
`endif

   endcase
   return tuple2 (byte_en, out_word);
endfunction

// ================================================================
// Extract bytes from word read from fabric.
// The bytes of interest are already in the LSBs of 'word',
// they just have to be suitably extended.
// Arguments:
//  - a RISC-V LD/ST f3 value (encoding LB, LH, LW, LD, LBU, LHU, LWU)
//  - a byte-address
//  - a load-word (loaded from fabric)
// result:
//  - word with correct byte(s), properly extended.

function Bit #(64) fn_extend_bytes (Bit #(3) f3, Bit #(64) word64);
   Bit #(64) result = 0;
   case (f3)
      f3_LB:  result = signExtend (word64 [ 7: 0]);
      f3_LBU: result = zeroExtend (word64 [ 7: 0]);

      f3_LH:  result = signExtend (word64 [15: 0]);
      f3_LHU: result = zeroExtend (word64 [15: 0]);

      f3_LW:  result = signExtend (word64 [31: 0]);
      f3_LWU: result = zeroExtend (word64 [31: 0]);

      f3_LD:  result = word64;
   endcase

   return result;
endfunction

// ================================================================

endpackage: Near_Mem_IFC
