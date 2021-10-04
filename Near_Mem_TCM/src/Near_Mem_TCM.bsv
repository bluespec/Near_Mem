// Copyright (c) 2016-2021 Bluespec, Inc. All Rights Reserved.

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
//
// The macro NM32 defines a near-mem with a 32-bit data path all
// the way from the CPU to the fabric. In the case of the TCMs,
// their word-width is sized to match the data path width.

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
// The tohost check is independent of other address checks.
// ----------------


package Near_Mem_TCM;

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
import ITCM             :: *;
import DTCM             :: *;

import TCM_DMA_AXI4_Adapter :: *;
import AXI4_Deburster   :: *;

import Fabric_Defs      :: *;
import AXI4_Types       :: *;

`ifdef FABRIC_AHBL
import AHBL_Types       :: *;
import AHBL_Defs        :: *;
import TCM_AHBL_Adapter :: *;
`endif


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
// Near_Mem_TCM module

(* synthesize *)
module mkNear_Mem (Near_Mem_IFC);

   // Verbosity: 0: quiet
   //            1: Requests and responses
   //            2: rule firings
   //            3: + detail
   Bit #(2) i_verbosity = 0;
   Bit #(2) d_verbosity = 0;

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

   // Back-door from fabric into ITCM
   interface imem_dma_server = itcm.dma_server;

   // ----------------
   // DMem

   // CPU side
   interface dmem = dtcm.dmem;

   // Fabric side
   interface dmem_master = dtcm.mem_master;

`ifdef DUAL_FABRIC
   // Near end (near-mem IO)
   interface nmio_master = dtcm.nmio_master;
`endif

   // Back-door from fabric into DTCM
   interface dmem_dma_server = dtcm.dma_server;

   // ----------------
   // Fence.I, Fence -- all fences are nops, right?
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
   // For ISA tests: watch memory writes to <tohost> addr

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
      dtcm.set_watch_tohost (watch_tohost, tohost_addr);
   endmethod

   method Bit #(64) mv_tohost_value = dtcm.mv_tohost_value;
`endif

endmodule: mkNear_Mem

// ================================================================

endpackage: Near_Mem_TCM
