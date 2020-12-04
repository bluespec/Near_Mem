// Copyright (c) 2016-2020 Bluespec, Inc. All Rights Reserved.

package TCM_DMA_AXI4_Adapter;

// ================================================================
// Adapter converting generic 64b-wide read/write requests into an
// AXI4 bus master. 'Client' upstream:
// - an MMIO: requests/responses are for 64b word or sub-word,
//            and where lane-alignment is already done.

// The AXI4 bus master can be used with 32b or 64b buses, and manages
// byte-lane alignment, number of beats in a burst, write-strobes,
// etc. accordingly.

// ================================================================
// BSV lib imports

import Vector           :: *;
import BRAMCore         :: *;
import ConfigReg        :: *;
import FIFOF            :: *;
import GetPut           :: *;
import ClientServer     :: *;
import Assert           :: *;

// ----------------
// BSV additional libs

import Cur_Cycle        :: *;
import GetPut_Aux       :: *;
import Semi_FIFOF       :: *;

// ================================================================
// Project imports

import ISA_Decls        :: *;
import TCM_Decls        :: *;
import MMU_Cache_Common :: *;
import SoC_Map          :: *;

import AXI4_Types       :: *;
import Fabric_Defs      :: *;

// ================================================================
// Adapter converting AXI4 slave requests into commands to a RAM. 
// 'Server' downstream:
// - a TCM RAM: requests/responses are for fabric-width only.

// The AXI4 bus slave can be used with 32b or 64b buses, and manages
// byte-lane alignment and write-strobes. However, this slave
// implementation only handles requests with a single beat.

// ================================================================
// Fabric Port
// Enables 'back-door' access of TCM by devices and debuggers.
// Supports only word-size requests.

interface TCM_DMA_AXI4_Adapter_IFC;
   // Reset
   method Action  reset;

   // Back-door slave interface from fabric into Near_Mem
   interface AXI4_Slave_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User)  dma_server;
endinterface

// ----------------------------------------------------------------
// Module state
typedef enum {STATE_READY,
              STATE_READ_RESPONDING,
              STATE_BURST_WRITE} State deriving (Bits, Eq, FShow);

// ----------------------------------------------------------------
module mkTCM_DMA_AXI4_Adapter #(
     BRAM_PORT_BE #(Addr, TCM_Word, Bytes_per_TCM_Word) ram
   , Bit #(2)                                           verbosity) (TCM_DMA_AXI4_Adapter_IFC);

   // Module state
   Reg #(State) rg_state <- mkReg (STATE_READY);

   // Requests from/responses to fabric
   AXI4_Slave_Xactor_IFC #(Wd_Id, Wd_Addr, Wd_Data, Wd_User) slave_xactor <- mkAXI4_Slave_Xactor;

   SoC_Map_IFC soc_map <- mkSoC_Map;

   // ----------------------------------------------------------------
   // BEHAVIOR

   // ----------------
   // Reset

   function Action fa_reset;
      action
         slave_xactor.reset;
         rg_state <= STATE_READY;
      endaction
   endfunction

   // ----------------------------------------------------------------
   // Handle fabric read request
   // Only full-word requests (32b in Fabric_32 and 64b in Fabric_64)

   // The head of the read request queue, and some functions on it
   let rda              = slave_xactor.o_rd_addr.first;
   let rd_byte_addr     = rda.araddr;
   let rd_ram_word_addr = ((rd_byte_addr - soc_map.m_itcm_addr_base) >> bits_per_byte_in_tcm_word);

   // XXX Is this check redundant because this device will only get requests meant for it, guaranteed by the fabric
   // Bool rd_addr_valid   = soc_map.m_is_itcm_addr_1 (rd_byte_addr);
   Bool rd_addr_valid   = True;

   Byte_in_TCM_Word rd_byte_in_tcm_word = rd_byte_addr [(bits_per_byte_in_tcm_word - 1) : 0];

   // Check alignment
`ifdef FABRIC32
   Bool rd_addr_aligned = (
         (rd_byte_in_tcm_word == 0)
      || (rd_byte_in_tcm_word == 4));
   Bool rd_lower_word = (rd_byte_in_tcm_word == 0);
`elsif FABRIC64
   Bool rd_addr_aligned = (rd_byte_in_tcm_word == 0);
`endif

   // Invalid read address: send error response
   rule rl_bad_rd_addr (   (rg_state == STATE_READY)
                        && ((! rd_addr_valid) || (! rd_addr_aligned)));
      slave_xactor.o_rd_addr.deq;

      let rdr = AXI4_Rd_Data {
           rid:   rda.arid
         , rresp: axi4_resp_slverr
         , rdata: rd_byte_addr
         , ruser: rda.aruser
         , rlast: True
      };
      slave_xactor.i_rd_data.enq (rdr);

      if (verbosity > 0)
         $display ("%0d: %m.rl_bad_rd_addr 0x%0h", cur_cycle, rd_byte_addr);
   endrule

   // Legal, well-formed read requests: initiate RAM read
   rule rl_rd_req ((rg_state == STATE_READY) && rd_addr_valid && rd_addr_aligned);
      // Initiate word read from ram
      ram.put (0, fv_Fabric_Addr_to_Addr (rd_ram_word_addr), ?);
      rg_state <= STATE_READ_RESPONDING;

      if (verbosity > 1)
         $display ("%0d: %m.rl_rd_req: addr 0x%0h", cur_cycle, rd_byte_addr);
   endrule

   // Read responses: get word from RAM and respond
   rule rl_rd_rsp (rg_state == STATE_READ_RESPONDING);
`ifdef FABRIC32
      let words = pack (ram.read);
      Bit#(Wd_Data) word = rd_lower_word ? words[31:0] : words [63:32];
`elsif FABRIC64
      Bit#(Wd_Data) word = pack(ram.read);
`endif
      
      let rdr = AXI4_Rd_Data {
           rid  : rda.arid
         , rresp: axi4_resp_okay
         , rdata: word
         , rlast: True
         , ruser: rda.aruser
      };
      slave_xactor.i_rd_data.enq (rdr);
      slave_xactor.o_rd_addr.deq;
      rg_state <= STATE_READY;

      if (verbosity > 1)
         $display ("%0d: %m.rl_rd_rsp: addr 0x%0h => data 0x%0h",
                   cur_cycle, rd_byte_addr, word);
   endrule

   // ----------------------------------------------------------------
   // Handle fabric write request
   // Only full-word requests (32b in RV32 and 64b in RV64)

   // The head of the write request queue, and some functions on it
   let wra = slave_xactor.o_wr_addr.first;
   let wrd = slave_xactor.o_wr_data.first;

   let wr_byte_addr     = wra.awaddr;
   let wr_ram_word_addr = ((wr_byte_addr - soc_map.m_itcm_addr_base) >> bits_per_byte_in_tcm_word);

   // XXX Is this check redundant because this device will only get requests meant for it, guaranteed by the fabric
   // Bool wr_addr_valid   = soc_map.m_is_itcm_addr_2 (wr_byte_addr);
   Bool wr_addr_valid   = True;

   Byte_in_TCM_Word wr_byte_in_tcm_word = wr_byte_addr [(bits_per_byte_in_tcm_word - 1) : 0];

   // Check alignment and strobe
`ifdef FABRIC32
   Bool wr_addr_aligned = (
         (wr_byte_in_tcm_word == 0)
      || (wr_byte_in_tcm_word == 4));
   Bool wr_lower_word = (wr_byte_in_tcm_word == 0);
`elsif FABRIC64
   Bool wr_addr_aligned = (wr_byte_in_tcm_word == 0);
`endif
   Bool wr_data_size_ok = (wrd.wstrb == '1);

   // Invalid write address or data size: send error response
   rule rl_bad_wr_addr (
         (rg_state == STATE_READY)
      && (   (! wr_addr_valid)
          || (! wr_addr_aligned)
          || (! wr_data_size_ok)));

      slave_xactor.o_wr_addr.deq;
      slave_xactor.o_wr_data.deq;

      let wrr = AXI4_Wr_Resp {
           bid:   wra.awid
         , bresp: axi4_resp_slverr
         , buser: wra.awuser
      };
      slave_xactor.i_wr_resp.enq (wrr);

      if (verbosity > 0) begin
         $display ("%0d: %m.rl_bad_wr_addr", cur_cycle);
         $display ("    ", fshow (wra));
         $display ("    ", fshow (wrd));
         $display ("    => ", fshow (wrr));
      end
   endrule

   // Legal, well-formed write requests
   rule rl_wr_req (
         (rg_state == STATE_READY)
      && (wr_addr_valid)
      && (wr_addr_aligned)
      && (wr_data_size_ok));

      slave_xactor.o_wr_addr.deq;
      slave_xactor.o_wr_data.deq;

      // Strobe generation
`ifdef FABRIC32
      Bit #(Bytes_per_TCM_Word) strobe = wr_lower_word ? 8'hf : 8'hf0; 
`elsif FABRIC64
      Bit #(Bytes_per_TCM_Word) strobe = 8'hff;
`endif

      // Write data generation
      TCM_Word tcm_wdata = ?;
`ifdef FABRIC32
      tcm_wdata = {wrd.wdata, wrd.wdata};
`elsif FABRIC64
      tcm_wdata = wrd.wdata;
`endif

      // Write word to ram
      ram.put (strobe, fv_Fabric_Addr_to_Addr (wr_ram_word_addr), tcm_wdata);

      // Send response
      let wrr = AXI4_Wr_Resp {
           bid:   wra.awid
         , bresp: axi4_resp_okay
         , buser: wra.awuser};
      slave_xactor.i_wr_resp.enq (wrr);

      if (verbosity > 1) begin
         $display ("%0d: %m.rl_wr_req", cur_cycle);
         $display ("    ", fshow (wra));
         $display ("    ", fshow (wrd));
      end
   endrule

   // ----------------------------------------------------------------
   // INTERFACE

   method Action reset;
      fa_reset;
      if (verbosity > 1)
         $display ("%0d: %m.reset", cur_cycle);
   endmethod

   interface dma_server = slave_xactor.axi_side;
endmodule

endpackage
