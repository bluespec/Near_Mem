// Copyright (c) 2020- Bluespec, Inc. All Rights Reserved.
// This package captures definitions used by the TCM logic

package TCM_Decls;
import Vector        :: *;
import ISA_Decls     :: *;

// TCM related type definitions
typedef 64 TCM_XLEN;
typedef Bit #(TCM_XLEN)                   TCM_Word;
typedef TDiv #(TCM_XLEN, Bits_per_Byte)   Bytes_per_TCM_Word;
typedef TLog #(Bytes_per_TCM_Word)        Bits_per_Byte_in_TCM_Word;
typedef Bit #(Bits_per_Byte_in_TCM_Word)  Byte_in_TCM_Word;
typedef Vector #(Bytes_per_TCM_Word, Byte) TCM_Word_B;
Integer bytes_per_tcm_word        = valueOf (Bytes_per_TCM_Word);
Integer bits_per_byte_in_tcm_word = valueOf (Bits_per_Byte_in_TCM_Word);
Integer addr_lo_byte_in_tcm_word = 0;
Integer addr_hi_byte_in_tcm_word = addr_lo_byte_in_tcm_word + bits_per_byte_in_tcm_word - 1;

function  Byte_in_TCM_Word fn_addr_to_byte_in_tcm_word (Addr a);
   return a [addr_hi_byte_in_tcm_word : addr_lo_byte_in_tcm_word ];
endfunction

// TCM Sizing
// Integer kB_per_TCM = 'h4;         // 4KB
   Integer kB_per_TCM = 'h40;     // 64KB
// Integer kB_per_TCM = 'h80;     // 128KB
// Integer kB_per_TCM = 'h400;    // 1 MB
// Integer kB_per_TCM = 'h4000;    // 16 MB
Integer bytes_per_TCM = kB_per_TCM * 'h400;

// size of the BRAM in TCM_Word(s). the addition of the extra term
// is to prevent rounding down in case bytes_per_word is not a
// power of two.
Integer n_words_BRAM = ((bytes_per_TCM + bytes_per_tcm_word - 1) / bytes_per_tcm_word);

endpackage

