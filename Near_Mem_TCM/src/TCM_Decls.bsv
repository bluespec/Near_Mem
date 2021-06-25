// Copyright (c) 2020- Bluespec, Inc. All Rights Reserved.
// This package captures definitions used by the TCM logic

package TCM_Decls;
import Vector        :: *;
import ISA_Decls     :: *;

// TCM related type definitions -- common to I and D TCMs
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

typedef 64 KB_PER_ITCM;
typedef 32 KB_PER_DTCM;

// TCM Sizing - separate sizing for I and D TCMs
Integer kB_per_ITCM = valueOf (KB_PER_ITCM);     // 64KB
Integer kB_per_DTCM = valueOf (KB_PER_DTCM);     // 32KB
Integer bytes_per_ITCM = kB_per_ITCM * 'h400;
Integer bytes_per_DTCM = kB_per_DTCM * 'h400;

// LSBs to address a byte in the TCMs
typedef TAdd# (TLog# (KB_PER_ITCM), TLog #(1024)) ITCM_Addr_LSB;
Integer itcm_addr_lsb = valueOf (ITCM_Addr_LSB);
typedef TAdd# (TLog# (KB_PER_DTCM), TLog #(1024)) DTCM_Addr_LSB;
Integer dtcm_addr_lsb = valueOf (DTCM_Addr_LSB);

// Indices into the ITCM and DTCM (TCM word aligned)
typedef Bit #(TAdd #(TLog #(KB_PER_ITCM), 7))    ITCM_INDEX;  // KB * 1024 / bytes_per_tcm_word
typedef Bit #(TAdd #(TLog #(KB_PER_DTCM), 7))    DTCM_INDEX;  // KB * 1024 / bytes_per_tcm_word

// size of the BRAM in TCM_Word(s). the addition of the extra term is to prevent rounding down
// in case bytes_per_word is not a power of two.
Integer n_words_IBRAM = (bytes_per_ITCM / bytes_per_tcm_word);
Integer n_words_DBRAM = (bytes_per_DTCM / bytes_per_tcm_word);

endpackage
