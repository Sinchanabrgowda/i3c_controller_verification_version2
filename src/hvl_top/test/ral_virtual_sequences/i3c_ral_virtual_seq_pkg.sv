`ifndef I3C_RAL_VIRTUAL_SEQ_PKG_INCLUDED_
`define I3C_RAL_VIRTUAL_SEQ_PKG_INCLUDED_

package i3c_ral_virtual_seq_pkg;

  `include "uvm_macros.svh"
  import uvm_pkg::*;
  import i3c_globals_pkg::*;
  import apb_global_pkg::*;
  import apb_master_pkg::*;
  import apb_master_seq_pkg::*;
  import i3c_target_pkg::*;
  import i3c_target_seq_pkg::*;
  import i3c_env_pkg::*;

  `define CCC_ENTDAA 8'h07

  // Base
  `include "top_virtual_base_seq.sv"

  // SDR virtual sequences
  `include "i3c_sdr_write_virtual_seq.sv"
  `include "i3c_sdr_read_virtual_seq.sv"
  `include "i3c_sdr_multi_write_read_virtual_seq.sv"
  `include "i3c_sdr_write_read_virtual_seq.sv"
  `include "i3c_sdr_write_read_write_read_virtual_seq.sv"
  `include "i3c_invalid_addr_write_virtual_seq.sv"
  `include "i3c_fifo_full_write_virtual_seq.sv"
  `include "i3c_ccc_coverage_virtual_seq.sv"
  `include "i3c_rdatab_ro_seq.sv"
  `include "i3c_verify_pos_ack_seq.sv"
  `include "i3c_verify_neg_ack_seq.sv"
  `include "i3c_verify_repeated_start_seq.sv"
  `include "i3c_start_stop_combination_seq.sv"
  `include "i3c_randomDataTransferWidth_vseq.sv"
  `include "i3c_random_rw_virtual_seq.sv"
  `include "i3c_multi_wdatab_seq.sv"
  `include "i3c_daa_sdr_virtual_seq.sv"
  `include "i3c_read_write_read_seq.sv"
  `include "i3c_multi_write_read_back_req.sv"

  // DAA virtual sequences
  `include "i3c_daa_virtual_seq.sv"
  `include "i3c_sdr_or_daa_virtual_seq.sv"
`include "i3c_hot_join_virtual_seq.sv"  //hot join seq
`include "i3c_ibi_virtual_seq.sv"
`include "i3c_hot_join_invalid_addr_virtual_seq.sv" 

//`include "i3c_ibi_t0_virtual_seq.sv"
endpackage : i3c_ral_virtual_seq_pkg

`endif

