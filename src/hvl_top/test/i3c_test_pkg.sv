`ifndef I3C_TEST_PKG_INCLUDED_
`define I3C_TEST_PKG_INCLUDED_

package i3c_test_pkg;

  `include "uvm_macros.svh"

  import uvm_pkg::*;
  import apb_global_pkg::*;
  import i3c_globals_pkg::*;
  import apb_master_pkg::*;
  import i3c_target_pkg::*;
  import i3c_env_pkg::*;
  import apb_master_seq_pkg::*;
  import i3c_target_seq_pkg::*;
  import i3c_ral_virtual_seq_pkg::*;

  // Base test
  `include "i3c_base_test.sv"

  // SDR tests
  `include "i3c_write_8b_test.sv"
  `include "i3c_multi_write_test.sv"
  `include "i3c_read_8b_test.sv"
  `include "i3c_write_read_8b_test.sv"
  `include "i3c_write_read_write_read_8b_test.sv"
  `include "i3c_invalid_addr_write_test.sv"
  `include "i3c_fifo_full_write_test.sv"
  `include "i3c_ccc_coverage_test.sv"

  // DAA tests
  `include "i3c_daa_write_8b_test.sv"
  `include "i3c_daa_read_8b_test.sv"
  `include "i3c_daa_write_read_write_read_8b_test.sv"
  `include "i3c_sdr_or_daa_write_8b_test.sv"

//hot join
`include "i3c_hot_join_test.sv"
`include "i3c_hot_join_invalid_reserved_addr.sv"
//ibi
`include "i3c_ibi_test.sv"
//`include "i3c_ibi_t0_test.sv"				

endpackage : i3c_test_pkg

`endif

