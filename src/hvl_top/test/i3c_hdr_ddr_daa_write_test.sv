`ifndef I3C_HDR_DDR_DAA_WRITE_TEST_INCLUDED_
`define I3C_HDR_DDR_DAA_WRITE_TEST_INCLUDED_

// HDR-DDR (Optional Feature F001) WRITE test, preceded by DAA -- NEW,
// additive only file. Mirrors i3c_daa_write_8b_test.sv exactly (DAA round
// first, dynamic addresses assigned into i3c_target_agent_cfg_h[i].
// targetAddress), then runs the HDR-DDR write to target[0] using that
// freshly-assigned dynamic address instead of the static default address.
//
// No changes were needed in i3c_hdr_ddr_write_virtual_seq.sv for this:
// it already reads i3c_env_cfg_h.i3c_target_agent_cfg_h[target_idx].
// targetAddress live (same pattern i3c_sdr_write_virtual_seq.sv uses),
// so whatever DAA leaves in that field - static (untouched) or dynamic
// (post-DAA) - is what the HDR-DDR Command Word's address field carries.
class i3c_hdr_ddr_daa_write_test extends i3c_base_test;
  `uvm_component_utils(i3c_hdr_ddr_daa_write_test)

  i3c_daa_virtual_seq            daaSeq;
  i3c_hdr_ddr_write_virtual_seq  hdrDdrWriteSeq;

  extern function new(string name = "i3c_hdr_ddr_daa_write_test",
                       uvm_component parent = null);
  extern virtual function void setup_target_agent_cfg();
  extern virtual task          run_phase(uvm_phase phase);

endclass : i3c_hdr_ddr_daa_write_test


function i3c_hdr_ddr_daa_write_test::new(string name = "i3c_hdr_ddr_daa_write_test",
                                          uvm_component parent = null);
  super.new(name, parent);
endfunction : new


// Enable DAA on every target, same as i3c_daa_write_8b_test.
function void i3c_hdr_ddr_daa_write_test::setup_target_agent_cfg();
  super.setup_target_agent_cfg();
  foreach (i3c_env_cfg_h.i3c_target_agent_cfg_h[i]) begin
    i3c_env_cfg_h.i3c_target_agent_cfg_h[i].has_daa = 1;
  end
  i3c_env_cfg_h.has_daa           = 1;
  i3c_env_cfg_h.no_of_daa_devices = NO_OF_TARGETS;
endfunction : setup_target_agent_cfg


task i3c_hdr_ddr_daa_write_test::run_phase(uvm_phase phase);
  phase.raise_objection(this, "i3c_hdr_ddr_daa_write_test");

  `uvm_info(get_type_name(),
    $sformatf("Starting DAA phase for %0d targets before HDR-DDR WRITE", NO_OF_TARGETS),
    UVM_LOW)

  daaSeq = i3c_daa_virtual_seq::type_id::create("daaSeq");
  daaSeq.i3c_env_cfg_h = i3c_env_cfg_h;
  daaSeq.start(i3c_env_h.top_virtual_seqr_h);

  `uvm_info(get_type_name(), "DAA phase complete", UVM_LOW)
  foreach (i3c_env_cfg_h.i3c_target_agent_cfg_h[i]) begin
    `uvm_info(get_type_name(),
      $sformatf("  target[%0d] dynamic addr = 0x%0h",
                i, i3c_env_cfg_h.i3c_target_agent_cfg_h[i].targetAddress),
      UVM_LOW)
  end

  `uvm_info(get_type_name(),
    "Starting HDR-DDR WRITE using target[0]'s dynamic address", UVM_LOW)

  hdrDdrWriteSeq = i3c_hdr_ddr_write_virtual_seq::type_id::create("hdrDdrWriteSeq");
  hdrDdrWriteSeq.i3c_env_cfg_h = i3c_env_cfg_h;
  hdrDdrWriteSeq.target_idx    = 0;
  hdrDdrWriteSeq.num_words     = 2;
  hdrDdrWriteSeq.start(i3c_env_h.top_virtual_seqr_h);

  #20us;

  `uvm_info(get_type_name(), "HDR-DDR (post-DAA) WRITE test complete", UVM_LOW)
  phase.drop_objection(this);
endtask : run_phase

`endif

