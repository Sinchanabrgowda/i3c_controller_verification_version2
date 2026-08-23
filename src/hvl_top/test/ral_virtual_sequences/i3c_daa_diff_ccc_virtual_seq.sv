`ifndef I3C_DAA_DIFF_CCC_VIRTUAL_SEQ_INCLUDED_
`define I3C_DAA_DIFF_CCC_VIRTUAL_SEQ_INCLUDED_
class i3c_daa_diff_ccc_virtual_seq extends top_virtual_base_seq;
  `uvm_object_utils(i3c_daa_diff_ccc_virtual_seq)
  uvm_status_e   status;
  uvm_reg_data_t ctrl_val;
  uvm_reg_data_t ctrl_mirror;
  int unsigned timeout_per_slave_ns = 50_000;
  function new(string name = "i3c_daa_diff_ccc_virtual_seq");
    super.new(name);
  endfunction
  task body();
    int num_targets;
    super.body();
    if (i3c_env_cfg_h == null)
      `uvm_fatal("CFG_NULL",
        "i3c_env_cfg_h is NULL inside i3c_daa_virtual_seq")
    if (i3c_env_cfg_h.regBlockHandle == null)
      `uvm_fatal("RAL_NULL",
        "regBlockHandle is NULL inside i3c_daa_virtual_seq")
    num_targets = i3c_env_cfg_h.no_of_targets;
    `uvm_info(get_type_name(),
      $sformatf("Starting multi-slave DAA for %0d targets", num_targets),
      UVM_LOW)
    foreach (p_sequencer.i3c_target_seqr_h[i]) begin
      automatic int idx = i;  // capture for fork
      fork
        begin
          i3c_target_daa_seq tgt_daa_seq;
          tgt_daa_seq = i3c_target_daa_seq::type_id::create(
                          $sformatf("tgt_daa_seq_%0d", idx));
          tgt_daa_seq.cfg_pid = i3c_env_cfg_h.i3c_target_agent_cfg_h[idx].pid;
          tgt_daa_seq.cfg_bcr = i3c_env_cfg_h.i3c_target_agent_cfg_h[idx].bcr;
          tgt_daa_seq.cfg_dcr = i3c_env_cfg_h.i3c_target_agent_cfg_h[idx].dcr;
          `uvm_info(get_type_name(),
            $sformatf("Launching DAA seq for target[%0d]", idx), UVM_LOW)
          tgt_daa_seq.start(p_sequencer.i3c_target_seqr_h[idx]);
          `uvm_info(get_type_name(),
            $sformatf("DAA seq DONE for target[%0d]", idx), UVM_LOW)
        end
      join_none
    end
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_type.set(2'd2);  // CMD_TYPE_DAA
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_ccc.set(8'h03);  // ENTDAA
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.start.set(1'b1);
    ctrl_val = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get();
    `uvm_info("CTRL_DEBUG",
      $sformatf("CTRL value before DAA write = 0x%0h", ctrl_val), UVM_LOW)
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.update(status, .parent(this));
    ctrl_mirror = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get_mirrored_value();
    `uvm_info("CTRL_DEBUG",
      $sformatf("CTRL mirrored value after DAA write = 0x%0h", ctrl_mirror),
      UVM_LOW)
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.mirror(status, UVM_NO_CHECK);
    begin
      int wait_ns = timeout_per_slave_ns * num_targets;
      `uvm_info(get_type_name(),
        $sformatf("Waiting %0d ns for all %0d slaves to complete DAA",
                  wait_ns, num_targets), UVM_LOW)
      #(wait_ns * 1ns);
    end
    `uvm_info(get_type_name(),
      $sformatf("Multi-slave DAA completed. Assigned addresses:"), UVM_LOW)
    foreach (i3c_env_cfg_h.i3c_target_agent_cfg_h[i]) begin
      `uvm_info(get_type_name(),
        $sformatf("  target[%0d]: dynamic addr = 0x%0h",
                  i, i3c_env_cfg_h.i3c_target_agent_cfg_h[i].targetAddress),
        UVM_LOW)
    end
  endtask : body
endclass : i3c_daa_diff_ccc_virtual_seq
`endif
