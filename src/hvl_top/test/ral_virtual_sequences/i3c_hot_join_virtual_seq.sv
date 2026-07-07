
`ifndef I3C_HOT_JOIN_VIRTUAL_SEQ_INCLUDED_
`define I3C_HOT_JOIN_VIRTUAL_SEQ_INCLUDED_

class i3c_hot_join_virtual_seq extends top_virtual_base_seq;
  `uvm_object_utils(i3c_hot_join_virtual_seq)

  uvm_status_e   status;
  uvm_reg_data_t ctrl_val;
  uvm_reg_data_t ctrl_mirror;

  int unsigned timeout_per_slave_ns = 50_000;

  bit [6:0] hotjoin_ibi_addr = 7'h20;

  function new(string name = "i3c_hot_join_virtual_seq");
    super.new(name);
  endfunction

  // --------------------------------------------------------------------------
  task body();
    int num_targets;
    int hot_join_idx;

    super.body();

    if (i3c_env_cfg_h == null)
      `uvm_fatal("CFG_NULL",
        "i3c_env_cfg_h is NULL inside i3c_hot_join_virtual_seq")

    if (i3c_env_cfg_h.regBlockHandle == null)
      `uvm_fatal("RAL_NULL",
        "regBlockHandle is NULL inside i3c_hot_join_virtual_seq")

    num_targets  = i3c_env_cfg_h.no_of_targets;
    hot_join_idx = num_targets - 1;

    if (num_targets < 2)
      `uvm_fatal("CFG_ERR",
        "i3c_hot_join_virtual_seq requires at least 2 targets (N-1 for the initial DAA round, 1 to hot-join)")

    `uvm_info(get_type_name(),
      $sformatf("Starting initial DAA for targets[0..%0d], reserving target[%0d] for hot-join",
                hot_join_idx - 1, hot_join_idx),
      UVM_LOW)

    for (int i = 0; i < hot_join_idx; i++) begin
      automatic int idx = i;
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

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_type.set(2'd3);  // CMD_TYPE_DAA
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_ccc.set(8'h07);  // ENTDAA
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.start.set(1'b1);

    ctrl_val = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get();
    `uvm_info("CTRL_DEBUG",
      $sformatf("CTRL value before initial DAA write = 0x%0h", ctrl_val),
      UVM_LOW)

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.update(status, .parent(this));

    ctrl_mirror = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get_mirrored_value();
    `uvm_info("CTRL_DEBUG",
      $sformatf("CTRL mirrored value after initial DAA write = 0x%0h",
                ctrl_mirror), UVM_LOW)

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.mirror(status, UVM_NO_CHECK);

    begin
      int wait_ns = timeout_per_slave_ns * hot_join_idx;
      `uvm_info(get_type_name(),
        $sformatf("Waiting %0d ns for initial DAA round (%0d targets)",
                  wait_ns, hot_join_idx), UVM_LOW)
      #(wait_ns * 1ns);
    end

    `uvm_info(get_type_name(), "Initial DAA round completed. Addresses:",
      UVM_LOW)
    for (int i = 0; i < hot_join_idx; i++) begin
      `uvm_info(get_type_name(),
        $sformatf("  target[%0d]: dynamic addr = 0x%0h",
                  i, i3c_env_cfg_h.i3c_target_agent_cfg_h[i].targetAddress),
        UVM_LOW)
    end

    `uvm_info(get_type_name(),
      $sformatf("Triggering HOT JOIN for target[%0d] with ibi_addr=0x%0h",
                hot_join_idx, hotjoin_ibi_addr),
      UVM_LOW)

    i3c_env_cfg_h.i3c_target_agent_cfg_h[hot_join_idx].hotjoin_addr    = hotjoin_ibi_addr;
    i3c_env_cfg_h.i3c_target_agent_cfg_h[hot_join_idx].pending_hot_join = 1;

    fork
      begin
        i3c_target_hot_join_seq tgt_hj_seq;
        tgt_hj_seq = i3c_target_hot_join_seq::type_id::create("tgt_hj_seq");

        tgt_hj_seq.cfg_pid          = i3c_env_cfg_h.i3c_target_agent_cfg_h[hot_join_idx].pid;
        tgt_hj_seq.cfg_bcr          = i3c_env_cfg_h.i3c_target_agent_cfg_h[hot_join_idx].bcr;
        tgt_hj_seq.cfg_dcr          = i3c_env_cfg_h.i3c_target_agent_cfg_h[hot_join_idx].dcr;
        tgt_hj_seq.cfg_hotjoin_addr = hotjoin_ibi_addr;

        `uvm_info(get_type_name(),
          $sformatf("Launching HOTJOIN seq for target[%0d]", hot_join_idx),
          UVM_LOW)
        tgt_hj_seq.start(p_sequencer.i3c_target_seqr_h[hot_join_idx]);
        `uvm_info(get_type_name(),
          $sformatf("HOTJOIN seq DONE for target[%0d]", hot_join_idx),
          UVM_LOW)
      end
    join_none

    #(timeout_per_slave_ns * 2 * 1ns);

    `uvm_info(get_type_name(),
      $sformatf("HOT JOIN complete. target[%0d] dynamic addr = 0x%0h",
                hot_join_idx,
                i3c_env_cfg_h.i3c_target_agent_cfg_h[hot_join_idx].targetAddress),
      UVM_LOW)

  endtask : body

endclass : i3c_hot_join_virtual_seq

`endif

