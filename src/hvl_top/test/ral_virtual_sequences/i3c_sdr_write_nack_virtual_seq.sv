`ifndef I3C_SDR_WRITE_NACK_VIRTUAL_SEQ_INCLUDED_
`define I3C_SDR_WRITE_NACK_VIRTUAL_SEQ_INCLUDED_
class i3c_sdr_write_nack_virtual_seq extends top_virtual_base_seq;
  `uvm_object_utils(i3c_sdr_write_nack_virtual_seq)
  uvm_status_e   status;
  uvm_reg_data_t ctrl_val;
  uvm_reg_data_t ctrl_mirror;
  uvm_reg_data_t wdatab_mirror;
  rand bit [7:0] wdata;
  rand bit [7:0] transfer_len;
  int unsigned target_idx = 0;
  constraint len_c {
    transfer_len == 2;
  }
  function new(string name = "i3c_sdr_nack_write_virtual_seq");
    super.new(name);
  endfunction
  task body();
    i3c_target_writeOperationWith8bitsData_seq target_seq_write;
    if (i3c_env_cfg_h == null)
      `uvm_fatal("CFG_NULL",
        "i3c_env_cfg_h is NULL inside i3c_sdr_nack_write_virtual_seq")
    if (i3c_env_cfg_h.regBlockHandle == null)
      `uvm_fatal("RAL_NULL",
        "regBlockHandle is NULL inside i3c_sdr_write_nack_virtual_seq")
    super.body();
    if (!this.randomize()) begin
      `uvm_error(get_type_name(), "Sequence randomization failed — using defaults")
      wdata        = 8'hA5;
      transfer_len = 8'd1;
    end else begin
      `uvm_info(get_type_name(),
        $sformatf("Randomized: wdata=0x%0x len=%0d", wdata, transfer_len),
        UVM_LOW)
    end
    `uvm_info(get_type_name(),
      $sformatf("Starting SDR WRITE to target[%0d] addr=0x%0h",
                target_idx,
                i3c_env_cfg_h.i3c_target_agent_cfg_h[target_idx].targetAddress),
      UVM_LOW)
    // -----------------------------------------------------------------
    // pending_sdr -- ADDED. Missing from the original paste. Tells the
    // monitor proxy (i3c_target_monitor_proxy) that the next START on
    // the bus for this target is an SDR frame, not another DAA
    // broadcast. Without this, since has_daa stays 1 for the whole test
    // (set in i3c_daa_write_nack_8b_test::setup_target_agent_cfg()), the
    // monitor's dispatch chain would keep calling sample_daa_data() and
    // misparse this SDR write as a DAA session. Mirrors
    // i3c_sdr_write_virtual_seq's use of the same flag.
    // -----------------------------------------------------------------
    i3c_env_cfg_h.i3c_target_agent_cfg_h[target_idx].pending_sdr = 1;
    fork
      begin
        target_seq_write =
          i3c_target_writeOperationWith8bitsData_seq::type_id::create(
            "target_seq_write");
        target_seq_write.start(
          p_sequencer.i3c_target_seqr_h[target_idx]);
      end
    join_none
    i3c_env_cfg_h.regBlockHandle.wdatab_inst.write(status, wdata);
    // NOTE: cmd_addr is intentionally hardcoded to 0x68 rather than the
    // dynamically assigned target address -- this is the deliberate
    // NACK scenario: the controller addresses a target address that
    // won't match any assigned dynamic address, so the target NACKs
    // the address phase.
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_addr.set(7'h68);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_len.set(transfer_len);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_dir.set(1'b0);   // WRITE
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_type.set(2'd0);  // SDR
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_ccc.set(8'd0);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_mode.set(1'b0);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.start.set(1'b1);
    ctrl_val = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get();
    `uvm_info("CTRL_DEBUG",
      $sformatf("CTRL before SDR write = 0x%0h", ctrl_val), UVM_LOW)
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.update(status, .parent(this));
    ctrl_mirror = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get_mirrored_value();
    `uvm_info("CTRL_DEBUG",
      $sformatf("CTRL mirrored after SDR write = 0x%0h", ctrl_mirror), UVM_LOW)
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.mirror(status, UVM_NO_CHECK);
    wdatab_mirror = i3c_env_cfg_h.regBlockHandle.wdatab_inst.get_mirrored_value();
    `uvm_info("WDATAB_DEBUG",
      $sformatf("WDATAB mirrored = 0x%0h", wdatab_mirror), UVM_LOW)
    #5000;
    // -----------------------------------------------------------------
    // pending_sdr -- ADDED. Belt-and-braces clear, same redundant-clear
    // pattern i3c_sdr_write_virtual_seq uses (covers the case where the
    // frame never completed -- e.g. this NACK scenario, where there's
    // no data phase to complete).
    // -----------------------------------------------------------------
    i3c_env_cfg_h.i3c_target_agent_cfg_h[target_idx].pending_sdr = 0;
    `uvm_info(get_type_name(),
      $sformatf("SDR WRITE to target[%0d] complete", target_idx), UVM_LOW)
  endtask : body
endclass : i3c_sdr_write_nack_virtual_seq
`endif
