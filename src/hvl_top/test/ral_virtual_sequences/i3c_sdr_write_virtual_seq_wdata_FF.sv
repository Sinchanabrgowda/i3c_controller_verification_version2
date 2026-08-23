`ifndef I3C_SDR_WRITE_VIRTUAL_SEQ_WDATA_FF_INCLUDED_
`define I3C_SDR_WRITE_VIRTUAL_SEQ_WDATA_FF_INCLUDED_
class i3c_sdr_write_virtual_seq_wdata_FF extends top_virtual_base_seq;
  `uvm_object_utils(i3c_sdr_write_virtual_seq_wdata_FF)
  uvm_status_e   status;
  uvm_reg_data_t ctrl_val;
  uvm_reg_data_t ctrl_mirror;
  int unsigned target_idx = 0;
  rand bit [7:0] wdata_bytes [0:15];
  rand int unsigned transfer_len;
  constraint len_c {
    transfer_len inside {[1:16]};
  }
  // -----------------------------------------------------------------
  // FIXED -- same missing-';' before the foreach closing brace seen in
  // every wdata_XX variant this session:
  //   wdata_bytes[i] == 8'hFF};
  // Corrected to standard foreach-constraint form below.
  // -----------------------------------------------------------------
  constraint wdata_val_c {
    foreach (wdata_bytes[i]) {
      wdata_bytes[i] == 8'hFF;
    }
  }
  function new(string name = "i3c_sdr_write_virtual_seq_wdata_FF");
    super.new(name);
  endfunction
  task body();
    i3c_target_writeOperationWith8bitsData_seq target_seq_write;
    if (i3c_env_cfg_h == null)
      `uvm_fatal("CFG_NULL",
        "i3c_env_cfg_h is NULL inside i3c_sdr_write_virtual_seq")
    if (i3c_env_cfg_h.regBlockHandle == null)
      `uvm_fatal("RAL_NULL",
        "regBlockHandle is NULL inside i3c_sdr_write_virtual_seq")
    super.body();
    if (!this.randomize()) begin
      `uvm_error(get_type_name(), "Randomization failed - using defaults")
      transfer_len    = 2;
      wdata_bytes[0]  = 8'h44;
      wdata_bytes[1]  = 8'hBE;
    end
    `uvm_info(get_type_name(),
      $sformatf("SDR WRITE to target[%0d] addr=0x%0h len=%0d bytes",
                target_idx,
                i3c_env_cfg_h.i3c_target_agent_cfg_h[target_idx].targetAddress,
                transfer_len),
      UVM_LOW)
    for (int i = 0; i < transfer_len; i++)
      `uvm_info(get_type_name(),
        $sformatf("  planned payload[%0d] = 0x%02h", i, wdata_bytes[i]),
        UVM_LOW)
    // -----------------------------------------------------------------
    // pending_sdr -- ADDED. Missing from the original paste, same gap as
    // every other new SDR write sequence this session. Tells the monitor
    // proxy the next START is an SDR frame, not another DAA broadcast
    // (has_daa stays 1 for the whole test).
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
    for (int i = 0; i < transfer_len; i++) begin
      `uvm_info(get_type_name(),
        $sformatf("Pushing WDATAB[%0d] = 0x%02h to RTL FIFO", i, wdata_bytes[i]),
        UVM_LOW)
      i3c_env_cfg_h.regBlockHandle.wdatab_inst.write(
        status, uvm_reg_data_t'(wdata_bytes[i]),
        UVM_FRONTDOOR, .parent(this));
      if (status != UVM_IS_OK)
        `uvm_error(get_type_name(),
          $sformatf("WDATAB write[%0d]=0x%02h failed status=%s",
                    i, wdata_bytes[i], status.name()))
      else
        `uvm_info(get_type_name(),
          $sformatf("WDATAB[%0d] write OK", i), UVM_LOW)
    end
    `uvm_info(get_type_name(),
      $sformatf("All %0d bytes loaded into WDATAB FIFO", transfer_len),
      UVM_LOW)
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_addr.set(
      i3c_env_cfg_h.i3c_target_agent_cfg_h[target_idx].targetAddress);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_len.set(transfer_len);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_dir.set(1'b0);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_type.set(2'd0);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_ccc.set(8'd0);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_mode.set(1'b0);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.start.set(1'b1);
    ctrl_val = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get();
    `uvm_info("CTRL_DEBUG",
      $sformatf("CTRL before SDR write = 0x%0h  addr=0x%0h len=%0d",
                ctrl_val,
                i3c_env_cfg_h.i3c_target_agent_cfg_h[target_idx].targetAddress,
                transfer_len),
      UVM_LOW)
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.update(status, .parent(this));
    ctrl_mirror =
      i3c_env_cfg_h.regBlockHandle.ctrl_inst.get_mirrored_value();
    `uvm_info("CTRL_DEBUG",
      $sformatf("CTRL mirrored after write = 0x%0h", ctrl_mirror), UVM_LOW)
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.mirror(status, UVM_NO_CHECK);
    #(5000 * (transfer_len + 2));
    // -----------------------------------------------------------------
    // pending_sdr -- ADDED. Belt-and-braces clear, same pattern as the
    // other SDR write sequences.
    // -----------------------------------------------------------------
    i3c_env_cfg_h.i3c_target_agent_cfg_h[target_idx].pending_sdr = 0;
    `uvm_info(get_type_name(),
      $sformatf("SDR WRITE to target[%0d] complete (%0d bytes)",
                target_idx, transfer_len),
      UVM_LOW)
  endtask : body
endclass : i3c_sdr_write_virtual_seq_wdata_FF
`endif
