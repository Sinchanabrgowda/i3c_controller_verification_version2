`ifndef I3C_SDR_READ_VIRTUAL_SEQ_INCLUDED_
`define I3C_SDR_READ_VIRTUAL_SEQ_INCLUDED_
class i3c_sdr_read_virtual_seq extends top_virtual_base_seq;
  `uvm_object_utils(i3c_sdr_read_virtual_seq)
  uvm_status_e status;
  uvm_reg_data_t rdata;
  uvm_reg_data_t rdata_get;
  uvm_reg_data_t rdata_mirror;
  function new(string name = "i3c_sdr_read_virtual_seq");
    super.new(name);
  endfunction
  task body();
    i3c_target_readOperationWith8bitsData_seq target_seq_read;
    super.body();
    `uvm_info(get_type_name(), "Starting SDR READ test", UVM_LOW)

    i3c_env_cfg_h.i3c_target_agent_cfg_h[0].pending_sdr = 1;
    fork
      begin
        target_seq_read =
          i3c_target_readOperationWith8bitsData_seq::type_id::create("target_seq_read");
        target_seq_read.start(p_sequencer.i3c_target_seqr_h[0]);
      end
    join_none;
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_addr.set(i3c_env_cfg_h.i3c_target_agent_cfg_h[0].targetAddress);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_len.set(8'd1);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_dir.set(1'b1);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_type.set(2'b00);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.start.set(1'b1);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.update(status, .parent(this));
    if(status != UVM_IS_OK)
      `uvm_error("RAL","CTRL register write failed")
    `uvm_info(get_type_name(), "SDR READ command issued", UVM_LOW)
    #5000;
    i3c_env_cfg_h.regBlockHandle.rdatab_inst.read(status, rdata);
    if(status != UVM_IS_OK)
      `uvm_error("RAL","RDATAB read failed")
    rdata_get =
      i3c_env_cfg_h.regBlockHandle.rdatab_inst.get();
    rdata_mirror =
      i3c_env_cfg_h.regBlockHandle.rdatab_inst.get_mirrored_value();
    `uvm_info("RDATAB_DEBUG",
      $sformatf("RDATAB read value (DUT)   = %0h", rdata),
      UVM_LOW)
    `uvm_info("RDATAB_DEBUG",
      $sformatf("RDATAB get() value (RAL)  = %0h", rdata_get),
      UVM_LOW)
    `uvm_info("RDATAB_DEBUG",
      $sformatf("RDATAB mirror value (RAL) = %0h", rdata_mirror),
      UVM_LOW)
    // Belt-and-braces clear -- the monitor proxy already clears
    // pending_sdr itself once it finishes sampling the frame, but clear
    // it here too (same redundant-clear pattern used in
    // i3c_sdr_write_virtual_seq / i3c_ibi_virtual_seq) in case the frame
    // never completed within the #5000 window above.
    i3c_env_cfg_h.i3c_target_agent_cfg_h[0].pending_sdr = 0;
    `uvm_info(get_type_name(),
      "SDR READ completed successfully",
      UVM_LOW)
  endtask
endclass
`endif
