class i3c_daa_write_8b_test_len_16 extends i3c_base_test;
  `uvm_component_utils(i3c_daa_write_8b_test_len_16)
  i3c_daa_virtual_seq       daaSeq;
  i3c_sdr_write_virtual_seq_len_16 sdrWriteSeq;
  function new(string name = "i3c_daa_write_8b_test_len_16",
         uvm_component parent = null);
    super.new(name, parent);
  endfunction
  function void setup_target_agent_cfg();
    super.setup_target_agent_cfg();
    foreach (i3c_env_cfg_h.i3c_target_agent_cfg_h[i]) begin
      i3c_env_cfg_h.i3c_target_agent_cfg_h[i].has_daa = 1;
    end
    i3c_env_cfg_h.has_daa           = 1;
    i3c_env_cfg_h.no_of_daa_devices = NO_OF_TARGETS;
    //i3c_env_cfg_h.no_of_daa_devices = 0;
  endfunction
  virtual task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    `uvm_info(get_type_name(),
      $sformatf("Starting multi-slave DAA test (%0d targets)", NO_OF_TARGETS),
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
      "Starting SDR WRITE with dynamic address of target[0]", UVM_LOW)
    sdrWriteSeq = i3c_sdr_write_virtual_seq_len_16::type_id::create("sdrWriteSeq");
    sdrWriteSeq.i3c_env_cfg_h = i3c_env_cfg_h;
    sdrWriteSeq.start(i3c_env_h.top_virtual_seqr_h);
    #50us;
    phase.drop_objection(this);
  endtask
endclass
