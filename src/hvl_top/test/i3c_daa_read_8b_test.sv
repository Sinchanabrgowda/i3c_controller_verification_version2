class i3c_daa_read_8b_test extends i3c_base_test;
  `uvm_component_utils(i3c_daa_read_8b_test)
  i3c_daa_virtual_seq         daaSeq;
  i3c_sdr_read_virtual_seq   sdrReadSeq;
  function new(string name = "i3c_daa_read_8b_test", uvm_component parent = null);
    super.new(name, parent);
  endfunction
 function void setup_target_agent_cfg();
    super.setup_target_agent_cfg();
    foreach(i3c_env_cfg_h.i3c_target_agent_cfg_h[i]) begin
      i3c_env_cfg_h.i3c_target_agent_cfg_h[i].has_daa = 1;
    end
  endfunction
  virtual task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    `uvm_info(get_type_name(), "Starting DAA sequence", UVM_LOW)
    daaSeq = i3c_daa_virtual_seq::type_id::create("daaSeq");
    daaSeq.i3c_env_cfg_h = i3c_env_cfg_h;
    daaSeq.start(i3c_env_h.top_virtual_seqr_h);
    `uvm_info(get_type_name(),
      "DAA done - updating target address to dynamic 0x08",
      UVM_LOW)
`uvm_info(get_type_name(),
  $sformatf("DAA done - target[0] dynamic addr = 0x%0h",
            i3c_env_cfg_h.i3c_target_agent_cfg_h[0].targetAddress),
  UVM_LOW)
    `uvm_info(get_type_name(),
      "Starting SDR Read with dynamic address", UVM_LOW)
    sdrReadSeq = i3c_sdr_read_virtual_seq::type_id::create("sdrReadSeq");
    sdrReadSeq.i3c_env_cfg_h = i3c_env_cfg_h;
    sdrReadSeq.start(i3c_env_h.top_virtual_seqr_h);
    #50us;
    phase.drop_objection(this);
  endtask
  endclass
