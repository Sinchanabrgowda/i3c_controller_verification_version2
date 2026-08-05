class i3c_ibi_t0_test extends i3c_base_test;
  `uvm_component_utils(i3c_ibi_t0_test)
  i3c_ibi_t0_virtual_seq ibiSeqt0;
  function new(string name = "i3c_ibi_t0_test",
               uvm_component parent = null);
    super.new(name, parent);
  endfunction
  function void setup_target_agent_cfg();
    super.setup_target_agent_cfg();
    foreach (i3c_env_cfg_h.i3c_target_agent_cfg_h[i]) begin
      i3c_env_cfg_h.i3c_target_agent_cfg_h[i].has_daa      = 1;
      i3c_env_cfg_h.i3c_target_agent_cfg_h[i].pending_ibi  = 0;
    end
    i3c_env_cfg_h.has_daa           = 1;
    i3c_env_cfg_h.no_of_daa_devices = NO_OF_TARGETS;
  endfunction
  virtual task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    `uvm_info(get_type_name(),
      $sformatf("Starting IBI test (%0d targets get DAA, target[0] sends IBI)",
                NO_OF_TARGETS),
      UVM_LOW)
    ibiSeqt0 = i3c_ibi_t0_virtual_seq::type_id::create("ibiSeqt0");
    ibiSeqt0.i3c_env_cfg_h    = i3c_env_cfg_h;
    ibiSeqt0.ibi_target_idx   = 0;
    ibiSeqt0.ibi_mdb_payload  = 8'h17;
    // T-bit  Flip to 1 to exercise
    // the 2nd-byte path.
    ibiSeqt0.send_second_ibi_byte = 0;
    ibiSeqt0.ibi_mdb2_payload     = 8'h2A;
    ibiSeqt0.start(i3c_env_h.top_virtual_seqr_h);
    `uvm_info(get_type_name(), "IBI test phase complete", UVM_LOW)
    foreach (i3c_env_cfg_h.i3c_target_agent_cfg_h[i]) begin
      `uvm_info(get_type_name(),
        $sformatf("  target[%0d] dynamic addr = 0x%0h",
                  i, i3c_env_cfg_h.i3c_target_agent_cfg_h[i].targetAddress),
        UVM_LOW)
    end
    #50us;
    phase.drop_objection(this);
  endtask
endclass
