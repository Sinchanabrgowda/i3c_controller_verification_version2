class i3c_ibi_test extends i3c_base_test;
  `uvm_component_utils(i3c_ibi_test)
  i3c_ibi_virtual_seq ibiSeq;
  function new(string name = "i3c_ibi_test",
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
    ibiSeq = i3c_ibi_virtual_seq::type_id::create("ibiSeq");
    ibiSeq.i3c_env_cfg_h    = i3c_env_cfg_h;
    ibiSeq.ibi_target_idx   = 0;
    ibiSeq.ibi_mdb_payload  = 8'h17;


    //   ibiSeq.ibi_num_extra_bytes = 3;
    //   ibiSeq.ibi_extra_data      = '{8'h2A, 8'h3B, 8'h4C};
    ibiSeq.ibi_num_extra_bytes = 1;
    ibiSeq.ibi_extra_data      = '{8'h2A};
    ibiSeq.start(i3c_env_h.top_virtual_seqr_h);
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
