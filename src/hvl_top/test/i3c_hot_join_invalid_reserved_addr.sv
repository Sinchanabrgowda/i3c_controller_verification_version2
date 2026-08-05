class i3c_hot_join_inv_addr_test extends i3c_base_test;
  `uvm_component_utils(i3c_hot_join_inv_addr_test)

  i3c_hot_join_invalid_addr_virtual_seq hotJoinSeq_inv_addr;

  function new(string name = "i3c_hot_join_inv_addr_test",
               uvm_component parent = null);
    super.new(name, parent);
  endfunction

  function void setup_target_agent_cfg();
    super.setup_target_agent_cfg();
    foreach (i3c_env_cfg_h.i3c_target_agent_cfg_h[i]) begin
      i3c_env_cfg_h.i3c_target_agent_cfg_h[i].has_daa           = 1;
      i3c_env_cfg_h.i3c_target_agent_cfg_h[i].pending_hot_join  = 0;
    end
    i3c_env_cfg_h.has_daa           = 1;

    i3c_env_cfg_h.no_of_daa_devices = NO_OF_TARGETS;
  endfunction

  virtual task run_phase(uvm_phase phase);
    phase.raise_objection(this);

    `uvm_info(get_type_name(),
      $sformatf("Starting hot-join test (%0d targets, last one hot-joins)",
                NO_OF_TARGETS),
      UVM_LOW)

    hotJoinSeq_inv_addr = i3c_hot_join_invalid_addr_virtual_seq::type_id::create("hotJoinSeq_inv_addr");
    hotJoinSeq_inv_addr.i3c_env_cfg_h    = i3c_env_cfg_h;
    hotJoinSeq_inv_addr.hotjoin_ibi_addr = 7'h05;
    hotJoinSeq_inv_addr.start(i3c_env_h.top_virtual_seqr_h);

    `uvm_info(get_type_name(), "Hot-join test phase complete", UVM_LOW)

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
