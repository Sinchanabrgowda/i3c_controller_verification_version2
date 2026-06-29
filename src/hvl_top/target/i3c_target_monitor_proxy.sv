`ifndef I3C_TARGET_MONITOR_PROXY_INCLUDED_
`define I3C_TARGET_MONITOR_PROXY_INCLUDED_

class i3c_target_monitor_proxy extends uvm_component;
  `uvm_component_utils(i3c_target_monitor_proxy)

  i3c_target_tx                      tx;
  i3c_target_agent_config            i3c_target_agent_cfg_h;
  virtual i3c_target_monitor_bfm     i3c_target_mon_bfm_h;
  uvm_analysis_port #(i3c_target_tx) target_analysis_port;

  localparam bit [7:0] BCAST_7E_W  = 8'hFC;
  localparam bit [7:0] ENTDAA_CODE = 8'h07;

  extern function new(string name = "i3c_target_monitor_proxy",
                      uvm_component parent = null);
  extern virtual function void build_phase(uvm_phase phase);
  extern virtual function void connect_phase(uvm_phase phase);
  extern virtual function void end_of_elaboration_phase(uvm_phase phase);
  extern virtual function void start_of_simulation_phase(uvm_phase phase);
  extern virtual task          run_phase(uvm_phase phase);

endclass : i3c_target_monitor_proxy


function i3c_target_monitor_proxy::new(
    string name = "i3c_target_monitor_proxy",
    uvm_component parent = null);
  super.new(name, parent);
  target_analysis_port = new("target_analysis_port", this);
  tx = new();
endfunction : new


function void i3c_target_monitor_proxy::build_phase(uvm_phase phase);

  super.build_phase(phase);
endfunction : build_phase


function void i3c_target_monitor_proxy::connect_phase(uvm_phase phase);
  super.connect_phase(phase);
endfunction : connect_phase


function void i3c_target_monitor_proxy::end_of_elaboration_phase(
    uvm_phase phase);
  string mon_key;
  super.end_of_elaboration_phase(phase);

  // Build the per-target key that i3c_target_agent_bfm registered
  mon_key = $sformatf("i3c_target_monitor_bfm_%0d",
                       i3c_target_agent_cfg_h.target_id);

  `uvm_info("TGT_MON_PROXY",
    $sformatf("[target_id=%0d] Looking up monitor BFM with key: %s",
              i3c_target_agent_cfg_h.target_id, mon_key), UVM_LOW)

  if (!uvm_config_db #(virtual i3c_target_monitor_bfm)::get(
        this, "", mon_key, i3c_target_mon_bfm_h)) begin
    `uvm_fatal("FATAL_MDP_CANNOT_GET_target_MONITOR_BFM",
      $sformatf("Cannot get i3c_target_monitor_bfm from config_db. key=%s",
                mon_key))
  end

  i3c_target_mon_bfm_h.i3c_target_mon_proxy_h = this;

endfunction : end_of_elaboration_phase


function void i3c_target_monitor_proxy::start_of_simulation_phase(
    uvm_phase phase);
  super.start_of_simulation_phase(phase);
endfunction : start_of_simulation_phase


task i3c_target_monitor_proxy::run_phase(uvm_phase phase);
  i3c_transfer_bits_s struct_packet;
  i3c_transfer_cfg_s  struct_cfg;

  `uvm_info(get_type_name(),
    $sformatf("[target_id=%0d] Monitor Proxy running",
              i3c_target_agent_cfg_h.target_id), UVM_HIGH)

  i3c_target_mon_bfm_h.wait_for_reset();
  i3c_target_mon_bfm_h.sample_idle_state();

  forever begin

    tx = i3c_target_tx::type_id::create("tx");

    // Build cfg struct
    i3c_target_cfg_converter::from_class(i3c_target_agent_cfg_h, struct_cfg);
    i3c_target_seq_item_converter::from_class(tx, struct_packet);

    if (i3c_target_agent_cfg_h != null &&
        i3c_target_agent_cfg_h.has_daa) begin

      `uvm_info(get_type_name(),
        $sformatf("[target_id=%0d] Waiting to sample DAA transaction",
                  i3c_target_agent_cfg_h.target_id), UVM_HIGH)

      i3c_target_mon_bfm_h.sample_daa_data(struct_packet, struct_cfg);

      i3c_target_seq_item_converter::to_class(struct_packet, tx);
      tx.txn_type = i3c_target_tx::DAA;

    end else begin

      `uvm_info(get_type_name(),
        $sformatf("[target_id=%0d] Waiting to sample SDR transaction",
                  i3c_target_agent_cfg_h.target_id), UVM_HIGH)

      i3c_target_mon_bfm_h.sample_data(struct_packet, struct_cfg);

      i3c_target_seq_item_converter::to_class(struct_packet, tx);

    end

    `uvm_info(get_type_name(),
      $sformatf("[target_id=%0d] Sampled transaction – writing to analysis port",
                i3c_target_agent_cfg_h.target_id), UVM_HIGH)

    target_analysis_port.write(tx);

  end // forever

endtask : run_phase

`endif
