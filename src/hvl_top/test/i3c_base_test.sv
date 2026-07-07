
`ifndef I3C_BASE_TEST_INCLUDED_
`define I3C_BASE_TEST_INCLUDED_

class i3c_base_test extends uvm_test;
  `uvm_component_utils(i3c_base_test)

  i3c_env             i3c_env_h;
  i3c_env_config      i3c_env_cfg_h;
  apb_env_config      apb_env_cfg_h;
  apb_master_agent_config apb_master_agent_cfg_h;

  extern function new(string name = "i3c_base_test",
                      uvm_component parent = null);
  extern virtual function void build_phase(uvm_phase phase);
  extern virtual function void setup_env_cfg();
  extern virtual function void setup_target_agent_cfg();
  extern virtual function void end_of_elaboration_phase(uvm_phase phase);
  extern virtual task          run_phase(uvm_phase phase);

endclass : i3c_base_test


function i3c_base_test::new(string name = "i3c_base_test",
                             uvm_component parent = null);
  super.new(name, parent);
endfunction


function void i3c_base_test::build_phase(uvm_phase phase);
  super.build_phase(phase);

  i3c_env_cfg_h = i3c_env_config::type_id::create("i3c_env_cfg_h");
  i3c_env_h     = i3c_env::type_id::create("i3c_env_h", this);

  apb_env_cfg_h = apb_env_config::type_id::create("apb_env_cfg_h");
  uvm_config_db #(apb_env_config)::set(
      this, "*", "apb_env_config", apb_env_cfg_h);

  apb_master_agent_cfg_h =
    apb_master_agent_config::type_id::create("apb_master_agent_cfg_h");
  apb_master_agent_cfg_h.no_of_slaves = NO_OF_SLAVES;
  apb_master_agent_cfg_h.has_coverage = 1;
  apb_master_agent_cfg_h.master_min_addr_range(0, 32'h0000_0000);
  apb_master_agent_cfg_h.master_max_addr_range(0, 32'h0000_007F);
  uvm_config_db #(apb_master_agent_config)::set(
      this, "*", "apb_master_agent_config", apb_master_agent_cfg_h);

  setup_env_cfg();

endfunction : build_phase


function void i3c_base_test::setup_env_cfg();

  i3c_env_cfg_h.no_of_targets          = NO_OF_TARGETS;
  i3c_env_cfg_h.has_scoreboard         = 1;
  i3c_env_cfg_h.has_virtual_sequencer  = 1;
  i3c_env_cfg_h.writeReadMode_h        = WRITE_READ;

  // Target agent configs
  i3c_env_cfg_h.i3c_target_agent_cfg_h =
    new[i3c_env_cfg_h.no_of_targets];
  foreach (i3c_env_cfg_h.i3c_target_agent_cfg_h[i]) begin
    i3c_env_cfg_h.i3c_target_agent_cfg_h[i] =
      i3c_target_agent_config::type_id::create(
        $sformatf("i3c_target_agent_cfg_h[%0d]", i));
  end
  setup_target_agent_cfg();

  foreach (i3c_env_cfg_h.i3c_target_agent_cfg_h[i]) begin
    uvm_config_db #(i3c_target_agent_config)::set(
      this,
      $sformatf("*i3c_target_agent_h[%0d]*", i),
      "i3c_target_agent_config",
      i3c_env_cfg_h.i3c_target_agent_cfg_h[i]);
    `uvm_info(get_type_name(),
      $sformatf("target_agent_cfg[%0d]:\n%s", i,
                i3c_env_cfg_h.i3c_target_agent_cfg_h[i].sprint()),
      UVM_NONE)
  end

  uvm_config_db #(i3c_env_config)::set(
      this, "*", "i3c_env_config", i3c_env_cfg_h);
  `uvm_info(get_type_name(),
    $sformatf("i3c_env_cfg:\n%s", i3c_env_cfg_h.sprint()), UVM_NONE)

endfunction : setup_env_cfg


function void i3c_base_test::setup_target_agent_cfg();
  foreach (i3c_env_cfg_h.i3c_target_agent_cfg_h[i]) begin
    i3c_env_cfg_h.i3c_target_agent_cfg_h[i].target_id    = i;
    i3c_env_cfg_h.i3c_target_agent_cfg_h[i].isActive     = UVM_ACTIVE;
    i3c_env_cfg_h.i3c_target_agent_cfg_h[i].dataTransferDirection =
      dataTransferDirection_e'(MSB_FIRST);
    i3c_env_cfg_h.i3c_target_agent_cfg_h[i].hasCoverage =
      hasCoverage_e'(TRUE);
    i3c_env_cfg_h.i3c_target_agent_cfg_h[i].has_daa          = 0;
    i3c_env_cfg_h.i3c_target_agent_cfg_h[i].daa_accept_address = 1;

    case (i)
      0: begin
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].targetAddress = TARGET0_ADDRESS;
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].pid           = TARGET0_PID;
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].bcr           = DEFAULT_BCR;
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].dcr           = TARGET0_DCR;
         end
      1: begin
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].targetAddress = TARGET1_ADDRESS;
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].pid           = TARGET1_PID;
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].bcr           = DEFAULT_BCR;
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].dcr           = TARGET1_DCR;
         end
      2: begin
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].targetAddress = TARGET2_ADDRESS;
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].pid           = TARGET2_PID;
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].bcr           = DEFAULT_BCR;
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].dcr           = TARGET2_DCR;
         end
      3: begin
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].targetAddress = TARGET3_ADDRESS;
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].pid           = TARGET3_PID;
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].bcr           = DEFAULT_BCR;
           i3c_env_cfg_h.i3c_target_agent_cfg_h[i].dcr           = TARGET3_DCR;
         end
      default: begin
        i3c_env_cfg_h.i3c_target_agent_cfg_h[i].pid = 48'h00_AABB_CC00_00 + i + 1;
        i3c_env_cfg_h.i3c_target_agent_cfg_h[i].bcr = DEFAULT_BCR;
        i3c_env_cfg_h.i3c_target_agent_cfg_h[i].dcr = 8'hC0 + i;
      end
    endcase
  end
endfunction : setup_target_agent_cfg


function void i3c_base_test::end_of_elaboration_phase(uvm_phase phase);
  uvm_top.print_topology();
  uvm_test_done.set_drain_time(this, 3000ns);
endfunction


task i3c_base_test::run_phase(uvm_phase phase);
  phase.raise_objection(this, "i3c_base_test");
  `uvm_info(get_type_name(), "Inside I3C_BASE_TEST", UVM_NONE)
  super.run_phase(phase);
  #10;
  `uvm_info(get_type_name(), "Done I3C_BASE_TEST", UVM_NONE)
  phase.drop_objection(this);
endtask

`endif

