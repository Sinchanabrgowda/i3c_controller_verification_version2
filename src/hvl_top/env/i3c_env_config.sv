`ifndef I3C_ENV_CONFIG_INCLUDED_
`define I3C_ENV_CONFIG_INCLUDED_

class i3c_env_config extends uvm_object;
  `uvm_object_utils(i3c_env_config)

  bit                          has_scoreboard        = 1;
  bit                          has_virtual_sequencer = 1;

  // Only target (slave) agents
  int                          no_of_targets;
  i3c_target_agent_config      i3c_target_agent_cfg_h[];

  writeReadMode_e              writeReadMode_h;
  i3c_ral_reg_block            regBlockHandle;

  // DAA
  bit                          has_daa           = 0;
  int                          no_of_daa_devices = 0;

  extern function new(string name = "i3c_env_config");
  extern function void do_print(uvm_printer printer);

endclass : i3c_env_config


function i3c_env_config::new(string name = "i3c_env_config");
  super.new(name);
endfunction


function void i3c_env_config::do_print(uvm_printer printer);
  super.do_print(printer);
  printer.print_field("has_scoreboard",
    has_scoreboard,        1,                        UVM_DEC);
  printer.print_field("has_virtual_sequencer",
    has_virtual_sequencer, 1,                        UVM_DEC);
  printer.print_field("no_of_targets",
    no_of_targets,         $bits(no_of_targets),     UVM_DEC);
  printer.print_field("has_daa",
    has_daa,               1,                        UVM_DEC);
  printer.print_field("no_of_daa_devices",
    no_of_daa_devices,     $bits(no_of_daa_devices), UVM_DEC);
endfunction

`endif

