`ifndef I3C_TARGET_AGENT_CONFIG_INCLUDED_
`define I3C_TARGET_AGENT_CONFIG_INCLUDED_

class i3c_target_agent_config extends uvm_object;
  `uvm_object_utils(i3c_target_agent_config)

 
  uvm_active_passive_enum            isActive             = UVM_ACTIVE;
  hasCoverage_e                      hasCoverage          = TRUE;
  dataTransferDirection_e            dataTransferDirection;
  bit [TARGET_ADDRESS_WIDTH-1:0]     targetAddress;
  bit [DATA_WIDTH-1:0]               defaultReadData      = 'hFF;


  int unsigned                       target_id            = 0;

 bit  has_daa               = 0;

 bit  pending_hot_join      = 0;
 bit [6:0] hotjoin_addr      = 7'h20;
 
 
  bit                                daa_accept_address   = 1;
  bit [47:0] pid = 48'hAABBCCDDEEFF;  
  bit [7:0]  bcr = 8'h00;          
  bit [7:0]  dcr = 8'hC2;
 // Set by the hot-join virtual sequence on every target EXCEPT the one
// currently hot-joining. While set, that target's monitor proxy stays
// completely off the bus.
bit hotjoin_in_progress_elsewhere = 0;

  extern function new(string name = "i3c_target_agent_config");
  extern function void do_print(uvm_printer printer);

endclass : i3c_target_agent_config


function i3c_target_agent_config::new(string name = "i3c_target_agent_config");
  super.new(name);
endfunction : new


function void i3c_target_agent_config::do_print(uvm_printer printer);
  super.do_print(printer);
  printer.print_string ("isActive",            isActive.name());
  printer.print_string ("dataTransferDirection",dataTransferDirection.name());
  printer.print_string ("hasCoverage",          hasCoverage.name());
  printer.print_field  ("targetAddress",        targetAddress,
                         $bits(targetAddress),  UVM_HEX);

  printer.print_field  ("target_id",            target_id,  32, UVM_DEC);
    printer.print_field  ("daa_accept_address",   daa_accept_address, 1, UVM_BIN);
    printer.print_field  ("pending_hot_join",     pending_hot_join, 1, UVM_BIN);
    printer.print_field  ("hotjoin_addr",         hotjoin_addr, 7, UVM_HEX);
printer.print_field ("pid",                   pid, 48, UVM_HEX);
  printer.print_field ("bcr",                   bcr,  8, UVM_HEX);
  printer.print_field ("dcr",                   dcr,  8, UVM_HEX);
endfunction : do_print

`endif

