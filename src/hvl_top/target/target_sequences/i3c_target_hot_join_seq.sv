`ifndef I3C_TARGET_HOT_JOIN_SEQ_INCLUDED_
`define I3C_TARGET_HOT_JOIN_SEQ_INCLUDED_

class i3c_target_hot_join_seq extends i3c_target_base_seq;
  `uvm_object_utils(i3c_target_hot_join_seq)

  bit [47:0] cfg_pid          = 48'h0;
  bit [7:0]  cfg_bcr          = 8'h0;
  bit [7:0]  cfg_dcr          = 8'h0;
  bit [6:0]  cfg_hotjoin_addr = 7'h2;

  extern function new(string name = "i3c_target_hot_join_seq");
  extern task body();

endclass : i3c_target_hot_join_seq


function i3c_target_hot_join_seq::new(string name = "i3c_target_hot_join_seq");
  super.new(name);
endfunction : new

task i3c_target_hot_join_seq::body();

  `uvm_info(get_type_name(),
    $sformatf("HOT JOIN sequence start (ibi_addr=0x%0h)", cfg_hotjoin_addr),
    UVM_LOW)

  req = i3c_target_tx::type_id::create("req_hotjoin");
  start_item(req);

  if (!req.randomize() with {
    txn_type     == i3c_target_tx::HOTJOIN;
    pid          == cfg_pid;
    bcr          == cfg_bcr;
    dcr          == cfg_dcr;
    hotjoin_addr == cfg_hotjoin_addr;
  }) begin
    `uvm_error(get_type_name(), "Randomization failed on HOTJOIN item")
  end else begin
    `uvm_info(get_type_name(),
      $sformatf("ibi_addr=0x%0h PID=0x%0x BCR=0x%0x DCR=0x%0x",
                req.hotjoin_addr, req.pid, req.bcr, req.dcr),
      UVM_LOW)
  end

  finish_item(req);

  if (req.daa_ack == ACK) begin
    `uvm_info(get_type_name(),
      $sformatf("HOT JOIN successful: dynamic_addr=0x%0h assigned via ENTDAA",
                req.dynamic_address),
      UVM_LOW)
  end else begin
    `uvm_error(get_type_name(),
      "HOT JOIN did not result in a dynamic address assignment (daa_ack != ACK)")
  end

  `uvm_info(get_type_name(),
    $sformatf("HOT JOIN sequence complete. daa_ack=%0d dynamic_addr=0x%0h",
              req.daa_ack, req.dynamic_address), UVM_LOW)

endtask : body

`endif

