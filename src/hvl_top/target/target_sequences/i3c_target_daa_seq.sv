`ifndef I3C_TARGET_DAA_SEQ_INCLUDED_
`define I3C_TARGET_DAA_SEQ_INCLUDED_

class i3c_target_daa_seq extends i3c_target_base_seq;
  `uvm_object_utils(i3c_target_daa_seq)

  bit [47:0] cfg_pid = 48'h0;
  bit [7:0]  cfg_bcr = 8'h0;
  bit [7:0]  cfg_dcr = 8'h0;

  extern function new(string name = "i3c_target_daa_seq");
  extern task body();

endclass : i3c_target_daa_seq


function i3c_target_daa_seq::new(string name = "i3c_target_daa_seq");
  super.new(name);
endfunction : new

task i3c_target_daa_seq::body();

  `uvm_info(get_type_name(),
    "Multi-slave DAA sequence start",
    UVM_LOW)

  req = i3c_target_tx::type_id::create("req_daa");
  start_item(req);

  if (!req.randomize() with {
    txn_type == i3c_target_tx::DAA;
    pid      == cfg_pid;
    bcr      == cfg_bcr;
    dcr      == cfg_dcr;
  }) begin
    `uvm_error(get_type_name(), "Randomization failed on DAA item")
  end else begin
    `uvm_info(get_type_name(),
      $sformatf("PID=0x%0x BCR=0x%0x DCR=0x%0x", req.pid, req.bcr, req.dcr),
      UVM_LOW)
  end

  finish_item(req);

  if (req.daa_ack == ACK) begin
    `uvm_info(get_type_name(),
      $sformatf("Address assigned: dynamic_addr=0x%0h", req.dynamic_address),
      UVM_LOW)
  end else begin
    `uvm_info(get_type_name(),
      "DAA session ended (STOP) without this target winning arbitration",
      UVM_LOW)
  end

  `uvm_info(get_type_name(),
    $sformatf("DAA sequence complete. daa_ack=%0d dynamic_addr=0x%0h",
              req.daa_ack, req.dynamic_address), UVM_LOW)

endtask : body

`endif
