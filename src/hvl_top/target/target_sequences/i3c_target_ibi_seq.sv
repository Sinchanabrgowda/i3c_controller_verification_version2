`ifndef I3C_TARGET_IBI_SEQ_INCLUDED_
`define I3C_TARGET_IBI_SEQ_INCLUDED_

class i3c_target_ibi_seq extends i3c_target_base_seq;
  `uvm_object_utils(i3c_target_ibi_seq)

  bit [7:0] cfg_mdb = 8'h17;   // Mandatory Data Byte to send with the IBI

  extern function new(string name = "i3c_target_ibi_seq");
  extern task body();

endclass : i3c_target_ibi_seq


function i3c_target_ibi_seq::new(string name = "i3c_target_ibi_seq");
  super.new(name);
endfunction : new

task i3c_target_ibi_seq::body();

  `uvm_info(get_type_name(),
    $sformatf("IBI sequence start (mdb=0x%0h)", cfg_mdb),
    UVM_LOW)

  req = i3c_target_tx::type_id::create("req_ibi");
  start_item(req);

  if (!req.randomize() with {
    txn_type == i3c_target_tx::IBI;
    ibi_mdb  == cfg_mdb;
  }) begin
    `uvm_error(get_type_name(), "Randomization failed on IBI item")
  end else begin
    `uvm_info(get_type_name(),
      $sformatf("mdb=0x%0h", req.ibi_mdb), UVM_LOW)
  end

  finish_item(req);

  if (req.daa_ack == ACK) begin
    `uvm_info(get_type_name(),
      $sformatf("IBI accepted: controller ACKed request from dynamic_addr=0x%0h",
                req.dynamic_address),
      UVM_LOW)
  end else begin
    `uvm_error(get_type_name(),
      "IBI request was NACKed by controller (daa_ack != ACK)")
  end

  `uvm_info(get_type_name(),
    $sformatf("IBI sequence complete. ack=%0d dynamic_addr=0x%0h mdb=0x%0h",
              req.daa_ack, req.dynamic_address, req.ibi_mdb), UVM_LOW)

endtask : body

`endif

