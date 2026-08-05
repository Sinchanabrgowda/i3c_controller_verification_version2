`ifndef I3C_TARGET_IBI_T0_SEQ_INCLUDED_
`define I3C_TARGET_IBI_T0_SEQ_INCLUDED_
class i3c_target_ibi_t0_seq extends i3c_target_base_seq;
  `uvm_object_utils(i3c_target_ibi_t0_seq)
  bit [7:0] cfg_mdb = 8'h17;
  bit       cfg_send_second_byte = 0;
  bit [7:0] cfg_mdb2             = 8'h2A;
  extern function new(string name = "i3c_target_ibi_t0_seq");
  extern task body();
endclass : i3c_target_ibi_t0_seq

function i3c_target_ibi_t0_seq::new(string name = "i3c_target_ibi_t0_seq");
  super.new(name);
endfunction : new

task i3c_target_ibi_t0_seq::body();
  `uvm_info(get_type_name(),
    $sformatf("IBI sequence start (mdb=0x%0h)", cfg_mdb),
    UVM_LOW)
  req = i3c_target_tx::type_id::create("req_ibi");
  start_item(req);
  if (!req.randomize() with {
    txn_type           == i3c_target_tx::IBI;
    ibi_mdb            == cfg_mdb;
    ibi_want_more_data == cfg_send_second_byte;
    ibi_data2          == cfg_mdb2;
  }) begin
    `uvm_error(get_type_name(), "Randomization failed on IBI item")
  end else begin
    `uvm_info(get_type_name(),
      $sformatf("mdb=0x%0h want_more_data=%0b mdb2=0x%0h",
                req.ibi_mdb, req.ibi_want_more_data, req.ibi_data2), UVM_LOW)
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
    $sformatf("IBI T-bit result: T1=%0b (driven by target after MDB) -> %0s",
              req.ibi_t1,
              req.ibi_t1 ? "target signaled MORE DATA follows" :
                           "target signaled NO MORE DATA (STOP)"),
    UVM_LOW)
  if (req.ibi_data2_sent) begin
    `uvm_info(get_type_name(),
      $sformatf("IBI 2nd data byte sent: mdb2=0x%0h T2=%0b",
                req.ibi_data2, req.ibi_t2), UVM_LOW)
  end else if (cfg_send_second_byte) begin
    `uvm_info(get_type_name(),
      "IBI 2nd data byte requested by sequence but NOT sent -- the IBI request itself was likely NACKed (target drives T1/T2 unconditionally once ACKed)",
      UVM_LOW)
  end
  `uvm_info(get_type_name(),
    $sformatf("IBI sequence complete. ack=%0d dynamic_addr=0x%0h mdb=0x%0h data2_sent=%0b",
              req.daa_ack, req.dynamic_address, req.ibi_mdb, req.ibi_data2_sent), UVM_LOW)
endtask : body
`endif
