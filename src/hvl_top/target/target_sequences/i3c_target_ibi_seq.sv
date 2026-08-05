`ifndef I3C_TARGET_IBI_SEQ_INCLUDED_
`define I3C_TARGET_IBI_SEQ_INCLUDED_
class i3c_target_ibi_seq extends i3c_target_base_seq;
  `uvm_object_utils(i3c_target_ibi_seq)
  bit [7:0] cfg_mdb = 8'h17;
  int unsigned cfg_num_extra_bytes = 0;
  bit [7:0]    cfg_extra_data[];
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
    txn_type            == i3c_target_tx::IBI;
    ibi_mdb             == cfg_mdb;
    ibi_num_extra_bytes == cfg_num_extra_bytes;
    foreach (cfg_extra_data[i])
      ibi_extra_data[i] == cfg_extra_data[i];
  }) begin
    `uvm_error(get_type_name(), "Randomization failed on IBI item")
  end else begin
    `uvm_info(get_type_name(),
      $sformatf("mdb=0x%0h num_extra_bytes=%0d",
                req.ibi_mdb, req.ibi_num_extra_bytes), UVM_LOW)
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
  if (req.ibi_extra_data_sent.size() > 0) begin
    foreach (req.ibi_extra_data_sent[i]) begin
      `uvm_info(get_type_name(),
        $sformatf("IBI extra byte[%0d] sent: data=0x%0h T=%0b -> %0s",
                  i, req.ibi_extra_data_sent[i], req.ibi_extra_t_bits[i],
                  req.ibi_extra_t_bits[i] ? "MORE DATA follows" :
                                             "NO MORE DATA (STOP)"),
        UVM_LOW)
    end
  end else if (cfg_num_extra_bytes > 0) begin
    `uvm_info(get_type_name(),
      "IBI extra data bytes requested by sequence but NOT sent -- the IBI request itself was likely NACKed (target drives T-bits unconditionally once ACKed)",
      UVM_LOW)
  end
  `uvm_info(get_type_name(),
    $sformatf("IBI sequence complete. ack=%0d dynamic_addr=0x%0h mdb=0x%0h extra_bytes_sent=%0d",
              req.daa_ack, req.dynamic_address, req.ibi_mdb,
              req.ibi_extra_data_sent.size()), UVM_LOW)
endtask : body
`endif
