`ifndef I3C_TARGET_HDR_DDR_SEQ_INCLUDED_
`define I3C_TARGET_HDR_DDR_SEQ_INCLUDED_

class i3c_target_hdr_ddr_seq extends i3c_target_base_seq;
  `uvm_object_utils(i3c_target_hdr_ddr_seq)

  int unsigned cfg_num_words = 1;

  extern function new(string name = "i3c_target_hdr_ddr_seq");
  extern task body();

endclass : i3c_target_hdr_ddr_seq


function i3c_target_hdr_ddr_seq::new(string name = "i3c_target_hdr_ddr_seq");
  super.new(name);
endfunction : new

task i3c_target_hdr_ddr_seq::body();

  `uvm_info(get_type_name(),
    $sformatf("HDR-DDR target sequence start (cfg_num_words=%0d)", cfg_num_words),
    UVM_LOW)

  req = i3c_target_tx::type_id::create("req_hdr_ddr");
  start_item(req);

  if (!req.randomize() with {
    txn_type          == i3c_target_tx::HDR_DDR;
    hdr_ddr_num_words == local::cfg_num_words;
  }) begin
    `uvm_error(get_type_name(), "Randomization failed on HDR-DDR item")
  end else begin
    `uvm_info(get_type_name(),
      $sformatf("hdr_ddr_num_words=%0d", req.hdr_ddr_num_words), UVM_LOW)
  end

  finish_item(req);

  `uvm_info(get_type_name(),
    $sformatf("HDR-DDR sequence complete. cmd_code=0x%0h cmd_ack=%0b crc_ok=%0b words=%0d restart=%0b exit=%0b",
              req.hdr_ddr_cmd_code, req.hdr_ddr_cmd_ack, req.hdr_ddr_crc_ok,
              req.hdr_ddr_num_words, req.hdr_ddr_got_restart, req.hdr_ddr_got_exit),
    UVM_LOW)

endtask : body

`endif

