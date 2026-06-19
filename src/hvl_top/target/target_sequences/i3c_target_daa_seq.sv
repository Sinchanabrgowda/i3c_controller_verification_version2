`ifndef I3C_TARGET_DAA_SEQ_INCLUDED_
`define I3C_TARGET_DAA_SEQ_INCLUDED_

class i3c_target_daa_seq extends i3c_target_base_seq;
  `uvm_object_utils(i3c_target_daa_seq)

  // -----------------------------------------------------------------------
  // Fixed identity fields – set by the virtual sequence BEFORE calling
  // start() so the single DAA item uses the same PID/BCR/DCR from agent
  // config for the entire bus session.
  // -----------------------------------------------------------------------
  bit [47:0] cfg_pid = 48'h0;
  bit [7:0]  cfg_bcr = 8'h0;
  bit [7:0]  cfg_dcr = 8'h0;

  extern function new(string name = "i3c_target_daa_seq");
  extern task body();

endclass : i3c_target_daa_seq


function i3c_target_daa_seq::new(string name = "i3c_target_daa_seq");
  super.new(name);
endfunction : new

// ============================================================================
// BUG FIX -- "after one target gets DAA, STOP is detected instead of
// Repeated-START":
//
// The previous version of this sequence wrapped start_item()/finish_item()
// in a `while (!address_assigned)` loop, calling finish_item() (and
// therefore drive_daa_data() in the BFM) once per "round". Each call to
// drive_daa_data() begins with detect_start() -- it is designed to handle
// ONE complete, continuous DAA bus session from the initial START all the
// way through to the final STOP, internally looping on every intermediate
// Repeated-START until either this target wins or the master ends the
// session with STOP.
//
// Because the sequence was calling finish_item() again after every loss,
// it forced this target's BFM to tear down and call detect_start() again
// -- expecting a brand-new START condition -- even though the real bus
// was still in the middle of the very same session (other targets' wins
// were still being separated by Rep-STARTs, not by STOP+START). The BFM
// had no way to know that; from its perspective each fresh finish_item()
// call looked like an entirely new ENTDAA transaction, so it resynced on
// whatever bus activity happened to be occurring at that moment --
// producing the "broadcast addr = 0x7e/0xfd" misalignment and the
// STOP-instead-of-Rep-START symptom seen across several iterations of
// this bug.
//
// FIX: call finish_item() exactly ONCE per target. drive_daa_data() in
// the BFM already internally loops across every Rep-START within the
// single bus session on this target's behalf -- losing a round there
// does not return to the sequence, it just re-enters arbitration after
// the next Rep-START. The task only returns when:
//   - this target wins and gets ACK'd (req.daa_ack == ACK), or
//   - the master ends the whole session with STOP before this target won
//     (req.daa_ack == NACK, dynamic_address invalid)
// Both outcomes are now handled directly from the single finish_item()
// result, with no sequence-level retry loop needed or wanted.
// ============================================================================
task i3c_target_daa_seq::body();

  `uvm_info(get_type_name(),
    "Multi-slave DAA sequence start",
    UVM_LOW)

  req = i3c_target_tx::type_id::create("req_daa");
  start_item(req);

  // ------------------------------------------------------------------
  // Constrain to the FIXED PID/BCR/DCR from the agent configuration.
  // Never randomize PID/BCR/DCR – they are the device's hardware
  // identity and must be stable for the entire arbitration session so
  // that open-drain bit-by-bit comparison is deterministic.
  // ------------------------------------------------------------------
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

  // ------------------------------------------------------------------
  // After finish_item() returns, the driver proxy has called
  // drive_daa_data() exactly once and it has already run the ENTIRE
  // bus session to completion for this target -- internally looping
  // across as many Rep-STARTs as needed. req.daa_ack / req.dynamic_address
  // reflect the final outcome:
  //   ACK  -> this target won an arbitration round and was assigned
  //           req.dynamic_address before the session ended.
  //   NACK -> the master issued STOP before this target ever won
  //           (e.g. another target's DAA_seq saw the same session and
  //           this target simply never matched arbitration).
  // ------------------------------------------------------------------
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
