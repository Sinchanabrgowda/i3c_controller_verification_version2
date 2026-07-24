`ifndef I3C_HDR_DDR_WRITE_VIRTUAL_SEQ_INCLUDED_
`define I3C_HDR_DDR_WRITE_VIRTUAL_SEQ_INCLUDED_

// ============================================================================
// FILE: i3c_hdr_ddr_write_virtual_seq.sv
//
// HDR-DDR (Optional Feature F001, MIPI I3C Basic Spec v1.2, Section 6.2)
// WRITE virtual sequence -- NEW, additive only file. Modeled directly on
// i3c_sdr_write_virtual_seq.sv (SDR write) and the CCC-trigger pattern in
// i3c_daa_virtual_seq.sv / i3c_ccc_coverage_virtual_seq.sv, driving the DUT
// through the same APB/RAL ctrl_inst register the rest of this AVIP uses.
//
// IMPORTANT - register-trigger semantics note (the DUT RTL was
// intentionally NOT inspected to derive this, per the request that
// accompanied this change; only the MIPI spec and the AVIP's own,
// pre-existing RAL model were used):
//   Step 1 (enter HDR-DDR via the ENTHDR0 Broadcast CCC) reuses the exact
//   cmd_type value (2'd1) that i3c_ccc_coverage_virtual_seq.sv already
//   uses to drive CCC byte 0x20 (which IS ENTHDR0) -- that is the only
//   place in this codebase that already exercises this CCC, so it is the
//   best available precedent. Note this is inconsistent with the
//   CMD_TYPE_CCC=2'b10 parameter in i3c_globals_pkg.sv, which does not
//   appear to be used by that existing sequence; this pre-existing
//   inconsistency was not introduced here and was left exactly as found.
//   Step 2 (the actual HDR-DDR Command Word: direction/address/length)
//   reuses the same cmd_addr/cmd_dir/cmd_len fields as a normal SDR
//   transfer, with cmd_mode=1 added. cmd_mode is a RAL field that already
//   exists specifically alongside cmd_type, and the STATUS register
//   already has hdr_busy/hdr_done/hdr_bit_busy/hdr_bit_done bits mirroring
//   sdr_busy/sdr_done/sdr_bit_busy/sdr_bit_done -- so cmd_mode=1 selecting
//   the HDR-DDR data path is a reasonable inference from the RAL model's
//   own field names, but it is a best-effort mapping, not a confirmed
//   contract. Please validate against your DUT and adjust if needed.
// ============================================================================
class i3c_hdr_ddr_write_virtual_seq extends top_virtual_base_seq;
  `uvm_object_utils(i3c_hdr_ddr_write_virtual_seq)

  uvm_status_e   status;
  uvm_reg_data_t ctrl_val;

  int unsigned target_idx = 0;
  int unsigned num_words  = 1;   // number of 16-bit HDR-DDR Data Words

  function new(string name = "i3c_hdr_ddr_write_virtual_seq");
    super.new(name);
  endfunction

  task body();
    i3c_target_hdr_ddr_seq target_seq_hdr_ddr;
    bit [7:0] wbyte;

    if (i3c_env_cfg_h == null)
      `uvm_fatal("CFG_NULL", "i3c_env_cfg_h is NULL inside i3c_hdr_ddr_write_virtual_seq")
    if (i3c_env_cfg_h.regBlockHandle == null)
      `uvm_fatal("RAL_NULL", "regBlockHandle is NULL inside i3c_hdr_ddr_write_virtual_seq")

    super.body();

    `uvm_info(get_type_name(),
      $sformatf("Starting HDR-DDR WRITE to target[%0d] addr=0x%0h, %0d word(s)",
                target_idx,
                i3c_env_cfg_h.i3c_target_agent_cfg_h[target_idx].targetAddress,
                num_words),
      UVM_LOW)

    i3c_env_cfg_h.i3c_target_agent_cfg_h[target_idx].pending_hdr_ddr = 1;

    fork
      begin
        target_seq_hdr_ddr = i3c_target_hdr_ddr_seq::type_id::create("target_seq_hdr_ddr");
        target_seq_hdr_ddr.cfg_num_words = num_words;
        target_seq_hdr_ddr.start(p_sequencer.i3c_target_seqr_h[target_idx]);
      end
    join_none

    // --- Step 1: enter HDR-DDR mode via the ENTHDR0 Broadcast CCC ---
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_type.set(2'd1);   // see note above
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_ccc.set(ENTHDR0_CCC_CODE);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_addr.set(I3C_BROADCAST_ADDR);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_len.set(8'd0);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_dir.set(1'b0);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_mode.set(1'b0);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.start.set(1'b1);

    ctrl_val = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get();
    `uvm_info("CTRL_DEBUG", $sformatf("CTRL before ENTHDR0 = 0x%0h", ctrl_val), UVM_LOW)

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.update(status, .parent(this));
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.mirror(status, UVM_NO_CHECK);

    #5000;

    // --- Push the Write payload bytes the same way SDR writes do (2 bytes
    //     per HDR-DDR Data Word) ---
    for (int i = 0; i < num_words*2; i++) begin
      wbyte = 8'hA5 + i[7:0];
      i3c_env_cfg_h.regBlockHandle.wdatab_inst.write(status, wbyte);
    end

    // --- Step 2: the HDR-DDR Command Word (Write) ---
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_addr.set(
      i3c_env_cfg_h.i3c_target_agent_cfg_h[target_idx].targetAddress);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_len.set(num_words*2);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_dir.set(1'b0);      // WRITE
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_type.set(CMD_TYPE_SDR);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_ccc.set(8'd0);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_mode.set(1'b1);     // HDR-DDR data phase
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.start.set(1'b1);

    ctrl_val = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get();
    `uvm_info("CTRL_DEBUG",
      $sformatf("CTRL before HDR-DDR WRITE Command Word = 0x%0h", ctrl_val), UVM_LOW)

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.update(status, .parent(this));
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.mirror(status, UVM_NO_CHECK);

    #10000;

    i3c_env_cfg_h.i3c_target_agent_cfg_h[target_idx].pending_hdr_ddr = 0;

    `uvm_info(get_type_name(),
      $sformatf("HDR-DDR WRITE to target[%0d] complete", target_idx), UVM_LOW)

  endtask : body

endclass : i3c_hdr_ddr_write_virtual_seq
`endif

