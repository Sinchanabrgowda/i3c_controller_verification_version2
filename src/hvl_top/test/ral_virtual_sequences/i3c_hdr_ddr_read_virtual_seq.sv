`ifndef I3C_HDR_DDR_READ_VIRTUAL_SEQ_INCLUDED_
`define I3C_HDR_DDR_READ_VIRTUAL_SEQ_INCLUDED_

// ============================================================================
// FILE: i3c_hdr_ddr_read_virtual_seq.sv
//
// HDR-DDR (Optional Feature F001, MIPI I3C Basic Spec v1.2, Section 6.2)
// READ virtual sequence -- NEW, additive only file. Mirrors
// i3c_hdr_ddr_write_virtual_seq.sv (see that file for the register-trigger
// semantics note - the same best-effort cmd_mode=1 / cmd_type=2'd1
// assumptions apply here).
// ============================================================================
class i3c_hdr_ddr_read_virtual_seq extends top_virtual_base_seq;
  `uvm_object_utils(i3c_hdr_ddr_read_virtual_seq)

  uvm_status_e   status;
  uvm_reg_data_t ctrl_val;
  uvm_reg_data_t rdata;

  int unsigned target_idx = 0;
  int unsigned num_words  = 1;   // number of 16-bit HDR-DDR Data Words

  function new(string name = "i3c_hdr_ddr_read_virtual_seq");
    super.new(name);
  endfunction

  task body();
    i3c_target_hdr_ddr_seq target_seq_hdr_ddr;

    if (i3c_env_cfg_h == null)
      `uvm_fatal("CFG_NULL", "i3c_env_cfg_h is NULL inside i3c_hdr_ddr_read_virtual_seq")
    if (i3c_env_cfg_h.regBlockHandle == null)
      `uvm_fatal("RAL_NULL", "regBlockHandle is NULL inside i3c_hdr_ddr_read_virtual_seq")

    super.body();

    `uvm_info(get_type_name(),
      $sformatf("Starting HDR-DDR READ from target[%0d] addr=0x%0h, %0d word(s)",
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
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_type.set(2'd1);   // see write-seq note
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

    // --- Step 2: the HDR-DDR Command Word (Read) ---
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_addr.set(
      i3c_env_cfg_h.i3c_target_agent_cfg_h[target_idx].targetAddress);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_len.set(num_words*2);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_dir.set(1'b1);      // READ
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_type.set(CMD_TYPE_SDR);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_ccc.set(8'd0);
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_mode.set(1'b1);     // HDR-DDR data phase
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.start.set(1'b1);

    ctrl_val = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get();
    `uvm_info("CTRL_DEBUG",
      $sformatf("CTRL before HDR-DDR READ Command Word = 0x%0h", ctrl_val), UVM_LOW)

    i3c_env_cfg_h.regBlockHandle.ctrl_inst.update(status, .parent(this));
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.mirror(status, UVM_NO_CHECK);

    #5000;

    for (int i = 0; i < num_words*2; i++) begin
      i3c_env_cfg_h.regBlockHandle.rdatab_inst.read(status, rdata);
      `uvm_info("RDATAB_DEBUG",
        $sformatf("HDR-DDR RDATAB[%0d] = 0x%0h", i, rdata), UVM_LOW)
    end

    #5000;

    i3c_env_cfg_h.i3c_target_agent_cfg_h[target_idx].pending_hdr_ddr = 0;

    `uvm_info(get_type_name(),
      $sformatf("HDR-DDR READ from target[%0d] complete", target_idx), UVM_LOW)

  endtask : body

endclass : i3c_hdr_ddr_read_virtual_seq
`endif

