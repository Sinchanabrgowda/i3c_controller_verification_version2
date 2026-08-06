`ifndef I3C_IBI_VIRTUAL_SEQ_INCLUDED_
`define I3C_IBI_VIRTUAL_SEQ_INCLUDED_
class i3c_ibi_virtual_seq extends top_virtual_base_seq;
  `uvm_object_utils(i3c_ibi_virtual_seq)
  uvm_status_e   status;
  uvm_reg_data_t ctrl_val;
  uvm_reg_data_t ctrl_mirror;


  int unsigned timeout_per_slave_ns = 50_000;


  int unsigned ibi_target_idx = 0;
  

  bit ibi_sweep_all_targets = 0;
  // Mandatory Data Byte the target sends with its IBI request.
  bit [7:0] ibi_mdb_payload = 8'h17;
  


  int unsigned ibi_num_extra_bytes = 0;
  bit [7:0]    ibi_extra_data[];
  function new(string name = "i3c_ibi_virtual_seq");
    super.new(name);
  endfunction
  task body();
    int num_targets;
    bit daa_seq_done[];
    super.body();
    if (i3c_env_cfg_h == null)
      `uvm_fatal("CFG_NULL", "i3c_env_cfg_h is NULL inside i3c_ibi_virtual_seq")
    if (i3c_env_cfg_h.regBlockHandle == null)
      `uvm_fatal("RAL_NULL", "regBlockHandle is NULL inside i3c_ibi_virtual_seq")
    num_targets = i3c_env_cfg_h.no_of_targets;
    if (!ibi_sweep_all_targets && ibi_target_idx >= num_targets)
      `uvm_fatal("CFG_ERR",
        $sformatf("ibi_target_idx=%0d out of range for %0d targets",
                  ibi_target_idx, num_targets))
    daa_seq_done = new[num_targets];
    foreach (daa_seq_done[i]) daa_seq_done[i] = 0;
    `uvm_info(get_type_name(),
      $sformatf("Starting DAA for all %0d targets before IBI", num_targets),
      UVM_LOW)
    for (int i = 0; i < num_targets; i++) begin
      automatic int idx = i;
      fork
        begin
          i3c_target_daa_seq tgt_daa_seq;
          tgt_daa_seq = i3c_target_daa_seq::type_id::create(
                          $sformatf("tgt_daa_seq_%0d", idx));
          tgt_daa_seq.cfg_pid = i3c_env_cfg_h.i3c_target_agent_cfg_h[idx].pid;
          tgt_daa_seq.cfg_bcr = i3c_env_cfg_h.i3c_target_agent_cfg_h[idx].bcr;
          tgt_daa_seq.cfg_dcr = i3c_env_cfg_h.i3c_target_agent_cfg_h[idx].dcr;
          `uvm_info(get_type_name(),
            $sformatf("Launching DAA seq for target[%0d]", idx), UVM_LOW)
          tgt_daa_seq.start(p_sequencer.i3c_target_seqr_h[idx]);
          daa_seq_done[idx] = 1;
          `uvm_info(get_type_name(),
            $sformatf("DAA seq DONE for target[%0d]", idx), UVM_LOW)
        end
      join_none
    end
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_type.set(2'd3);  // CMD_TYPE_DAA
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.cmd_ccc.set(8'h07);  // ENTDAA
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.start.set(1'b1);
    ctrl_val = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get();
    `uvm_info("CTRL_DEBUG",
      $sformatf("CTRL value before IBI-precursor DAA write = 0x%0h", ctrl_val),
      UVM_LOW)
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.update(status, .parent(this));
    ctrl_mirror = i3c_env_cfg_h.regBlockHandle.ctrl_inst.get_mirrored_value();
    `uvm_info("CTRL_DEBUG",
      $sformatf("CTRL mirrored value after IBI DAA write = 0x%0h",
                ctrl_mirror), UVM_LOW)
    i3c_env_cfg_h.regBlockHandle.ctrl_inst.mirror(status, UVM_NO_CHECK);
    begin
      bit all_daa_done;
      int daa_timeout_ns = timeout_per_slave_ns * num_targets * 4;
      `uvm_info(get_type_name(),
        $sformatf("Waiting for DAA round (%0d targets) to complete, timeout=%0dns",
                  num_targets, daa_timeout_ns), UVM_LOW)
      fork
        begin : wait_for_daa_done
          forever begin
            all_daa_done = 1;
            foreach (daa_seq_done[i])
              if (!daa_seq_done[i]) all_daa_done = 0;
            if (all_daa_done) break;
            #100ns;
          end
        end
        begin : daa_timeout_guard
          #(daa_timeout_ns * 1ns);
        end
      join_any
      disable fork;
      if (!all_daa_done)
        `uvm_error(get_type_name(),
          $sformatf("DAA round did not complete for all %0d targets within %0dns -- proceeding to IBI anyway, results may be invalid",
                    num_targets, daa_timeout_ns))
    end
    `uvm_info(get_type_name(), "DAA round completed. Addresses:", UVM_LOW)
    for (int i = 0; i < num_targets; i++) begin
      `uvm_info(get_type_name(),
        $sformatf("  target[%0d]: dynamic addr = 0x%0h",
                  i, i3c_env_cfg_h.i3c_target_agent_cfg_h[i].targetAddress),
        UVM_LOW)
    end
//IBI FLOW
    // Target sweep


    begin
      int sweep_start_idx;
      int sweep_end_idx;
      if (ibi_sweep_all_targets) begin
        sweep_start_idx = 0;
        sweep_end_idx   = num_targets - 1;
        `uvm_info(get_type_name(),
          $sformatf("IBI target sweep enabled -- will trigger IBI for all %0d targets, one at a time",
                    num_targets), UVM_LOW)
      end else begin
        sweep_start_idx = ibi_target_idx;
        sweep_end_idx   = ibi_target_idx;
      end
      for (int cur_idx = sweep_start_idx; cur_idx <= sweep_end_idx; cur_idx++) begin
        `uvm_info(get_type_name(),
          $sformatf("Triggering IBI for target[%0d] (dynamic addr=0x%0h) mdb=0x%0h",
                    cur_idx,
                    i3c_env_cfg_h.i3c_target_agent_cfg_h[cur_idx].targetAddress,
                    ibi_mdb_payload),
          UVM_LOW)
        i3c_env_cfg_h.i3c_target_agent_cfg_h[cur_idx].ibi_mdb     = ibi_mdb_payload;
        i3c_env_cfg_h.i3c_target_agent_cfg_h[cur_idx].pending_ibi = 1;
        // T-bit / N extra data bytes 

        i3c_env_cfg_h.i3c_target_agent_cfg_h[cur_idx].ibi_num_extra_bytes = ibi_num_extra_bytes;
        i3c_env_cfg_h.i3c_target_agent_cfg_h[cur_idx].ibi_extra_data      = ibi_extra_data;
        begin
          i3c_target_ibi_seq tgt_ibi_seq;
          bit ibi_done;
          int ibi_timeout_ns = timeout_per_slave_ns * (6 + ibi_num_extra_bytes * 4);
          tgt_ibi_seq = i3c_target_ibi_seq::type_id::create(
                          $sformatf("tgt_ibi_seq_%0d", cur_idx));
          tgt_ibi_seq.cfg_mdb             = ibi_mdb_payload;
          tgt_ibi_seq.cfg_num_extra_bytes = ibi_num_extra_bytes;
          tgt_ibi_seq.cfg_extra_data      = ibi_extra_data;
          ibi_done = 0;
          fork
            begin : run_ibi
              `uvm_info(get_type_name(),
                $sformatf("Launching IBI seq for target[%0d]", cur_idx),
                UVM_LOW)
              tgt_ibi_seq.start(p_sequencer.i3c_target_seqr_h[cur_idx]);
              ibi_done = 1;
              `uvm_info(get_type_name(),
                $sformatf("IBI seq DONE for target[%0d]", cur_idx), UVM_LOW)
            end
            begin : ibi_timeout_guard
              #(ibi_timeout_ns * 1ns);
            end
          join_any
          disable fork;
          if (!ibi_done)
            `uvm_error(get_type_name(),
              $sformatf("IBI did not complete for target[%0d] within %0dns",
                        cur_idx, ibi_timeout_ns))
        end
        i3c_env_cfg_h.i3c_target_agent_cfg_h[cur_idx].pending_ibi = 0;
        `uvm_info(get_type_name(),
          $sformatf("IBI complete for target[%0d]", cur_idx), UVM_LOW)
      end
    end
  endtask : body
endclass : i3c_ibi_virtual_seq
`endif
