`ifndef I3C_TARGET_MONITOR_BFM_INCLUDED_
`define I3C_TARGET_MONITOR_BFM_INCLUDED_

import i3c_globals_pkg::*;

interface i3c_target_monitor_bfm (
  input pclk,
  input areset,
  input scl_i,
  input scl_o,
  input scl_oen,
  input sda_i,
  input sda_o,
  input sda_oen
);

  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import i3c_target_pkg::*;
  import i3c_target_pkg::i3c_target_monitor_proxy;

  i3c_target_monitor_proxy i3c_target_mon_proxy_h;
  i3c_fsm_state_e          state;

  string name = "I3C_TARGET_MONITOR_BFM";

  localparam logic [7:0] BCAST_7E_W  = 8'hFC;
  localparam logic [7:0] ENTDAA_CODE = 8'h07;
  localparam logic [7:0] BCAST_7E_R  = 8'hFD;
  localparam int         ARB_BIT_CNT = 64;

  initial begin
    $display(name);
  end


  task wait_for_reset();
    @(negedge areset);
    @(posedge areset);
  endtask : wait_for_reset

  task sample_idle_state();
    @(posedge pclk);
  endtask : sample_idle_state

  task wait_for_idle_state();
    @(posedge pclk);
    while (scl_i != 1 && sda_i != 1)
      @(posedge pclk);
    state = IDLE;
  endtask : wait_for_idle_state

  task sample_data(inout i3c_transfer_bits_s struct_packet,
                   inout i3c_transfer_cfg_s  struct_cfg);
    detect_start();
    sample_target_address(struct_packet);
    sample_operation(struct_packet.operation);
    sampleAddressAck(struct_packet.targetAddressStatus);
    if (struct_packet.targetAddressStatus == ACK) begin
      if (struct_packet.operation == WRITE)
        sampleWriteDataAndAck(struct_packet, struct_cfg);
      else
        sampleReadDataAndAck(struct_packet, struct_cfg);
    end else begin
      detect_stop();
    end
  endtask : sample_data

  task sampleWriteDataAndAck(inout i3c_transfer_bits_s pkt,
                              inout i3c_transfer_cfg_s  cfg);
    fork
      begin
        for (int i = 0; i < MAXIMUM_BYTES; i++) begin
          sample_write_data(cfg, pkt, i);
          sampleWdataAck(pkt.writeDataStatus[i]);
          if (pkt.writeDataStatus[i] == NACK) break;
        end
      end
    join_none
    detect_stop();
    disable fork;
  endtask : sampleWriteDataAndAck

  task sampleReadDataAndAck(inout i3c_transfer_bits_s pkt,
                             inout i3c_transfer_cfg_s  cfg);
    fork
      begin
        for (int i = 0; i < MAXIMUM_BYTES; i++) begin
          sample_read_data(pkt, i, cfg.dataTransferDirection);
          sampleReadAck(pkt.readDataStatus[i]);
          if (pkt.readDataStatus[i] == NACK) break;
        end
      end
    join_none
    detect_stop();
    disable fork;
  endtask : sampleReadDataAndAck

  // -------------------------------------------------------------------------
  // DAA monitoring
  //
  // KEY DESIGN DECISION:
  //   Open-drain arbitration means the bus wire-ANDs all drivers. Every
  //   monitor instance therefore sees ONLY the winner's PID/BCR/DCR bits
  //   in each round, not its own target's bits. Reading arb bits off the
  //   bus and reporting them as "this target's PID" is always wrong for
  //   losers — they all see round-1 winner's identity, as confirmed by
  //   the debug log showing pid=0xaabbcc0001 for all 4 targets.
  //
  //   The driver proxy already correctly updates
  //   i3c_target_agent_cfg_h.targetAddress with the assigned dynamic
  //   address (dyn_addr_out) after DAA. The cfg struct passed into this
  //   task via sample_daa_data() already contains the correct per-target
  //   pid/bcr/dcr from agent config.
  //
  //   Correct approach:
  //     1. Loop through ALL DAA rounds (START → preamble → Rep-START →
  //        arb → address → ACK → Rep-START/STOP repeat).
  //     2. In each round, read the 64 arb bits off the bus to find the
  //        winning PID/BCR/DCR for THAT round.
  //     3. Compare the winning PID against THIS target's cfg.pid.
  //        If they match, THIS target won this round: record dynamic_address
  //        and daa_ack from the bus, and return.
  //        If they don't match, this target lost this round: continue to
  //        the next Rep-START.
  //     4. If the bus sends STOP without this target winning, it was never
  //        assigned: return with daa_ack=NACK and dynamic_address=0.
  //
  //   This guarantees each monitor reports the correct per-target result
  //   regardless of which round its target won arbitration.
  // -------------------------------------------------------------------------
  task sample_daa_data(inout i3c_transfer_bits_s pkt,
                       inout i3c_transfer_cfg_s  cfg);
    bit [63:0] arb_shift;
    bit [7:0]  dyn_byte;
    bit [47:0] round_pid;
    bit [7:0]  round_bcr;
    bit [7:0]  round_dcr;
    bit [1:0]  scl_loc;
    bit [1:0]  sda_loc;
    bit        got_rep_start;
    int        round;

    // Default: not assigned
    pkt.pid             = cfg.pid;
    pkt.bcr             = cfg.bcr;
    pkt.dcr             = cfg.dcr;
    pkt.daa_ack         = NACK;
    pkt.dynamic_address = 7'h00;

    round = 0;

    // ------------------------------------------------------------------
    // Steps 1-3: START + 7E+W + ENTDAA (common preamble, once only)
    // ------------------------------------------------------------------
    detect_start();
    `uvm_info(name, "DAA MON: START detected", UVM_HIGH)

    // Consume 7E+W byte (8 POSEDGEs) + ACK slot
    for (int k = 7; k >= 0; k--)
      detectEdge_scl(POSEDGE);
    detectEdge_scl(NEGEDGE);
    detectEdge_scl(POSEDGE);   // ACK
    detectEdge_scl(NEGEDGE);

    // Consume ENTDAA byte (8 POSEDGEs) + ACK slot
    for (int k = 7; k >= 0; k--)
      detectEdge_scl(POSEDGE);
    detectEdge_scl(NEGEDGE);
    detectEdge_scl(POSEDGE);   // ACK
    detectEdge_scl(NEGEDGE);

    // ------------------------------------------------------------------
    // Steps 4-11: Loop per round until STOP
    // ------------------------------------------------------------------
    // Detect first Rep-START (SDA falls while SCL=1)
    detect_rep_start(scl_loc, sda_loc);
    `uvm_info(name, "DAA MON: initial Rep-START detected", UVM_HIGH)

    forever begin
      round++;
      `uvm_info(name, $sformatf("DAA MON: starting round %0d", round), UVM_HIGH)

      // Step 5: consume 7E+R byte (need NEGEDGE first after Rep-START)
      detectEdge_scl(NEGEDGE);
      for (int k = 7; k >= 0; k--)
        detectEdge_scl(POSEDGE);
      detectEdge_scl(NEGEDGE);
      detectEdge_scl(POSEDGE);   // ACK
      detectEdge_scl(NEGEDGE);

      // Steps 6-7: sample 64 arb bits (bus wire-AND = winner's bits)
      arb_shift = '0;
      for (int k = 63; k >= 0; k--) begin
        detectEdge_scl(POSEDGE);
        arb_shift[k] = sda_i;
        detectEdge_scl(NEGEDGE);
      end

      round_pid = arb_shift[63:16];
      round_bcr = arb_shift[15:8];
      round_dcr = arb_shift[7:0];

      `uvm_info(name,
        $sformatf("DAA MON [round %0d] bus winner PID=0x%0h BCR=0x%0h DCR=0x%0h | my PID=0x%0h",
                  round, round_pid, round_bcr, round_dcr, cfg.pid),
        UVM_NONE)

      // Steps 9: sample dynamic address byte from master (8 bits)
      for (int k = 7; k >= 0; k--) begin
        detectEdge_scl(POSEDGE);
        dyn_byte[k] = sda_i;
      end

      // Step 10: sample winner's ACK (slave drives SDA low = ACK=0)
      detectEdge_scl(NEGEDGE);
      detectEdge_scl(POSEDGE);
      // daa_ack on bus: 0=winner ACKed, 1=NACK (no winner or rejected)
      // We read it but only store it if this round's winner is our target

      // Step 11: detect Rep-START or STOP
      detect_rep_start_or_stop(scl_loc, sda_loc, got_rep_start);
      detectEdge_scl(NEGEDGE);  // consume trailing NEGEDGE

      // Does this round's winner match THIS target?
      if (round_pid == cfg.pid &&
          round_bcr == cfg.bcr &&
          round_dcr == cfg.dcr) begin
        // This target won this round
        pkt.pid             = cfg.pid;
        pkt.bcr             = cfg.bcr;
        pkt.dcr             = cfg.dcr;
        pkt.dynamic_address = dyn_byte[7:1];
        pkt.daa_ack         = ACK;
        `uvm_info(name,
          $sformatf("DAA MON [round %0d] THIS TARGET WON: dyn_addr=0x%0h",
                    round, pkt.dynamic_address),
          UVM_NONE)
        return;
      end

      if (!got_rep_start) begin
        // STOP: DAA complete, this target was never assigned
        `uvm_info(name,
          $sformatf("DAA MON: STOP detected after round %0d — target not assigned (pid=0x%0h)",
                    round, cfg.pid),
          UVM_NONE)
        // pkt already defaulted to NACK/0 at top
        return;
      end

      `uvm_info(name,
        $sformatf("DAA MON [round %0d] this target did not win, continuing to round %0d",
                  round, round+1),
        UVM_HIGH)
      // Rep-START: loop to next round
    end

  endtask : sample_daa_data


  // -------------------------------------------------------------------------
  // Detect Repeated-START: SDA falls while SCL=1
  // -------------------------------------------------------------------------
  task automatic detect_rep_start(ref bit [1:0] scl_loc, ref bit [1:0] sda_loc);
    scl_loc = {scl_i, scl_i};
    sda_loc = {sda_i, sda_i};
    do begin
      @(negedge pclk);
      scl_loc = {scl_loc[0], scl_i};
      sda_loc = {sda_loc[0], sda_i};
    end while (!(sda_loc == 2'b10 && scl_loc == 2'b11));
    `uvm_info(name, "DAA MON: Rep-START detected", UVM_HIGH)
  endtask : detect_rep_start


  // -------------------------------------------------------------------------
  // Detect Rep-START (SDA falls, SCL=1) or STOP (SDA rises, SCL=1)
  // -------------------------------------------------------------------------
  task automatic  detect_rep_start_or_stop(
      ref  bit [1:0] scl_loc,
      ref  bit [1:0] sda_loc,
      output bit     got_rep_start);

    scl_loc = {scl_i, scl_i};
    sda_loc = {sda_i, sda_i};

    forever begin
      @(negedge pclk);
      scl_loc = {scl_loc[0], scl_i};
      sda_loc = {sda_loc[0], sda_i};

      // SDA falls while SCL=1 → Rep-START
      if (scl_loc == 2'b11 && sda_loc == 2'b10) begin
        `uvm_info(name, "DAA MON: Rep-START detected", UVM_HIGH)
        got_rep_start = 1;
        return;
      end

      // SDA rises while SCL=1 → STOP
      if (scl_loc == 2'b11 && sda_loc == 2'b01) begin
        `uvm_info(name, "DAA MON: STOP detected", UVM_HIGH)
        got_rep_start = 0;
        return;
      end
    end
  endtask : detect_rep_start_or_stop


  // -------------------------------------------------------------------------
  // Helpers
  // -------------------------------------------------------------------------
  task detect_start();
    bit [1:0] scl_d;
    bit [1:0] sda_d;
    state = START;
    do begin
      @(negedge pclk);
      scl_d = {scl_d[0], scl_i};
      sda_d = {sda_d[0], sda_i};
    end while (!(sda_d == NEGEDGE && scl_d == 2'b11));
  endtask : detect_start

  task detect_stop();
    bit [1:0] scl_d;
    bit [1:0] sda_d;
    state = STOP;
    do begin
      @(negedge pclk);
      #1;
      scl_d = {scl_d[0], scl_i};
      sda_d = {sda_d[0], sda_i};
    end while (!(sda_d == POSEDGE && scl_d == 2'b11));
  endtask : detect_stop

  task sample_target_address(inout i3c_transfer_bits_s pkt);
    bit [TARGET_ADDRESS_WIDTH-1:0] addr;
    state = ADDRESS;
    detectEdge_scl(NEGEDGE);
    for (int k = TARGET_ADDRESS_WIDTH-1; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      addr[k] = sda_i;
    end
    pkt.targetAddress = addr;
  endtask : sample_target_address

  task sample_operation(output operationType_e op);
    bit b;
    state = WR_BIT;
    detectEdge_scl(POSEDGE);
    b  = sda_i;
    op = (b == 1'b0) ? WRITE : READ;
  endtask : sample_operation

  task sampleAddressAck(output bit ack);
    state = ACK_NACK;
    detectEdge_scl(NEGEDGE);
    detectEdge_scl(POSEDGE);
    ack = sda_i;
    detectEdge_scl(NEGEDGE);
  endtask : sampleAddressAck

  task sample_write_data(
      input  i3c_transfer_cfg_s cfg,
      inout  i3c_transfer_bits_s pkt,
      input  int i);
    bit [DATA_WIDTH-1:0] wdata;
    state = WRITE_DATA;
    for (int k = 0, bit_no = 0; k < DATA_WIDTH; k++) begin
      bit_no = (cfg.dataTransferDirection == MSB_FIRST) ?
               ((DATA_WIDTH-1) - k) : k;
      detectEdge_scl(POSEDGE);
      wdata[bit_no] = sda_i;
      pkt.no_of_i3c_bits_transfer++;
    end
    pkt.writeData[i] = wdata;
  endtask : sample_write_data

  task sampleWdataAck(output bit ack);
    state = ACK_NACK;
    detectEdge_scl(NEGEDGE);
    detectEdge_scl(POSEDGE);
    ack = sda_i;
    detectEdge_scl(NEGEDGE);
  endtask : sampleWdataAck

  task sample_read_data(
      inout  i3c_transfer_bits_s pkt,
      input  int i,
      input  dataTransferDirection_e dir);
    bit [DATA_WIDTH-1:0] rdata;
    state = READ_DATA;
    for (int k = 0, bit_no = 0; k < DATA_WIDTH; k++) begin
      bit_no = (dir == MSB_FIRST) ? ((DATA_WIDTH-1) - k) : k;
      detectEdge_scl(POSEDGE);
      rdata[bit_no] = sda_i;
      pkt.no_of_i3c_bits_transfer++;
    end
    pkt.readData[i] = rdata;
  endtask : sample_read_data

  task sampleReadAck(output bit ack);
    state = ACK_NACK;
    detectEdge_scl(POSEDGE);
    ack = sda_i;
    detectEdge_scl(NEGEDGE);
  endtask : sampleReadAck

  task automatic detectEdge_scl(input edge_detect_e edgeSCL);
    bit [1:0] scl_loc_m = 2'b11;
    do begin
      @(negedge pclk);
      scl_loc_m = {scl_loc_m[0], scl_i};
    end while (!(scl_loc_m == edgeSCL));
  endtask : detectEdge_scl

endinterface : i3c_target_monitor_bfm

`endif
