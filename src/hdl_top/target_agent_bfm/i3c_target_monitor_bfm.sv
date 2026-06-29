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

  // DAA monitoring  (passive – captures one ARB round per call)

  // Module-level counter: incremented after each completed round
  int unsigned mon_round = 0;

  // Lightweight REP_START / STOP detector used inside sample_daa_data
  task automatic detect_rep_start_or_stop_mon(output bit got_rep_start);
    bit [1:0] scl_loc;
    bit [1:0] sda_loc;
    scl_loc = {scl_i, scl_i};
    sda_loc = {sda_i, sda_i};
    forever begin
      @(negedge pclk);
      scl_loc = {scl_loc[0], scl_i};
      sda_loc = {sda_loc[0], sda_i};
      // REP_START: SDA falls while SCL is high
      if (scl_loc == 2'b11 && sda_loc == 2'b10) begin
        got_rep_start = 1; return;
      end
      // STOP: SDA rises while SCL is high
      if (scl_loc == 2'b11 && sda_loc == 2'b01) begin
        got_rep_start = 0; return;
      end
    end
  endtask : detect_rep_start_or_stop_mon

  task sample_daa_data(inout i3c_transfer_bits_s pkt,
                       inout i3c_transfer_cfg_s  cfg);
    bit [63:0] arb_shift;
    bit [7:0]  dyn_byte;
    bit [1:0]  scl_loc;
    bit [1:0]  sda_loc;
    bit        got_rep_start;

    // ------------------------------------------------------------------
    // Round 0 only: START + 7E+W + ACK + ENTDAA + ACK + REP_START + 7E+R + ACK
    // ------------------------------------------------------------------
    if (mon_round == 0) begin

      detect_start();

      // 7E+W (8 bits) + ACK
      for (int k = 7; k >= 0; k--)
        detectEdge_scl(POSEDGE);
      detectEdge_scl(NEGEDGE);
      detectEdge_scl(POSEDGE);   // ACK
      detectEdge_scl(NEGEDGE);

      // ENTDAA CCC byte (8 bits) + ACK
      for (int k = 7; k >= 0; k--)
        detectEdge_scl(POSEDGE);
      detectEdge_scl(NEGEDGE);
      detectEdge_scl(POSEDGE);   // ACK
      detectEdge_scl(NEGEDGE);

      // First REP_START
      do begin
        @(negedge pclk);
        scl_loc = {scl_loc[0], scl_i};
        sda_loc = {sda_loc[0], sda_i};
      end while (!(sda_loc == 2'b10 && scl_loc == 2'b11));

      // 7E+R header (8 bits) + ACK
      detectEdge_scl(NEGEDGE);
      for (int k = 7; k >= 0; k--)
        detectEdge_scl(POSEDGE);
      detectEdge_scl(NEGEDGE);
      detectEdge_scl(POSEDGE);   // ACK
      detectEdge_scl(NEGEDGE);

    end
    else begin

      // 7E+R header (8 bits) + ACK
      detectEdge_scl(NEGEDGE);
      for (int k = 7; k >= 0; k--)
        detectEdge_scl(POSEDGE);
      detectEdge_scl(NEGEDGE);
      detectEdge_scl(POSEDGE);   // ACK
      detectEdge_scl(NEGEDGE);

    end

    // ARB phase: sample 64 bits (winner's PID|BCR|DCR on the wire)
    for (int k = 63; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      arb_shift[k] = sda_i;
      detectEdge_scl(NEGEDGE);
    end

    pkt.pid = arb_shift[63:16];
    pkt.bcr = arb_shift[15:8];
    pkt.dcr = arb_shift[7:0];

    // Dynamic address byte (8 bits) driven by master
    for (int k = 7; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      dyn_byte[k] = sda_i;
    end
    pkt.dynamic_address = dyn_byte[7:1];

    // ACK from winning slave
    detectEdge_scl(NEGEDGE);
    detectEdge_scl(POSEDGE);
    pkt.daa_ack = sda_i;
    detectEdge_scl(NEGEDGE);

  
    detect_rep_start_or_stop_mon(got_rep_start);

    mon_round++;

  endtask : sample_daa_data

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

  task automatic detectEdge_scl(input edge_detect_e edgeSCL);                         //made as automatic in driver bfm too
    bit [1:0] scl_loc_m = 2'b11;
    do begin
      @(negedge pclk);
      scl_loc_m = {scl_loc_m[0], scl_i};
    end while (!(scl_loc_m == edgeSCL));
  endtask : detectEdge_scl

endinterface : i3c_target_monitor_bfm

`endif
