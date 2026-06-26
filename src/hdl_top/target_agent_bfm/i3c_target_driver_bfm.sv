`ifndef I3C_TARGET_DRIVER_BFM_INCLUDED_
`define I3C_TARGET_DRIVER_BFM_INCLUDED_

import i3c_globals_pkg::*;

interface i3c_target_driver_bfm (
  input  pclk,
  input  areset,
  input  scl_i,
  output reg scl_o,
  output reg scl_oen,
  input  sda_i,
  output reg sda_o,
  output reg sda_oen
);

  i3c_fsm_state_e state;
  bit [7:0]  rdata;
  bit [1:0]  scl_local = 2'b11;
  static int count=0;
  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import i3c_target_pkg::i3c_target_driver_proxy;

  i3c_target_driver_proxy i3c_target_drv_proxy_h;

  bit [DATA_WIDTH-1:0] targetFIFOMemory[$];

  string name = "I3C_TARGET_DRIVER_BFM";

  initial begin
    $display(name);
  end

  // =========================================================================
  // Reset / Idle helpers
  // =========================================================================
  task wait_for_system_reset();
    state = RESET_DEACTIVATED;
    @(negedge areset);
    state = RESET_ACTIVATED;
    @(posedge areset);
    state = RESET_DEACTIVATED;
  endtask : wait_for_system_reset

  task drive_idle_state();
    @(posedge pclk);
    drive_scl(1);
    drive_sda(1);
    state <= IDLE;
    `uvm_info(name, "inside idle state", UVM_HIGH)
  endtask : drive_idle_state

  task wait_for_idle_state();
    @(posedge pclk);
    while (scl_i != 1 && sda_i != 1) begin
      @(posedge pclk);
    end
    state = IDLE;
    `uvm_info(name, "I3C bus is free state detected", UVM_NONE)
  endtask : wait_for_idle_state

  // =========================================================================
  // SDR (non-DAA) transaction
  // =========================================================================
  task drive_data(inout i3c_transfer_bits_s dataPacketStruck,
                  input i3c_transfer_cfg_s  configPacketStruck);
    `uvm_info(name, "target txn started", UVM_HIGH)
    detect_start();
    sample_target_address(configPacketStruck, dataPacketStruck);
    sample_operation(dataPacketStruck.operation);
    driveAddressAck(dataPacketStruck.targetAddressStatus);

    if (dataPacketStruck.targetAddressStatus == ACK) begin
      `uvm_info(name, "targetAddressStatus is ACK", UVM_HIGH)
      if (dataPacketStruck.operation == WRITE)
        sampleWriteDataAndDriveACK(dataPacketStruck, configPacketStruck);
      else
        driveReadDataAndSampleACK(dataPacketStruck, configPacketStruck);
    end else begin
      `uvm_info(name, "targetAddressStatus is NACK", UVM_HIGH)
      detect_stop();
    end
  endtask : drive_data

  task sampleWriteDataAndDriveACK(
      inout i3c_transfer_bits_s dataPacketStruck,
      input i3c_transfer_cfg_s  configPacketStruck);
    `uvm_info(name, "sampleWriteDataAndDriveACK started", UVM_HIGH)
    fork
      begin
        for (int i = 0; i < MAXIMUM_BYTES; i++) begin
          sample_write_data(configPacketStruck, dataPacketStruck, i);
          driveWdataAck(dataPacketStruck.writeDataStatus[i]);
          if (dataPacketStruck.writeDataStatus[i] == NACK)
            break;
        end
      end
    join_none
    `uvm_info(name, "sampleWriteDataAndDriveACK done", UVM_HIGH)
    wrDetect_stop();
    disable fork;
  endtask : sampleWriteDataAndDriveACK

  task driveReadDataAndSampleACK(
      inout i3c_transfer_bits_s dataPacketStruck,
      input i3c_transfer_cfg_s  configPacketStruck);
    `uvm_info(name, "driveReadDataAndSampleACK started", UVM_HIGH)
    fork
      begin
        for (int i = 0; i < MAXIMUM_BYTES; i++) begin
          if (targetFIFOMemory.size() == 0)
            rdata = configPacketStruck.defaultReadData;
          else
            rdata = targetFIFOMemory.pop_front();

          drive_read_data(rdata, dataPacketStruck, i,
                          configPacketStruck.dataTransferDirection);
          sample_ack(dataPacketStruck.readDataStatus[i]);
          if (dataPacketStruck.readDataStatus[i] == NACK)
            break;
        end
      end
    join_none
    wrDetect_stop();
    disable fork;
  endtask : driveReadDataAndSampleACK

  // =========================================================================
  // DAA TRANSACTION  (multi-slave arbitration)


  // Flag: once set this target has an address and ignores further DAA rounds
  bit has_address = 0;

  task drive_daa_data(
      inout i3c_transfer_bits_s dataPacketStruck,
      input i3c_transfer_cfg_s  configPacketStruck,
      output bit [47:0] pid_out,
      output bit [7:0]  bcr_out,
      output bit [7:0]  dcr_out,
      output bit [6:0]  dyn_addr_out,
      output bit        daa_ack_out);

    bit [63:0] my_id;
    bit        won_arb;
    bit [6:0]  assigned_addr;
    bit [7:0]  dyn_addr_byte;
    int        lost_bit;       // which arb bit we lost at (i value from arb loop)
    bit        got_rep_start;

    `uvm_info(name, "DAA transaction started", UVM_NONE)

    // Bail out early if already assigned
    if (has_address) begin
      `uvm_info("HAS_ADDR", "Already has dynamic address - ignoring DAA", UVM_NONE)
      daa_ack_out  = NACK;
      pid_out      = configPacketStruck.pid;
      bcr_out      = configPacketStruck.bcr;
      dcr_out      = configPacketStruck.dcr;
      dyn_addr_out = 7'h00;
      return;
    end

    // Build the 64-bit value to drive: PID[47:0] | BCR[7:0] | DCR[7:0]
    my_id = {configPacketStruck.pid,
             configPacketStruck.bcr,
             configPacketStruck.dcr};

    detect_start();
    `uvm_info(name, "DAA: START detected", UVM_NONE)

    sample_daa_broadcast_address(dataPacketStruck);
    sample_daa_ccc_byte(dataPacketStruck);
    detect_repeated_start();
    sample_daa_broadcast_read(dataPacketStruck);


    won_arb = 0;

    while (!won_arb) begin

      // ----------------------------------------------------------------
      // ARB PHASE: drive 64 bits, sample bus each SCL cycle
      // ----------------------------------------------------------------
      drive_daa_arb_bits(my_id, won_arb, lost_bit);

      if (!won_arb) begin
        // LOST arbitration this round — release SDA immediately.
        drive_sda(1);
        `uvm_info(name,
          "DAA: Lost arbitration - waiting for next Rep-START or STOP",
          UVM_NONE)

        skip_remaining_winner_frame(lost_bit);

        detect_rep_start_or_stop(got_rep_start);

        if (!got_rep_start) begin
          `uvm_info(name,
            "DAA: STOP detected after losing arb - DAA complete", UVM_NONE)
          daa_ack_out  = NACK;
          pid_out      = configPacketStruck.pid;
          bcr_out      = configPacketStruck.bcr;
          dcr_out      = configPacketStruck.dcr;
          dyn_addr_out = 7'h00;
          return;
        end

        `uvm_info(name,
          "DAA: Rep-START detected after losing arb - re-entering",
          UVM_NONE)
        // Rep-START: sample the 7E+R broadcast header then re-enter arb
        sample_daa_broadcast_read(dataPacketStruck);
      end
    end // while !won_arb

    // ------------------------------------------------------------------
    // Phase 3: Winner samples the dynamic address byte from master
    // ------------------------------------------------------------------
    sample_daa_dynamic_address(assigned_addr, dyn_addr_byte, daa_ack_out);

    driveAddressAck(daa_ack_out);

      if(i3c_target_driver_proxy::daa_count==3)   //added
      begin
      `uvm_info(name,"DAA count=3 entered", UVM_NONE)
      detect_repeated_start();
      sample_daa_broadcast_read(dataPacketStruck);  
      
      driveAddressAck(1);
      detect_stop();
      end
else
begin
    detect_rep_start_or_stop(got_rep_start);

    if (got_rep_start) begin
      `uvm_info(name,
        "DAA: Rep-START after address assignment - more targets remain",
        UVM_NONE)
    end else begin
      `uvm_info(name,
        "DAA: STOP after address assignment - all targets assigned",
        UVM_NONE)
    end
end
    if (daa_ack_out == ACK) begin
      has_address = 1;
      `uvm_info(name,
        $sformatf("DAA: slave assigned dynamic address 0x%0x", assigned_addr),
        UVM_NONE)
    end

    // Return results
    pid_out      = configPacketStruck.pid;
    bcr_out      = configPacketStruck.bcr;
    dcr_out      = configPacketStruck.dcr;
    dyn_addr_out = assigned_addr;

    dataPacketStruck.targetAddress       = 7'h7E;
    dataPacketStruck.targetAddressStatus = ACK;

  endtask : drive_daa_data

  // =========================================================================
  // ARB PHASE – drive 64-bit PID+BCR+DCR with open-drain arbitration

  task automatic drive_daa_arb_bits(
      input  bit [63:0] my_id,
      output bit        won_arb,
      output int        lost_bit_out);

    bit my_bit;
    bit bus_bit;
    bit[63:0] all_bits=0;
  bit first_bit=0;
bit[63:0] bus_val=0;
    won_arb      = 1;
    lost_bit_out = 0;
      first_bit= sda_i;
   `uvm_info("DRIVE_DAA_BITS",$sformatf("first bus bit =%b",first_bit),UVM_LOW)
    for (int i = 63; i >= 0; i--) begin
      my_bit = my_id[i];
      all_bits[i]=my_id[i];
     //`uvm_info("DRIVE_DAA_BITS",$sformatf("sent bit =%b  before detect negedge",my_id[i]),UVM_LOW)
      // Step 1: wait for falling SCL → safe window
      detectEdge_scl(NEGEDGE);


      // Step 2: drive our bit
      drive_sda(my_bit);

      // Step 3: wait for rising SCL → master samples
      detectEdge_scl(POSEDGE);

      // Step 4: sample the bus
      bus_bit = sda_i;
      bus_val[i]=sda_i;
      `uvm_info("DRIVE_DAA_BITS",$sformatf("curretn bus bit=%b",bus_val[i]),UVM_LOW)

      if (i > 0) begin
        if (my_bit == 1'b1 && bus_bit == 1'b0) begin
          `uvm_info(name,
            $sformatf("DAA ARB: Lost at bit %0d (drove 1, bus=0)", i),
            UVM_NONE)
          drive_sda(1);   // release bus immediately
          won_arb      = 0;
          lost_bit_out = i;
          return;
        end
      end

      // Consistent: either both-0 (dominant) or both-1 (recessive)
    end
    `uvm_info("DRIVE_DAA_BITS",$sformatf("all 64 bits =%h",all_bits),UVM_LOW)
    `uvm_info("DRIVE_DAA_BITS",$sformatf("all total bus bit=%h",bus_val),UVM_LOW)
    `uvm_info(name, "DAA ARB: WON all 64 bits", UVM_NONE)

  endtask : drive_daa_arb_bits
/*
task automatic drive_daa_arb_bits(
    input  bit [63:0] my_id,
    output bit        won_arb,
    output int        lost_bit_out);

  bit my_bit;
  bit bus_bit;
  bit [63:0] all_bits = 0;
  bit [63:0] bus_val  = 0;

  won_arb      = 1;
  lost_bit_out = 0;

  // ─────────────────────────────────────────────────────────────────
  // CRITICAL: resync scl_local to actual bus state on entry.
  // sample_daa_broadcast_read() exits with SCL already LOW.
  // Without this resync, detectEdge_scl(NEGEDGE) below will skip
  // the current low and wait for the *next* negedge, causing bit 63
  // to never be driven → SDA stays high (pull-up) for that bit.
  // ─────────────────────────────────────────────────────────────────
  scl_local = {scl_i, scl_i};   // seed both history slots from live bus

  for (int i = 63; i >= 0; i--) begin
    my_bit      = my_id[i];
    all_bits[i] = my_id[i];

    // Step 1: SCL low → safe window to change SDA
    detectEdge_scl(NEGEDGE);

    // Step 2: drive our bit
    drive_sda(my_bit);
    `uvm_info("DRIVE_DAA_BITS",
      $sformatf("sent bit =%b", my_bit), UVM_LOW)

    // Step 3: SCL rising → master samples
    detectEdge_scl(POSEDGE);

    // Step 4: sample the bus
    bus_bit    = sda_i;
    bus_val[i] = sda_i;
    `uvm_info("DRIVE_DAA_BITS",
      $sformatf("current bus bit=%b", bus_val[i]), UVM_LOW)

    // Step 5: arbitration check
    if (i > 0) begin
      if (my_bit == 1'b1 && bus_bit == 1'b0) begin
        `uvm_info(name,
          $sformatf("DAA ARB: Lost at bit %0d (drove 1, bus=0)", i),
          UVM_NONE)
        drive_sda(1);
        won_arb      = 0;
        lost_bit_out = i;
        return;
      end
    end
  end

  `uvm_info("DRIVE_DAA_BITS",
    $sformatf("all 64 bits =%h", all_bits), UVM_LOW)
  `uvm_info("DRIVE_DAA_BITS",
    $sformatf("all bus bits =%h", bus_val), UVM_LOW)
  `uvm_info(name, "DAA ARB: WON all 64 bits", UVM_NONE)

*/
 task automatic skip_remaining_winner_frame(input int lost_at_bit);
    `uvm_info(name,
      $sformatf("skip_remaining_winner_frame: lost_at_bit=%0d, stepping through winner's remaining frame",
                lost_at_bit),
      UVM_HIGH)

    // Remaining arbitration bits: i-1 downto 0 → lost_at_bit more bits
    for (int i = 0; i < lost_at_bit; i++) begin
      detectEdge_scl(NEGEDGE);
      detectEdge_scl(POSEDGE);
    end

    // 8-bit dynamic address byte
    for (int k = 0; k < 8; k++) begin
      detectEdge_scl(NEGEDGE);
      detectEdge_scl(POSEDGE);
    end

    // ACK bit: driveAddressAck() does
    //   detectEdge_scl(NEGEDGE); drive_sda(ack); detectEdge_scl(POSEDGE);
    //   detectEdge_scl(NEGEDGE); drive_sda(1);
    // i.e. two NEGEDGEs and one POSEDGE in between, ending on a NEGEDGE
    // with SDA released high. Mirror that exactly.
    detectEdge_scl(NEGEDGE);
    detectEdge_scl(POSEDGE);
    detectEdge_scl(NEGEDGE);

    `uvm_info(name,
      "skip_remaining_winner_frame: stepped past winner's full ACK release",
      UVM_HIGH)
  endtask : skip_remaining_winner_frame


  // =========================================================================
  // Phase helpers
  // =========================================================================

  task sample_daa_broadcast_address(inout i3c_transfer_bits_s pkt);
    bit [6:0] addr_bits;
    bit       rw_bit;
    bit [7:0] full_byte;

    `uvm_info(name, "DAA: sampling broadcast 0x7E+W", UVM_HIGH)

    for (int k = 6; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      addr_bits[k] = sda_i;
      drive_sda(1);
    end

    detectEdge_scl(POSEDGE);
    rw_bit    = sda_i;
    drive_sda(1);

    full_byte = {addr_bits, rw_bit};
    `uvm_info(name,
      $sformatf("DAA: broadcast addr = 0x%0x (expect 0xFC)", full_byte),
      UVM_NONE)

    // ACK the broadcast (drive 0 for one SCL cycle)
    detectEdge_scl(NEGEDGE);
    drive_sda(1'b0);
    detectEdge_scl(POSEDGE);
    detectEdge_scl(NEGEDGE);
    drive_sda(1'b1);

  endtask : sample_daa_broadcast_address

  task sample_daa_ccc_byte(inout i3c_transfer_bits_s pkt);
    bit [7:0] ccc_byte;
    `uvm_info(name, "DAA: sampling ENTDAA CCC byte", UVM_HIGH)

    for (int k = 7; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      ccc_byte[k] = sda_i;
      drive_sda(1);
    end
    `uvm_info(name,
      $sformatf("DAA: CCC byte = 0x%0x (expect 0x07)", ccc_byte), UVM_NONE)

    // ACK
    detectEdge_scl(NEGEDGE);
    drive_sda(1'b0);
    detectEdge_scl(POSEDGE);
    detectEdge_scl(NEGEDGE);
    drive_sda(1'b1);

  endtask : sample_daa_ccc_byte

  task detect_repeated_start();
    bit [1:0] scl_loc;
    bit [1:0] sda_loc;

    do begin
      @(negedge pclk);
      scl_loc = {scl_loc[0], scl_i};
      sda_loc = {sda_loc[0], sda_i};
      end while (!(sda_loc == NEGEDGE && scl_loc == 2'b11));
    `uvm_info(name, "DAA: Repeated START detected", UVM_HIGH)
    scl_local = {scl_i, scl_i};
  endtask : detect_repeated_start


  task automatic detect_rep_start_or_stop(output bit got_rep_start);
    bit [1:0] scl_loc;
    bit [1:0] sda_loc;

    // Seed from the live bus state, not a hardcoded assumption.
    scl_loc = {scl_i, scl_i};
    sda_loc = {sda_i, sda_i};

    `uvm_info(name,
      $sformatf("detect_rep_start_or_stop: ENTRY scl_i=%0b sda_i=%0b",
                scl_i, sda_i),
      UVM_NONE)

    forever begin
      @(negedge pclk);
      scl_loc = {scl_loc[0], scl_i};
      sda_loc = {sda_loc[0], sda_i};

      // SCL held high AND SDA falling  → Repeated-START
      if (scl_loc == 2'b11 && sda_loc == 2'b10) begin
        `uvm_info(name,
          $sformatf("detect_rep_start_or_stop: Repeated-START (scl_loc=%b sda_loc=%b)",
                    scl_loc, sda_loc),
          UVM_NONE)
        got_rep_start = 1;
        return;
      end

      // SCL held high AND SDA rising   → STOP
      if (scl_loc == 2'b11 && sda_loc == 2'b01) begin
        `uvm_info(name,
          $sformatf("detect_rep_start_or_stop: STOP (scl_loc=%b sda_loc=%b)",
                    scl_loc, sda_loc),
          UVM_NONE)
        got_rep_start = 0;
        return;
      end

      `uvm_info(name,
        $sformatf("detect_rep_start_or_stop: sample scl_loc=%b sda_loc=%b (scl_i=%0b sda_i=%0b)",
                  scl_loc, sda_loc, scl_i, sda_i),
        UVM_HIGH)
    end
  endtask : detect_rep_start_or_stop

  task sample_daa_broadcast_read(inout i3c_transfer_bits_s pkt);
    bit [6:0] addr_bits;
    bit       rw_bit;
    bit [7:0] full_byte;

    `uvm_info(name, "DAA: sampling broadcast 0x7E+R", UVM_HIGH)

    detectEdge_scl(NEGEDGE);

    for (int k = 6; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      addr_bits[k] = sda_i;
      drive_sda(1);
    end

    detectEdge_scl(POSEDGE);
    rw_bit    = sda_i;
    drive_sda(1);

    full_byte = {addr_bits, rw_bit};
    `uvm_info(name,
      $sformatf("DAA: broadcast read addr = 0x%0x (expect 0xFD)", full_byte),
      UVM_NONE)

    // ACK (open-drain: all remaining unassigned slaves pull low together)
    detectEdge_scl(NEGEDGE);
    drive_sda(1'b0);
    detectEdge_scl(POSEDGE);
    //detectEdge_scl(NEGEDGE);
    //drive_sda(1'b1);

  endtask : sample_daa_broadcast_read

  task sample_daa_dynamic_address(
      output bit [6:0] dyn_addr_out,
      output bit [7:0] full_byte_out,
      output bit       ack_out);

    bit [7:0] addr_byte;
    bit       parity_received;
    bit       parity_calc;
    i3c_target_driver_proxy::daa_count++;     //added                                                                      /////////////////////////////////

    `uvm_info(name, "DAA: sampling dynamic address", UVM_HIGH)

    for (int k = 7; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      addr_byte[k] = sda_i;
      drive_sda(1);
    end

    dyn_addr_out     = addr_byte[7:1];
    parity_received  = addr_byte[0];
    full_byte_out    = addr_byte;

    parity_calc = ~^addr_byte[7:1];
    `uvm_info("COUNT_SLAVE",$sformatf("Slave addr assignment done count=%d",i3c_target_driver_proxy::daa_count),UVM_NONE)
    if (parity_calc == parity_received) begin
      ack_out = ACK;
      `uvm_info(name,
        $sformatf("DAA: dynamic addr=%0h dyn_add+parity=%0h parity OK =%0b→ ACK",dyn_addr_out,addr_byte,parity_received),UVM_NONE)
        end
    else begin
      ack_out = NACK;
      `uvm_info(name,
        $sformatf("DAA: dynamic addr=0x%0x parity FAIL → NACK", dyn_addr_out),
        UVM_NONE)
    end

  endtask : sample_daa_dynamic_address

  // =========================================================================
  // SDR helpers (unchanged)
  // =========================================================================

  task detect_start();
    bit [1:0] scl_local_d;
    bit [1:0] sda_local_d;
    state = START;
    `uvm_info(name, "detect_start waiting", UVM_HIGH)
    do begin
      @(negedge pclk);
      scl_local_d = {scl_local_d[0], scl_i};
      sda_local_d = {sda_local_d[0], sda_i};
    end while (!(sda_local_d == NEGEDGE && scl_local_d == 2'b11));
    `uvm_info(name, "Start condition detected", UVM_HIGH)
    scl_local = {scl_i, scl_i};  // resync: uses local vars, never updates scl_local
  endtask : detect_start

  task sample_target_address(
      input  i3c_transfer_cfg_s cfg_pkt,
      inout  i3c_transfer_bits_s pkt);

    bit [TARGET_ADDRESS_WIDTH-1:0] local_addr;
    `uvm_info(name, "sample_target_address started", UVM_HIGH)
    state = ADDRESS;

    detectEdge_scl(NEGEDGE);

    for (int k = TARGET_ADDRESS_WIDTH-1; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      local_addr[k] = sda_i;
      drive_sda(1);
    end

    `uvm_info(name,
      $sformatf("DEBUG :: local_addr = 0x%0x", local_addr[6:0]), UVM_NONE)
    pkt.targetAddress = local_addr;

    if (local_addr != cfg_pkt.targetAddress)
      pkt.targetAddressStatus = NACK;
    else
      pkt.targetAddressStatus = ACK;

  endtask : sample_target_address

  task sample_operation(output operationType_e wr_rd);
    bit operation;
    state = WR_BIT;
    detectEdge_scl(POSEDGE);
    operation = sda_i;
    drive_sda(1);

    if (operation == 1'b0) begin
      wr_rd = WRITE;
      `uvm_info(name, "operation = WRITE", UVM_HIGH)
    end else begin
      wr_rd = READ;
      `uvm_info(name, "operation = READ", UVM_HIGH)
    end
  endtask : sample_operation

  task driveAddressAck(input bit ack);
    `uvm_info(name, $sformatf("driveAddressAck = %0d", ack), UVM_HIGH)
    state = ACK_NACK;
    detectEdge_scl(NEGEDGE);
    drive_sda(ack);
    detectEdge_scl(POSEDGE);
    detectEdge_scl(NEGEDGE);
    drive_sda(1'b1);
  endtask : driveAddressAck

  task sample_write_data(
      input  i3c_transfer_cfg_s cfg_pkt,
      inout  i3c_transfer_bits_s pkt,
      input  int i);

    bit [DATA_WIDTH-1:0] wdata;
    state = WRITE_DATA;

    for (int k = 0, bit_no = 0; k < DATA_WIDTH; k++) begin
      bit_no = (cfg_pkt.dataTransferDirection == MSB_FIRST) ?
               ((DATA_WIDTH - 1) - k) : k;
      detectEdge_scl(POSEDGE);
      wdata[bit_no] = sda_i;
      pkt.no_of_i3c_bits_transfer++;
    end

    targetFIFOMemory.push_back(wdata);
    pkt.writeData[i] = wdata;
  endtask : sample_write_data

  task driveWdataAck(input bit ack);
    state = ACK_NACK;
    detectEdge_scl(NEGEDGE);
    drive_sda(ack);
    detectEdge_scl(POSEDGE);
    detectEdge_scl(NEGEDGE);
    drive_sda(1'b1);
  endtask : driveWdataAck

  task drive_read_data(
      input  bit [7:0]            rdata_in,
      inout  i3c_transfer_bits_s  pkt,
      input  int                  i,
      input  dataTransferDirection_e dir);

    state = READ_DATA;

    for (int k = 0, bit_no = 0; k < DATA_WIDTH; k++) begin
      bit_no = (dir == MSB_FIRST) ? ((DATA_WIDTH - 1) - k) : k;
      drive_sda(rdata_in[bit_no]);
      pkt.no_of_i3c_bits_transfer++;
      detectEdge_scl(NEGEDGE);
    end
    pkt.readData[i] = rdata_in;
    drive_sda(1);
  endtask : drive_read_data

  task sample_ack(output bit ack);
    state = ACK_NACK;
    detectEdge_scl(POSEDGE);
    ack = sda_i;
    detectEdge_scl(NEGEDGE);
  endtask : sample_ack

  task wrDetect_stop();
    bit [1:0] scl_d;
    bit [1:0] sda_d;
    do begin
      @(negedge pclk);
      #1;
      scl_d = {scl_d[0], scl_i};
      sda_d = {sda_d[0], sda_i};
    end while (!(sda_d == POSEDGE && scl_d == 2'b11));
    state = STOP;
    `uvm_info(name, "Stop condition detected", UVM_HIGH)
  endtask : wrDetect_stop

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
    `uvm_info(name, "Stop condition detected", UVM_HIGH)
  endtask : detect_stop

  // =========================================================================
  // Low-level drive/edge helpers
  // =========================================================================

  task drive_sda(input bit value);
    // TRISTATE_BUF_ON  = 1 (driving)
    // TRISTATE_BUF_OFF = 0 (high-Z)
    sda_oen <= value ? TRISTATE_BUF_OFF : TRISTATE_BUF_ON;
    sda_o   <= value;
  endtask : drive_sda

  task drive_scl(input bit value);
    scl_oen <= value ? TRISTATE_BUF_OFF : TRISTATE_BUF_ON;
    scl_o   <= value;
  endtask : drive_scl

  task detectEdge_scl(input edge_detect_e edgeSCL);
    edge_detect_e scl_edge_value;
    do begin
      @(negedge pclk);
      scl_local = {scl_local[0], scl_i};
    end while (!(scl_local == edgeSCL));
    scl_edge_value = edge_detect_e'(scl_local);
    `uvm_info("TARGET_DRIVER_BFM",
      $sformatf("scl %s detected", scl_edge_value.name()), UVM_HIGH)
  endtask : detectEdge_scl

endinterface : i3c_target_driver_bfm

`endif
