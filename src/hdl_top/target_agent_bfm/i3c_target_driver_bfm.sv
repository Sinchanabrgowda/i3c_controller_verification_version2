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
  // SDR (non-DAA) transaction
/*
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
                        begin
        //sampleWriteDataAndDriveACK(dataPacketStruck, configPacketStruck);
                                sample_write_data(configPacketStruck,dataPacketStruck,0);
      detect_stop();
      `uvm_info("stop_detect_sdr", "sdr stop detected", UVM_HIGH)
                        end
                        else
        driveReadDataAndSampleACK(dataPacketStruck, configPacketStruck);
      end
                else begin
      `uvm_info(name, "targetAddressStatus is NACK", UVM_HIGH)
      detect_stop();
    end
  endtask : drive_data
*/
task drive_data(
    inout i3c_transfer_bits_s dataPacketStruck,
    input i3c_transfer_cfg_s  configPacketStruck
);
  `uvm_info(name, "target txn started", UVM_HIGH)
  detect_start();
  sample_target_address(configPacketStruck, dataPacketStruck);
  sample_operation(dataPacketStruck.operation);
  driveAddressAck(dataPacketStruck.targetAddressStatus);
  if (dataPacketStruck.targetAddressStatus == ACK) begin
    `uvm_info(name, "targetAddressStatus is ACK", UVM_HIGH)
   if (dataPacketStruck.operation == WRITE)
        begin
         fork
          begin
           for (int i = 0; i < MAXIMUM_BYTES; i++) begin
            sample_write_data(configPacketStruck, dataPacketStruck, i);
          end
        end
    join_none
      detect_stop();
      `uvm_info("stop_detect_sdr", "sdr stop detected", UVM_HIGH)
    end
    else begin
      driveReadDataAndSampleACK(dataPacketStruck, configPacketStruck);
    end
  end
  else begin
    `uvm_info(name, "targetAddressStatus is NACK", UVM_HIGH)
    detect_stop();
  end
endtask : drive_data
task sampleWriteDataAndDriveACK(
      inout i3c_transfer_bits_s dataPacketStruck,
      input i3c_transfer_cfg_s  configPacketStruck);
                                bit last_byte;
                                `uvm_info(name, "sampleWriteDataAndDriveACK started", UVM_HIGH)
    fork
      begin
        for (int i = 0; i < MAXIMUM_BYTES; i++) begin
          sample_write_data(configPacketStruck, dataPacketStruck, i);
                                                                //sample_write_data(configPacketStruck, dataPacketStruck, i, last_byte);
          `uvm_info(name,
            $sformatf("Sampled WDATA[%0d] = 0x%02h (%08b) status=%s",
                      i,
                      dataPacketStruck.writeData[i],
                      dataPacketStruck.writeData[i],
                      dataPacketStruck.writeDataStatus[i] == ACK ? "ACK" : "NACK"),
            UVM_HIGH)
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
  // DAA TRANSACTION  (multi-slave arbitration)
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
    int        lost_bit;
    bit        got_rep_start;
    `uvm_info(name, "DAA transaction started", UVM_NONE)
    if (has_address) begin
      `uvm_info("HAS_ADDR", "Already has dynamic address - ignoring DAA", UVM_NONE)
      daa_ack_out  = NACK;
      pid_out      = configPacketStruck.pid;
      bcr_out      = configPacketStruck.bcr;
      dcr_out      = configPacketStruck.dcr;
      dyn_addr_out = 7'h00;
      return;
    end
    // Build 64-bit arb value: PID[47:0] | BCR[7:0] | DCR[7:0]
    my_id = {configPacketStruck.pid,
             configPacketStruck.bcr,
             configPacketStruck.dcr};
    detect_start();
    `uvm_info(name, "DAA: START detected", UVM_NONE)
    sample_daa_broadcast_address(dataPacketStruck);   // 7E+W + ACK
    sample_daa_ccc_byte(dataPacketStruck);            // ENTDAA + ACK
    detect_repeated_start();                          // first REP_START
    sample_daa_broadcast_read(dataPacketStruck);      // 7E+R + ACK (unassigned pulls low)
    // ARBITRATION LOOP
    won_arb = 0;
    while (!won_arb) begin
      drive_daa_arb_bits(my_id, won_arb, lost_bit);
      if (!won_arb) begin
        drive_sda(1);
        `uvm_info(name, "DAA: Lost arbitration", UVM_NONE)
        skip_remaining_winner_frame(lost_bit);
        detect_rep_start_or_stop(got_rep_start);
        if (!got_rep_start) begin
          `uvm_info(name, "DAA: STOP after losing arb - DAA complete without winning", UVM_NONE)
          daa_ack_out  = NACK;
          pid_out      = configPacketStruck.pid;
          bcr_out      = configPacketStruck.bcr;
          dcr_out      = configPacketStruck.dcr;
          dyn_addr_out = 7'h00;
          return;
        end
        `uvm_info(name, "DAA: REP_START - re-entering arb with 7E+R header", UVM_NONE)
        sample_daa_broadcast_read(dataPacketStruck);
      end
    end // while !won_arb
    // WON arbitration
    sample_daa_dynamic_address(assigned_addr, dyn_addr_byte, daa_ack_out);
    driveAddressAck(daa_ack_out);
    if (daa_ack_out == ACK) begin
      has_address = 1;
      `uvm_info(name,
        $sformatf("DAA: slave assigned dynamic address 0x%0x", assigned_addr),
        UVM_NONE)
    end
    pid_out      = configPacketStruck.pid;
    bcr_out      = configPacketStruck.bcr;
    dcr_out      = configPacketStruck.dcr;
    dyn_addr_out = assigned_addr;
    dataPacketStruck.targetAddress       = 7'h7E;
    dataPacketStruck.targetAddressStatus = ACK;
    forever begin
      detect_rep_start_or_stop(got_rep_start);
      if (!got_rep_start) begin
        `uvm_info(name, "DAA: STOP detected - all devices assigned, exiting", UVM_NONE)
        return;
      end
      `uvm_info(name,
        "DAA: REP_START after assignment - consuming 7E+R, driving NACK",
        UVM_NONE)
      sample_daa_broadcast_read(dataPacketStruck);
    end // forever
  endtask : drive_daa_data
  // ARB PHASE  drive 64-bit PID+BCR+DCR with open-drain arbitration
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
      detectEdge_scl(NEGEDGE);
      drive_sda(my_bit);
      detectEdge_scl(POSEDGE);
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
  scl_local = {scl_i, scl_i};   // seed both history slots from live bus
  for (int i = 63; i >= 0; i--) begin
    my_bit      = my_id[i];
    all_bits[i] = my_id[i];
    // Step 1: SCL low ? safe window to change SDA
    detectEdge_scl(NEGEDGE);
    // Step 2: drive our bit
    drive_sda(my_bit);
    `uvm_info("DRIVE_DAA_BITS",
      $sformatf("sent bit =%b", my_bit), UVM_LOW)
    // Step 3: SCL rising ? master samples
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
    for (int i = 0; i < lost_at_bit; i++) begin
      detectEdge_scl(NEGEDGE);
      detectEdge_scl(POSEDGE);
    end
    for (int k = 0; k < 8; k++) begin
      detectEdge_scl(NEGEDGE);
      detectEdge_scl(POSEDGE);
    end
    detectEdge_scl(NEGEDGE);
    detectEdge_scl(POSEDGE);
    detectEdge_scl(NEGEDGE);
    `uvm_info(name,
      "skip_remaining_winner_frame: stepped past winner's full ACK release",
      UVM_HIGH)
  endtask : skip_remaining_winner_frame
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
    scl_loc = {scl_i, scl_i};
    sda_loc = {sda_i, sda_i};
    forever begin
      @(negedge pclk);
      scl_loc = {scl_loc[0], scl_i};
      sda_loc = {sda_loc[0], sda_i};
      if (scl_loc == 2'b11 && sda_loc == 2'b10) begin
         `uvm_info(name,$sformatf("[target_id=%0d] start detected[REPEATED START]@ time=%0t",i3c_target_drv_proxy_h.i3c_target_agent_cfg_h.target_id, $time),UVM_NONE)
        got_rep_start = 1;
        return;
      end
      if (scl_loc == 2'b11 && sda_loc == 2'b01) begin
 `uvm_info(name,$sformatf("[target_id=%0d] detect_rep_start_or_stop(STOP): STOP  @ time=%0t",i3c_target_drv_proxy_h.i3c_target_agent_cfg_h.target_id, $time),UVM_NONE)
        got_rep_start = 0;
        return;
      end
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
    detectEdge_scl(NEGEDGE);
`uvm_info("DAA_ACK_DEBUG",$sformatf("[target_id=%0d] has_address=%0b -> driving %s on 7E+R ACK slot",i3c_target_drv_proxy_h.i3c_target_agent_cfg_h.target_id,has_address,has_address ? "NACK" : "ACK"),UVM_NONE)
    drive_sda(has_address ? 1'b1 : 1'b0);
    detectEdge_scl(POSEDGE);
    //detectEdge_scl(NEGEDGE);  //had one issue
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
        $sformatf("DAA: dynamic addr=%0h dyn_add+parity=%0h parity OK =%0b? ACK",dyn_addr_out,addr_byte,parity_received),UVM_NONE)
        end
    else begin
      ack_out = NACK;
      `uvm_info(name,
        $sformatf("DAA: dynamic addr=0x%0x parity FAIL ? NACK", dyn_addr_out),
        UVM_NONE)
    end
  endtask : sample_daa_dynamic_address
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
     `uvm_info(name,$sformatf("[target_id=%0d] start detected@ time=%0t",i3c_target_drv_proxy_h.i3c_target_agent_cfg_h.target_id, $time),UVM_NONE)     ////addedd
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
    `uvm_info(name,
      $sformatf("sampled bit %0d = %0d", k, sda_i), UVM_HIGH)
      drive_sda(1);
    end
    `uvm_info(name,
      $sformatf("DEBUG :: local_addr = 0x%0x", local_addr[6:0]), UVM_NONE)
    pkt.targetAddress = local_addr;
 `uvm_info(name,
    $sformatf("DEBUG :: cfg target_addr = 0x%0x",
              cfg_pkt.targetAddress), UVM_NONE)
    if (local_addr != cfg_pkt.targetAddress) begin
      pkt.targetAddressStatus = NACK;
`uvm_info(name, "address mismatch NACK", UVM_HIGH)
   end else begin
      pkt.targetAddressStatus = ACK;
`uvm_info(name, "address match ACK", UVM_HIGH)
end
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
//added here
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
  `uvm_info(name, $sformatf("data=0x%02h (%08b)", wdata, wdata), UVM_HIGH)
  targetFIFOMemory.push_back(wdata);
  pkt.writeData[i]       = wdata;
  pkt.writeDataStatus[i] = ACK;
  driveWdataAck(pkt.writeDataStatus[i]);   // drives the real ACK slot immediately
endtask : sample_write_data
        /*
  task sample_write_data(
      input  i3c_transfer_cfg_s cfg_pkt,
      inout  i3c_transfer_bits_s pkt,
      input  int i);
    bit [DATA_WIDTH-1:0] wdata;
    bit                  t_bit;  // T-bit: flow control driven by CONTROLLER
                                 // 0 = more bytes coming
                                 // 1 = last byte (STOP follows)
    state = WRITE_DATA;
    // Step 1: sample 8 data bits on each POSEDGE
    for (int k = 0, bit_no = 0; k < DATA_WIDTH; k++) begin
      bit_no = (cfg_pkt.dataTransferDirection == MSB_FIRST) ?
               ((DATA_WIDTH - 1) - k) : k;
      detectEdge_scl(POSEDGE);
      wdata[bit_no] = sda_i;
      pkt.no_of_i3c_bits_transfer++;
    end
    detectEdge_scl(POSEDGE);
    t_bit = sda_i;
    `uvm_info(name,
      $sformatf("T-bit=%0b (%s)  data=0x%02h (%08b)",
                t_bit,
                t_bit ? "LAST BYTE" : "MORE BYTES",
                wdata, wdata),
      UVM_HIGH)
    targetFIFOMemory.push_back(wdata);
    pkt.writeData[i]      = wdata;
    pkt.writeDataStatus[i] = ACK;
    `uvm_info("debug_ack", $sformatf("driveAddressAck "), UVM_HIGH)
                                        driveWdataAck(pkt.writeDataStatus[i]);
                return;
  endtask : sample_write_data
*/
  task driveWdataAck(input bit ack);
    state = ACK_NACK;
    detectEdge_scl(NEGEDGE);
    `uvm_info("debug_ack1", $sformatf("driveAddressAck = %0d", ack), UVM_HIGH)
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
     `uvm_info(name,$sformatf("[target_id=%0d] stop detected@ time=%0t",i3c_target_drv_proxy_h.i3c_target_agent_cfg_h.target_id, $time),UVM_NONE)  //aded
  endtask : detect_stop
  task drive_sda(input bit value);
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
  // HOT JOIN (IBI-initiated DAA)
  task wait_for_stable_bus_idle();
    int idle_cycles;
    idle_cycles = 0;
    forever begin
      @(posedge pclk);
      if (scl_i == 1'b1 && sda_i == 1'b1)
        idle_cycles++;
      else
        idle_cycles = 0;
      if (idle_cycles > 4)
        return;
    end
  endtask : wait_for_stable_bus_idle
  task automatic drive_hot_join_ibi(input bit [6:0] hotjoin_addr,output bit ibi_ack);
    bit [7:0] full_byte;
//bit ibi_ack=0;
    bit bus_bit=0;
    full_byte = {hotjoin_addr, 1'b0};  // 7-bit addr + RnW(=0, target->ctrl)
    `uvm_info(name,
      "HOT_JOIN: waiting for stable bus idle before asserting IBI request",
      UVM_NONE)
    wait_for_stable_bus_idle();
    `uvm_info(name,
      "HOT_JOIN: asserting IBI request (SDA 1->0 while SCL high)",
      UVM_NONE)
    drive_sda(1'b0);
    detectEdge_scl(NEGEDGE);
    `uvm_info(name,
      $sformatf("HOT_JOIN: driving IBI address byte = 0x%0h (addr=0x%0h, RnW=1)",
                full_byte, hotjoin_addr),
      UVM_NONE)
    for (int k = 7; k >= 0; k--) begin
    // detectEdge_scl(NEGEDGE);
      drive_sda(full_byte[k]);
      detectEdge_scl(POSEDGE);
      bus_bit = sda_i;
      `uvm_info("HOT_JOIN_BUS_BIT",$sformatf("curretn bus bit=%b",bus_bit),UVM_LOW)
      detectEdge_scl(NEGEDGE);
    end
    drive_sda(1'b1);
    sample_ack(ibi_ack);   // <-- add this: sample master's ACK on the 9th clock
    `uvm_info("CHECK HOT JOIN", $sformatf("HOT_JOIN: address ACK/NACK = %0b", ibi_ack), UVM_NONE)
    `uvm_info(name,
      "HOT_JOIN: IBI address phase complete  controller will restart ENTDAA for hot-join",
      UVM_NONE)
//sample_daa_broadcast_Address();
  endtask : drive_hot_join_ibi
/*
task sample_daa_broadcast_Address();
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
    detectEdge_scl(NEGEDGE);
    drive_sda(1'b0);
    detectEdge_scl(POSEDGE);
    detectEdge_scl(NEGEDGE);
    drive_sda(1'b1);
  endtask : sample_daa_broadcast_Address
*/
//HOT JOIN TASK
  task drive_hot_join_data(
      input  bit [6:0]   hotjoin_addr,
      inout  i3c_transfer_bits_s dataPacketStruck,
      input  i3c_transfer_cfg_s  configPacketStruck,
      output bit [47:0]  pid_out,
      output bit [7:0]   bcr_out,
      output bit [7:0]   dcr_out,
      output bit [6:0]   dyn_addr_out,
      output bit         daa_ack_out);
    bit ack;
    `uvm_info(name, "HOT_JOIN: transaction started", UVM_NONE)
    drive_hot_join_ibi(hotjoin_addr,ack);
    if(ack==0)
    begin
    drive_daa_data(
      dataPacketStruck,
      configPacketStruck,
      pid_out,
      bcr_out,
      dcr_out,
      dyn_addr_out,
      daa_ack_out
    );
    `uvm_info(name,
      $sformatf("HOT_JOIN: complete. daa_ack=%0b dynamic_addr=0x%0h",
                daa_ack_out, dyn_addr_out),
      UVM_NONE)
   end
   else
    detect_stop();
  endtask : drive_hot_join_data
  task automatic drive_ibi_request(
      input  bit [6:0] my_dyn_addr,
      output bit       ibi_ack_out);
    bit [7:0] full_byte;
    bit       bus_bit;
    full_byte = {my_dyn_addr, 1'b1};  // dynamic addr + RnW=1 (target->ctrl read)
    `uvm_info(name,
      "IBI: waiting for stable bus idle before asserting IBI request",
      UVM_NONE)
    wait_for_stable_bus_idle();
    `uvm_info(name,
      "IBI: asserting IBI request (SDA 1->0 while SCL high)",
      UVM_NONE)
    drive_sda(1'b0);
    detectEdge_scl(NEGEDGE);
    `uvm_info(name,
      $sformatf("IBI: driving address byte = 0x%0h (addr=0x%0h, RnW=1)",
                full_byte, my_dyn_addr),
      UVM_NONE)
    for (int k = 7; k >= 0; k--) begin
      drive_sda(full_byte[k]);
      detectEdge_scl(POSEDGE);
      bus_bit = sda_i;
      `uvm_info("IBI_BUS_BIT", $sformatf("current bus bit=%b", bus_bit), UVM_LOW)
      detectEdge_scl(NEGEDGE);
    end
`uvm_info("IBI", $sformatf("done sending dynamic addr "), UVM_LOW)
    drive_sda(1'b1);
    sample_ack(ibi_ack_out);
    `uvm_info("CHECK IBI",
      $sformatf("IBI: address ACK/NACK = %0b", ibi_ack_out), UVM_NONE)
  endtask : drive_ibi_request
  task automatic drive_ibi_payload_byte(
      input  bit [7:0] data_in,
      input  bit       t_bit_in);
    `uvm_info(name,
      $sformatf("IBI: driving data byte = 0x%0h, T-bit=%0b (%0s)",
                data_in, t_bit_in,
                t_bit_in ? "MORE DATA" : "NO MORE DATA/STOP"), UVM_NONE)
    for (int k = 7; k >= 0; k--) begin
      drive_sda(data_in[k]);
      detectEdge_scl(NEGEDGE);
    end
    // T-bit (9th clock) -- driven the same way as any other data bit.
    drive_sda(t_bit_in);
    detectEdge_scl(NEGEDGE);
    if (!t_bit_in)
      drive_sda(1'b1);   // last byte: release SDA so the controller can drive STOP
    `uvm_info("IBI_DEBUG_T",
      $sformatf("IBI: data byte 0x%0h complete, T-bit driven=%0b",
                data_in, t_bit_in),
      UVM_NONE)
  endtask : drive_ibi_payload_byte
//IBI TASK
  // Generalized N-extra-byte version -- NEW, additive replacement. The
  // target decides up front how many extra bytes (num_extra_bytes) it
  // wants to send beyond the MDB, and drives T=1 after every one of them
  // except the last, where it drives T=0 and lets the controller issue
  // STOP. num_extra_bytes==0 reproduces the original MDB-only behavior
  // (T1=0 immediately after the MDB).
  task drive_ibi_data(
      input  bit [6:0] my_dyn_addr,
      input  bit [7:0] mdb_in,
      input  int unsigned num_extra_bytes,
      input  bit [7:0] extra_data_in[],
      output bit       ibi_ack_out,
      output bit       t1_out,
      output bit [7:0] extra_data_sent_out[],
      output bit       extra_t_bits_out[]);
    t1_out               = 1'b0;
    extra_data_sent_out  = new[0];
    extra_t_bits_out     = new[0];
    `uvm_info(name, "IBI: transaction started", UVM_NONE)
    drive_ibi_request(my_dyn_addr, ibi_ack_out);
    if (ibi_ack_out == ACK)
    begin
      // Byte 1: Mandatory Data Byte. The target itself decides T1:
      // 1 if any extra bytes follow, 0 (end of data) otherwise.
      t1_out = (num_extra_bytes > 0);
      drive_ibi_payload_byte(mdb_in, t1_out);
      if (num_extra_bytes > 0) begin
        extra_data_sent_out = new[num_extra_bytes];
        extra_t_bits_out    = new[num_extra_bytes];
        for (int k = 0; k < num_extra_bytes; k++) begin
          bit t_this;
          // T=1 after every extra byte except the last, where T=0.
          t_this = (k < num_extra_bytes - 1) ? 1'b1 : 1'b0;
          `uvm_info(name,
            $sformatf("IBI: target driving extra byte[%0d]=0x%0h, T=%0b (%0s)",
                      k, extra_data_in[k], t_this,
                      t_this ? "MORE DATA" : "NO MORE DATA/STOP"), UVM_NONE)
          drive_ibi_payload_byte(extra_data_in[k], t_this);
          extra_data_sent_out[k] = extra_data_in[k];
          extra_t_bits_out[k]    = t_this;
        end
      end
      detect_stop();
      `uvm_info(name, "IBI: complete, controller issued STOP", UVM_NONE)
    end else begin
      `uvm_info(name,
        "IBI: request NACKed by controller, no payload sent", UVM_NONE)
    end
  endtask : drive_ibi_data
//HDR DDR///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  task automatic sample_hdr_ddr_enthdr0(inout i3c_transfer_bits_s pkt);
    bit [7:0] ccc_byte;
    `uvm_info(name, "HDR-DDR: waiting for START (ENTHDR0 entry)", UVM_HIGH)
    detect_start();
    sample_daa_broadcast_address(pkt);   // 0x7E + W, ACKed by all Targets
    for (int k = 7; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      ccc_byte[k] = sda_i;
      drive_sda(1);
    end
    `uvm_info(name,
      $sformatf("HDR-DDR: CCC byte = 0x%0h (expect 0x%0h ENTHDR0)",
                ccc_byte, ENTHDR0_CCC_CODE), UVM_NONE)
    detectEdge_scl(NEGEDGE);   // SCL low after CCC byte's 8th bit
    detectEdge_scl(POSEDGE);   // T-Bit clock edge - not driven/ACKed
    detectEdge_scl(NEGEDGE);   // T-Bit falling edge = HDR-DDR mode begins
    `uvm_info(name, "HDR-DDR: mode entered (post T-Bit falling edge)", UVM_HIGH)
  endtask : sample_hdr_ddr_enthdr0
  task automatic sample_hdr_ddr_command_word(
      inout  i3c_transfer_bits_s pkt,
      input  i3c_transfer_cfg_s  cfg,
      inout  bit [4:0]           crc_state,
      output bit                 addr_match);
    bit [15:0] payload;
    bit        pa1, pa0;
    bit [1:0]  parity_calc;
    detectEdge_scl(POSEDGE); drive_sda(1);   // PRE1 (fixed 0 value not checked)
    detectEdge_scl(NEGEDGE); drive_sda(1);   // PRE0
    for (int i = 15; i >= 0; i--) begin
      detectEdge_scl(i[0] ? POSEDGE : NEGEDGE);
      payload[i] = sda_i;
      drive_sda(1);
      crc_state = i3c_hdr_ddr_crc5_next(crc_state, payload[i]);
    end
    detectEdge_scl(POSEDGE); pa1 = sda_i; drive_sda(1);
    detectEdge_scl(NEGEDGE); pa0 = sda_i; drive_sda(1);
    parity_calc = i3c_hdr_ddr_parity(payload);
    pkt.operation        = payload[15] ? READ : WRITE;
    pkt.hdr_ddr_cmd_code = payload[14:8];
    pkt.targetAddress    = payload[7:1];
    addr_match = (payload[7:1] == cfg.targetAddress) &&
                 ({pa1, pa0} == parity_calc);
    `uvm_info(name,
      $sformatf("HDR-DDR CMD: rw=%s cmd_code=0x%0h addr=0x%0h parity_ok=%0b addr_match=%0b",
                operationType_e'(pkt.operation).name(), pkt.hdr_ddr_cmd_code, pkt.targetAddress,({pa1, pa0} == parity_calc), addr_match), UVM_NONE)
        endtask : sample_hdr_ddr_command_word
  task automatic handle_hdr_ddr_word0_handshake(
      input  bit accept_cmd,
      output bit accepted);
    detectEdge_scl(POSEDGE);
    drive_sda(1);
    detectEdge_scl(NEGEDGE);
    accepted = accept_cmd;
    drive_sda(accepted ? 1'b0 : 1'b1);
  endtask : handle_hdr_ddr_word0_handshake
  task automatic handle_hdr_ddr_continue_preamble(
      input  i3c_transfer_bits_s pkt,
      input  bit                 more_words,   // meaningful for READ only
      output bit                 crc_next);
    bit pre1, pre0;
    if (pkt.operation == WRITE) begin
      detectEdge_scl(POSEDGE); pre1 = sda_i; drive_sda(1);
      detectEdge_scl(NEGEDGE); pre0 = sda_i; drive_sda(1);
      crc_next = (pre1 == 1'b0);
      if (pre0 !== 1'b1)
        `uvm_warning(name,
          "HDR-DDR: unexpected preamble in after-DATA context (mid-transfer ENDXFER abort/END is not modeled in this version)")
    end else begin
      crc_next = !more_words;
      drive_sda(crc_next ? 1'b0 : 1'b1);   // PRE1
      detectEdge_scl(POSEDGE);
      drive_sda(1'b1);                     // PRE0 (fixed)
      detectEdge_scl(NEGEDGE);
    end
  endtask : handle_hdr_ddr_continue_preamble
  // A single Data Word's 16-bit payload + PA1/PA0 (preamble already
  task automatic handle_hdr_ddr_data_payload(
      inout i3c_transfer_bits_s pkt,
      inout bit [4:0]           crc_state,
      input i3c_transfer_cfg_s  cfg,
      input int                 word_idx);
    bit [15:0] payload;
    bit [1:0]  parity_calc;
    bit        pa1, pa0;
    if (pkt.operation == WRITE) begin
      for (int i = 15; i >= 0; i--) begin
        detectEdge_scl(i[0] ? POSEDGE : NEGEDGE);
        payload[i] = sda_i;
        drive_sda(1);
        crc_state = i3c_hdr_ddr_crc5_next(crc_state, payload[i]);
      end
      detectEdge_scl(POSEDGE); pa1 = sda_i; drive_sda(1);
      detectEdge_scl(NEGEDGE); pa0 = sda_i; drive_sda(1);
      parity_calc = i3c_hdr_ddr_parity(payload);
      if ({pa1, pa0} !== parity_calc)
        `uvm_error(name,
          $sformatf("HDR-DDR: Data Word[%0d] parity error (rcvd=%0b%0b calc=%0b%0b)",
                    word_idx, pa1, pa0, parity_calc[1], parity_calc[0]))
      if (2*word_idx+1 < MAXIMUM_BYTES) begin
        pkt.writeData[2*word_idx]   = payload[15:8];
        pkt.writeData[2*word_idx+1] = payload[7:0];
      end
      targetFIFOMemory.push_back(payload[15:8]);
      targetFIFOMemory.push_back(payload[7:0]);
      pkt.no_of_i3c_bits_transfer += 16;
    end else begin
      bit [15:0] hdr_rdata;
      if (targetFIFOMemory.size() >= 2) begin
        hdr_rdata[15:8] = targetFIFOMemory.pop_front();
        hdr_rdata[7:0]  = targetFIFOMemory.pop_front();
      end else begin
        hdr_rdata = {cfg.defaultReadData, cfg.defaultReadData};
      end
      for (int i = 15; i >= 0; i--) begin
        drive_sda(hdr_rdata[i]);
        crc_state = i3c_hdr_ddr_crc5_next(crc_state, hdr_rdata[i]);
        detectEdge_scl(i[0] ? POSEDGE : NEGEDGE);
      end
      parity_calc = i3c_hdr_ddr_parity(hdr_rdata);
      drive_sda(parity_calc[1]); detectEdge_scl(POSEDGE);
      drive_sda(parity_calc[0]); detectEdge_scl(NEGEDGE);
      if (2*word_idx+1 < MAXIMUM_BYTES) begin
        pkt.readData[2*word_idx]   = hdr_rdata[15:8];
        pkt.readData[2*word_idx+1] = hdr_rdata[7:0];
      end
      pkt.no_of_i3c_bits_transfer += 16;
    end
  endtask : handle_hdr_ddr_data_payload
  // CRC Word  2'b01 preamble
  // caller's continue-preamble step) + 4'hC token + 5-bit CRC-5 + 1 setup
  // bit. Write: Controller drives, Target checks against its own running
  // CRC-5. Read: Target drives its own running CRC-5, Controller checks.
  task automatic handle_hdr_ddr_crc_word(
      inout i3c_transfer_bits_s pkt,
      input bit [4:0]           crc_calc);
    bit [3:0] token;
    bit [4:0] crc_rcvd;
    if (pkt.operation == WRITE) begin
      for (int i = 3; i >= 0; i--) begin
        detectEdge_scl(i[0] ? POSEDGE : NEGEDGE);
        token[i] = sda_i;
        drive_sda(1);
      end
      for (int i = 4; i >= 0; i--) begin
        detectEdge_scl(i[0] ? NEGEDGE : POSEDGE);
        crc_rcvd[i] = sda_i;
        drive_sda(1);
      end
      detectEdge_scl(NEGEDGE);   // setup bit (kept High by the Controller)
      drive_sda(1);
      pkt.hdr_ddr_crc_calc = crc_calc;
      pkt.hdr_ddr_crc_rcvd = crc_rcvd;
      pkt.hdr_ddr_crc_ok   = (crc_rcvd == crc_calc);
      if (token != HDR_DDR_CRC_TOKEN)
        `uvm_error(name,
          $sformatf("HDR-DDR: CRC token=0x%0h, expected 0x%0h", token, HDR_DDR_CRC_TOKEN))
      if (pkt.hdr_ddr_crc_ok)
        `uvm_info(name, $sformatf("HDR-DDR: CRC OK (0x%0h)", crc_rcvd), UVM_NONE)
      else
        `uvm_error(name,
          $sformatf("HDR-DDR: CRC MISMATCH received=0x%0h calc=0x%0h", crc_rcvd, crc_calc))
    end else begin
      for (int i = 3; i >= 0; i--) begin
        drive_sda(HDR_DDR_CRC_TOKEN[i]);
        detectEdge_scl(i[0] ? POSEDGE : NEGEDGE);
      end
      for (int i = 4; i >= 0; i--) begin
        drive_sda(crc_calc[i]);
        detectEdge_scl(i[0] ? NEGEDGE : POSEDGE);
      end
      drive_sda(1'b1);           // setup bit
      detectEdge_scl(NEGEDGE);
      pkt.hdr_ddr_crc_calc = crc_calc;
      pkt.hdr_ddr_crc_rcvd = crc_calc;
      pkt.hdr_ddr_crc_ok   = 1'b1;
      `uvm_info(name, $sformatf("HDR-DDR: drove CRC = 0x%0h", crc_calc), UVM_NONE)
    end
    // SDA parks High for >= tDIG_H, then the Restart/Exit Pattern begins.
    drive_sda(1);
  endtask : handle_hdr_ddr_crc_word
  task automatic detect_hdr_exit_or_restart_pattern(
      output bit is_restart,
      output bit is_exit);
    int fall_count;
    bit prev_sda;
    is_restart = 0;
    is_exit    = 0;
    fall_count = 0;
    prev_sda   = sda_i;
    forever begin
      @(negedge pclk);
      if (scl_i == 1'b1) begin
        if (fall_count == 2 && sda_i == 1'b1) begin
          is_restart = 1;
          `uvm_info(name, "HDR-DDR: HDR Restart Pattern detected", UVM_NONE)
        end
        return;
      end
      if (prev_sda == 1'b1 && sda_i == 1'b0) begin
        fall_count++;
        if (fall_count >= 4) begin
          is_exit = 1;
          `uvm_info(name, "HDR-DDR: HDR Exit Pattern detected", UVM_NONE)
          return;
        end
      end
      prev_sda = sda_i;
    end
  endtask : detect_hdr_exit_or_restart_pattern
  task automatic drive_hdr_ddr_data(
      inout i3c_transfer_bits_s dataPacketStruck,
      input i3c_transfer_cfg_s  configPacketStruck,
      input bit                 skip_enthdr0 = 1'b0);
    bit [4:0] crc_state;
    bit       addr_match;
    bit       accepted;
    bit       crc_next;
    bit       is_restart, is_exit;
    int       word_idx;
    int       max_words;
    `uvm_info(name,
      $sformatf("HDR-DDR: transaction started (skip_enthdr0=%0b)", skip_enthdr0),
      UVM_NONE)
    crc_state                            = HDR_DDR_CRC5_INIT;
    dataPacketStruck.no_of_i3c_bits_transfer = 0;
    dataPacketStruck.hdr_ddr_got_restart = 0;
    dataPacketStruck.hdr_ddr_got_exit    = 0;
    if (!skip_enthdr0)
      sample_hdr_ddr_enthdr0(dataPacketStruck);
    else
      `uvm_info(name,
        "HDR-DDR: chained Command Word after HDR Restart Pattern - skipping ENTHDR0 sampling",
        UVM_NONE)
    sample_hdr_ddr_command_word(dataPacketStruck, configPacketStruck,
                                 crc_state, addr_match);
    handle_hdr_ddr_word0_handshake(addr_match, accepted);
    dataPacketStruck.hdr_ddr_cmd_ack = accepted ? ACK : NACK;
    if (accepted) begin
      handle_hdr_ddr_data_payload(dataPacketStruck, crc_state,
                                   configPacketStruck, 0);
      word_idx  = 1;
      max_words = (dataPacketStruck.hdr_ddr_num_words > 0) ?
                    dataPacketStruck.hdr_ddr_num_words : 1;
      if (max_words > MAXIMUM_BYTES/2) max_words = MAXIMUM_BYTES/2;
      forever begin
        handle_hdr_ddr_continue_preamble(
          dataPacketStruck, (word_idx < max_words), crc_next);
        if (crc_next) break;
        handle_hdr_ddr_data_payload(dataPacketStruck, crc_state,
                                     configPacketStruck, word_idx);
        word_idx++;
        if (word_idx >= (MAXIMUM_BYTES/2)) begin
          `uvm_warning(name, "HDR-DDR: word-count safety cap reached, forcing CRC")
          break;
        end
      end
      dataPacketStruck.hdr_ddr_num_words = word_idx;
      handle_hdr_ddr_crc_word(dataPacketStruck, crc_state);
    end else begin
      `uvm_info(name,
        "HDR-DDR: Command ignored (address mismatch/parity error) - no data phase",
        UVM_NONE)
    end
    detect_hdr_exit_or_restart_pattern(is_restart, is_exit);
    dataPacketStruck.hdr_ddr_got_restart = is_restart;
    dataPacketStruck.hdr_ddr_got_exit    = is_exit;
    if (!is_restart && !is_exit)
      `uvm_warning(name, "HDR-DDR: neither Restart nor Exit Pattern observed before task exit")
    if (is_restart)
      `uvm_info(name,
        "HDR-DDR: HDR Restart Pattern observed - caller (i3c_target_hdr_ddr_seq) may issue a follow-up sequence item with skip_enthdr0=1 to drive/sample the next chained Command Word",
        UVM_NONE)
    `uvm_info(name,
      $sformatf("HDR-DDR: transaction complete. cmd_ack=%0b crc_ok=%0b words=%0d restart=%0b exit=%0b",
                dataPacketStruck.hdr_ddr_cmd_ack, dataPacketStruck.hdr_ddr_crc_ok,
                dataPacketStruck.hdr_ddr_num_words, dataPacketStruck.hdr_ddr_got_restart,
                dataPacketStruck.hdr_ddr_got_exit),
      UVM_NONE)
  endtask : drive_hdr_ddr_data
*/
endinterface : i3c_target_driver_bfm
`endif
