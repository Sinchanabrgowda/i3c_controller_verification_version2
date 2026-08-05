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
 bit has_address = 0;
function bit is_addressed();
  return has_address;
endfunction : is_addressed
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
    // Steps 1-3: START + 7E+W + ENTDAA (common preamble, once only)
    detect_start();
    `uvm_info(name, "DAA MON: START detected", UVM_HIGH)
 if (has_address) begin
    `uvm_info(name,
      "DAA MON: already has dynamic address - passively skipping this DAA session",
      UVM_NONE)
    skip_daa_session_passively();
    return;
  end
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
    // Steps 4-11: Loop per round until STOP
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
        has_address         = 1;
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
task automatic skip_daa_session_passively();
  bit [1:0] scl_loc;
  bit [1:0] sda_loc;
  bit       got_rep_start;
  // consume 7E+W + ACK
  for (int k = 7; k >= 0; k--) detectEdge_scl(POSEDGE);
  detectEdge_scl(NEGEDGE); detectEdge_scl(POSEDGE); detectEdge_scl(NEGEDGE);
  // consume ENTDAA + ACK
  for (int k = 7; k >= 0; k--) detectEdge_scl(POSEDGE);
  detectEdge_scl(NEGEDGE); detectEdge_scl(POSEDGE); detectEdge_scl(NEGEDGE);
  detect_rep_start(scl_loc, sda_loc);
  forever begin
    detectEdge_scl(NEGEDGE);
    for (int k = 7; k >= 0; k--) detectEdge_scl(POSEDGE);   // 7E+R
    detectEdge_scl(NEGEDGE); detectEdge_scl(POSEDGE); detectEdge_scl(NEGEDGE); // ACK
    for (int k = 63; k >= 0; k--) begin                      // 64 arb bits
      detectEdge_scl(POSEDGE);
      detectEdge_scl(NEGEDGE);
    end
    for (int k = 7; k >= 0; k--) detectEdge_scl(POSEDGE);    // dyn addr byte
    detectEdge_scl(NEGEDGE); detectEdge_scl(POSEDGE);        // ACK/NACK
    detect_rep_start_or_stop(scl_loc, sda_loc, got_rep_start);
    detectEdge_scl(NEGEDGE);
    if (!got_rep_start) return;   // STOP — session over
  end
endtask : skip_daa_session_passively
  // Detect Repeated-START: SDA falls while SCL=1
  task automatic detect_rep_start(ref bit [1:0] scl_loc, ref bit [1:0] sda_loc);
    scl_loc = {scl_i, scl_i};
    sda_loc = {sda_i, sda_i};
    do begin
      @(negedge pclk);
      scl_loc = {scl_loc[0], scl_i};
      sda_loc = {sda_loc[0], sda_i};
    end while (!(sda_loc == 2'b10 && scl_loc == 2'b11));
   `uvm_info(name,$sformatf(" repeated start detected@ time=%0t", $time),UVM_NONE)
        endtask : detect_rep_start
  // Detect Rep-START (SDA falls, SCL=1) or STOP (SDA rises, SCL=1)
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
         `uvm_info(name,$sformatf("DAA repeated start detected@ time=%0t", $time),UVM_NONE)
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
  // Helpers
  task detect_start();
    bit [1:0] scl_d;
    bit [1:0] sda_d;
    state = START;
    do begin
      @(negedge pclk);
      scl_d = {scl_d[0], scl_i};
      sda_d = {sda_d[0], sda_i};
    end while (!(sda_d == NEGEDGE && scl_d == 2'b11));
   //`uvm_info(name,$sformatf("[target_id=%0d] DAA MON: START detected @ time=%0t",i3c_target_drv_proxy_h.i3c_target_agent_cfg_h.target_id, $time),UVM_NONE)
  `uvm_info(name,$sformatf(" start detected@ time=%0t", $time),UVM_NONE)
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
  `uvm_info(name,$sformatf(" stop detected@ time=%0t", $time),UVM_NONE)
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
  // HOT JOIN monitoring — NEW, additive only.
  task sample_hot_join_ibi(output bit [6:0] ibi_addr_out ,output bit ack);
    bit [7:0] full_byte;
    //bit       ack;
    `uvm_info(name,
      "HOT_JOIN MON: waiting for IBI request (SDA fall while SCL high)",
      UVM_NONE)
    detect_start();   // electrically identical pattern to an IBI request
    `uvm_info(name, "HOT_JOIN MON: sampling IBI address byte", UVM_NONE)
    detectEdge_scl(NEGEDGE);
    for (int k = 7; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      full_byte[k] = sda_i;
    end
    ibi_addr_out = full_byte[7:1];
    // ACK slot — driven by the controller for this read-direction byte.
    detectEdge_scl(NEGEDGE);
    detectEdge_scl(POSEDGE);
    ack = sda_i;
    detectEdge_scl(NEGEDGE);
    `uvm_info(name,
      $sformatf("HOT_JOIN MON: ibi_addr=0x%0h ack=%0b (controller will restart ENTDAA)",
                ibi_addr_out, ack),
      UVM_NONE)
  endtask : sample_hot_join_ibi
  task sample_hot_join_data(
      inout  i3c_transfer_bits_s pkt,
      inout  i3c_transfer_cfg_s  cfg,
      output bit [6:0]           observed_ibi_addr);
  bit ack;
    sample_hot_join_ibi(observed_ibi_addr,ack);
if(ack==0)begin
    sample_daa_data(pkt, cfg);
end
else
  detect_stop();
  endtask : sample_hot_join_data
  // IBI (In-Band Interrupt)
  task sample_ibi_request(
      output bit [6:0] ibi_addr_out,
      output bit       ack_out);
    bit [7:0] full_byte;
    `uvm_info(name,
      "IBI MON: waiting for IBI request (SDA fall while SCL high)",
      UVM_NONE)
    detect_start();
    `uvm_info(name, "IBI MON: sampling IBI address byte", UVM_NONE)
    detectEdge_scl(NEGEDGE);
    for (int k = 7; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      full_byte[k] = sda_i;
    end
    ibi_addr_out = full_byte[7:1];
    // ACK slot -- driven by the controller for this read-direction byte.
    detectEdge_scl(NEGEDGE);
    detectEdge_scl(POSEDGE);
    ack_out = sda_i;
    detectEdge_scl(NEGEDGE);
    `uvm_info(name,
      $sformatf("IBI MON: addr=0x%0h ack=%0b", ibi_addr_out, ack_out),
      UVM_NONE)
  endtask : sample_ibi_request
  // Samples one IBI data byte plus the following T-bit (9th clock).
  // Per the I3C spec the TARGET drives the T-bit in this direction; the
  // monitor is purely passive and just samples what it observes on SDA.
  task sample_ibi_payload_byte(
      output bit [7:0] data_out,
      output bit       t_bit_out);
    for (int k = 7; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      data_out[k] = sda_i;
    end
    detectEdge_scl(NEGEDGE);
    detectEdge_scl(POSEDGE);   // T-bit slot
    t_bit_out = sda_i;
    detectEdge_scl(NEGEDGE);
    `uvm_info(name,
      $sformatf("IBI MON: data=0x%0h T-bit=%0b (%0s)",
                data_out, t_bit_out,
                t_bit_out ? "MORE DATA" : "NO MORE DATA/STOP"),
      UVM_NONE)
  endtask : sample_ibi_payload_byte
  // Generalized N-extra-byte version -- NEW, additive replacement. The
  // monitor is purely passive: it doesn't need to know in advance how
  // many extra bytes are coming, it just keeps sampling bytes as long as
  // it observes T=1, and stops the moment it samples T=0 -- exactly the
  // "T=1 -> byte -> T-bit -> ..." loop from the I3C spec (4.3.6.2).
  task sample_ibi_data(
      output bit [6:0] ibi_addr_out,
      output bit       ack_out,
      output bit [7:0] mdb_out,
      output bit       t1_out,
      output bit [7:0] extra_data_out[],
      output bit       extra_t_bits_out[]);
    bit [7:0] extra_data_q[$];
    bit       extra_t_q[$];
    bit       more_data;
    mdb_out           = 8'h00;
    t1_out            = 1'b0;
    extra_data_out    = new[0];
    extra_t_bits_out  = new[0];
    sample_ibi_request(ibi_addr_out, ack_out);
    if (ack_out == ACK) begin
      sample_ibi_payload_byte(mdb_out, t1_out);
      more_data = t1_out;
      while (more_data) begin
        bit [7:0] cur_data;
        bit       cur_t;
        `uvm_info(name,
          $sformatf("IBI MON: T=1 sampled, sampling extra byte[%0d]",
                    extra_data_q.size()), UVM_NONE)
        sample_ibi_payload_byte(cur_data, cur_t);
        extra_data_q.push_back(cur_data);
        extra_t_q.push_back(cur_t);
        more_data = cur_t;
      end
      extra_data_out   = extra_data_q;
      extra_t_bits_out = extra_t_q;
      detect_stop();
    end else begin
      `uvm_info(name, "IBI MON: request NACKed, no payload phase", UVM_NONE)
    end
  endtask : sample_ibi_data
 /*
        task automatic sample_hdr_ddr_enthdr0();
    bit [7:0] ccc_byte;
    `uvm_info(name, "HDR-DDR MON: waiting for START (ENTHDR0 entry)", UVM_HIGH)
    detect_start();
    // 0x7E + W (8 bits) + ACK slot
    detectEdge_scl(NEGEDGE);
    for (int k = 7; k >= 0; k--)
      detectEdge_scl(POSEDGE);
    detectEdge_scl(NEGEDGE);
    detectEdge_scl(POSEDGE);   // ACK
    detectEdge_scl(NEGEDGE);
    for (int k = 7; k >= 0; k--) begin
      detectEdge_scl(POSEDGE);
      ccc_byte[k] = sda_i;
    end
    `uvm_info(name,
      $sformatf("HDR-DDR MON: CCC byte = 0x%0h (expect 0x%0h ENTHDR0)",
                ccc_byte, ENTHDR0_CCC_CODE), UVM_NONE)
    detectEdge_scl(NEGEDGE);   // SCL low after CCC byte's 8th bit
    detectEdge_scl(POSEDGE);   // T-Bit clock edge
    detectEdge_scl(NEGEDGE);   // T-Bit falling edge = HDR-DDR mode begins
  endtask : sample_hdr_ddr_enthdr0
  task automatic sample_hdr_ddr_command_word(
      inout  i3c_transfer_bits_s pkt,
      input  i3c_transfer_cfg_s  cfg,
      inout  bit [4:0]           crc_state,
      output bit                 addr_match);
    bit [15:0] payload;
    bit        pa1, pa0;
    bit [1:0]  parity_calc;
    detectEdge_scl(POSEDGE);   // PRE1
    detectEdge_scl(NEGEDGE);   // PRE0
    for (int i = 15; i >= 0; i--) begin
      detectEdge_scl(i[0] ? POSEDGE : NEGEDGE);
      payload[i] = sda_i;
      crc_state = i3c_hdr_ddr_crc5_next(crc_state, payload[i]);
    end
    detectEdge_scl(POSEDGE); pa1 = sda_i;
    detectEdge_scl(NEGEDGE); pa0 = sda_i;
    parity_calc = i3c_hdr_ddr_parity(payload);
    pkt.operation        = payload[15] ? READ : WRITE;
    pkt.hdr_ddr_cmd_code = payload[14:8];
    pkt.targetAddress    = payload[7:1];
    addr_match = (payload[7:1] == cfg.targetAddress) &&
                 ({pa1, pa0} == parity_calc);
    `uvm_info(name,
      $sformatf("HDR-DDR MON CMD: rw=%s cmd_code=0x%0h addr=0x%0h addr_match=%0b",
                operationType_e'(pkt.operation).name(), pkt.hdr_ddr_cmd_code, pkt.targetAddress,
                addr_match), UVM_NONE)
        endtask : sample_hdr_ddr_command_word
  task automatic sample_hdr_ddr_word0_handshake(output bit accepted);
    bit pre1, pre0;
    detectEdge_scl(POSEDGE); pre1 = sda_i;
    detectEdge_scl(NEGEDGE); pre0 = sda_i;
    accepted = (pre0 == 1'b0);
  endtask : sample_hdr_ddr_word0_handshake
  task automatic sample_hdr_ddr_continue_preamble(output bit crc_next);
    bit pre1, pre0;
    detectEdge_scl(POSEDGE); pre1 = sda_i;
    detectEdge_scl(NEGEDGE); pre0 = sda_i;
    crc_next = (pre1 == 1'b0);
  endtask : sample_hdr_ddr_continue_preamble
  task automatic sample_hdr_ddr_data_payload(
      inout i3c_transfer_bits_s pkt,
      inout bit [4:0]           crc_state,
      input int                 word_idx);
    bit [15:0] payload;
    for (int i = 15; i >= 0; i--) begin
      detectEdge_scl(i[0] ? POSEDGE : NEGEDGE);
      payload[i] = sda_i;
      crc_state = i3c_hdr_ddr_crc5_next(crc_state, payload[i]);
    end
    detectEdge_scl(POSEDGE);   // PA1
    detectEdge_scl(NEGEDGE);   // PA0
    if (2*word_idx+1 < MAXIMUM_BYTES) begin
      if (pkt.operation == WRITE) begin
        pkt.writeData[2*word_idx]   = payload[15:8];
        pkt.writeData[2*word_idx+1] = payload[7:0];
      end else begin
        pkt.readData[2*word_idx]   = payload[15:8];
        pkt.readData[2*word_idx+1] = payload[7:0];
      end
    end
    pkt.no_of_i3c_bits_transfer += 16;
  endtask : sample_hdr_ddr_data_payload
  task automatic sample_hdr_ddr_crc_word(
      inout i3c_transfer_bits_s pkt,
      input bit [4:0]           crc_calc);
    bit [3:0] token;
    bit [4:0] crc_rcvd;
    for (int i = 3; i >= 0; i--) begin
      detectEdge_scl(i[0] ? POSEDGE : NEGEDGE);
      token[i] = sda_i;
    end
    for (int i = 4; i >= 0; i--) begin
      detectEdge_scl(i[0] ? NEGEDGE : POSEDGE);
      crc_rcvd[i] = sda_i;
    end
    detectEdge_scl(NEGEDGE);   // setup bit
    pkt.hdr_ddr_crc_calc = crc_calc;
    pkt.hdr_ddr_crc_rcvd = crc_rcvd;
    pkt.hdr_ddr_crc_ok   = (crc_rcvd == crc_calc);
    if (token != HDR_DDR_CRC_TOKEN)
      `uvm_warning(name,
        $sformatf("HDR-DDR MON: CRC token=0x%0h, expected 0x%0h", token, HDR_DDR_CRC_TOKEN))
    if (pkt.hdr_ddr_crc_ok)
      `uvm_info(name, $sformatf("HDR-DDR MON: CRC OK (0x%0h)", crc_rcvd), UVM_NONE)
    else
      `uvm_error(name,
        $sformatf("HDR-DDR MON: CRC MISMATCH observed=0x%0h calc=0x%0h", crc_rcvd, crc_calc))
  endtask : sample_hdr_ddr_crc_word
  // Identical (already purely observational) algorithm as the driver
  // BFM's detector - see i3c_target_driver_bfm.sv for the spec reference
  // and abstraction-level notes.
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
          `uvm_info(name, "HDR-DDR MON: HDR Restart Pattern detected", UVM_NONE)
        end
        return;
      end
      if (prev_sda == 1'b1 && sda_i == 1'b0) begin
        fall_count++;
        if (fall_count >= 4) begin
          is_exit = 1;
          `uvm_info(name, "HDR-DDR MON: HDR Exit Pattern detected", UVM_NONE)
          return;
        end
      end
      prev_sda = sda_i;
    end
  endtask : detect_hdr_exit_or_restart_pattern
  // Top-level orchestrator - called from i3c_target_monitor_proxy for the
  // pending_hdr_ddr flag, mirroring how sample_data()/sample_daa_data()
  // are called for SDR/DAA.
  task automatic sample_hdr_ddr_data(
      inout i3c_transfer_bits_s pkt,
      inout i3c_transfer_cfg_s  cfg,
      input bit                 skip_enthdr0 = 1'b0);
    bit [4:0] crc_state;
    bit       addr_match;
    bit       accepted;
    bit       crc_next;
    bit       is_restart, is_exit;
    int       word_idx;
    int       max_words;
    crc_state                 = HDR_DDR_CRC5_INIT;
    pkt.no_of_i3c_bits_transfer = 0;
    pkt.hdr_ddr_got_restart    = 0;
    pkt.hdr_ddr_got_exit       = 0;
    if (!skip_enthdr0)
      sample_hdr_ddr_enthdr0();
    else
      `uvm_info(name,
        "HDR-DDR MON: chained Command Word after HDR Restart Pattern - skipping ENTHDR0 sampling",
        UVM_NONE)
    sample_hdr_ddr_command_word(pkt, cfg, crc_state, addr_match);
    sample_hdr_ddr_word0_handshake(accepted);
    pkt.hdr_ddr_cmd_ack = accepted ? ACK : NACK;
    if (accepted) begin
      sample_hdr_ddr_data_payload(pkt, crc_state, 0);
      word_idx  = 1;
      max_words = MAXIMUM_BYTES/2;   // Monitor has no a-priori word count;
      forever begin
        sample_hdr_ddr_continue_preamble(crc_next);
        if (crc_next) break;
        sample_hdr_ddr_data_payload(pkt, crc_state, word_idx);
        word_idx++;
        if (word_idx >= max_words) begin
          `uvm_warning(name, "HDR-DDR MON: word-count safety cap reached, forcing CRC")
          break;
        end
      end
      pkt.hdr_ddr_num_words = word_idx;
      sample_hdr_ddr_crc_word(pkt, crc_state);
    end else begin
      `uvm_info(name,
        "HDR-DDR MON: Command ignored (address mismatch) - no data phase",
        UVM_NONE)
    end
    detect_hdr_exit_or_restart_pattern(is_restart, is_exit);
    pkt.hdr_ddr_got_restart = is_restart;
    pkt.hdr_ddr_got_exit    = is_exit;
  endtask : sample_hdr_ddr_data
*/
endinterface : i3c_target_monitor_bfm
`endif
//added
