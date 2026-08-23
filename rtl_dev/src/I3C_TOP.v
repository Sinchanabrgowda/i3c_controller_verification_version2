///TOP
`include "param.v"
module I3C_TOP(
  input   wire          clk,
  input   wire          rst_n,
  // CPU interface
  input   wire          wr_en,
  input   wire          rd_en,
  input   wire  [6:0]   addrs,
  input   wire  [31:0]  w_reg_data,
  input   wire  [7:0]   w_data,
  input   wire          cmd_dc_type,
  output  wire  [31:0]  rd_data,
  output  wire  [7:0]   r_data,
  output  wire          scl_i,
  input   wire          sda_i,
  output  wire          sda_o,
  output  wire          sda_oe
);
 
wire        scl_oe;
wire        scl_o;
 
///// FIFO wires////////////////////
wire        tx_wr_en;
wire [7:0]  tx_wdata;
wire        tx_valid;
wire        tx_ready1;
wire        tx_ready2;
wire        tx_ready = tx_ready1 || tx_ready2;
wire [7:0]  tx_rdata;
 
wire        rx_rd_en;
wire [7:0]  rx_wdata1;
wire        rx_wdata2;
wire [7:0]  rx_wdata;
 
wire        rx_valid1;
wire        rx_valid2;
wire        rx_valid = rx_valid1 || rx_valid2;
assign rx_wdata = rx_valid1 ? rx_wdata1 :
                  rx_valid2 ? rx_wdata2 : 'b0;
wire [7:0]  rx_rdata;
 
////// CMD wires/////////////////
wire        cmd_start;
wire [1:0]  cmd_type;
wire        sdr_dev_type;
wire        dev_type;
wire [6:0]  cmd_addr;
wire [7:0]  cmd_ccc;
wire [7:0]  cmd_len;
wire        cmd_dir;
wire        hdr_pending;
wire        shift_hold;
 
/////// From FSM signals//////////////
wire        sdr_done;
wire        sdr_busy;
wire        be_nack;
wire [7:0]  sdr_ccc_code;
wire [7:0]  sdr_read_len;
wire        sdr_ccc_is_read;
wire        sdr_has_restart;
////////SDR FSM/////////////////
wire        sdr_error;
wire        push_pull1;
wire        start_sdr;
wire        start_daa;
wire        sdr_dir1;
wire [6:0]  sdr_addr;
wire [7:0]  sdr_len;
wire        cmd_busy;
wire        stop_read;
wire        ccc_read;
wire        s_r1;
wire        shift_done;
////////DAA wires/////////////////
wire         daa_start;  
wire         daa_error;
wire [6:0]   dyn_addr;
wire         daa_busy;
wire         daa_done;
wire [6:0]   daa_dyn_addr;
wire [47:0]  pid;
wire [7:0]   bcr;
wire [7:0]   dcr;
wire         start_daa_final;
 
wire        be_done1;
wire        arb_mode1;
wire        start_r1;
wire        start_r2;
wire        sdr_active;
wire [7:0]  be_tx_data1;
wire [7:0]  be_rx_data;
 
wire [7:0]  sdr_be_tx_data;
wire [7:0]  daa_be_tx_data;
wire        be_start;
wire        be_rw;
wire        be_valid;
 
wire        sdr_be_rw;
wire        daa_be_rw;
wire        sdr_valid;
wire        daa_valid;
wire        sdr_push_pull;
wire        daa_push_pull;
wire        be_busy1;
wire        start;
wire        stop;
wire        is_i3c1;
wire [6:0]  sdr_addr1;
wire        parity_error;
wire        arbitration_lost;
wire        device_known;
wire        ibi_addr_cap_en;   // 1-cycle pulse: 7-bit addr ready
wire [6:0]  ibi_addr_cap;      // captured target address
wire        ibi_ack_ok;
// device table interface
wire         lookup_en;
wire [6:0]   lookup_addr;
wire         hotjoin_req;
//In-band interrupt
wire        ibi_request1;
wire        ibi_valid;
wire [6:0]  ibi_addr;
wire [7:0]  ibi_payload;
wire [7:0]  irq_payload;
wire [6:0]  irq_addr;
wire        irq;
wire        bus_idle;
 
//FOR DDR
wire [1:0]  mode;
wire        cmd_mode;
wire        start_hdr;
 
wire        hdr_valid;
wire        hdr_rw;
wire [15:0] hdr_tx_data;   // DDR → 2 bytes per cycle
wire [15:0] hdr_rx_data;
wire        hdr_done;
wire        hdr_done1;
wire        hdr_busy;
wire        hdr_busy1;
wire        hdr_active;
wire        hdr_start_sig;
wire        hdr_stop_sig;
 
//NEW SCL
wire        scl_start_sdr;
wire        scl_stop_sdr;
wire        push_pull_sdr;
 
wire        scl_start_hdr;
wire        scl_stop_hdr;
wire        push_pull_hdr;
 
wire        hotjoin_request;
wire [70:0] dt_wr_data;
wire        valid_adrs;
 
assign hdr_active = start_hdr | hdr_busy;
assign sdr_active =
    ((cmd_mode == 0) || (cmd_mode == 1 && hdr_pending) // HDR address phase
    )
& ~hdr_active
& ~(cmd_type == 2'b11)
& ~(sdr_ccc_code == 8'h07) & ~daa_busy;

 
assign push_pull1 =
    hdr_active      ? 1'b1 :   // HDR always push-pull
    sdr_active      ? sdr_push_pull :
                      daa_push_pull;
 
assign s_r1 =
    hdr_active      ? 1'b0 :
    sdr_active      ? start_r1 :
                      start_r2;
//IBI
wire  ibi_be_valid;
wire  ibi_be_rw;
wire  ibi_be_done;
reg   ibi_active;
wire  ibi_tbit;
wire  ibi_reg_valid;
 
assign bus_idle = ~sdr_busy && ~daa_busy && ~be_busy1 && !hdr_busy1;
 
always @(posedge clk or negedge rst_n) begin
  if (!rst_n)
    ibi_active <= 1'b0;
 
  else if (ibi_request1)
    ibi_active <= 1'b1;
 
  else if (ibi_valid || hotjoin_req)  
    ibi_active <= 1'b0;
end
 
assign ibi_request     = ibi_request1 &  device_known;
assign hotjoin_request = hotjoin_req & ~device_known;
assign start_daa_final = start_daa | hotjoin_req;
 
////////////////////////////////////////////////////////////////////////////////
assign be_rw =
    ibi_active  ? ibi_be_rw :
    sdr_active  ? sdr_be_rw :
                  daa_be_rw;
 
assign be_valid =
    ibi_active  ? ibi_be_valid :
    sdr_active  ? sdr_valid :
                  daa_valid;
 
assign be_tx_data1 =
    daa_busy      ? daa_be_tx_data :
    ibi_active    ? 8'h00 :
    sdr_active    ? sdr_be_tx_data :
                    daa_be_tx_data;
assign ibi_be_done = be_done1;
// Tri state (pulled up)
assign scl_i       = scl_oe        ? scl_o          : 1'bz;
assign sda_i       = sda_oe        ? sda_o          : 1'bz;
 
parameter integer CLK_FREQ_HZ = 100_000_000;
wire sda_o_sdr;
wire sda_oe_sdr;
wire sda_o_hdr;
wire sda_oe_hdr;
 
assign sda_o =
    hdr_active ? sda_o_hdr :
                 sda_o_sdr;
 
assign sda_oe =
    hdr_active ? sda_oe_hdr :
                 sda_oe_sdr;
assign scl_start_final =
    hdr_active ? hdr_start_sig : start;
assign scl_stop_final =
    hdr_active ? hdr_stop_sig : stop;
 
// Instantiate REG block
i3c_reg_interface x1(
  .clk            (clk),
  .rst_n          (rst_n),  
  .wr_en          (wr_en),
  .rd_en          (rd_en),
  .addrs          (addrs),
  .w_reg_data     (w_reg_data),
  .w_data         (w_data), 
  .rd_data        (rd_data),
  .r_data         (r_data), 
  .Tx_wr_en       (tx_wr_en),
  .Tx_wdata       (tx_wdata),//Write to Tx_fifo
  .Rx_rd_en       (rx_rd_en),
  .Rx_rdata       (rx_rdata),//Read from Rx_fifo ...slave
  .CMD_BUSY       (cmd_busy),
  .SDR_BUSY       (sdr_busy),
  .SDR_DONE       (sdr_done),
  .DAA_DONE       (daa_done),
  .DAA_BUSY       (daa_busy ),
  .Tx_FULL        (tx_full),
  .Tx_EMPTY       (tx_empty),
  .Rx_FULL        (rx_full),
  .Rx_EMPTY       (rx_empty),
  .SDR_BIT_DONE   (be_done1),
  .SDR_BIT_BUSY   (be_busy1),
  .BUS_IDLE       (bus_idle),
  .HDR_BUSY       (hdr_busy),
  .HDR_DONE       (hdr_done),
  .HDR_BIT_DONE   (hdr_done1),
  .HDR_BIT_BUSY   (hdr_busy1),
  .IRQ            (irq),
  .daa_dyn_addr   (dyn_addr),
  .cmd_dc_type    (cmd_dc_type),
  .cmd_start      (cmd_start), 
  .cmd_type       (cmd_type),
  .cmd_mode       (cmd_mode),
  .cmd_addr       (cmd_addr),
  .cmd_ccc        (cmd_ccc),
  .cmd_len        (cmd_len),
  .cmd_dir        (cmd_dir),
  .cmd_dev_type   (cmd_dev_type) 
);
 
// Instantiate CMD block
i3c_cmd_ctrl x2(
  .clk              (clk),
  .rst_n            (rst_n), 
  .cmd_start        (cmd_start),
  .cmd_type         (cmd_type),
  .cmd_mode         (cmd_mode),
  .cmd_addr         (cmd_addr),
  .cmd_ccc          (cmd_ccc),
  .cmd_len          (cmd_len),
  .cmd_dir          (cmd_dir),  
  .start_sdr        (start_sdr),
  .sdr_addr         (sdr_addr),
  .sdr_len          (sdr_len),
  .sdr_dir          (sdr_dir1),
  .sdr_is_ccc       (sdr_is_ccc), 
  .sdr_ccc_code     (sdr_ccc_code),
  .sdr_ccc_is_read  (sdr_ccc_is_read),
  .sdr_read_len     (sdr_read_len),
  .sdr_has_restart  (sdr_has_restart),
  .start_daa        (start_daa),
  .start_hdr        (start_hdr),
  .sdr_done         (sdr_done),
  .daa_done         (daa_done),
  .hdr_done         (hdr_done),
  .cmd_busy         (cmd_busy),
  .be_done          (be_done1),
  .hdr_pending      (hdr_pending),
  .shift_hold       (shift_hold),
  .rstdaa_done      (rstdaa_done)
);
 
// Instantiate FIFOs
i3c_Tx_FIFO x3(
  .clk            (clk),
  .rst_n          (rst_n),
  .wr_en          (tx_wr_en),
  .din            (tx_wdata),   
  .rd_en          (tx_ready),
  .dout           (tx_rdata),
  .valid          (tx_valid),
  .full           (tx_full),
  .empty          (tx_empty)
);
 
i3c_Rx_FIFO x4 ( 
  .clk            (clk),
  .rst_n          (rst_n),        
  .wr_en          (rx_valid),
  .din            (rx_wdata),  
  .rd_en          (rx_rd_en),
  .dout           (rx_rdata),
  .valid          (rx_ready),  
  .full           (rx_full),
  .empty          (rx_empty) 
);
 
//SDR_FSM//////////////////
i3c_sdr_fsm x5 (
  .clk              (clk),
  .rst_n            (rst_n),
  // Command
  .start            (start_sdr),
  .addr             (sdr_addr),
  .dir              (sdr_dir1),
  .len              (sdr_len),
  .sdr_read_len     (sdr_read_len),
  .dev_type         (cmd_dev_type),
  // TX FIFO
  .tx_data          (tx_rdata),
  .tx_valid         (tx_valid),
  .tx_ready         (tx_ready1),
  // RX FIFO
  .rx_data          (rx_wdata1),
  .rx_valid         (rx_valid1),
  .rx_ready         (rx_ready),
  // Bit engine interface
  .be_valid         (sdr_valid),
  .be_rd_wr         (sdr_be_rw),
  .be_tx_data       (sdr_be_tx_data),
  .last_byte        (last_byte), //from bit_engine
  .be_rx_data       (be_rx_data),
  .be_busy          (be_busy1),
  .be_done          (be_done1), 
  .be_nack          (be_nack),
  // Status
  .busy             (sdr_busy),
  .done             (sdr_done),
  .error            (sdr_error),
  .parity_error     (parity_error), //from bit
  .arbitration_lost (arbitration_lost), //from bit
  .sdr_has_restart  (sdr_has_restart),
  .s_r              (start_r1),
  .push_pull        (sdr_push_pull),
  .sdr_addr         (sdr_addr1),
  .dt_is_i3c        (is_i3c1),  // Device table
  .dyn_addr         (dyn_addr),
  .start_hdr        (hdr_start_sig),
  .cmd_mode         (cmd_mode),
  .sdr_ccc_code     (sdr_ccc_code),
  .sdr_is_ccc       (sdr_is_ccc),
  .pid_done         (pid_done),
  .shift_done       (shift_done)
);
 
//DAA_FSM/////////////////
i3c_daa_fsm x6 (
  .clk            (clk),
  .rst_n          (rst_n),
  // Control from command FSM / host
  .start_daa      (start_daa_final),
  .daa_done       (daa_done),
  .start_r        (start_r2),
  .push_pull      (daa_push_pull),
  .valid          (daa_valid),
  .rd_wr          (daa_be_rw),
  .tx_data        (daa_be_tx_data),
  .arb_mode       (arb_mode1),
  .sda_i          (sda_i),
  .scl_i          (scl_i),
  .busy           (be_busy1),
  .nack           (be_nack),
  .be_done        (be_done1),
  .pid            (pid),
  .bcr            (bcr),
  .dcr            (dcr),
  .dyn_addr       (dyn_addr),
  .dt_en          (dt_en),
  .daa_busy       (daa_busy),
  .dt_wr_data     (dt_wr_data)
  );
 
//BIT ENGINE + START?STOP GENERATOR
i3c_device_table x7(
  .clk            (clk),
  .rst_n          (rst_n),
  .wr_en          (dt_en), // write enable
  .wr_data        (dt_wr_data),
  .sdr_addr       (sdr_addr1),
  .is_i3c_dyn     (is_i3c1),
  .lookup_en      (lookup_en),
  .lookup_addr    (lookup_addr),
  .device_known   (device_known),
  .rstdaa_done    (rstdaa_done),
  .ibi_addr_cap_en(ibi_addr_cap_en),
  .ibi_addr_cap   (ibi_addr_cap),
  .ibi_ack_ok     (ibi_ack_ok)
);
 
i3c_bit_engine x8 (
  .clk              (clk),
  .rst_n            (rst_n),
  .valid            (be_valid),
  .rd_wr            (be_rw),
  .tx_data          (be_tx_data1),
  .rx_data          (be_rx_data),
  .arb_mode         (arb_mode1),
  .s_r              (s_r1), 
  .scl_i            (scl_i),     // sampled SCL
  .sda_i            (sda_i),
  .sda_o            (sda_o_sdr),
  .sda_oe           (sda_oe_sdr),
  .busy             (be_busy1),
  .nack             (be_nack),
  .push_pull        (push_pull1),
  .txn_done         (be_done1),
  .start_done       (start),
  .stop_done        (stop),
  .last_byte        (last_byte),
  .parity_error     (parity_error),
  .arbitration_lost (arbitration_lost),
  .shift_hold       (shift_hold),
  .pid_done         (pid_done),
  .shift_done       (shift_done),
  .ibi_adrs_active  (ibi_adrs_active),
  .ibi_active       (ibi_active),
  .ibi_tbit         (ibi_tbit),
  .hotjoin_req      (hotjoin_req),
  .valid_adrs       (valid_adrs),
  .ibi_addr_cap_en(ibi_addr_cap_en),
  .ibi_addr_cap   (ibi_addr_cap),
  .ibi_ack_ok     (ibi_ack_ok)
);
//SCL GENERATOR/////////////////
i3c_scl_gen x9 (
  .clk          (clk),
  .rst_n        (rst_n),
  .scl_start    (scl_start_final),
  .scl_stop     (scl_stop_final), 
  .push_pull    (push_pull1), 
  .scl_hz_cfg   (12500000),
  .scl_o        (scl_o),   
  .scl_oe       (scl_oe) 
);
 
i3c_ibi_detector x10(
  .clk          (clk),
  .rst_n        (rst_n),
  .scl          (scl_i),
  .sda          (sda_i),
  .bus_idle     (bus_idle),
  .ibi_request  (ibi_request1)
);
 
i3c_ibi_fsm x11(
  .clk              (clk),
  .rst_n            (rst_n),
  .ibi_request      (ibi_request1),
// bit engine interface
  .be_valid         (ibi_be_valid),//op
  .be_rd_wr         (ibi_be_rw),//op
  .be_rx_data       (be_rx_data),
  .be_busy          (be_busy1),
  .be_done          (be_done1),
  .ibi_addr         (ibi_addr),
  .ibi_payload      (ibi_payload),
  .ibi_valid        (ibi_valid),
  .lookup_en        (lookup_en),
  .lookup_addr      (lookup_addr),
  .device_known     (device_known),
  .hotjoin_req      (hotjoin_req),
  .ibi_adrs_active  (ibi_adrs_active),
  .daa_done         (daa_done),
  .ibi_tbit         (ibi_tbit),
  .ibi_reg_valid    (ibi_reg_valid),
  .nvalid_adrs      (valid_adrs)
);
 
i3c_ibi_regs x12(
  .clk            (clk),
  .rst_n          (rst_n),
  .ibi_addr       (ibi_addr),
  .ibi_payload    (ibi_payload),
  .ibi_reg_valid  (ibi_reg_valid),
  .irq            (irq),
  .irq_addr       (irq_addr),
  .irq_payload    (irq_payload)
);
//hdr FSM
i3c_hdr_fsm x13 (
  .clk          (clk),
  .rst_n        (rst_n),
  .start_hdr    (start_hdr),
  .hdr_len      (sdr_len),
  .hdr_dir      (sdr_dir1),
  .tx_data      (tx_rdata),
  .tx_valid     (tx_valid),
  .tx_ready     (tx_ready2),
  .rx_data      (rx_wdata2),
  .rx_valid     (rx_valid2),
  .rx_ready     (rx_ready),
  .hdr_valid    (hdr_valid),
  .hdr_rw       (hdr_rw),
  .hdr_tx_data  (hdr_tx_data),
  .hdr_rx_data  (hdr_rx_data),
  .hdr_done     (hdr_done1),
  .hdr_busy     (hdr_busy1),
  .hdr_start    (hdr_start_sig),
  .hdr_stop     (hdr_stop_sig),
  .busy         (hdr_busy),
  .done         (hdr_done)
);
 
i3c_hdr_ddr_engine x14 (
  .clk          (clk),
  .rst_n        (rst_n),
  .valid        (hdr_valid),
  .rw           (hdr_rw),
  .tx_data      (hdr_tx_data),
  .rx_data      (hdr_rx_data),
  .scl_i        (scl_i),
  .sda_i        (sda_i),
  .sda_o        (sda_o_hdr),
  .sda_oe       (sda_oe_hdr),
  .hdr_start    (hdr_start_sig),
  .hdr_stop     (hdr_stop_sig),
  .busy         (hdr_busy1),
  .done         (hdr_done1)
);
 
endmodule
