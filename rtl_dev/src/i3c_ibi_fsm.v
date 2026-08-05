//IBO_FSM
module i3c_ibi_fsm(
      input  wire        clk,
      input  wire        rst_n,
      input  wire        ibi_request,
      // bit engine interface
      output reg         be_valid,
      output reg         be_rd_wr,
      input  wire [7:0]  be_rx_data,
      input  wire        be_busy,
      input  wire        be_done,
      // device table interface
      output reg         lookup_en,
      output reg [6:0]   lookup_addr,
      input  wire        device_known,
      output reg  [6:0]  ibi_addr,
      output reg  [7:0]  ibi_payload,
      output reg         ibi_valid,
      output reg         ibi_reg_valid,
      output reg         hotjoin_req,
      input  wire        daa_done,
      output reg         ibi_adrs_active,
      input  wire        ibi_tbit
);
 
//======================================================
// STATES
//======================================================
localparam  IDLE         = 4'd0,
            ACK_IBI      = 4'd1,
            READ_ADDR    = 4'd2,
            CHECK        = 4'd3,
            CHECK_WAIT   = 4'd4,
            READ_DATA    = 4'd5,
            DONE         = 4'd6,
            HOTJOIN      = 4'd7,
            READ_MORE    = 4'd8;
 
reg [3:0] state, next_state;
reg       hotjoin_detected;
 
always @(posedge clk or negedge rst_n) begin
  if (!rst_n)
    state <= IDLE;
  else
    state <= next_state;
end
 
always @(*) begin
  next_state = state;
  hotjoin_req = 0;
  case(state)
 
    IDLE: begin
      if (ibi_request)
        next_state = ACK_IBI;
    end
 
    ACK_IBI: begin
      if (!be_busy)
        next_state = READ_ADDR;
    end
 
    READ_ADDR: begin
      if (!be_busy && be_done)
        next_state = CHECK;
    end
 
    CHECK: begin
      next_state = CHECK_WAIT;
    end
 
    CHECK_WAIT: begin
      if (device_known)
        next_state = READ_DATA;
      else begin
        next_state = HOTJOIN;
        hotjoin_req = 1'b1;
        end
    end
 
    READ_DATA: begin
      if (!be_busy && be_done) begin
          if (ibi_tbit)
              next_state = READ_MORE;
          else
              next_state = DONE;
        end
    end
    READ_MORE: begin
      if(!be_busy)
          next_state = READ_DATA;
    end
    HOTJOIN: begin
    if(daa_done)
      next_state = DONE;
    end
 
    DONE: begin
      next_state = IDLE;
    end
 
    default:
      next_state = IDLE;
 
  endcase
end
 
always @(*) begin
  be_valid    = 0;
  ibi_adrs_active = 0;
  be_rd_wr    = 1'b1;
 
  lookup_en   = 0;
  lookup_addr = ibi_addr;
 
  ibi_valid   = 0;
// hotjoin_req = 0;
 
  case(state)
 
    READ_ADDR: begin
 
      if (!be_busy)
        be_valid = 1'b1;
      ibi_adrs_active=1'b1;
      be_rd_wr = 1'b1;
    end
    CHECK,
    CHECK_WAIT: begin
      lookup_en   = 1'b1;
      lookup_addr = ibi_addr;
      be_valid = 1'b0;
    end
 
    READ_DATA: begin
      be_valid=1;
      if (!be_busy)
        be_valid = 1'b0;
 
      be_rd_wr = 1'b1;
    end
    READ_MORE: begin
      be_valid = 1'b1;
      be_rd_wr = 1'b1;
    end
    HOTJOIN: begin
      be_rd_wr    = 1'b0;
     // hotjoin_req = 1'b1;
    end
 
    DONE: begin
      if (!hotjoin_detected) begin
        ibi_valid = 1'b1;
        be_rd_wr    = 1'b0;
      end
    end
 
  endcase
end
 
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    ibi_addr          <= 7'd0;
    ibi_payload       <= 8'd0;
    hotjoin_detected  <= 1'b0;
    ibi_reg_valid<=0;
  end
  else begin
 
    if (state == IDLE)
      hotjoin_detected <= 1'b0;
    if (state == READ_ADDR && be_done)
      ibi_addr <= be_rx_data[7:1];
 
    if (state == READ_DATA && be_done) begin
     ibi_payload <= be_rx_data;       
    end
    if(state==READ_DATA &&be_done)
            ibi_reg_valid<=1'b1;
     else
            ibi_reg_valid<=0;
    if (state == HOTJOIN)
      hotjoin_detected <= 1'b1;
 
  end
end
 
endmodule
