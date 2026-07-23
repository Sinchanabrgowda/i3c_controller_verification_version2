//fixed
module i3c_sdr_fsm (

  input  wire        clk,
  input  wire        rst_n,

  input  wire        start,
  input  wire [6:0]  addr,
  input  wire        dir,        // 0 = write, 1 = read
  input  wire [7:0]  len,
  input  wire [7:0]  sdr_read_len,
  input  wire        dev_type, //1=i3c stat ,0=i2c stat

  input  wire [7:0]  tx_data,
  input  wire        tx_valid,
  output reg         tx_ready,

  output reg  [7:0]  rx_data,
  output reg         rx_valid,
  input  wire        rx_ready,

  // Bit engine interface
  output reg         be_valid,
  output reg         be_rd_wr,
  output reg  [7:0]  be_tx_data,
  output reg         last_byte,

  input  wire [7:0]  be_rx_data,
  input  wire        be_busy,
  input  wire        be_done,
  input  wire        be_nack,
  input  wire        parity_error, 
  input  wire        arbitration_lost, 
  input  wire        sdr_has_restart ,  // indicates directed CCC read transaction
  input  wire        sdr_is_ccc,
  input  wire [7:0]  sdr_ccc_code,
  
  output reg         busy,
  output reg         done,
  output reg         error,
  output reg         s_r ,
  output reg         push_pull,
  output reg  [6:0]  sdr_addr,

  input  wire        dt_is_i3c,   // Device table
  input  wire [6:0]  dyn_addr,
  input  wire        start_hdr,
  input  wire        cmd_mode,
  input  wire        pid_done,
  input  wire        shift_done
);

localparam [3:0]
  IDLE        = 4'd0,
  SEND_ADDR   = 4'd1,
  WAIT_ADDR   = 4'd2,
  SEND_WR     = 4'd3,
  WAIT_WR     = 4'd4,
  RESTART     = 4'd5,
  SEND_ADDR_R = 4'd6,
  WAIT_ADDR_R = 4'd7,
  SEND_RD     = 4'd8,
  WAIT_RD     = 4'd9,
  STOP        = 4'd10,
  ERROR       = 4'd11;

  
reg  [3:0]  state, next_state;
reg  [7:0]  byte_cnt;
reg         push_pull_hold;
wire 	      is_i3c;

assign    is_i3c = dt_is_i3c || dev_type;

// STATE REGISTER
always @(posedge clk or negedge rst_n) begin
  if (!rst_n)
    state <= IDLE;
  else
    state <= next_state;
end

// BYTE COUNTER + RX CAPTURE
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    byte_cnt <= 8'd0;
    rx_valid <= 1'b0;
  end else begin
    rx_valid <= 1'b0;
    case (state)
      
      IDLE: begin
        if (start)begin
			    byte_cnt <= len;
			  end 
		  end
		
      RESTART: begin
        if (!be_busy)
        byte_cnt <= sdr_read_len;
      end

      WAIT_WR: begin      
        if (!be_busy && be_done)
          byte_cnt <= byte_cnt - 1'b1;
      end

      WAIT_RD: begin
        if ((!be_busy && be_done) || shift_done ) begin
          rx_data  <= be_rx_data;
          rx_valid <= 1'b1;
          byte_cnt <= byte_cnt - 1'b1;
        end
      end

      default: ;
    endcase
  end
end

// LAST BYTE GENERATION (T-BIT SUPPORT)
always @(*) begin
  last_byte = (byte_cnt == 8'd1);
end

// PUSH-PULL CONTROL (PER DEVICE)
always @(posedge clk or negedge rst_n) begin
  if (!rst_n)
    push_pull_hold <= 1'b0;
  else begin
    case (state)
      SEND_WR,
      WAIT_WR,
      SEND_RD,
      WAIT_RD:
        push_pull_hold <= is_i3c;  // I3C ? push-pull

      default:
        push_pull_hold <= 1'b0;       // I2C ? open-drain
    endcase
  end
end

always @(*) begin
  push_pull = push_pull_hold;
end

// NEXT STATE LOGIC
always @(*) begin

  next_state = state;
  be_valid   = 1'b0;
  be_rd_wr   = 1'b0;
  be_tx_data = 8'd0;
  tx_ready   = 1'b0;
  s_r        = 1'b0;

  busy  = (state != IDLE);
  done  = 1'b0;
  error = 1'b0;

  // GLOBAL ARBITRATION LOSS HANDLING

  if (arbitration_lost) begin
    next_state = ERROR;
  end else begin

    case (state)
      IDLE: begin
        if (start)
          next_state = SEND_ADDR;
      end

      SEND_ADDR: begin
      be_valid = 1;
      be_rd_wr = 0;

      //be_tx_data = is_i3c ? {dyn_addr, dir} : {addr, dir};
      be_tx_data = {addr, dir};
      sdr_addr   = addr;

      if (!be_busy)
        next_state = WAIT_ADDR;
    end

      WAIT_ADDR: begin
        if (!be_busy && be_done) begin
          if (is_i3c) begin
            if (parity_error)
              next_state = ERROR;
            else if (dir)
              next_state = SEND_RD;
            else
              next_state = SEND_WR;
          end 
          else begin
            if (be_nack)
              next_state = ERROR;
            else if (dir)
              next_state = SEND_RD;
            else
              next_state = SEND_WR;
          end
        end
      end

      SEND_WR: begin
        if(start_hdr || cmd_mode==1)
          next_state = STOP;
        else if (tx_valid) begin
          tx_ready   = 1'b1;
          be_valid   = 1'b1;
          be_rd_wr   = 1'b0;
         
        if (be_valid && !be_busy)   // wait till bit engine ready
          next_state = WAIT_WR;
        end
      end

      WAIT_WR: begin
      if(sdr_has_restart) begin
        be_tx_data = sdr_ccc_code;
      end
      else begin
       be_tx_data = tx_data;
       end
        if (!be_busy && be_done) begin
			     if (is_i3c && parity_error)
				   next_state = ERROR;

			     else if (sdr_has_restart && sdr_is_ccc)
				   next_state = RESTART;

			     else if (byte_cnt == 8'd1)
				   next_state = STOP;

			     else
				   next_state = SEND_WR;
			  end	
		  end
				
      RESTART: begin
  
      be_valid = 1'b1;
      be_rd_wr = 1'b0;
      // trigger restart in bit engine
      s_r = 1'b1;
  
      if (!be_busy)
        next_state = SEND_ADDR_R;
  
      end
	  
      SEND_ADDR_R: begin
      be_rd_wr   = 1'b0;
      be_valid   = 1'b1;
      be_tx_data = {addr, 1'b1};
    
      if (!be_busy)
        next_state = WAIT_ADDR_R;
  
      end
      
     WAIT_ADDR_R: begin
          if (!be_busy && be_done) begin
            if (is_i3c && parity_error)
              next_state = ERROR;
            else if (!is_i3c && be_nack)
              next_state = ERROR;
            else
              next_state = SEND_RD;
          end
     end
     
      SEND_RD: begin
      
       be_valid = 1'b1;
        be_rd_wr = 1'b1;
       if(start_hdr || cmd_mode==1)
          next_state = STOP;

       else if (!be_busy)
         next_state = WAIT_RD;
		 end

      WAIT_RD: begin
        be_rd_wr = 1'b1;
        if(pid_done)
            next_state = STOP;
            
        if (!be_busy && be_done) begin
          if (!rx_ready)
            next_state = ERROR;
          else if (byte_cnt == 8'd1)
            next_state = STOP;
          else
            next_state = SEND_RD;

			  end
	    end

      STOP: begin
        done       = 1'b1;
        next_state = IDLE;
      end

      ERROR: begin
        error      = 1'b1;
        done       = 1'b1;
        next_state = IDLE;
      end

      default: next_state = IDLE;

    endcase
  end
end

endmodule
