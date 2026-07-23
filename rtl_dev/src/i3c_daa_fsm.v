module i3c_daa_fsm (
  input   wire          clk,
  input   wire          rst_n,

  input   wire          start_daa,
  output  reg           daa_done,
  output  reg           start_r,
  output  reg           push_pull,

  output  reg           valid,
  output  reg           rd_wr,
  output  reg   [7:0]   tx_data,
  output  reg           arb_mode,
  input   wire          sda_i,
  input   wire          scl_i,
  input   wire          busy,
  input   wire          nack,
  input   wire          be_done,
  output  reg           daa_busy,

  output  reg   [47:0]  pid,
  output  reg   [7:0]   bcr,
  output  reg   [7:0]   dcr,
  output  reg   [6:0]   dyn_addr,
  output  reg           dt_en,
  output  reg   [70:0]  dt_wr_data
);

localparam [4:0]
  IDLE        = 5'd0,
  SEND_7E_W   = 5'd1,
  SEND_ENTDAA = 5'd2,
  REP_START   = 5'd3,
  SEND_7E_R   = 5'd4,
  ARB_BITS    = 5'd5,
  ASSIGN      = 5'd6,
  LOOP        = 5'd7,
  STOP        = 5'd8;

reg   [4:0]   state, next;
reg   [6:0]   bit_cnt;
reg   [63:0]  arb_shift;

wire          parity = ~^dyn_addr;

reg           scl_q, scl_qq;

wire scl_rise = (scl_q == 1'b1) && (scl_qq == 1'b0);
wire scl_fall = (scl_q == 1'b0) && (scl_qq == 1'b1);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    scl_q  <= 1'b1;
    scl_qq <= 1'b1;
  end else begin
    scl_q  <= scl_i;
    scl_qq <= scl_q;
  end
end

always @(posedge clk or negedge rst_n) begin
  if (!rst_n)
    state <= IDLE;
  else
    state <= next;
end

always @* begin
  case (state)

    IDLE        : next = start_daa          ? SEND_7E_W : IDLE;

    SEND_7E_W   : next = be_done && !busy
                        ? nack ? STOP : SEND_ENTDAA
                        : SEND_7E_W;

    SEND_ENTDAA : next = be_done && !busy
                        ? nack ? STOP : REP_START
                        : SEND_ENTDAA;

    REP_START   : next = !busy && start_r
                        ? SEND_7E_R
                        : REP_START;

    SEND_7E_R   : next = be_done
                        ? nack ? STOP : ARB_BITS
                        : SEND_7E_R;

    ARB_BITS    : next = (bit_cnt == 7'd0)
                        ? ASSIGN
                        : ARB_BITS;

    ASSIGN      : next = be_done && !busy
                        ? LOOP
                        : ASSIGN;

    LOOP        : next = nack ? STOP : REP_START;                      //next=REP_START;     

    STOP        : next = IDLE;

    default     : next = IDLE;

  endcase
end

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    valid      <= 'b0;
    rd_wr      <= 'b0;
    tx_data    <= 'b0;
    arb_mode   <= 'b0;
    bit_cnt    <= 7'd64;
    arb_shift  <= 'b0;
    pid        <= 'b0;
    bcr        <= 'b0;
    dcr        <= 'b0;
    dyn_addr   <= 7'h08;
    daa_done   <= 'b0;
    start_r    <= 'b0;
    push_pull  <= 'b0;
    dt_en      <= 'b0;
    dt_wr_data <= 'b0;

  end else begin

    valid    <= 1'b0;
    start_r  <= 1'b0;
    daa_done <= 1'b0;
    dt_en    <= 1'b0;

    daa_busy <= (state != IDLE);

    case (state)

      SEND_7E_W: begin
        push_pull <= 1'b0;
        valid     <= 1'b1;
        rd_wr     <= 1'b0;
        tx_data   <= {7'h7E, 1'b0};

        if (be_done && !busy)
          $display("[%0t] SEND_7E_W completed, NACK=%0b",
                   $time, nack);
      end

      SEND_ENTDAA: begin
        push_pull <= 1'b1;
        valid     <= !busy && !be_done ? 1'b1 : 1'b0;
        rd_wr     <= 1'b0;
        tx_data   <= 8'h07;

        if (be_done && !busy)
          $display("[%0t] SEND_ENTDAA completed, NACK=%0b",
                   $time, nack);
      end

      REP_START: begin
        start_r <= 1'b1;

        if (!busy)
          $display("[%0t] REP_START completed",
                   $time);
      end

      SEND_7E_R: begin
        push_pull <= 1'b0;
        valid     <= !busy && !be_done ? 1'b1 : 1'b0;
        rd_wr     <= 1'b0;
        tx_data   <= {7'h7E, 1'b1};
        arb_mode  <= 1'b0;

        if (be_done)
          $display("[%0t] SEND_7E_R completed, NACK=%0b",
                   $time, nack);
      end

      ARB_BITS: begin
        push_pull <= 1'b1;
        arb_mode  <= 1'b1;

        if (scl_rise) begin
          arb_shift <= {arb_shift[62:0], sda_i};
          bit_cnt   <= bit_cnt - 1;

          if (bit_cnt == 7'd1)
            $display("[%0t] ARB_BITS completed, Captured Data = %h",
                     $time, {arb_shift[62:0], sda_i});
        end
      end

      ASSIGN: begin
        arb_mode  <= 1'b0;
        pid       <= arb_shift[63:16];
        bcr       <= arb_shift[15:8];
        dcr       <= arb_shift[7:0];

        valid     <= !busy ? 1 : 0;
        rd_wr     <= 1'b0;
        tx_data   <= {dyn_addr, parity};

        dt_wr_data <= {
                         arb_shift[62:16],
                         arb_shift[15:8],
                         arb_shift[7:0],
                         sda_i,
                         dyn_addr
                       };

        bit_cnt   <= 7'd64;

        if (!busy)
          $display("[%0t] ASSIGN completed : PID=%h BCR=%h DCR=%h DYN_ADDR=%h",
                   $time,
                   arb_shift[63:16],
                   arb_shift[15:8],
                   arb_shift[7:0],
                   dyn_addr);
      end

      LOOP: begin
        dt_en <= 1'b1;

        if(nack) begin
          daa_done <= 1'b1;
          $display("[%0t] LOOP completed : No more devices responded, DAA Done",
                   $time);
        end
        else begin
          $display("[%0t] LOOP completed : Next Dynamic Address = 0x%0h",
                   $time, dyn_addr + 1'b1);
        end

        dyn_addr <= dyn_addr + 1;
      end

      STOP: begin
        valid    <= 1'b0;
        daa_done <= 1'b1;

        $display("[%0t] STOP state reached. DAA Finished.",
                 $time);
      end

    endcase
  end
end

endmodule
