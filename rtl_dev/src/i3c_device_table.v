//device table
module i3c_device_table #(
  parameter A_WIDTH = 7,    
  parameter D_WIDTH = 71    // {PID[47:0], BCR[7:0], DCR[7:0], DYN_ADDR[6:0]}
)(
  input   wire                clk,
  input   wire                rst_n,
 
  input   wire                wr_en,
  input   wire [D_WIDTH-1:0]  wr_data,
 
  input   wire [6:0]          sdr_addr,
  output  reg                 is_i3c_dyn,
 
  input   wire                lookup_en,
  input   wire [6:0]          lookup_addr,
  output  reg                 device_known,
  input   wire                rstdaa_done,
  input   wire                ibi_addr_cap_en,   // 1-cycle pulse: 7-bit addr ready
  input   wire [6:0]          ibi_addr_cap,      // captured target address
  output  reg                 ibi_ack_ok 
);
 
  reg [D_WIDTH-1:0] mem [0:(1<<A_WIDTH)-1];
  reg [A_WIDTH-1:0] idx;
  integer i;
 
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      idx <= 0;
      for (i = 0; i < (1<<A_WIDTH); i = i + 1)
      mem[i] <= {D_WIDTH{1'b0}};
    end 
     else if (rstdaa_done) begin
    idx <= 0;
    for (i = 0;i < (1<<A_WIDTH);i = i + 1)
      mem[i] <= {D_WIDTH{1'b0}};
  end
    else begin
      if (wr_en) begin
        mem[idx] <= wr_data;
        idx      <= idx + 1'b1;
      end
    end
  end
 
 
  always @(*) begin
    is_i3c_dyn = 1'b0;
    for (i = 0; i < (1<<A_WIDTH); i = i + 1) begin
      if ((mem[i] != 0) && (mem[i][6:0] == sdr_addr))
        is_i3c_dyn = 1'b1;
    end
  end
 
  always @(*) begin
    device_known = 1'b0;
   // ibi_ack_ok   = 1'b0;
    if (lookup_en) begin
      for (i = 0; i < (1<<A_WIDTH); i = i + 1) begin
        if ((mem[i] != 0) && (mem[i][6:0] == lookup_addr))
          device_known = 1'b1;
      end
     end
     if (ibi_addr_cap_en) begin
     ibi_ack_ok   = 1'b0;
      for (i = 0; i < (1<<A_WIDTH); i = i + 1) begin
        if ((mem[i] != 0) && (mem[i][6:0] == ibi_addr_cap))
          ibi_ack_ok = 1'b1;
      end
    end
  end
endmodule
