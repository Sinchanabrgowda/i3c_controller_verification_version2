//IBI_REGS
module i3c_ibi_regs(
 
input wire clk,
input wire rst_n,
 
input wire        ibi_reg_valid,
input wire [6:0]  ibi_addr,
input wire [7:0]  ibi_payload,
 
output reg        irq,
output reg [6:0]  irq_addr,
output reg [7:0]  irq_payload
 
);
 
always @(posedge clk or negedge rst_n) begin
 
  if(!rst_n) begin
    irq         <= 1'b0;
    irq_addr    <= 7'd0;
    irq_payload <= 8'd0;
  end
 
  else begin
 
    if(ibi_reg_valid) begin
 
      irq         <= 1'b1;
      irq_addr    <= ibi_addr;
      irq_payload <= ibi_payload;
 
    end
    else
      irq <= 1'b0;
 
  end
 
end
endmodule
