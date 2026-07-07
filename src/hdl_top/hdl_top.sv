
`ifndef HDL_TOP_INCLUDED_
`define HDL_TOP_INCLUDED_

import i3c_globals_pkg::*;
import apb_global_pkg::*;

module hdl_top;

  bit clk;
  bit rst;

  wire I3C_SCL;
  wire I3C_SDA;

  wire pclk;
  wire preset_n;

  assign pclk    = clk;
  assign preset_n = rst;

  logic        wr_en;
  logic        rd_en;
  logic [6:0]  addrs;
  logic [31:0] w_reg_data;
  logic [7:0]  w_data;
  logic [31:0] rd_data;
  logic [7:0]  r_data;

  logic scl_o;
  wire  sda_o;
  logic sda_oe;

  initial begin
    $display("HDL TOP – multi-slave (%0d targets)", NO_OF_TARGETS);
  end

  initial begin
    clk = 1'b0;
    forever #10 clk = ~clk;
  end

  initial begin
    rst = 1'b1;
    repeat (2) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);
    rst = 1'b1;
  end

  apb_if apb_intf(.pclk(pclk), .preset_n(preset_n));

  pullup p_scl (I3C_SCL);
  pullup p_sda (I3C_SDA);

  i3c_if #(.NO_OF_TARGETS(NO_OF_TARGETS)) intf_i3c (
    .pclk   (clk),
    .areset (rst),
    .SCL    (I3C_SCL),
    .SDA    (I3C_SDA)
  );

  apb_i3c_wrapper wrapper (
    .apb        (apb_intf),
    .wr_en      (wr_en),
    .rd_en      (rd_en),
    .addrs      (addrs),
    .w_reg_data (w_reg_data),
    .w_data     (w_data),
    .rd_data    (rd_data),
    .r_data     (r_data)
  );

  I3C_TOP dut (
    .clk        (clk),
    .rst_n      (rst),
    .wr_en      (wr_en),
    .rd_en      (rd_en),
    .addrs      (addrs),
    .w_reg_data (w_reg_data),
    .w_data     (w_data),
    .rd_data    (rd_data),
    .r_data     (r_data),
    .scl_i      (I3C_SCL),
    
    .sda_i      (I3C_SDA),
    .sda_o      (sda_o),
    .sda_oe     (sda_oe)
  );
//.scl_o      (scl_o),
  apb_master_agent_bfm apb_master_agent_bfm_h (apb_intf);

  genvar i;
  generate
    for (i = 0; i < NO_OF_TARGETS; i++) begin : gen_target_bfm
      i3c_target_agent_bfm #(
        .target_ID(i)
      ) i3c_target_agent_bfm_inst (
        .intf (intf_i3c)
      );
    end
  endgenerate

  initial begin
    $dumpfile("i3c_avip.vcd");
    $dumpvars();
  end

endmodule : hdl_top

`endif

