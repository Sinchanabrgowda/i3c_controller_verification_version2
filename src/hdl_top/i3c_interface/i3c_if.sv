

interface i3c_if #(parameter int NO_OF_TARGETS = 1)
    (input pclk, input areset, inout SCL, inout SDA);

  import i3c_globals_pkg::*;

  logic scl_i;
  logic sda_i;

  logic [NO_OF_TARGETS-1:0] scl_o;
  logic [NO_OF_TARGETS-1:0] scl_oen;  // 1 = drive, 0 = tristate
  logic [NO_OF_TARGETS-1:0] sda_o;
  logic [NO_OF_TARGETS-1:0] sda_oen;  // 1 = drive, 0 = tristate

  genvar gi;
  generate
    for (gi = 0; gi < NO_OF_TARGETS; gi++) begin : tgt_drive
      assign SCL = (scl_oen[gi] && !scl_o[gi]) ? 1'b0 : 1'bz;
      assign SDA = (sda_oen[gi] && !sda_o[gi]) ? 1'b0 : 1'bz;
    end
  endgenerate

  assign scl_i = SCL;
  assign sda_i = SDA;

endinterface : i3c_if

