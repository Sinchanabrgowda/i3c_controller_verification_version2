
`ifndef I3C_GLOBALS_PKG_INCLUDED_
`define I3C_GLOBALS_PKG_INCLUDED_

package i3c_globals_pkg;

  parameter int NO_OF_CONTROLLERS = 1;
  parameter int NO_OF_TARGETS     = 4;   

  parameter int NO_OF_REG              = 1;
  parameter int DATA_WIDTH             = 8;
  parameter int TARGET_ADDRESS_WIDTH   = 7;
  parameter int REGISTER_ADDRESS_WIDTH = 8;
  parameter int MAXIMUM_BITS           = 1024;
  parameter int MAXIMUM_BYTES          = MAXIMUM_BITS / DATA_WIDTH;

  parameter TARGET0_ADDRESS = 7'b110_1000;   // 7'h68
  parameter TARGET1_ADDRESS = 7'b110_1100;   // 7'h6C
  parameter TARGET2_ADDRESS = 7'b111_1100;   // 7'h7C
  parameter TARGET3_ADDRESS = 7'b100_1100;   // 7'h4C

  parameter bit TRISTATE_BUF_ON  = 1;   // driving
  parameter bit TRISTATE_BUF_OFF = 0;   // high-Z

  parameter BUS_IDLE_TIME = 1;
  parameter BUS_FREE_TIME = 1;

  parameter bit [6:0] I3C_BROADCAST_ADDR = 7'h7E;
  parameter bit [7:0] ENTDAA_CCC_CODE    = 8'h07;
  parameter bit [7:0] BCAST_ADDR_WRITE   = 8'hFC;  // {7'h7E, W=0}
  parameter bit [7:0] BCAST_ADDR_READ    = 8'hFD;  // {7'h7E, R=1}
  parameter int       DAA_ARB_BIT_COUNT  = 64;
  parameter bit [6:0] DAA_FIRST_DYN_ADDR = 7'h08;

  parameter bit [1:0] CMD_TYPE_DAA = 2'd3;
  parameter bit [1:0] CMD_TYPE_SDR = 2'b00;
  parameter bit [1:0] CMD_TYPE_CCC = 2'b10;

  parameter bit [47:0] TARGET0_PID = 48'h00_AABB_CC00_01;
  parameter bit [47:0] TARGET1_PID = 48'h00_AABB_CC00_02;
  parameter bit [47:0] TARGET2_PID = 48'h00_AABB_CC00_03;
  parameter bit [47:0] TARGET3_PID = 48'h00_AABB_CC00_04;
  parameter bit [47:0] TARGET4_PID = 48'h00_AABB_CC00_05;


  parameter bit [7:0]  DEFAULT_BCR = 8'h00;
  parameter bit [7:0]  TARGET0_DCR = 8'hC2;
  parameter bit [7:0]  TARGET1_DCR = 8'hC3;
  parameter bit [7:0]  TARGET2_DCR = 8'hC4;
  parameter bit [7:0]  TARGET3_DCR = 8'hC5;
  parameter bit [7:0]  TARGET4_DCR = 8'hC6;


  typedef enum bit {
    MSB_FIRST = 1'b0,
    LSB_FIRST = 1'b1
  } dataTransferDirection_e;

  typedef enum bit {
    TRUE  = 1'b1,
    FALSE = 1'b0
  } hasCoverage_e;

  typedef enum bit {
    WRITE = 1'b0,
    READ  = 1'b1
  } operationType_e;

  typedef enum bit [1:0] {
    ONLY_WRITE = 2'b00,
    ONLY_READ  = 2'b01,
    WRITE_READ = 2'b10
  } writeReadMode_e;

  typedef enum bit {
    SDR = 1'b0,
    DAA = 1'b1
  } txn_type_e;

  typedef struct {
    bit [TARGET_ADDRESS_WIDTH-1:0]   targetAddress;
    bit                              operation;
    bit                              targetAddressStatus;
    bit                              writeDataStatus[MAXIMUM_BYTES];
    bit                              readDataStatus[MAXIMUM_BYTES];
    bit [DATA_WIDTH-1:0]             writeData[MAXIMUM_BYTES];
    bit [DATA_WIDTH-1:0]             readData[MAXIMUM_BYTES];
    int                              no_of_i3c_bits_transfer;
    bit [REGISTER_ADDRESS_WIDTH-1:0] register_address;
    bit                              txn_type;
    bit [47:0]                       pid;
    bit [7:0]                        bcr;
    bit [7:0]                        dcr;
    bit [6:0]                        dynamic_address;
    bit                              daa_ack;
  } i3c_transfer_bits_s;

  typedef struct {
    dataTransferDirection_e           dataTransferDirection;
    bit                               operation;
    int                               clockRateDividerValue;
    bit [TARGET_ADDRESS_WIDTH-1:0]    targetAddress;
    bit [DATA_WIDTH-1:0]              defaultReadData;
    bit [47:0]                        pid;
    bit [7:0]                         bcr;
    bit [7:0]                         dcr;
    bit                               daa_accept_address;
  } i3c_transfer_cfg_s;

  typedef enum int {
    RESET_DEACTIVATED,
    RESET_ACTIVATED,
    IDLE,
    FREE,
    START,
    ADDRESS,
    WR_BIT,
    ACK_NACK,
    WRITE_DATA,
    READ_DATA,
    STOP
  } i3c_fsm_state_e;

  typedef enum bit [3:0] {
    DAA_IDLE      = 4'd0,
    DAA_SEND_7E_W = 4'd1,
    DAA_ENTDAA    = 4'd2,
    DAA_REP_START = 4'd3,
    DAA_SEND_7E_R = 4'd4,
    DAA_ARB_BITS  = 4'd5,
    DAA_ASSIGN    = 4'd6,
    DAA_LOOP      = 4'd7,
    DAA_STOP      = 4'd8
  } daa_fsm_state_e;

  typedef enum bit [1:0] {
    POSEDGE = 2'b01,
    NEGEDGE = 2'b10
  } edge_detect_e;

  typedef enum bit {
    ACK  = 1'b0,
    NACK = 1'b1
  } acknowledge_e;

endpackage : i3c_globals_pkg

`endif

