// LTPI common definitions.
// Reference: OCP DC-SCM 2.0 LTPI Specification v1.0 (spec/OCP_DC-SCM_2.0_LTPI_v1.0.pdf)
package ltpi_pkg;

    // 8b values of the comma control symbols (spec Table 19).
    localparam logic [7:0] COMMA_LINK = 8'hBC;  // K28.5 - Link Detect / Link Speed
    localparam logic [7:0] COMMA_CFG  = 8'hDC;  // K28.6 - Advertise / Configure / Accept
    localparam logic [7:0] COMMA_OP   = 8'hFC;  // K28.7 - Operational frames

    // Frame Subtype field values (spec Tables 20, 25).
    localparam logic [7:0] SUB_LINK_DETECT = 8'h00;
    localparam logic [7:0] SUB_LINK_SPEED  = 8'h01;
    localparam logic [7:0] SUB_ADVERTISE   = 8'h00;
    localparam logic [7:0] SUB_CONFIGURE   = 8'h01;
    localparam logic [7:0] SUB_ACCEPT      = 8'h02;

    localparam int unsigned FRAME_LEN = 16;     // bytes per frame incl. comma + CRC

    typedef enum logic [2:0] {
        FRAME_INVALID     = 3'd0,
        FRAME_LINK_DETECT = 3'd1,
        FRAME_LINK_SPEED  = 3'd2,
        FRAME_ADVERTISE   = 3'd3,
        FRAME_CONFIGURE   = 3'd4,
        FRAME_ACCEPT      = 3'd5,
        FRAME_OPERATIONAL = 3'd6,   // Default I/O frame (subtype 0h00)
        FRAME_OP_DATA     = 3'd7    // Default Data frame (subtype 0h01)
    } frame_type_t;

    // Data channel commands (byte 5 of the Data Frame payload)
    localparam logic [7:0] DC_CMD_WRITE_REQ  = 8'h01;
    localparam logic [7:0] DC_CMD_READ_REQ   = 8'h02;
    localparam logic [7:0] DC_CMD_WRITE_RESP = 8'h81;
    localparam logic [7:0] DC_CMD_READ_RESP  = 8'h82;
    localparam logic [7:0] DC_CMD_ERROR      = 8'h04;   // Table 12: CRC err

    typedef enum logic [2:0] {
        ST_DETECT_ALIGN = 3'd0,   // Link Detect - frame alignment part (Fig. 27)
        ST_DETECT       = 3'd1,   // Link Detect - main part
        ST_SPEED        = 3'd2,   // Link Speed
        ST_ADV_ALIGN    = 3'd3,   // Advertise - re-alignment at operational freq
        ST_ADV          = 3'd4,   // Advertise - main part
        ST_CFG_ACC      = 3'd5,   // SCM: Configure / HPM: Accept
        ST_OPERATIONAL  = 3'd6    // Operational mode
    } link_state_t;

    // Operational frame subtypes (spec Table 32).
    localparam logic [7:0] SUB_OP_IO   = 8'h00;
    localparam logic [7:0] SUB_OP_DATA = 8'h01;

    // I2C/SMBus relay events (spec Table 10).
    localparam logic [3:0] I2C_EV_IDLE       = 4'b0000;
    localparam logic [3:0] I2C_EV_START      = 4'b0001;
    localparam logic [3:0] I2C_EV_START_RCVD = 4'b0010;
    localparam logic [3:0] I2C_EV_STOP       = 4'b0011;
    localparam logic [3:0] I2C_EV_STOP_RCVD  = 4'b0100;
    localparam logic [3:0] I2C_EV_DATA_RCVD  = 4'b0101;
    localparam logic [3:0] I2C_EV_DATA0      = 4'b0110;
    localparam logic [3:0] I2C_EV_DATA1      = 4'b0111;
    localparam logic [3:0] I2C_EV_START_ECHO = 4'b1000;
    localparam logic [3:0] I2C_EV_STOP_ECHO  = 4'b1001;
    localparam logic [3:0] I2C_EV_DATA0_ECHO = 4'b1010;
    localparam logic [3:0] I2C_EV_DATA1_ECHO = 4'b1011;
    localparam logic [3:0] I2C_EV_DRCVD_ECHO = 4'b1100;
    // 0b1101-0b1111 reserved

    // Speed capability bits (spec Table 21): [7:0] = x1,x2,x3,x4,x6,x8,x10,x12
    // (25..300MHz), [11:8] = x16,x24,x32,x40 (400,600,800,1000MHz), [15] DDR.
    localparam logic [15:0] CAP_25M_SDR  = 16'h0001;  // mandatory base
    localparam logic [15:0] CAP_200M_SDR = 16'h0020;  // x8
    localparam logic [15:0] CAP_400M_SDR = 16'h0100;  // x16
    localparam logic [15:0] CAP_DDR      = 16'h8000;  // DDR-capable I/O

    // Project default: 25MHz base + 200MHz SDR + 400MHz SDR + DDR-capable,
    // i.e. peers negotiate up to 400MHz DDR (800Mbps) and can fall back to
    // 200MHz/400MHz SDR against SDR-only implementations.
    localparam logic [15:0] CAPS_DEFAULT = CAP_25M_SDR | CAP_200M_SDR
                                         | CAP_400M_SDR | CAP_DDR;

    // The CRC-8 update function lives in ltpi_crc8_func.svh, included inside
    // each module body: yosys cannot elaborate package function bodies that
    // are called from module context (the result wire ends up undriven).

endpackage
