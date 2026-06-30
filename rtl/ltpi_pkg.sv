// =============================================================================
// ltpi_pkg.sv  -  Constants and types for the DC-SCM 2.0 LTPI core
//
// Single source of truth derived from "DC-SCM 2.0 LVDS Tunneling Protocol &
// Interface Specification (LTPI) Revision 1.0, Version 1.0" (OCP).  Section /
// table references in comments point back at that document.
// =============================================================================
`ifndef LTPI_PKG_SV
`define LTPI_PKG_SV
package ltpi_pkg;

  // ---------------------------------------------------------------------------
  // Roles (the only top-level SCM/HPM asymmetry)
  // ---------------------------------------------------------------------------
  localparam bit ROLE_SCM = 1'b0;  // Secure Control Module - drives training
  localparam bit ROLE_HPM = 1'b1;  // Host Processor Module - responds

  // ---------------------------------------------------------------------------
  // LTPI version (spec rev 1.0 / ver 1.0), BCD major.minor  (Table 22)
  // ---------------------------------------------------------------------------
  localparam logic [3:0] LTPI_VER_MAJOR = 4'h1;
  localparam logic [3:0] LTPI_VER_MINOR = 4'h0;
  // Literal (not a concatenation) — iverilog mis-infers a 1-bit width for a
  // localparam built by concatenating other localparams, which injects X/Z into
  // the version byte and breaks CRC.  Keep as an explicit 8-bit literal.
  localparam logic [7:0] LTPI_VERSION   = 8'h10; // BCD 1.0

  // ---------------------------------------------------------------------------
  // Frame geometry: every LTPI frame is 16 symbols/bytes (spec Sec 3, Table 19)
  // ---------------------------------------------------------------------------
  localparam int  FRAME_BYTES   = 16;
  localparam int  PAYLOAD_BYTES = 14;            // bytes 1..14 (subtype..last) under CRC
  localparam int  SYMBOL_BITS   = 10;            // after 8b/10b
  localparam int  FRAME_BITS    = FRAME_BYTES*SYMBOL_BITS; // 160 (Sec 3)

  // ---------------------------------------------------------------------------
  // Comma symbols.  8b/10b K-code byte value = (y<<5)|xx for Kxx.y  (Table 19)
  //   K28.5 -> 0xBC  Link Detect / Link Speed
  //   K28.6 -> 0xDC  Advertise / Configure / Accept
  //   K28.7 -> 0xFC  Operational (I/O and Data frames)
  // ---------------------------------------------------------------------------
  localparam logic [7:0] K28_5 = 8'hBC;
  localparam logic [7:0] K28_6 = 8'hDC;
  localparam logic [7:0] K28_7 = 8'hFC;

  typedef enum logic [1:0] {
    COMMA_TRAIN = 2'd0,   // K28.5
    COMMA_CFG   = 2'd1,   // K28.6
    COMMA_OPER  = 2'd2,   // K28.7
    COMMA_NONE  = 2'd3    // not a known comma
  } comma_e;

  function automatic logic [7:0] comma_byte(input comma_e c);
    case (c)
      COMMA_TRAIN: comma_byte = K28_5;
      COMMA_CFG:   comma_byte = K28_6;
      default:     comma_byte = K28_7;
    endcase
  endfunction

  // returns a comma_e value as logic [1:0] (yosys can't infer an enum return width)
  function automatic logic [1:0] comma_decode(input logic kflag, input logic [7:0] b);
    // explicit 2-bit literals (= comma_e values) so yosys infers the width
    if (!kflag)        comma_decode = 2'd3;  // COMMA_NONE
    else if (b==K28_5) comma_decode = 2'd0;  // COMMA_TRAIN
    else if (b==K28_6) comma_decode = 2'd1;  // COMMA_CFG
    else if (b==K28_7) comma_decode = 2'd2;  // COMMA_OPER
    else               comma_decode = 2'd3;  // COMMA_NONE
  endfunction

  // ---------------------------------------------------------------------------
  // Frame subtypes  (Tables 20, 25, 32)
  // ---------------------------------------------------------------------------
  // K28.5 family
  localparam logic [7:0] SUB_LINK_DETECT = 8'h00;
  localparam logic [7:0] SUB_LINK_SPEED  = 8'h01;
  // K28.6 family
  localparam logic [7:0] SUB_ADVERTISE   = 8'h00;
  localparam logic [7:0] SUB_CONFIGURE   = 8'h01;
  localparam logic [7:0] SUB_ACCEPT      = 8'h02;
  // K28.7 family
  localparam logic [7:0] SUB_IO          = 8'h00;
  localparam logic [7:0] SUB_DATA        = 8'h01;

  // ---------------------------------------------------------------------------
  // Link state machine encodings  (CSR Table 36, offset 0x00 bits [19:16])
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    ST_DETECT   = 4'h0,
    ST_SPEED    = 4'h1,
    ST_ADVERTISE= 4'h2,
    ST_CONFIG   = 4'h3,   // SCM: Configuration / HPM: Accept
    ST_OPER     = 4'h4
  } link_state_e;

  // ---------------------------------------------------------------------------
  // Speed capability bit positions inside the 16-bit Speed word (Table 21/23).
  //   word[7:0]  = byte0 : x1,x2,x3,x4,x6,x8,x10,x12
  //   word[15:8] = byte1 : x16,x24,x32,x40,res,res,res,DDR
  // CSR "Link Speed" 4-bit encoding (Table 36 offset 0x00 [11:8]) listed below.
  // ---------------------------------------------------------------------------
  localparam int SPB_X1  = 0,  SPB_X2  = 1,  SPB_X3  = 2,  SPB_X4  = 3;
  localparam int SPB_X6  = 4,  SPB_X8  = 5,  SPB_X10 = 6,  SPB_X12 = 7;
  localparam int SPB_X16 = 8,  SPB_X24 = 9,  SPB_X32 = 10, SPB_X40 = 11;
  localparam int SPB_DDR = 15;

  // Highest-priority-first ordering of the 12 frequency bits for "highest common
  // speed" selection (Link Speed stage).  Index 0 = fastest.
  // Returns the bit index, and the matching 4-bit CSR speed code.
  function automatic logic [3:0] speed_csr_code(input int bit_idx);
    case (bit_idx)
      SPB_X1:  speed_csr_code = 4'h0;
      SPB_X2:  speed_csr_code = 4'h1;
      SPB_X3:  speed_csr_code = 4'h2;
      SPB_X4:  speed_csr_code = 4'h3;
      SPB_X6:  speed_csr_code = 4'h4;
      SPB_X8:  speed_csr_code = 4'h5;
      SPB_X10: speed_csr_code = 4'h6;
      SPB_X12: speed_csr_code = 4'h7;
      SPB_X16: speed_csr_code = 4'h8;
      SPB_X24: speed_csr_code = 4'h9;
      SPB_X32: speed_csr_code = 4'hA;
      SPB_X40: speed_csr_code = 4'hB;
      default: speed_csr_code = 4'h0;
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // Default LTPI Capabilities bit map  (Table 28).  8 bytes total.
  //   byte0 Supported Channels: [0]GPIO [1]I2C [2]UART [3]Data [4]OEM
  //   byte1 NL GPIO count [7:0]
  //   byte2 NL GPIO count [9:8]
  //   byte3 I2C enables [5:0], [6]Echo support
  //   byte4 I2C speeds   [5:0]
  //   byte5 UART: [3:0] max baud, [5]flow ctrl, [6]UART0 en, [7]UART1 en
  //   byte6/7 OEM
  // ---------------------------------------------------------------------------
  localparam int CAP_CH_GPIO = 0, CAP_CH_I2C = 1, CAP_CH_UART = 2,
                 CAP_CH_DATA = 3, CAP_CH_OEM = 4;

  // ---------------------------------------------------------------------------
  // I2C/SMBus event encodings (4 bits, Table 10), packed 2/byte (Table 11)
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    I2C_IDLE       = 4'h0,
    I2C_START      = 4'h1,
    I2C_START_RCVD = 4'h2,
    I2C_STOP       = 4'h3,
    I2C_STOP_RCVD  = 4'h4,
    I2C_DATA_RCVD  = 4'h5,
    I2C_DATA0      = 4'h6,
    I2C_DATA1      = 4'h7,
    I2C_START_ECHO = 4'h8,
    I2C_STOP_ECHO  = 4'h9,
    I2C_DATA0_ECHO = 4'hA,
    I2C_DATA1_ECHO = 4'hB,
    I2C_DRCVD_ECHO = 4'hC
  } i2c_evt_e;

  // ---------------------------------------------------------------------------
  // Data channel command encodings (8 bits, Table 12)
  // ---------------------------------------------------------------------------
  typedef enum logic [7:0] {
    DC_READ_REQ   = 8'h00,
    DC_WRITE_REQ  = 8'h01,
    DC_READ_CPL   = 8'h02,
    DC_WRITE_CPL  = 8'h03,
    DC_CRC_ERROR  = 8'h04
  } dc_cmd_e;

  localparam logic [3:0] DC_STATUS_OK      = 4'h0;
  localparam logic [3:0] DC_STATUS_INVALID = 4'h1;

  // ---------------------------------------------------------------------------
  // CSR offsets within the LTPI Control & Status block (Table 36).
  // The block base is 0x200 in the general map (Table 35); these are block-local.
  // ---------------------------------------------------------------------------
  localparam logic [7:0] CSR_LINK_STATUS      = 8'h00;
  localparam logic [7:0] CSR_DETECT_CAP_LOC   = 8'h04;
  localparam logic [7:0] CSR_DETECT_CAP_REM   = 8'h08;
  localparam logic [7:0] CSR_PLATFORM_LOC     = 8'h0C;
  localparam logic [7:0] CSR_PLATFORM_REM     = 8'h10;
  localparam logic [7:0] CSR_ADV_CAP_LOC_LO   = 8'h14;
  localparam logic [7:0] CSR_ADV_CAP_LOC_HI   = 8'h18;
  localparam logic [7:0] CSR_ADV_CAP_REM_LO   = 8'h1C;
  localparam logic [7:0] CSR_ADV_CAP_REM_HI   = 8'h20;
  localparam logic [7:0] CSR_DEF_CFG_LO       = 8'h24;
  localparam logic [7:0] CSR_DEF_CFG_HI       = 8'h28;
  localparam logic [7:0] CSR_ALIGN_ERR_CNT    = 8'h2C;
  localparam logic [7:0] CSR_LOST_ERR_CNT     = 8'h30;
  localparam logic [7:0] CSR_CRC_ERR_CNT      = 8'h34;
  localparam logic [7:0] CSR_COMMA_ERR_CNT    = 8'h38;
  localparam logic [7:0] CSR_SPEED_TO_CNT     = 8'h3C;
  localparam logic [7:0] CSR_CFG_TO_CNT       = 8'h40;
  localparam logic [7:0] CSR_TRAIN_RX_LO      = 8'h44;
  localparam logic [7:0] CSR_TRAIN_RX_HI      = 8'h48;
  localparam logic [7:0] CSR_TRAIN_TX_LO      = 8'h4C;
  localparam logic [7:0] CSR_TRAIN_TX_HI      = 8'h50;
  localparam logic [7:0] CSR_OPER_RX_CNT      = 8'h54;
  localparam logic [7:0] CSR_OPER_TX_CNT      = 8'h58;
  localparam logic [7:0] CSR_LINK_CONTROL     = 8'h80;

  // CSR Link Control (0x80) bit positions
  localparam int LC_SW_RESET     = 0;
  localparam int LC_RETRAIN      = 1;
  localparam int LC_DATA_RESET   = 9;
  localparam int LC_AUTO_CONFIG  = 10;
  localparam int LC_TRIG_CONFIG  = 11;

  // ---------------------------------------------------------------------------
  // Training thresholds & timeouts (spec Sec 4, Tables 37/38/42/43/47).
  // Frame counts are exact spec values.  Cycle-based timers (1ms advertise) are
  // overridable so sim/formal can shrink them.
  // ---------------------------------------------------------------------------
  localparam int DETECT_TX_MIN   = 255; // Sec 4.1.1.1
  localparam int DETECT_RX_MIN   = 7;   // 7 consecutive correct RX
  localparam int ALIGN_FRAMES    = 3;   // frame-alignment: 3 correct CRC
  localparam int SPEED_TX_MIN    = 7;   // SCM TX (Sec 4.1.1.2)
  localparam int SPEED_RX_MIN    = 3;   // HPM RX
  localparam int SPEED_TO_TX     = 255; // timeout -> Detect
  localparam int CONFIG_TX_MAX   = 31;  // SCM Configure max (Sec 4.1.2.2)
  localparam int ACCEPT_TX_MAX   = 15;  // HPM Accept max
  localparam int LOST_CFG_MAX    = 3;   // 3 consecutive lost -> Detect (config phase)
  localparam int LOST_OPER_MAX   = 7;   // 7 consecutive lost -> Detect (operational)

endpackage
`endif
