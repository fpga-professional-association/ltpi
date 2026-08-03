// Peer feature decode: identifies WHO is on the far end of the LTPI link
// and WHAT it can do, from the Advertise frame (spec 3.1.2.1).
//
// Two independent decodes:
//  1. FEATURE ROW (authoritative, vendor-independent): the Default LTPI
//     Capabilities bytes (Table 28) carried in Advertise bytes 5..12 -
//     supported channels, NL GPIO count, per-I2C enables/speeds, UART
//     baud/flow. This is what interop decisions should use.
//  2. VENDOR (best effort): the OEM-defined Platform Type ID (Table 26,
//     Advertise bytes 2-3) looked up in a small ROM. Platform IDs have no
//     public registry - the default entries target the peers we expect
//     (ASPEED AST1700-class BMCs, Lattice DC-SCM IP, the OCP/Intel
//     reference, Microchip CoreLTPI) and MUST be aligned with each
//     partner's platform agreement. Unknown IDs decode to VENDOR_UNKNOWN
//     with the feature row still fully valid.
//
// Advertise payload mapping into rx_payload[103:0] (= frame bytes 2..14):
//   bytes 2-3  platform ID      -> payload[15:0]
//   byte  4    capabilities type-> payload[23:16]  (0h00 = default, T29)
//   bytes 5-12 LTPI capabilities-> payload[87:24]  (Table 28 bytes 0..7)
import ltpi_pkg::*;

module ltpi_peer_decode (
    input  logic        clk,
    input  logic        rst,

    // From ltpi_frame_rx
    input  logic        frame_valid,
    input  logic        frame_crc_ok,
    input  ltpi_pkg::frame_type_t frame_type,
    input  logic [103:0] payload,

    // Peer identity
    output logic        peer_valid,        // an Advertise frame was captured
    output logic [15:0] peer_platform_id,
    output ltpi_pkg::peer_vendor_t peer_vendor,

    // Feature row (Table 28, valid when peer_valid && caps_default)
    output logic        caps_default,      // capabilities type == 0h00
    output logic        feat_gpio,         // supported channels
    output logic        feat_i2c,
    output logic        feat_uart,
    output logic        feat_data,
    output logic        feat_oem,
    output logic [9:0]  feat_nl_gpio_cnt,  // number of NL GPIOs
    output logic [5:0]  feat_i2c_en,       // per-channel I2C enables
    output logic [5:0]  feat_i2c_speed,    // 0=100kHz 1=400kHz per channel
    output logic        feat_i2c_echo,     // echo support (always 1 >=2.1)
    output logic [3:0]  feat_uart_baud,    // Table 27 encoding
    output logic        feat_uart_flow,
    output logic [1:0]  feat_uart_en,
    output logic [15:0] peer_oem_caps
);

    function automatic ltpi_pkg::peer_vendor_t vendor_of(input logic [15:0] id);
        begin
            case (id)
                PLATID_THIS_CORE: vendor_of = VENDOR_THIS_CORE;
                PLATID_ASPEED:    vendor_of = VENDOR_ASPEED;
                PLATID_LATTICE:   vendor_of = VENDOR_LATTICE;
                PLATID_INTEL_REF: vendor_of = VENDOR_INTEL_REF;
                PLATID_MICROCHIP: vendor_of = VENDOR_MICROCHIP;
                default:          vendor_of = VENDOR_UNKNOWN;
            endcase
        end
    endfunction

    logic adv_good;
    assign adv_good = frame_valid && frame_crc_ok
                      && frame_type == FRAME_ADVERTISE;

    always_ff @(posedge clk) begin
        if (rst) begin
            peer_valid       <= 1'b0;
            peer_platform_id <= '0;
            peer_vendor      <= VENDOR_UNKNOWN;
            caps_default     <= 1'b0;
            {feat_oem, feat_data, feat_uart, feat_i2c, feat_gpio} <= '0;
            feat_nl_gpio_cnt <= '0;
            feat_i2c_en      <= '0;
            feat_i2c_speed   <= '0;
            feat_i2c_echo    <= 1'b0;
            feat_uart_baud   <= '0;
            feat_uart_flow   <= 1'b0;
            feat_uart_en     <= '0;
            peer_oem_caps    <= '0;
        end else if (adv_good) begin
            peer_valid       <= 1'b1;
            peer_platform_id <= payload[15:0];
            peer_vendor      <= vendor_of(payload[15:0]);
            caps_default     <= (payload[23:16] == 8'h00);
            // Table 28 byte 0: supported channels
            feat_gpio        <= payload[24];
            feat_i2c         <= payload[25];
            feat_uart        <= payload[26];
            feat_data        <= payload[27];
            feat_oem         <= payload[28];
            // bytes 1-2: NL GPIO count [9:0]
            feat_nl_gpio_cnt <= {payload[41:40], payload[39:32]};
            // byte 3: I2C enables + echo
            feat_i2c_en      <= payload[53:48];
            feat_i2c_echo    <= payload[54];
            // byte 4: I2C speeds
            feat_i2c_speed   <= payload[61:56];
            // byte 5: UART baud/flow/enables
            feat_uart_baud   <= payload[67:64];
            feat_uart_flow   <= payload[68];
            feat_uart_en     <= payload[70:69];
            // bytes 6-7: OEM capabilities
            peer_oem_caps    <= payload[87:72];
        end
    end

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk)
        f_past_valid <= 1'b1;

    initial assume (rst);

    // PD1: capture happens only on CRC-good Advertise frames; everything
    // holds otherwise.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && !$past(adv_good)) begin
            assert (peer_valid == $past(peer_valid));
            assert (peer_platform_id == $past(peer_platform_id));
            assert (peer_vendor == $past(peer_vendor));
            assert (feat_nl_gpio_cnt == $past(feat_nl_gpio_cnt));
        end
    end

    // PD2: after a capture, the vendor decode is consistent with the ROM.
    always_ff @(posedge clk) begin
        if (f_past_valid && !$past(rst) && $past(adv_good)) begin
            assert (peer_valid);
            assert (peer_platform_id == $past(payload[15:0]));
            assert (peer_vendor == vendor_of(peer_platform_id));
        end
    end

    // PD3: the decode is total - every ID yields a defined vendor code.
    always_comb
        if (f_past_valid)
            assert (peer_vendor <= VENDOR_MICROCHIP);

    // Covers: an ASPEED peer decode, an unknown-ID capture with a valid
    // feature row, and a full-featured peer.
    always_ff @(posedge clk) begin
        if (f_past_valid) begin
            cover (peer_valid && peer_vendor == VENDOR_ASPEED);
            cover (peer_valid && peer_vendor == VENDOR_UNKNOWN
                   && caps_default && feat_i2c);
            cover (peer_valid && feat_gpio && feat_i2c && feat_uart
                   && feat_data && feat_i2c_en == 6'h3F);
        end
    end
`endif

endmodule
