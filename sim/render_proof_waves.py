"""Render human-reviewable waveform evidence for the key formal proofs.

Each PNG shows the system-simulation (sim/tb_ltpi_system.sv) exercising the
behavior a proof suite guarantees, with callouts telling a reviewer exactly
what to check. Run AFTER the sim (needs sim/tb_ltpi_system.vcd):

    cd sim && vvp tb.vvp && python render_proof_waves.py

Images land in docs/proof_waveforms/.
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from vcdvcd import VCDVCD

VCD = "tb_ltpi_system.vcd"
OUTDIR = os.path.join("..", "docs", "proof_waveforms")
os.makedirs(OUTDIR, exist_ok=True)

INK, MUTED, ACCENT, GOOD, WARN = "#1F2937", "#9CA3AF", "#2563EB", "#059669", "#DC2626"
PAPER = "#FFFFFF"

STATE_NAMES = {0: "DETECT\nALIGN", 1: "DETECT", 2: "SPEED", 3: "ADV\nALIGN",
               4: "ADV", 5: "CFG/ACC", 6: "OPER"}
RELAY_NAMES = {0: "IDLE", 1: "START", 2: "WAIT", 3: "LDATA", 4: "RDATA",
               5: "RD\nRCVD", 6: "STOP", 7: "STOP2"}

vcd = VCDVCD(VCD)
NAMES = list(vcd.references_to_ids.keys())


def sig(path):
    for n in NAMES:
        if n == path or n.startswith(path + "["):
            return vcd[n]
    # generate-block / escaped-name fallback: match on suffix
    tail = path.split(".")[-1]
    cands = [n for n in NAMES if path in n or
             (all(p in n for p in path.split(".")[1:]))]
    for n in cands:
        if n.endswith(tail) or n.startswith(path):
            return vcd[n]
    raise KeyError(f"{path} not in VCD; closest: {cands[:5]}")


def tv(path):
    out = []
    for t, v in sig(path).tv:
        try:
            out.append((t / 1e6, int(v, 2)))    # ps -> us
        except ValueError:
            out.append((t / 1e6, None))
    return out


T_END = vcd.endtime / 1e6


def render(fname, title, lanes, t0, t1, callouts=(), spans=(), xlabel=None,
           lane_h=0.78):
    """lanes: (label, path, kind) with kind in bit/bit_good/bit_warn/bus/
    state/relay. callouts: (t_us, lane_idx, text, dy). spans: (t0, t1, text)."""
    fig, ax = plt.subplots(figsize=(15, lane_h * len(lanes) + 2.2), dpi=150)
    ax.set_facecolor(PAPER); fig.patch.set_facecolor(PAPER)
    H, AMP = lane_h, 0.32

    for k, (label, path, kind) in enumerate(lanes):
        y0 = -k * H
        ax.axhline(y0 - H / 2 + 0.02, color=MUTED, lw=0.4, alpha=0.5)
        changes = tv(path)
        if kind in ("bit", "bit_good", "bit_warn"):
            color = {"bit": INK, "bit_good": GOOD, "bit_warn": WARN}[kind]
            xs, ys, prev = [t0], [None], 0
            # value at window start
            val = 0
            for t, v in changes:
                if t <= t0 and v is not None:
                    val = 1 if v else 0
            xs, ys, prev = [t0], [val * AMP], val
            for t, v in changes:
                if t < t0 or t > t1:
                    continue
                v = 0 if v is None else (1 if v else 0)
                xs += [t, t]; ys += [prev * AMP, v * AMP]; prev = v
            xs.append(t1); ys.append(prev * AMP)
            ax.plot(xs, [y + y0 for y in ys], color=color, lw=1.5,
                    solid_joinstyle="miter")
        else:
            segs, cur, cur_t = [], None, t0
            for t, v in changes:
                if t <= t0:
                    cur = v; continue
                if t > t1:
                    break
                segs.append((cur_t, t, cur)); cur, cur_t = v, t
            segs.append((cur_t, t1, cur))
            for (a, b, v) in segs:
                if v is None:
                    continue
                if b - a <= 0:
                    continue
                ax.add_patch(Rectangle((a, y0 - AMP * 0.75), b - a, AMP * 1.5,
                                       facecolor=ACCENT, alpha=0.14,
                                       edgecolor=ACCENT, lw=0.7))
                if kind == "state":
                    txt, fs = STATE_NAMES.get(v, str(v)), 6.2
                elif kind == "relay":
                    txt, fs = RELAY_NAMES.get(v, str(v)), 6.2
                else:
                    txt, fs = f"{v:X}", 6.8
                if (b - a) > (t1 - t0) * 0.03:
                    ax.text((a + b) / 2, y0, txt, ha="center", va="center",
                            fontsize=fs, color=INK, family="monospace")
        ax.text(t0 - (t1 - t0) * 0.012, y0, label, ha="right", va="center",
                fontsize=8.5, color=INK)

    ytop = AMP + 0.30
    for a, b, name in spans:
        ax.annotate("", xy=(b, ytop), xytext=(a, ytop),
                    arrowprops=dict(arrowstyle="<->", color=MUTED, lw=0.8))
        ax.text((a + b) / 2, ytop + 0.15, name, ha="center", fontsize=7.5,
                color=INK)

    for t, lane, text, dy in callouts:
        y = -lane * H
        ax.annotate(text, xy=(t, y), xytext=(t, y + dy),
                    fontsize=7.6, color=WARN, ha="center",
                    arrowprops=dict(arrowstyle="->", color=WARN, lw=1.1),
                    bbox=dict(boxstyle="round,pad=0.25", fc="#FEF2F2",
                              ec=WARN, lw=0.8))

    ax.set_xlim(t0 - (t1 - t0) * 0.14, t1 + (t1 - t0) * 0.02)
    ax.set_ylim(-len(lanes) * H + 0.15, ytop + 0.5)
    ax.set_yticks([])
    ax.set_xlabel(xlabel or "time [µs]  (200 MHz link clock in sim; "
                  "properties are speed-independent)", fontsize=9, color=INK)
    ax.tick_params(axis="x", labelsize=8, colors=INK)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_color(MUTED)
    ax.set_title(title, fontsize=10.5, color=INK, pad=30, wrap=True)
    plt.tight_layout()
    out = os.path.join(OUTDIR, fname)
    plt.savefig(out, bbox_inches="tight", facecolor=PAPER)
    plt.close(fig)
    print("wrote", out)


TB = "tb_ltpi_system"

# ---------------------------------------------------------------- 1: training
render(
    "01_link_training_loopback_L1-L4_T1-T3.png",
    "Proof ltpi_loopback L1–L4 / T1–T3 — SCM↔HPM training: "
    "handshake-gated Operational entry and one-hot speed agreement\n"
    "REVIEW: states advance together; speed_select = 8100 (400MHz+DDR, both ends); "
    "link_up rises only after Config/Accept; HPM never Operational before SCM",
    [("rst",            f"{TB}.rst",        "bit"),
     ("SCM link state", f"{TB}.scm_state",  "state"),
     ("HPM link state", f"{TB}.hpm_state",  "state"),
     ("SCM link_up",    f"{TB}.scm_up",     "bit_good"),
     ("HPM link_up",    f"{TB}.hpm_up",     "bit_good"),
     ("SCM speed_select", f"{TB}.scm_speed", "bus"),
     ("HPM speed_select", f"{TB}.hpm_speed", "bus")],
    0.0, 25.0,
    callouts=[(21.4, 3, "L1/L2: link_up only after\nConfigure->Accept handshake", 0.9),
              (19.0, 5, "T3: both sides pick the SAME one-hot\nmutually-supported speed (proven unbounded)", 1.1)],
    spans=[(0.05, 7.1, "Link Detect"), (7.1, 9.0, "Link Speed"),
           (9.0, 19.9, "Advertise"), (19.9, 21.2, "Cfg/Acc"),
           (21.2, 25.0, "OPERATIONAL")])

# ------------------------------------------------- 2: bidirectional I2C relay
render(
    "02_i2c_bidirectional_relay_R1-R12.png",
    "Proofs ltpi_i2c_relay R1–R12 + ltpi_i2c_loopback — bidirectional I2C with "
    "clock stretching (SPDM path) and bus-mastership mutual exclusion\n"
    "REVIEW: initiator stretches SCL until the far side confirms; STARTs are regenerated on the far bus "
    "in BOTH directions; relays return to IDLE after Stop",
    [("SCM raw SCL",        f"{TB}.scm_scl",         "bit"),
     ("SCM raw SDA",        f"{TB}.scm_sda",         "bit"),
     ("SCM SCL stretch",    f"{TB}.scm_scl_stretch", "bit_warn"),
     ("START gen → HPM bus", f"{TB}.hpm_start_gen",  "bit_good"),
     ("HPM raw SCL",        f"{TB}.hpm_scl",         "bit"),
     ("HPM SCL stretch",    f"{TB}.hpm_scl_stretch", "bit_warn"),
     ("START gen → SCM bus", f"{TB}.scm_start_gen",  "bit_good"),
     ("SCM relay state",    f"{TB}.u_scm.g_i2c[0].u_i2c.state", "relay"),
     ("HPM relay state",    f"{TB}.u_hpm.g_i2c[0].u_i2c.state", "relay")],
    34.0, 66.0,
    callouts=[(37.5, 2, "R3: initiator stretches until\nStart Received returns", 0.8),
              (48.9, 6, "bidirectional: HPM-initiated START\nregenerated on SCM bus (SPDM response path)", 0.9),
              (63.5, 8, "both relays IDLE after Stop\n(mutual exclusion held throughout)", 0.8)],
    spans=[(34.0, 44.0, "SCM-initiated START"),
           (44.0, 52.0, "HPM-initiated START (reverse direction)"),
           (52.0, 66.0, "data bit + Stop handshakes")])

# ---------------------------------------------------------- 3: tSP filter F1
# locate the injected spike: shortest HIGH pulse on scm_scl (TB test 5b
# drives SCL high for 5 cycles = 25 ns while the bus is held low mid-bit)
scl = [x for x in tv(f"{TB}.scm_scl") if x[1] is not None]
spike_t, spike_w = None, 1e9
for i in range(len(scl) - 1):
    if scl[i][1] == 1:
        w = scl[i + 1][0] - scl[i][0]
        if w < spike_w:
            spike_w, spike_t = w, scl[i][0]
w_ns = spike_w * 1000.0
render(
    "03_i2c_tsp_spike_filter_F1.png",
    f"Proof ltpi_i2c_cond F1 — 50 ns tSP spike filter: pulses shorter than the window are "
    f"provably invisible\nREVIEW: the {w_ns:.0f} ns glitch on raw SCL never appears on filtered SCL and fires no "
    "edge pulse; relay state unchanged. The later real edge (>50 ns) DOES pass, after the tSP delay",
    [("raw SCL (bus)",      f"{TB}.scm_scl",  "bit_warn"),
     ("filtered SCL",       f"{TB}.u_scm.g_i2c[0].u_cond.scl_filt", "bit_good"),
     ("scl_fall pulse",     f"{TB}.u_scm.g_i2c[0].u_cond.scl_fall", "bit"),
     ("scl_rise pulse",     f"{TB}.u_scm.g_i2c[0].u_cond.scl_rise", "bit"),
     ("SCM relay state",    f"{TB}.u_scm.g_i2c[0].u_i2c.state",     "relay")],
    spike_t - 0.35, spike_t + 0.55,
    callouts=[(spike_t + spike_w / 2, 0,
               f"{w_ns:.0f} ns spike < 50 ns tSP window", 0.75),
              (spike_t + spike_w / 2 + 0.10, 1,
               "F1: filtered level unchanged — proven for\nEVERY pulse < FILT_CYC cycles, any alignment", -0.85),
              (spike_t - 0.02, 4,
               "START→WAIT = Start handshake completing\n(stretch release, just before the spike) —\nstate then HOLDS through spike + settle", 0.95)],
    xlabel="time [µs]  (zoom: 25 ns glitch injected by TB test 5b; "
           "the wide SCL rise afterwards is the legitimate STOP edge)")

# ------------------------------------------------------------- 4: OEM APB O1-O5
render(
    "04_oem_apb_tunnel_O1-O5.png",
    "Proof ltpi_oem_apb O1–O5 — AMBA APB tunneled through I/O-frame OEM bytes 11–14\n"
    "REVIEW: near-side PREADY asserts only AFTER the far-side APB access completed and the response frame "
    "returned; write data CAFED00D lands in the far memory and reads back identically",
    [("SCM apb PSEL",     f"{TB}.apb_psel",        "bit"),
     ("SCM apb PENABLE",  f"{TB}.apb_penable",     "bit"),
     ("SCM apb PREADY",   f"{TB}.apb_pready",      "bit_good"),
     ("SCM apb PWRITE",   f"{TB}.apb_pwrite",      "bit"),
     ("SCM apb PADDR",    f"{TB}.apb_paddr",       "bus"),
     ("SCM apb PWDATA",   f"{TB}.apb_pwdata",      "bus"),
     ("SCM apb PRDATA",   f"{TB}.apb_prdata",      "bus"),
     ("HPM apb PSEL (far master)",    f"{TB}.hpm_apb_psel",    "bit_good"),
     ("HPM apb PENABLE (far master)", f"{TB}.hpm_apb_penable", "bit"),
     ("HPM apb PREADY (far slave)",   f"{TB}.hpm_apb_pready",  "bit")],
    142.5, 151.5,
    callouts=[(146.0, 2, "O1: PREADY only in ACCESS, only after\nthe tunneled 2-beat response returned", 0.85),
              (147.9, 7, "far side runs a REAL APB access\n(O2–O4: SETUP→ACCESS, signals stable)", 0.9),
              (150.4, 6, "read back = CAFED00D\n(round trip through 2×16-byte frames)", 0.9)],
    spans=[(142.9, 146.2, "APB WRITE tunneled SCM→HPM"),
           (146.4, 150.7, "APB READ tunneled SCM→HPM")])

# --------------------------------------------------------- 5: GPIO/UART channels
render(
    "05_channels_gpio_uart.png",
    "Proofs ltpi_gpio_channel + ltpi_uart_channel — LL/NL GPIO exact-slice transport and "
    "UART tunneling\nREVIEW: LL pattern A5C3 and NL pattern DEADBEEF appear unchanged on the far side "
    "(hold-on-CRC-error, no cross-slice writes); UART txd level follows across the link",
    [("SCM LL GPIO in",   f"{TB}.scm_ll_in",   "bus"),
     ("HPM LL GPIO out",  f"{TB}.hpm_ll_out",  "bus"),
     ("SCM NL GPIO in",   f"{TB}.scm_nl_in",   "bus"),
     ("HPM NL GPIO out",  f"{TB}.hpm_nl_out",  "bus"),
     ("SCM UART txd in",  f"{TB}.scm_uart_txd",     "bit"),
     ("HPM UART txd out", f"{TB}.hpm_uart_txd_out", "bit_good")],
    21.0, 36.0,
    callouts=[(27.5, 1, "LL GPIO A5C3 delivered (refreshed every frame)", 0.8),
              (31.5, 3, "NL DEADBEEF via ⌈2 frame slices⌉\n(proven: exact slice addressing)", 0.9)])

print("done:", OUTDIR)
