"""Render key LTPI loopback signals from the simulation VCD as a timing
diagram PNG (link training -> operational -> channel traffic -> I2C
handshake)."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from vcdvcd import VCDVCD

VCD = "tb_ltpi_system.vcd"
OUT = "ltpi_loopback_waveform.png"

INK    = "#1F2937"   # waveform ink
MUTED  = "#9CA3AF"   # lane separators / grid
ACCENT = "#2563EB"   # bus value blocks
GOOD   = "#059669"   # link-up highlight
PAPER  = "#FFFFFF"

STATE_NAMES = {0: "DETECT\nALIGN", 1: "DETECT", 2: "SPEED", 3: "ADV\nALIGN",
               4: "ADV", 5: "CFG/ACC", 6: "OPER"}

vcd = VCDVCD(VCD)
names = vcd.references_to_ids.keys()

def sig(path):
    for n in names:
        if n == path or n.startswith(path + "["):
            return vcd[n]
    raise KeyError(path)

def tv(path):
    """[(time_ns, int_value)] change list."""
    out = []
    for t, v in sig(path).tv:
        try:
            out.append((t / 1000.0, int(v, 2)))   # ps -> ns
        except ValueError:
            out.append((t / 1000.0, None))        # x/z
    return out

T_END = vcd.endtime / 1000.0

LANES = [
    ("rst",              "tb_ltpi_system.rst",              "bit"),
    ("SCM state",        "tb_ltpi_system.scm_state",        "state"),
    ("HPM state",        "tb_ltpi_system.hpm_state",        "state"),
    ("SCM link_up",      "tb_ltpi_system.scm_up",           "bit_good"),
    ("HPM link_up",      "tb_ltpi_system.hpm_up",           "bit_good"),
    ("speed_select",     "tb_ltpi_system.scm_speed",        "bus"),
    ("LL GPIO in (SCM)", "tb_ltpi_system.scm_ll_in",        "bus"),
    ("LL GPIO out (HPM)","tb_ltpi_system.hpm_ll_out",       "bus"),
    ("NL GPIO out (HPM)","tb_ltpi_system.hpm_nl_out",       "bus"),
    ("UART txd (SCM)",   "tb_ltpi_system.scm_uart_txd",     "bit"),
    ("UART txd (HPM)",   "tb_ltpi_system.hpm_uart_txd_out", "bit"),
    ("I2C stretch (SCM)","tb_ltpi_system.scm_scl_stretch",  "bit"),
    ("I2C START>HPM",    "tb_ltpi_system.hpm_start_gen",    "bit"),
    ("I2C stretch (HPM)","tb_ltpi_system.hpm_scl_stretch",  "bit"),
    ("I2C START>SCM",    "tb_ltpi_system.scm_start_gen",    "bit"),
]

fig, ax = plt.subplots(figsize=(16, 0.78 * len(LANES) + 1.6), dpi=150)
ax.set_facecolor(PAPER); fig.patch.set_facecolor(PAPER)

H = 0.78      # lane pitch
AMP = 0.34    # waveform amplitude

for k, (label, path, kind) in enumerate(LANES):
    y0 = -k * H
    ax.axhline(y0 - H / 2 + 0.02, color=MUTED, lw=0.4, alpha=0.5)
    changes = tv(path)
    if kind in ("bit", "bit_good"):
        color = GOOD if kind == "bit_good" else INK
        xs, ys = [], []
        prev_v = 0
        for i, (t, v) in enumerate(changes):
            v = 0 if v is None else (1 if v else 0)
            if i == 0:
                xs, ys = [t], [v * AMP]
            else:
                xs += [t, t]
                ys += [prev_v * AMP, v * AMP]
            prev_v = v
        xs.append(T_END); ys.append(prev_v * AMP)
        ax.plot([x / 1000 for x in xs], [y + y0 for y in ys],
                color=color, lw=1.4, solid_joinstyle="miter")
    else:
        segs = []
        for i, (t, v) in enumerate(changes):
            t_next = changes[i + 1][0] if i + 1 < len(changes) else T_END
            segs.append((t, t_next, v))
        for (t0, t1, v) in segs:
            if v is None:
                continue
            x0, x1 = t0 / 1000, t1 / 1000
            if x1 - x0 < 0.02:
                continue
            ax.add_patch(Rectangle((x0, y0 - AMP * 0.75), x1 - x0,
                                   AMP * 1.5, facecolor=ACCENT, alpha=0.14,
                                   edgecolor=ACCENT, lw=0.7))
            if kind == "state":
                txt = STATE_NAMES.get(v, str(v))
                fs = 6.2
            else:
                txt = f"{v:X}"
                fs = 6.8
            if x1 - x0 > 0.55:
                ax.text((x0 + x1) / 2, y0, txt, ha="center", va="center",
                        fontsize=fs, color=INK, family="monospace")
    ax.text(-0.4, y0, label, ha="right", va="center", fontsize=8.5,
            color=INK)

# Phase annotations along the top
phases = [(0.05, 7.1, "Link Detect"), (7.1, 9.0, "Link Speed"),
          (9.0, 19.9, "Advertise"), (19.9, 21.2, "Cfg/Acc"),
          (21.2, T_END / 1000, "OPERATIONAL - channels live")]
ytop = AMP + 0.28
for x0, x1, name in phases:
    ax.annotate("", xy=(x1, ytop), xytext=(x0, ytop),
                arrowprops=dict(arrowstyle="<->", color=MUTED, lw=0.8))
    ax.text((x0 + x1) / 2, ytop + 0.16, name, ha="center", fontsize=7.5,
            color=INK)

ax.set_xlim(-3.2, T_END / 1000 + 0.3)
ax.set_ylim(-len(LANES) * H + 0.1, ytop + 0.45)
ax.set_yticks([])
ax.set_xlabel("time [µs]   (200 MHz link clock, SDR)", fontsize=9, color=INK)
ax.tick_params(axis="x", labelsize=8, colors=INK)
for s in ("top", "right", "left"):
    ax.spines[s].set_visible(False)
ax.spines["bottom"].set_color(MUTED)
ax.set_title("LTPI SCM"u"↔""HPM loopback - training, channels, BIDIRECTIONAL I2C (SPDM-ready)",
             fontsize=11, color=INK, pad=26)

plt.tight_layout()
plt.savefig(OUT, bbox_inches="tight", facecolor=PAPER)
print("wrote", OUT)
