# I2C/SMBus over LTPI - Limitations and Link-Speed Envelope

The relay is **protocol-correct at every link speed** (clock stretching is
unbounded and formally proven - the local bus can never outrun the
tunnel). What degrades at low link rates is *performance* and
*interoperability with real masters*. This note quantifies both and
explains the field observation that I2C-over-LTPI setups tend to work at
**400 Mbps and above but not slower**.

## Where the time goes

Every I2C event (Start, each data bit, Stop) is a frame-borne round trip:
the event must ride an I/O frame, be regenerated on the far bus, and its
Received/Echo confirmations must ride frames back. With the 2.1v1.1
triple-send requirement on Data/Data-Received Echoes, one data bit costs
roughly **8-10 frame times of SCL stretch** (Data ->1f, Echo ->3f,
Received ->1f, Received-Echo ->3f, plus framing skew). A frame is 160
wire bits (16 symbols x 10b).

| Link rate | Frame time | Stretch per bit (~9 frames) | Effective max bit rate | 100 kHz bus | 400 kHz bus |
|---|---|---|---|---|---|
| 25 MHz SDR (25 Mbps) | 6.4 µs | ~58 µs | ~15 kbit/s | 13% of nominal | 4% |
| 100 MHz SDR | 1.6 µs | ~14 µs | ~60 kbit/s | ~40% | 15% |
| 200 MHz SDR | 0.8 µs | ~7 µs | ~110 kbit/s | ~60% | 25% |
| **400 MHz SDR (400 Mbps)** | 0.4 µs | ~3.6 µs | ~200 kbit/s | **~75%** | ~40% |
| **400 MHz DDR (800 Mbps)** | 0.2 µs | ~1.8 µs | ~330 kbit/s | **~85%** | ~55% |

(Data-frame interleave and CRC-dropped frames add on top: every dropped
frame inserts a full extra frame time, and a dropped Echo restarts a
triple-send round.)

## Why "works at 400 Mbps+, fails slower" in real systems

1. **Master stretch tolerance.** The relay stretches SCL for the full
   round trip of every bit. Many hardware I2C masters (BMC SoCs, PMBus
   controllers) enforce their own clock-stretch or transaction timeouts -
   commonly 1-35 ms, sometimes far less in hardware state machines, and
   some Fm+ masters support **no stretching at all**. At 25 MHz SDR a
   single byte holds SCL ~0.5 ms and a 32-byte SMBus block ~15 ms -
   inside SMBus's T_LOW:SEXT=25 ms only with zero retries, and past many
   masters' real-world limits. At 400 Mbps+ the same block costs <1 ms
   and everything comfortable clears.
2. **SMBus timeout ceilings.** SMBus T_TIMEOUT (25-35 ms max clock-low)
   and T_LOW:SEXT (25 ms cumulative stretch per message) are hard spec
   ceilings. CRC-retry storms on a marginal slow link multiply the
   per-bit stretch directly into these budgets.
3. **Peer implementations without unbounded stretch.** Several
   ecosystem relays tie event turnaround to fixed timeouts tuned for
   fast links; below ~400 Mbps those fire mid-transaction. Our relay
   does not (proven R3), but the far end may.
4. **tSP filter realization floor.** The I2C-required 50 ns spike filter
   (`ltpi_i2c_cond`, proven) is built from link-clock samples: 20 crisp
   2.5 ns steps at 400 MHz, but only 2 coarse 40 ns steps at 25 MHz -
   and **below 40 MHz a 50 ns filter is unrealizable** (elaboration
   fails). Between 40-100 MHz the window quantizes to 40-80 ns, eroding
   both noise margin and Fm+ edge fidelity.

## Supported envelope (this core)

| Link rate | I2C channel status |
|---|---|
| >= 400 Mbps (400M SDR / 400M DDR) | **Recommended.** Validated envelope; matches field experience across vendor cores. |
| 100-300 MHz SDR | Functional; verify the *master's* stretch tolerance and derate throughput per the table. |
| 40-75 MHz | Bring-up/debug only: coarse tSP filter, heavy throughput collapse, most masters' timeouts at risk. |
| 25 MHz (base) | **Not recommended for I2C traffic.** Use it for link training and GPIO/UART/CSR; raise the link rate before enabling I2C/SMBus (BMC can do this via CSR 0x04 + retrain). |
| < 40 MHz link clock | I2C channel unavailable (tSP filter unrealizable; elaboration error by design). |

## Fixed structural limits (any speed)

- One in-flight event per channel direction (spec event model); no event
  pipelining across bits.
- Multi-master arbitration on the *local* segment is resolved by the
  local bus; simultaneous starts across the two LTPI ends are resolved
  by `ARB_PRIORITY` + deferral (proven), not by SDA bit-arbitration.
- SMBus-specific services (PEC, ARP, alert) pass through untouched -
  they are payload to the relay - but PEC does not protect against the
  relay's own timeout-driven aborts on the far segment.
- 10-bit addressing and repeated-start work (address bits are just data
  bits to the relay); the direction tracker assumes standard 9-bit
  (8+ACK) framing.
