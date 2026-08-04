# Proof Waveforms — human-reviewable evidence

Each image shows the **system simulation** (`sim/tb_ltpi_system.sv`, two
complete `ltpi_top` endpoints cross-connected at the serial level)
exercising the behavior a formal suite guarantees. The formal proofs are
the authority — they hold for *all* reachable states, unboundedly; these
waveforms let a human reviewer see one concrete run and check the claim
with their own eyes. Every title carries the proof IDs and a "REVIEW:"
line saying exactly what to look for.

| Image | Proof suite(s) | What the reviewer should confirm |
|---|---|---|
| `01_link_training_loopback_L1-L4_T1-T3.png` | `ltpi_loopback` | Both endpoints walk Detect → Speed → Advertise → Cfg/Accept → Operational together; `link_up` rises only after the Configure→Accept handshake (L1/L2); both sides latch the **same** one-hot speed 0x8100 = 400 MHz + DDR (T3) |
| `02_i2c_bidirectional_relay_R1-R12.png` | `ltpi_i2c_relay`, `ltpi_i2c_loopback` | The initiating side stretches SCL (red) until the far side confirms (R3); START pulses are regenerated on the far bus in **both** directions — the HPM-initiated one is the SPDM response path; both relay state lanes return to IDLE after Stop (mutual exclusion) |
| `03_i2c_tsp_spike_filter_F1.png` | `ltpi_i2c_cond` | A 25 ns glitch on raw SCL (red) never reaches filtered SCL (green), fires no `scl_rise`/`scl_fall` pulse, and leaves the relay state unchanged; the *legitimate* wide edge right after it **does** pass, ~50 ns later (the tSP delay) |
| `04_oem_apb_tunnel_O1-O5.png` | `ltpi_oem_apb` | Near-side PREADY (green) asserts only after the far side ran a real APB access (HPM PSEL/PENABLE/PREADY pulses) and the 2-beat response frame returned (O1); write data CAFED00D reads back identically |
| `05_channels_gpio_uart.png` | `ltpi_gpio_channel`, `ltpi_uart_channel` | LL pattern A5C3 and NL pattern DEADBEEF (2 frame slices) arrive unchanged; UART txd level follows across the link |

Reading the lanes: black = logic levels, **red** = the signal whose
behavior is under review (stretch, raw/glitched bus), **green** = the
"success" signal (link_up, regenerated START, filtered output, PREADY),
blue blocks = bus/state values (hex, or decoded state names).

The sim runs a 200 MHz link clock; every property shown is proven
speed-independent. Timestamps match the `PASS:` lines the TB prints.

## Regenerate

```powershell
cd sim
vvp tb.vvp                      # writes tb_ltpi_system.vcd (see USER_GUIDE §4 for the compile line)
python render_proof_waves.py    # writes docs/proof_waveforms/*.png
```
