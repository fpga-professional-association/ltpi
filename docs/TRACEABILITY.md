# LTPI Spec ↔ Verification Traceability Matrix

Target specification: **OCP DC-SCM 2.2 LTPI, Version 1.0** (Rev 1.2,
Aug 11 2025) — the latest public release. The core was originally built
against DC-SCM 2.0 LTPI v1.0 and updated with every normative change in
the 2.1v1.0 / 2.1v1.1 / 2.2v1.0 revision history:
100 ms Advertise alignment timeout, Echo triple-send, the refined
Operational link-lost rule (3 CRC-dropped IO frames Data-frame-immune /
7 total), and the Stop→Start-without-Idle optimization.

**How to read "does formal cover the entire spec?"** — no tool can prove
that by construction; spec coverage is an argument built from four legs,
all present here: (1) this requirement-level trace, (2) 100% reachability
of the 39 formal cover points (vacuity control), (3) mutation coverage
quantifying the property set's bug-detection power
(`formal/mutation_report.txt`), and (4) composition proofs that judge each
side against an independent reference rather than its own decode logic.
The rows marked ⚠/✗ below are the honest boundary of the claim.

Legend: **F** formal property (suite:property) · **S** system sim test ·
**✓** covered · **⚠** partial · **✗** out of scope

| Spec clause | Requirement | Coverage | Where |
|---|---|---|---|
| 2.2.1.1, T6 | LL GPIO refreshed every frame, bit-per-GPIO | ✓ | F gpio:G1-G2, S test 2 |
| 2.2.1.1 | NL GPIO over N frames, counter-decoded slices, X-offset formula | ✓ | F gpio:G3-G5 (incl. partial last slice), S test 3 |
| 2.2.1.2, T8 | UART 3× oversample, D[0]=oldest, per-frame flow ctrl | ✓ | F uart:U1-U4, S test 4 |
| 2.2.1.2 (2.1v1.0) | CRC checked before UART regeneration | ✓ | RX replay only from CRC-good frames (uart:U3 + frame gating) |
| 2.2.1.3, T9/T10 | I2C event relay: Start/Data/Stop + Echo/Received handshakes | ✓ | F i2c:R1-R12, S tests 5-7 |
| 2.2.1.3 | Clock stretching while awaiting far side (both dirs) | ✓ | F i2c:R3 (+pacing), R10 target-stretch; S stretch checks |
| 2.2.1.3 (2.1v1.1) | Data/DRcvd Echo sent ≥3 times | ✓ | F i2c pacing (echo3 gate) + cover |
| 2.2.1.3 (2.1v1.1) | Start after Stop-Received w/o Idle | ✓ | F i2c covers (both roles) |
| 2.2.1.3 | Events continuous until new; Echo separates repeats | ✓ | F rx_new change semantics, pacing R12 |
| 2.2.1.3 | Direction tracking (addr byte, R/W, ACK slots) | ✓ | F i2c:R7 + read-decode cover |
| — (SPDM/MCTP need) | Bidirectional initiation + arbitration + deferral | ✓ | F i2c:R9, composition S1 (mutual exclusion, unbounded), S test 6 |
| 2.2.1.3 | Full SMBus timeout/bus-error micro-architecture | ✗ | spec declares out of LTPI scope (impl-specific) |
| 2.2.1.4, T12/13, 3.1.1.2 T34 | Data channel R/W with tags, single-tracking, in-order | ✓ | F data:D1-D5, S test 11 |
| T34 | Data-frame LL GPIO bytes 2-3 | ⚠ | zeros carried; LL rides IO frames only (1-frame interleave keeps LL jitter in budget) |
| T12 note (2.2) | DC CRC-Error completion event | ⚠ | DC_CMD_ERROR defined+proven; requester timeout not implemented (BMC retry model) |
| 2.3 | Clock topology / PLL reconfig at speed switch | ✗ | system integration (documented in README + vendor notes) |
| 2.4 | CRC-8 x⁸+x²+x+1, init 0, no reflection, payload-after-comma | ✓ | F crc8 (zero-remainder theorem, 0xF4 KAT), frame_tx/rx proofs |
| 2.5 / T14 | CRC error ⇒ frame dropped; per-channel hold + recovery | ✓ | F gpio:G1, uart:U3, fsm:P7/P9; realign logic |
| 2.5 (2.1v1.1) | Operational link lost: 3 consec CRC-dropped IO (Data-immune) / 7 total | ✓ | F fsm io_bad_run props + link_lost |
| 2.7 | Latency tables / bandwidth math | ✗ | analytical, not RTL behavior |
| 3, T19 | 16-symbol frames, comma classes K28.5/6/7 | ✓ | F frame_tx/rx (cadence, comma legality, classification) |
| 3.1.1.x T20-24 | Detect/Speed frames: subtypes, version, caps/select words | ✓ | F frame proofs + fsm speed props; payload mux in top |
| 3.1.2.x T25-31 | Advertise/Configure/Accept frames | ✓ | F classification + fsm handshake props (P3) |
| T28 detail | Per-sub-channel capability bit semantics (I2C speeds, baud, NL count) | ⚠ | default-caps profile; cfg_match simplified to profile equality |
| 3.1.1 T32-33 | Operational IO/Data frame layouts | ✓ | F frame types incl. FRAME_OP_DATA; top payload mux; S all channel tests |
| 3.2, T35/36 | CSR: status/state/speed encodings, RWC errors, counters, link control | ✓ (subset) | F csr:C1-C5, S test 10 |
| T36 full map | Platform IDs, advertise mirrors, per-channel resets, 0x44-0x50 training counters | ⚠ | same proven RWC/RO patterns; registers not all populated |
| 4 / Fig 27 | Full training FSM, all arcs + timeouts | ✓ | F fsm:P1-P12 (Tables 37/38/42/43/47 exit disciplines) |
| 4.1.1.1 | 255 TX / 7-consecutive-RX detect exit; early exit on Link Speed | ✓ | F fsm:P4 + adopt-select (see below) |
| 4.1.1.2 | Highest-common speed; both sides converge | ✓ | F loopback L3/L4+T3 (agreement, one-hot, mutual support, DDR AND) |
| — (gap found) | Side exiting via received Link Speed must ADOPT its Speed Select | ✓ | sanitized adoption + loopback proof (documented spec finding) |
| 4.1.2.1 (2.1v1.0) | Advertise: 1 ms min TX; **100 ms** alignment timeout | ✓ | F fsm split params + props |
| 4.1.2.2 | Configure 31-frame budget → Advertise; Accept 15 | ✓ | F fsm:P10 + covers |
| 4.1.3 / T47 | Operational: soft reset→Advertise, retrain/hard→Detect | ✓ | F fsm:P5/P6, S tests 8/9 + CSR retrain |
| 4.1.x | Bring-up end-to-end (both sides reach Operational) | ✓ | F loopback covers (incl. 400M-DDR + 200M-SDR traces), S test 1 |
| phys/LVDS, 8b10b | Electrical layer, 10b symbol coding, DC balance | ✗ | SERDES/vendor layer (wrappers + constraints provided; OCP ref uses same split with encoder_8b10b.v at PHY) |

Cross-implementation check: constants and thresholds verified equal to the
official OCP reference RTL (`opencomputeproject/HWMgmt-Module-DCSCM-LTPI`):
comma bytes BC/DC/FC, subtype codes, 7/3/3 lock counts, 255 detect TX,
CSR error counters. The strongest possible future step is formal
equivalence checking (yosys miter + sby) between these modules and the
reference implementation's corresponding blocks.
