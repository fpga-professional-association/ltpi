# Simulation

`run.sh` compiles and runs the end-to-end testbench with Icarus Verilog.

```bash
sim/run.sh        # -> "PASS 9/9  (LTPI SCM<->HPM end-to-end)"
```

`tb_ltpi_link.sv` instantiates `ltpi_scm_top` and `ltpi_hpm_top`, cross-connects
their serial pads on a shared clock, and drives every channel through the live
link (with a small `ADVERTISE_CYCLES` so training completes quickly):

1. link trains to **Operational** on both sides;
2. **LL + NL GPIO** tunnel in both directions;
3. a **UART** TXD line level tunnels SCM→HPM;
4. an **I2C START** event relays SCM→HPM and is regenerated on the HPM bus;
5. an **Avalon-MM read** issued on the SCM returns HPM memory via the **Data channel**.

All RTL is pulled in via include-guarded `` `include `` from the two tops, so only
the testbench file is passed to `iverilog` (`-I rtl` resolves the includes).
The latest run is captured in [`../reports/sim_run.log`](../reports/sim_run.log).
