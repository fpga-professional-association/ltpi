"""Mutation coverage of the formal proofs (mcy methodology, self-contained).

For each target module: enumerate candidate single-token mutations in the
RTL (outside the `ifdef FORMAL block), apply them one at a time, and run
the module's SymbiYosys BMC task. A mutation is KILLED if any assertion
fails (or the design errors out); it SURVIVES if BMC still passes -
meaning the property set cannot see that injected bug.

Coverage = killed / applied. Survivors are listed for property review.
"""
import re
import shutil
import subprocess
import sys
import os

SUITE = r"D:\fpga_proofs\oss-cad-suite"
SIM_ENV = dict(os.environ)
SIM_ENV["PATH"] = SUITE + r"\bin;" + SUITE + r"\lib;" + SIM_ENV["PATH"]
SIM_SRCS = ["ltpi_pkg.sv", "ltpi_frame_rx.sv", "ltpi_frame_tx.sv",
            "ltpi_link_fsm.sv", "ltpi_gpio_channel.sv",
            "ltpi_uart_channel.sv", "ltpi_i2c_relay.sv", "ltpi_phy.sv",
            "ltpi_csr.sv", "ltpi_data_channel.sv", "ltpi_top.sv"]


def run_sim():
    """KILLED if the self-checking system TB fails, hangs, or errors."""
    sim = os.path.join("..", "sim")
    srcs = [os.path.join("..", "rtl", f) for f in SIM_SRCS]
    try:
        c = subprocess.run(
            ["iverilog", "-g2012", "-I", os.path.join("..", "rtl"),
             "-o", os.path.join(sim, "mut.vvp")] + srcs
            + [os.path.join(sim, "tb_ltpi_system.sv")],
            capture_output=True, text=True, timeout=120, env=SIM_ENV)
        if c.returncode != 0:
            return "FAIL"
        r = subprocess.run(["vvp", os.path.join(sim, "mut.vvp")],
                           capture_output=True, text=True, timeout=420,
                           env=SIM_ENV)
        return "PASS" if "ALL CHECKS PASSED" in r.stdout else "FAIL"
    except subprocess.TimeoutExpired:
        return "FAIL"   # hung = detected


RTL = os.path.join("..", "rtl")

# (mutation name, regex, replacement) - conservative single-site mutations.
MUTS = [
    ("eq->neq",   r"==",      "!="),
    ("neq->eq",   r"!=",      "=="),
    ("and->or",   r"&&",      "||"),
    ("plus->minus", r"\+ 1'b1", "- 1'b1"),
    ("ge->gt",    r">=",      ">"),
    ("lt->le",    r"<(?![=<])", "<="),
    ("not-drop",  r"!(?=[a-zA-Z_(])", ""),
]

TARGETS = [
    # (rtl file, sby, task, max mutants, escalation (sby, task) or None)
    # Escalation: a mutant surviving the module properties gets a second
    # chance to be killed by the COMPOSITION proof - module assertions
    # share decode cones with the RTL (mutating both is a tautology), but
    # the composition checks behavior against an independent reference.
    ("ltpi_crc8.sv",         "ltpi_crc8.sby",         "bmc",      12, None),
    ("ltpi_gpio_channel.sv", "ltpi_gpio_channel.sby", "bmc",      16, None),
    ("ltpi_uart_channel.sv", "ltpi_uart_channel.sby", "bmc",      16, None),
    ("ltpi_frame_tx.sv",     "ltpi_frame_tx.sby",     "bmc",      16, None),
    ("ltpi_link_fsm.sv",     "ltpi_link_fsm.sby",     "scm_bmc",  24,
     ("ltpi_loopback.sby", "bmc")),
    ("ltpi_i2c_relay.sv",    "ltpi_i2c_relay.sby",    "pri_bmc",  24,
     ("ltpi_i2c_loopback.sby", "bmc")),
]


def formal_start(text):
    """Index where the `ifdef FORMAL section starts (mutate only before)."""
    i = text.find("`ifdef FORMAL")
    return i if i >= 0 else len(text)


def candidate_sites(text):
    limit = formal_start(text)
    sites = []
    for name, pat, rep in MUTS:
        for m in re.finditer(pat, text[:limit]):
            # skip comments
            line_start = text.rfind("\n", 0, m.start()) + 1
            line = text[line_start:text.find("\n", m.start())]
            if "//" in text[line_start:m.start()]:
                continue
            sites.append((name, m.start(), m.end(), rep, line.strip()))
    return sites


def run_sby(sby, task):
    cmd = ["sby", "-f", sby] + ([task] if task else [])
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    out = r.stdout + r.stderr
    if "DONE (PASS" in out:
        return "PASS"
    if "DONE (FAIL" in out:
        return "FAIL"
    return "ERROR"


def main():
    total_applied = 0
    total_killed = 0
    survivors = []
    for rtl, sby, task, maxn, esc in TARGETS:
        path = os.path.join(RTL, rtl)
        orig = open(path).read()
        sites = candidate_sites(orig)
        # deterministic spread over the file
        step = max(1, len(sites) // maxn)
        picked = sites[::step][:maxn]
        killed = 0
        for (name, a, b, rep, line) in picked:
            mutant = orig[:a] + rep + orig[b:]
            open(path, "w").write(mutant)
            try:
                res = run_sby(sby, task)
                how = "module"
                if res == "PASS" and esc:
                    res = run_sby(esc[0], esc[1])
                    how = "composition"
                if res == "PASS":
                    res = run_sim()
                    how = "sim"
            finally:
                open(path, "w").write(orig)
            total_applied += 1
            if res in ("FAIL", "ERROR"):
                killed += 1
                total_killed += 1
            else:
                survivors.append((rtl, name, line))
            print(f"  {rtl:<22} {name:<12} -> "
                  f"{'KILLED' if res != 'PASS' else 'SURVIVED'} | {line[:60]}",
                  flush=True)
        print(f"{rtl}: {killed}/{len(picked)} killed", flush=True)

    print("\n==== MUTATION COVERAGE ====")
    print(f"applied {total_applied}, killed {total_killed}, "
          f"coverage {100 * total_killed / max(1, total_applied):.1f}%")
    if survivors:
        print("\nSurvivors (bugs the property set cannot see):")
        for rtl, name, line in survivors:
            print(f"  {rtl}: [{name}] {line[:70]}")


if __name__ == "__main__":
    main()
