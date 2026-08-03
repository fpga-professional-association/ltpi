"""Per-file line-coverage summary + uncovered-line listing from an lcov
coverage.info produced by verilator_coverage --write-info."""
import collections
import sys

files = collections.defaultdict(lambda: [0, 0])
uncov = collections.defaultdict(list)
cur = None
for line in open("coverage.info"):
    line = line.strip()
    if line.startswith("SF:"):
        cur = line[3:].replace("\\", "/").split("/")[-1]
    elif line.startswith("DA:"):
        num, cnt = line[3:].split(",")
        files[cur][1] += 1
        if int(cnt) > 0:
            files[cur][0] += 1
        else:
            uncov[cur].append(int(num))

print(f"{'file':<24}{'hit':>6}{'total':>7}{'line%':>8}")
th = tt = 0
for k in sorted(files):
    h, t = files[k]
    th += h
    tt += t
    print(f"{k:<24}{h:>6}{t:>7}{100*h/t:>7.1f}%")
print(f"{'TOTAL':<24}{th:>6}{tt:>7}{100*th/tt:>7.1f}%")

if "-u" in sys.argv:
    print("\nUncovered lines per RTL file (excluding testbench):")
    for k in sorted(uncov):
        if k.startswith("tb_"):
            continue
        runs = []
        for n in sorted(uncov[k]):
            if runs and n == runs[-1][1] + 1:
                runs[-1][1] = n
            else:
                runs.append([n, n])
        txt = ", ".join(f"{a}" if a == b else f"{a}-{b}" for a, b in runs)
        print(f"  {k}: {txt}")
