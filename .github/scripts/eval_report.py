#!/usr/bin/env python3
"""Render a master-vs-PR markdown block from mins_eval run_comparison output.

Parses the LaTeX ATE/NEES/Time table that `run_comparison` (viz_type:=0) prints
and emits a collapsible markdown section comparing the MINS_master and MINS_PR
"algorithms" for one sensor config. Informational only -- never fails the build.

Usage: eval_report.py <config_name> <run_comparison_stdout.txt>
"""
import re
import sys

FLOAT = re.compile(r"-?\d+\.?\d*(?:[eE][-+]?\d+)?")


def parse(path):
    """Return {'master': {...}, 'pr': {...}} with rmse/nees/time number lists."""
    rows = {}
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            m = re.match(r"^MINS\\?_(\w+)\s*&(.*)", line)
            if not m:
                continue
            alg = m.group(1).lower()                 # master | pr
            rest = m.group(2).split("\\\\")[0]       # drop everything after the row terminator
            rest = (rest.replace("\\textbf{", "")
                        .replace("\\colorbox{orange}{", "")
                        .replace("}", ""))
            cells = rest.split("&")
            nums = lambda s: [float(x) for x in FLOAT.findall(s)]
            rows[alg] = {
                "rmse": nums(cells[0]) if len(cells) > 0 else [],   # [ori_mean, ori_std, pos_mean, pos_std]
                "nees": nums(cells[1]) if len(cells) > 1 else [],   # [ori_mean, ori_std, pos_mean, pos_std]
                "time": nums(cells[2]) if len(cells) > 2 else [],   # [mean, std]
            }
    return rows


def cell(mean, std):
    return f"{mean:.3f} +/- {std:.3f}"


def delta(pr_mean, ma_mean):
    if ma_mean == 0:
        return "n/a"
    pct = (pr_mean - ma_mean) / abs(ma_mean) * 100.0
    sign = "+" if pct >= 0 else ""
    return f"{sign}{pct:.1f}%"


def main():
    config = sys.argv[1]
    rows = parse(sys.argv[2])
    out = [f"<details><summary><b>{config}</b></summary>\n"]

    if "master" not in rows or "pr" not in rows:
        out.append(f"> Note: could not parse results for `{config}` "
                   "(a master or PR run may have failed - see job logs).\n")
        out.append("</details>")
        print("\n".join(out))
        return

    out.append("| Metric | master | PR | Delta (mean) |")
    out.append("|---|---|---|---|")

    def row(label, key, mi, si):
        ma, pr = rows["master"][key], rows["pr"][key]
        if len(ma) <= mi or len(pr) <= mi:
            return
        out.append(f"| {label} | {cell(ma[mi], ma[si])} | {cell(pr[mi], pr[si])} "
                   f"| {delta(pr[mi], ma[mi])} |")

    row("RMSE ori (deg)", "rmse", 0, 1)
    row("RMSE pos (m)",  "rmse", 2, 3)
    row("NEES ori",      "nees", 0, 1)
    row("NEES pos",      "nees", 2, 3)
    row("Time (s)",      "time", 0, 1)
    out.append("\n</details>")
    print("\n".join(out))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # never break the report job
        cfg = sys.argv[1] if len(sys.argv) > 1 else "?"
        print(f"<details><summary><b>{cfg}</b></summary>\n\n"
              f"> Note: report parsing error: `{exc}`\n\n</details>")
