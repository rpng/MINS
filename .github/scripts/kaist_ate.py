"""KAIST Urban accuracy vs Table 6 of the MINS paper (arXiv 2309.15390).

Metric: position ATE after SE3 alignment divided by the GT path length (m/km).

usage: kaist_ate.py <seq> <gt.txt> <out dir>   -> JSON line for one sequence
       kaist_ate.py report <results dir>        -> markdown tables vs the paper
"""
import glob
import json
import os
import sys

# Table 6 (m/km), from the paper's stored trajectories scored with this script (matches the printed
# table to +-0.01). "-" = the paper has no result (camera/lidar-only on highways).
# HTML collapses runs of plain spaces, so the padding that keeps the columns equal has to be hard spaces.
NBSP = "\u00a0"

SEQS = list(range(18, 40))
_PAPER = {
    "IC": "- - - - - - - - 2.04 2.86 1.45 3.43 3.50 5.39 4.69 1.98 1.47 - - - 1.13 1.17",
    "ICW": "7.13 8.74 8.24 8.47 9.16 7.10 8.17 10.73 1.57 3.85 1.10 3.16 2.03 29.79 4.15 2.46 2.43 8.15 16.47 18.50 0.94 1.23",
    "IL": "- - - - - - - - 2.69 2.97 2.53 3.96 6.21 5.11 5.68 9.18 5.30 - - - 3.50 2.17",
    "ILW": "6.96 8.11 4.20 2.99 6.90 4.24 6.85 3.76 2.24 4.15 2.71 4.39 5.47 5.01 4.73 7.92 7.86 6.13 12.01 7.96 3.24 2.50",
    "ICGLW": "1.52 3.46 0.85 0.79 1.49 1.38 1.85 3.44 1.06 1.72 0.79 2.58 1.83 0.70 0.77 1.16 0.94 5.46 2.38 0.60 0.64 0.56",
}
PAPER = {c: dict(zip(SEQS, [None if x == "-" else float(x) for x in v.split()])) for c, v in _PAPER.items()}
REFS = ("master", "pr")


def ate_se3(gt, est):
    import numpy as np
    idx = np.clip(np.searchsorted(gt[:, 0], est[:, 0]), 1, len(gt) - 1)
    idx -= (est[:, 0] - gt[idx - 1, 0]) < (gt[idx, 0] - est[:, 0])
    ok = np.abs(gt[idx, 0] - est[:, 0]) < 0.01
    p_e, p_g = est[ok, 1:4], gt[idx[ok], 1:4]
    ce, cg = p_e.mean(0), p_g.mean(0)
    U, _, Vt = np.linalg.svd((p_g - cg).T @ (p_e - ce))
    R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    err = (p_e - ce) @ R.T + cg - p_g
    return float(np.sqrt((err ** 2).sum(1).mean())), int(ok.sum())


def score(seq, gt_path, out_dir):
    import numpy as np
    gt = np.loadtxt(gt_path, usecols=range(4))
    km = float(np.linalg.norm(np.diff(gt[:, 1:4], axis=0), axis=1).sum() / 1000)
    res = {"seq": seq, "km": km}
    for name in ("%s_%s" % (r, c) for r in REFS for c in PAPER):
        try:
            est = np.loadtxt(os.path.join(out_dir, name + ".txt"), usecols=range(4), ndmin=2)
            ate, n = ate_se3(gt, est)
            # A run that died early scores well on its short prefix, so report coverage too
            cover = float((est[-1, 0] - gt[0, 0]) / (gt[-1, 0] - gt[0, 0]))
            res[name] = {"ate": ate, "m_km": ate / km, "poses": n, "cover": cover}
        except Exception as e:
            res[name] = {"error": str(e)[:80]}
    print(json.dumps(res))


def report(results_dir):
    rows = {}
    for path in glob.glob(os.path.join(results_dir, "*", "score.json")):
        try:
            r = json.loads(open(path).read().strip().splitlines()[-1])
            rows[int(r["seq"])] = r
        except (IndexError, ValueError, KeyError):
            print("skipping unreadable %s\n" % path)

    def val(seq, name):
        x = rows.get(seq, {}).get(name, {})
        return x["m_km"] if x.get("cover", 0) > 0.98 else None

    def fmt(v):
        return "%.2f" % v if v < 1000 else ">1000"

    def cell(seq, name, paper):
        x = rows.get(seq, {}).get(name, {})
        if "m_km" not in x:
            return "-" if paper is None else "fail"
        # a diverged run's exact error means nothing, and printing it widens every column
        s = "%.2f" % x["m_km"] if x["m_km"] < 1000 else ">1000"
        return s if x["cover"] > 0.98 else s + "*"

    def badge(v, color):
        return '<img src="https://img.shields.io/badge/%s-%s?style=flat-square" width="48" height="20" alt="%s">' % (
            v.replace("-", "--").replace(">", "%3E"), color, v)

    def num(x):
        if x == ">1000":
            return float("inf")
        try:
            return float(x)
        except ValueError:
            return None  # fail, missing or partial coverage: no call to make

    def tint(pr, master):
        a, b = num(pr), num(master)
        if a is None or b is None:
            return pr
        return ("badge", pr, "9f9f9f" if pr == master else "e05d44" if a > b else "4c1")

    def emit(table, lead=2):
        """Print a table of cells, padding every value to one width so all columns match.

        GitHub's mobile app scales a cell image to the width of its column, and a column is
        as wide as its widest text, so without this the badges come out different sizes.
        The first lead columns are labels and stay plain text.
        """
        width = max(len(c) for row in table[1:] for c in row[lead:] if isinstance(c, str))

        def render(c, n, i):
            if n == 0 or i < lead:
                return c
            return "`%s`" % c.rjust(width, NBSP) if isinstance(c, str) else badge(c[1], c[2])

        for n, row in enumerate(table):
            print("| %s |" % " | ".join(render(c, n, i) for i, c in enumerate(row)))
            if n == 0:
                print("|" + "---|" * len(row))

    print("# KAIST Urban vs MINS paper Table 6 (SE3 ATE per GT km)\n")
    print("Mean m/km over the sequences where the paper, master and PR all have a full-length result.\n")
    table = [["config", "seq", "paper"] + list(REFS)]
    for c in PAPER:
        done = [s for s in SEQS if PAPER[c][s] is not None and all(val(s, r + "_" + c) is not None for r in REFS)]
        means = [fmt(sum(val(s, r + "_" + c) for s in done) / len(done)) if done else "-" for r in REFS]
        paper = "%.2f" % (sum(PAPER[c][s] for s in done) / len(done)) if done else "-"
        means[-1] = tint(means[-1], means[0])
        table.append([c, str(len(done)), paper] + means)
    emit(table)
    print("\n- = no result in the paper (and none here), fail = no trajectory, * = run ended early, >1000 = diverged. Green = PR better than master, grey = same, red = worse.\n")
    table = [["config", "row"] + [str(s) for s in SEQS]]
    table.append(["", "km"] + ["%.1f" % rows[s]["km"] if s in rows else "-" for s in SEQS])
    for c in PAPER:
        table.append([c, "paper"] + ["-" if PAPER[c][s] is None else "%.2f" % PAPER[c][s] for s in SEQS])
        for r in REFS:
            row = [cell(s, r + "_" + c, PAPER[c][s]) for s in SEQS]
            if r == "pr":
                row = [tint(x, cell(s, "master_" + c, PAPER[c][s])) for s, x in zip(SEQS, row)]
            table.append(["", r] + row)
    emit(table)

if __name__ == "__main__":
    if sys.argv[1] == "report":
        report(sys.argv[2])
    else:
        score(int(sys.argv[1]), sys.argv[2], sys.argv[3])
