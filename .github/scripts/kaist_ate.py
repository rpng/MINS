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
SEQS = list(range(18, 40))
_PAPER = {
    "ICGLW": "1.52 3.46 0.85 0.79 1.49 1.38 1.85 3.44 1.06 1.72 0.79 2.58 1.83 0.70 0.77 1.16 0.94 5.46 2.38 0.60 0.64 0.56",
    "ICW": "7.13 8.74 8.24 8.47 9.16 7.10 8.17 10.73 1.57 3.85 1.10 3.16 2.03 29.79 4.15 2.46 2.43 8.15 16.47 18.50 0.94 1.23",
    "ILW": "6.96 8.11 4.20 2.99 6.90 4.24 6.85 3.76 2.24 4.15 2.71 4.39 5.47 5.01 4.73 7.92 7.86 6.13 12.01 7.96 3.24 2.50",
    "IC": "- - - - - - - - 2.04 2.86 1.45 3.43 3.50 5.39 4.69 1.98 1.47 - - - 1.13 1.17",
    "IL": "- - - - - - - - 2.69 2.97 2.53 3.96 6.21 5.11 5.68 9.18 5.30 - - - 3.50 2.17",
}
PAPER = {c: dict(zip(SEQS, [None if x == "-" else float(x) for x in v.split()])) for c, v in _PAPER.items()}


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
    for c in PAPER:
        try:
            est = np.loadtxt(os.path.join(out_dir, c + ".txt"), usecols=range(4), ndmin=2)
            ate, n = ate_se3(gt, est)
            # A run that died early scores well on its short prefix, so report coverage too
            cover = float((est[-1, 0] - gt[0, 0]) / (gt[-1, 0] - gt[0, 0]))
            res[c] = {"ate": ate, "m_km": ate / km, "poses": n, "cover": cover}
        except Exception as e:
            res[c] = {"error": str(e)[:80]}
    print(json.dumps(res))


def report(results_dir):
    rows = {}
    for path in glob.glob(os.path.join(results_dir, "*", "score.json")):
        try:
            r = json.loads(open(path).read().strip().splitlines()[-1])
            rows[int(r["seq"])] = r
        except (IndexError, ValueError, KeyError):
            print("skipping unreadable %s\n" % path)

    def cell(seq, c):
        x = rows.get(seq, {}).get(c, {})
        if "m_km" not in x:
            return "fail"
        s = "%.2f" % x["m_km"]
        return s if x["cover"] > 0.98 else s + " (%d%%)" % (100 * x["cover"])

    print("# KAIST Urban vs MINS paper Table 6 (SE3 ATE per GT km)\n")
    print("(nn%) = run ended early, value covers only that fraction.\n")
    for c in PAPER:
        full = {s: rows[s][c]["m_km"] for s in SEQS
                if PAPER[c][s] is not None and rows.get(s, {}).get(c, {}).get("cover", 0) > 0.98}
        close = sum(abs(v / PAPER[c][s] - 1) <= 0.2 for s, v in full.items())
        total = sum(p is not None for p in PAPER[c].values())
        print("## %s: %d/%d within 20%% of paper\n" % (c, close, total))
        print("| seq | km | paper | this run |")
        print("|---|---|---|---|")
        for seq in SEQS:
            p = PAPER[c][seq]
            km = "%.2f" % rows[seq]["km"] if seq in rows else "-"
            print("| urban%d | %s | %s | %s |" % (seq, km, "-" if p is None else "%.2f" % p, cell(seq, c)))
        if full:
            print("| **mean (%d seq)** | | %.2f | %.2f |" % (len(full), sum(PAPER[c][s] for s in full) / len(full),
                                                          sum(full.values()) / len(full)))
        print()


if __name__ == "__main__":
    if sys.argv[1] == "report":
        report(sys.argv[2])
    else:
        score(int(sys.argv[1]), sys.argv[2], sys.argv[3])
