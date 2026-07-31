#!/usr/bin/env python3
"""
Generate calibration convergence plots from MINS _est/_std/_gt state files.

Usage:
  calib_plot.py --prefix <sensor_prefix> --calib-type <dt|ext|int_wheel|int_cam>
                --seeds-dir <dir> --output-png <path.png> --output-md <path.md>
                --title <str>

seeds-dir must contain numeric subdirectories (0/, 1/, ...) each holding the
MINS state output files produced with sys_save_state:=true.
"""
import argparse, os, sys, base64
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BLUE      = '#29B6F6'
BLUE_FILL = 0.18
ERR_COLOR = '#E53935'
GT_COLOR  = '#212121'
GRID_CLR  = '#E0E0E0'

LABELS = {
    'dt':        ['time offset (s)'],
    'ext':       ['φ_x (rad)', 'φ_y (rad)', 'φ_z (rad)', 'p_x (m)', 'p_y (m)', 'p_z (m)'],
    'int_wheel': ['left radius (m)', 'right radius (m)', 'baseline (m)'],
    'int_cam':   ['fu (px)', 'fv (px)', 'cu (px)', 'cv (px)', 'k₁', 'k₂', 'p₁', 'p₂'],
}

# Absolute error tolerance per calib_type (indexed by param order).
# A parameter passes if |error| <= k*sigma OR |error| <= abs_tol.
# Prevents physically tiny residuals from failing due to an overconfident filter.
ABS_TOL = {
    'dt':        [0.002],                                       # 2 ms
    'ext':       [0.005, 0.005, 0.005, 0.02, 0.02, 0.02],     # 5 mrad, 20 mm
    'int_wheel': [0.001, 0.001, 0.005],                        # 1 mm radii, 5 mm baseline
    'int_cam':   [2.0, 2.0, 2.0, 2.0, 0.002, 0.002, 0.002, 0.002],  # 2 px, 0.002 distortion
}


def load_txt(path):
    if not os.path.isfile(path):
        return None
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            rows.append([float(v) for v in line.split()])
    return np.array(rows) if rows else None


def jpl_to_rotmat(q):
    """JPL quaternion [qx qy qz qw] -> 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-w*z),    2*(x*z+w*y)],
        [2*(x*y+w*z),    1-2*(x*x+z*z),  2*(y*z-w*x)],
        [2*(x*z-w*y),    2*(y*z+w*x),    1-2*(x*x+y*y)],
    ])


def rot_to_rotvec(R):
    """Rotation matrix -> axis-angle vector [rad]."""
    # Rodrigues formula
    cos_a = np.clip((np.trace(R) - 1) / 2, -1, 1)
    angle = np.arccos(cos_a)
    if abs(angle) < 1e-10:
        return np.zeros(3)
    return angle / (2 * np.sin(angle)) * np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])


def process(calib_type, est, std_, gt):
    n = min(len(est), len(std_), len(gt))
    est, std_, gt = est[:n], std_[:n], gt[:n]
    t = est[:, 0] - est[0, 0]

    if calib_type == 'dt':
        err = (est[:, 1] - gt[:, 1]).reshape(-1, 1)
        s   = std_[:, 1].reshape(-1, 1)

    elif calib_type in ('int_wheel', 'int_cam'):
        err = est[:, 1:] - gt[:, 1:]
        s   = std_[:, 1:]

    elif calib_type == 'ext':
        # Position-only extrinsic (e.g. GPS lever arm): cols = [t, p0, p1, p2]
        if est.shape[1] == 4:
            err = est[:, 1:] - gt[:, 1:]
            s   = std_[:, 1:]
        else:
            # Full extrinsic: cols = [t, q0, q1, q2, q3, p0, p1, p2]
            err = np.zeros((n, 6))
            s   = np.zeros((n, 6))
            for i in range(n):
                R_e = jpl_to_rotmat(est[i, 1:5])
                R_g = jpl_to_rotmat(gt[i,  1:5])
                err[i, :3] = rot_to_rotvec(R_e @ R_g.T)
                err[i, 3:] = est[i, 5:8] - gt[i, 5:8]
                s[i, :3]   = std_[i, 1:4]
                s[i, 3:]   = std_[i, 4:7]
    else:
        raise ValueError(f'Unknown calib_type: {calib_type}')

    return t, err, s


def make_figure(all_t, all_err, all_s, labels, title, output_png):
    n = len(labels)
    if n <= 3:
        ncols, nrows = n, 1
    elif n <= 6:
        ncols, nrows = 3, (n + 2) // 3
    else:
        ncols, nrows = 4, (n + 3) // 4

    fig, axes = plt.subplots(nrows, ncols,
                             figsize=(3.5 * ncols, 2.8 * nrows),
                             squeeze=False)
    fig.suptitle(title, fontsize=11, fontweight='bold')

    flat = [axes[r][c] for r in range(nrows) for c in range(ncols)]
    first_handles = None

    for j, (ax, lab) in enumerate(zip(flat[:n], labels)):
        for k, (t, err, s) in enumerate(zip(all_t, all_err, all_s)):
            e  = err[:, j]
            sv = s[:, j]
            h1, = ax.plot(t, e, color=ERR_COLOR, alpha=0.8, linewidth=1.2,
                          label='error (seed)')
            h2  = ax.fill_between(t, -3*sv, 3*sv,
                                  color=BLUE, alpha=BLUE_FILL, label='±3σ bound')
            if k == 0 and j == 0:
                first_handles = [h1, h2]
        h3 = ax.axhline(0, color=GT_COLOR, linestyle='--', linewidth=0.8, label='GT (zero error)')
        if j == 0:
            first_handles.append(h3)

        ax.set_xlabel('time (s)', fontsize=8)
        ax.set_ylabel(lab, fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, color=GRID_CLR, linewidth=0.5)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

    for ax in flat[n:]:
        ax.set_visible(False)

    if first_handles:
        fig.legend(first_handles,
                   [h.get_label() for h in first_handles],
                   loc='lower right', fontsize=7, framealpha=0.8)

    plt.tight_layout()
    plt.savefig(output_png, dpi=150, bbox_inches='tight')
    plt.close()


def convergence_ok(all_err, all_s, calib_type, k=3.0):
    tols = ABS_TOL.get(calib_type, [0.0])
    for err, s in zip(all_err, all_s):
        e_last = np.abs(err[-1])
        s_last = s[-1]
        for j, (e_j, s_j) in enumerate(zip(e_last, s_last)):
            tol = tols[j] if j < len(tols) else 0.0
            if e_j > k * s_j and e_j > tol:
                return False
    return True


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--prefix',     required=True)
    p.add_argument('--calib-type', required=True,
                   choices=['dt', 'ext', 'int_wheel', 'int_cam'])
    p.add_argument('--seeds-dir',  required=True)
    p.add_argument('--output-png', required=True)
    p.add_argument('--output-md',  required=True)
    p.add_argument('--title',      required=True)
    args = p.parse_args()

    seeds = sorted(d for d in os.listdir(args.seeds_dir)
                   if d.isdigit() and
                   os.path.isdir(os.path.join(args.seeds_dir, d)))
    if not seeds:
        sys.exit(f'No seed dirs in {args.seeds_dir}')

    all_t, all_err, all_s = [], [], []
    for seed in seeds:
        d   = os.path.join(args.seeds_dir, seed)
        est = load_txt(os.path.join(d, f'{args.prefix}_est.txt'))
        std_= load_txt(os.path.join(d, f'{args.prefix}_std.txt'))
        gt  = load_txt(os.path.join(d, f'{args.prefix}_gt.txt'))
        if est is None or std_ is None or gt is None:
            print(f'seed {seed}: missing files, skip', file=sys.stderr)
            continue
        t, err, s = process(args.calib_type, est, std_, gt)
        all_t.append(t); all_err.append(err); all_s.append(s)

    if not all_t:
        sys.exit('No usable seed data')

    # Position-only ext (e.g. GPS): err has 3 cols not 6
    if args.calib_type == 'ext' and all_err[0].shape[1] == 3:
        labels = ['p_x (m)', 'p_y (m)', 'p_z (m)']
    else:
        labels = LABELS[args.calib_type]
    make_figure(all_t, all_err, all_s, labels, args.title, args.output_png)

    # Per-param convergence table
    ok_overall = convergence_ok(all_err, all_s, args.calib_type)
    icon = '✅' if ok_overall else '❌'

    mean_err = np.mean([np.abs(e[-1]) for e in all_err], axis=0)
    mean_std = np.mean([s[-1]        for s in all_s],    axis=0)
    tols = ABS_TOL.get(args.calib_type, [0.0])

    rows = []
    for j, lab in enumerate(labels):
        e   = mean_err[j] if j < len(mean_err) else float('nan')
        sv  = mean_std[j] if j < len(mean_std) else float('nan')
        tol = tols[j] if j < len(tols) else 0.0
        tick = '✅' if e <= 3 * sv or e <= tol else '❌'
        rows.append(f'| {lab} | {e:.3g} | {sv:.3g} | {tick} |')

    md = '\n'.join([
        f'### {icon} {args.title}',
        '',
        '| Parameter | final |error| | final σ | ±3σ |',
        '|---|---|---|---|',
    ] + rows)

    with open(args.output_md, 'w') as f:
        f.write(md)

    # Embed plot into step summary if the env var is set
    summary = os.environ.get('GITHUB_STEP_SUMMARY')
    if summary:
        with open(args.output_png, 'rb') as f:
            b64 = base64.b64encode(f.read()).decode()
        with open(summary, 'a') as f:
            f.write(f'\n## {args.title}\n')
            f.write(f'<img src="data:image/png;base64,{b64}" alt="{args.title}">\n\n')


if __name__ == '__main__':
    main()
