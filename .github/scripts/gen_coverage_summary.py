"""Turn a gcovr json-summary into the unit test code coverage comment posted on each PR."""
import json, os, sys

MARKER = '<!-- mins-coverage-report -->'

# Directories we care about enough to give their own row. Order is the order shown.
GROUPS = [
    ('update/wheel', 'mins/src/update/wheel/'),
    ('update', 'mins/src/update/'),
    ('state', 'mins/src/state/'),
    ('init', 'mins/src/init/'),
    ('options', 'mins/src/options/'),
    ('core', 'mins/src/core/'),
    ('utils', 'mins/src/utils/'),
]


def pct(covered, total):
    return 100.0 * covered / total if total else 0.0


def group_of(filename):
    for name, prefix in GROUPS:
        if prefix in filename:
            return name
    return 'other'


def summarize(report):
    """Accumulate per-file line/branch counts into the group buckets."""
    totals = {}
    for entry in report.get('files', []):
        name = group_of(entry['filename'])
        bucket = totals.setdefault(name, [0, 0, 0, 0])
        bucket[0] += entry.get('line_covered', 0)
        bucket[1] += entry.get('line_total', 0)
        bucket[2] += entry.get('branch_covered', 0)
        bucket[3] += entry.get('branch_total', 0)
    return totals


def delta(current, previous):
    """Change against master, coloured by direction. GitHub renders the colour as inline math."""
    if previous is None:
        return 'n/a'
    change = current - previous
    if abs(change) < 0.05:
        return 'same'
    return r'$\color{%s}%+.1f$' % ('green' if change > 0 else 'red', change)


def render(report, baseline):
    totals = summarize(report)
    was = summarize(baseline) if baseline else {}
    lines = [MARKER, '## Unit Test Code Coverage', '',
             '| Area | Lines | Line % | vs master | Branches | Branch % | vs master |',
             '|------|-------|--------|-----------|----------|----------|-----------|']
    for name, _ in GROUPS + [('other', None)]:
        if name not in totals:
            continue
        line_covered, line_total, branch_covered, branch_total = totals[name]
        before = was.get(name)
        lines.append('| `%s` | %d/%d | %.1f%% | %s | %d/%d | %.1f%% | %s |' % (
            name, line_covered, line_total, pct(line_covered, line_total),
            delta(pct(line_covered, line_total), pct(before[0], before[1]) if before else None),
            branch_covered, branch_total, pct(branch_covered, branch_total),
            delta(pct(branch_covered, branch_total), pct(before[2], before[3]) if before else None)))
    lines.append('')
    line_percent = report.get('line_percent', 0.0)
    branch_percent = report.get('branch_percent', 0.0)
    lines.append('**Overall %.1f%% lines (%s), %.1f%% branches (%s)** across `mins/src`.' % (
        line_percent, delta(line_percent, baseline.get('line_percent') if baseline else None),
        branch_percent, delta(branch_percent, baseline.get('branch_percent') if baseline else None)))
    lines.append('')
    if baseline is None:
        lines.append('No master baseline was available for this run, so the comparison columns '
                     'are empty. They fill in once a master build has published a report.')
        lines.append('')
    lines.append('Full HTML report is in the `coverage-report` artifact of this run.')
    return '\n'.join(lines)


json_path = sys.argv[1]
out_file = sys.argv[2] if len(sys.argv) > 2 else None
baseline_path = sys.argv[3] if len(sys.argv) > 3 else None

baseline = None
if baseline_path and os.path.exists(baseline_path):
    baseline = json.load(open(baseline_path))

if os.path.exists(json_path):
    summary = render(json.load(open(json_path)), baseline)
else:
    summary = '\n'.join([MARKER, '## Unit Test Code Coverage', '',
                         '*Report not generated - check the CI log.*'])

print(summary)

gss = os.environ.get('GITHUB_STEP_SUMMARY')
if gss:
    with open(gss, 'a') as f:
        f.write(summary + '\n')
if out_file:
    with open(out_file, 'w') as f:
        f.write(summary + '\n')
