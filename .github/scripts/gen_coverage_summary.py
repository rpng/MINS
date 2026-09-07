"""Turn a gcovr json-summary into the unit test code coverage comment posted on each PR."""
import json, os, sys

MARKER = '<!-- mins-coverage-report -->'

# Directories we care about enough to give their own row. Order is the order shown, and the
# first matching prefix wins, so the per-sensor rows have to come before the update catch-all.
GROUPS = [
    ('update/cam', 'mins/src/update/cam/'),
    ('update/gps', 'mins/src/update/gps/'),
    ('update/lidar', 'mins/src/update/lidar/'),
    ('update/vicon', 'mins/src/update/vicon/'),
    ('update/wheel', 'mins/src/update/wheel/'),
    ('update', 'mins/src/update/'),
    ('state', 'mins/src/state/'),
    ('init', 'mins/src/init/'),
    ('options', 'mins/src/options/'),
    ('core', 'mins/src/core/'),
    ('sim', 'mins/src/sim/'),
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


def coloured(current, previous, covered, total):
    """This branch's coverage, green above master and red below. GitHub renders the colour as math."""
    counts = '(%d/%d)' % (covered, total)
    if previous is None or abs(current - previous) < 0.05:
        return '%.1f%% %s' % (current, counts)
    colour = 'green' if current > previous else 'red'
    return r'$\color{%s}{%.1f\%%}$ %s' % (colour, current, counts)


def render(report, baseline):
    totals = summarize(report)
    was = summarize(baseline) if baseline else {}
    lines = [MARKER, '## Unit Test Code Coverage', '',
             '| Area | master | this branch |',
             '|------|--------|-------------|']
    for name, _ in GROUPS + [('other', None)]:
        if name not in totals:
            continue
        line_covered, line_total = totals[name][0], totals[name][1]
        before = was.get(name)
        was_pct = pct(before[0], before[1]) if before else None
        lines.append('| `%s` | %s | %s |' % (
            name, 'n/a' if was_pct is None else '%.1f%%' % was_pct,
            coloured(pct(line_covered, line_total), was_pct, line_covered, line_total)))
    lines.append('')
    line_percent = report.get('line_percent', 0.0)
    was_overall = baseline.get('line_percent') if baseline else None
    lines.append('**Overall %s across `mins/src`, master is %s.**' % (
        coloured(line_percent, was_overall, report.get('line_covered', 0),
                 report.get('line_total', 0)),
        'n/a' if was_overall is None else '%.1f%%' % was_overall))
    lines.append('')
    if baseline is None:
        lines.append('No master baseline was available for this run. It fills in once a master '
                     'build has published a coverage report.')
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
