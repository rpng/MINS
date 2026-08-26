"""Turn a gcovr json-summary into the markdown coverage comment posted on each PR."""
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


def render(report):
    totals = summarize(report)
    lines = [MARKER, '## Coverage', '', '| Area | Lines | Line % | Branch % |',
             '|------|-------|--------|----------|']
    for name, _ in GROUPS + [('other', None)]:
        if name not in totals:
            continue
        line_covered, line_total, branch_covered, branch_total = totals[name]
        lines.append('| `%s` | %d/%d | %.1f%% | %.1f%% |' % (
            name, line_covered, line_total, pct(line_covered, line_total),
            pct(branch_covered, branch_total)))
    lines.append('')
    lines.append('**Overall %.1f%% lines, %.1f%% branches** across `mins/src`.' % (
        report.get('line_percent', 0.0), report.get('branch_percent', 0.0)))
    lines.append('')
    lines.append('Full HTML report is in the `coverage-html` artifact of this run.')
    return '\n'.join(lines)


json_path = sys.argv[1]
out_file = sys.argv[2] if len(sys.argv) > 2 else None

if os.path.exists(json_path):
    summary = render(json.load(open(json_path)))
else:
    summary = '\n'.join([MARKER, '## Coverage', '',
                         '*Report not generated - check the CI log.*'])

print(summary)

gss = os.environ.get('GITHUB_STEP_SUMMARY')
if gss:
    with open(gss, 'a') as f:
        f.write(summary + '\n')
if out_file:
    with open(out_file, 'w') as f:
        f.write(summary + '\n')
