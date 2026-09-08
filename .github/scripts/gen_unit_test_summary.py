import glob, re, os, sys
import xml.etree.ElementTree as ET

text = open(sys.argv[1]).read() if len(sys.argv) > 1 and os.path.exists(sys.argv[1]) else ''
out_file = sys.argv[2] if len(sys.argv) > 2 else None
# Directory the test binaries drop their GTEST_OUTPUT xml into. ctest counts binaries, which
# says 3 no matter how many cases are in them, so the case counts come from the xml instead.
xml_dir = sys.argv[3] if len(sys.argv) > 3 else None

tests = re.findall(r'\d+/\d+ Test #\d+: (\S+)[\s.]+(\w+)\s+([\d.]+) sec', text)
total_line = re.search(r'(\d+)% tests passed, (\d+) tests failed out of (\d+)', text)


def read_cases(directory):
    """{binary name: (passed, total)} plus the names of the cases that did not pass."""
    counts, failures = {}, []
    for path in sorted(glob.glob(os.path.join(directory or '', '*.xml'))):
        root = ET.parse(path).getroot()
        total = int(root.get('tests', 0)) - int(root.get('disabled', 0))
        bad = int(root.get('failures', 0)) + int(root.get('errors', 0))
        counts[os.path.splitext(os.path.basename(path))[0]] = (total - bad, total)
        for case in root.iter('testcase'):
            if case.find('failure') is not None or case.find('error') is not None:
                failures.append(case.get('classname') + '.' + case.get('name'))
    return counts, failures


cases, failed_cases = read_cases(xml_dir) if xml_dir else ({}, [])

lines = ['<!-- mins-unit-test-report -->', '## Unit Test Results', '',
         '| Test | Cases | Result | Duration |', '|------|-------|--------|----------|']
for name, status, duration in tests:
    icon = '✅' if status == 'Passed' else '❌'
    passed, total = cases.get(name, (None, None))
    count = f'{passed}/{total}' if total is not None else '-'
    lines.append(f'| `{name}` | {count} | {icon} {status} | {duration}s |')
lines.append('')
if failed_cases:
    lines.append('Failed cases:')
    lines += [f'- `{name}`' for name in failed_cases]
    lines.append('')
if total_line:
    _, failed, total = total_line.groups()
    passed = int(total) - int(failed)
    tally = f'**{passed}/{total} test binaries passed**'
    if cases:
        case_total = sum(t for _, t in cases.values())
        case_passed = sum(p for p, _ in cases.values())
        tally += f', {case_passed}/{case_total} cases'
    lines.append(tally)
elif not tests:
    lines.append('*No test results found (build may have failed)*')

summary = '\n'.join(lines)
print(summary)

gss = os.environ.get('GITHUB_STEP_SUMMARY')
if gss:
    with open(gss, 'a') as f:
        f.write(summary + '\n')
if out_file:
    with open(out_file, 'w') as f:
        f.write(summary + '\n')
