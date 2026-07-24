import re, os, sys

text = open(sys.argv[1]).read() if len(sys.argv) > 1 and os.path.exists(sys.argv[1]) else ''
tests = re.findall(r'\d+/\d+ Test #\d+: (\S+)[\s.]+(\w+)\s+([\d.]+) sec', text)
total_line = re.search(r'(\d+)% tests passed, (\d+) tests failed out of (\d+)', text)

lines = ['## Unit Test Results', '', '| Test | Result | Duration |', '|------|--------|----------|']
for name, status, duration in tests:
    icon = '✅' if status == 'Passed' else '❌'
    lines.append(f'| `{name}` | {icon} {status} | {duration}s |')
lines.append('')
if total_line:
    _, failed, total = total_line.groups()
    passed = int(total) - int(failed)
    lines.append(f'**{passed}/{total} passed**')
elif not tests:
    lines.append('*No test results found (build may have failed)*')

summary = '\n'.join(lines)
print(summary)
gss = os.environ.get('GITHUB_STEP_SUMMARY')
if gss:
    with open(gss, 'a') as f:
        f.write(summary + '\n')
