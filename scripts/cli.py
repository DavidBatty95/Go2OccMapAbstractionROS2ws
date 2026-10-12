#!/usr/bin/env python3
"""Build and run the self-contained Docker/ROS 2 Go2 simulation workspace."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
COMPOSE = ['docker', 'compose', '--project-directory', str(ROOT), '-f', str(ROOT/'compose.yaml')]


def runtime_hash():
    paths = [p for folder in ('src', 'docker') for p in (ROOT/folder).rglob('*')
             if p.is_file() and '__pycache__' not in p.parts]
    digest = hashlib.sha256()
    for path in sorted(paths):
        digest.update(str(path.relative_to(ROOT)).encode()+b'\0'+path.read_bytes())
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('setup', help='download pinned public assets and build/install both ROS packages')
    sub.add_parser('test', help='run geometry, controller and physics tests')
    sub.add_parser('shell', help='ROS-enabled shell in the simulation DDS domain')
    results = sub.add_parser('results', help='serve local run reports on loopback only')
    results.add_argument('--port', type=int, default=8088)
    demo = sub.add_parser('demo', help='run closed-loop exploration, export evidence and check the outcome')
    demo.add_argument('--world', choices=('room', 'partitioned'), default='partitioned')
    demo.add_argument('--duration', type=float, default=120.)
    demo.add_argument('--seed', type=int, default=1)
    demo.add_argument('--speed', type=float, default=1.)
    demo.add_argument('--dose-budget', type=float, default=2.)
    demo.add_argument('--radiation-weight', type=float, default=1.)
    demo.add_argument('--gui', action='store_true', help='open MuJoCo and RViz; requires X11')
    demo.add_argument('--require-return', action='store_true')
    args = parser.parse_args()
    os.environ['GO2_UID'], os.environ['GO2_GID'] = str(os.getuid()), str(os.getgid())
    if args.command == 'results':
        (ROOT/'results').mkdir(exist_ok=True)
        print(f'Open http://127.0.0.1:{args.port} and select a run directory.', flush=True)
        raise SystemExit(subprocess.call([sys.executable, '-m', 'http.server', str(args.port), '--bind', '127.0.0.1',
                                         '--directory', str(ROOT/'results')]))
    digest = runtime_hash()
    if args.command == 'setup':
        subprocess.run([sys.executable, str(ROOT/'scripts/fetch_assets.py')], check=True)
        subprocess.run([*COMPOSE, 'build', '--build-arg', f'SOURCE_SHA256={digest}'], check=True)
        return
    image = subprocess.run(['docker', 'image', 'inspect', 'go2-occmap:humble'], capture_output=True, text=True)
    if image.returncode:
        parser.error('Image is missing. Run ./go2 setup first.')
    identity = json.loads(image.stdout)[0]
    if identity['Config'].get('Labels', {}).get('go2.runtime_source_sha256') != digest:
        parser.error('Runtime sources changed since the image build. Run ./go2 setup again.')
    if args.command == 'test':
        raise SystemExit(subprocess.call([*COMPOSE, 'run', '--rm', '-T', 'go2-sim',
                                         'python3', '-m', 'pytest', '-q', 'tests'], cwd=ROOT))
    if args.command == 'shell':
        raise SystemExit(subprocess.call([*COMPOSE, 'run', '--rm', 'go2-sim', 'bash'], cwd=ROOT))
    if (not all(math.isfinite(x) for x in (args.duration, args.speed, args.dose_budget, args.radiation_weight))
            or args.duration < 15 or not .1 <= args.speed <= 8 or args.dose_budget <= 0
            or args.radiation_weight < 0 or args.seed < 0):
        parser.error('duration >= 15, speed 0.1–8, positive dose budget and nonnegative radiation weight required')
    if args.gui and not os.environ.get('DISPLAY'):
        parser.error('DISPLAY is unset; omit --gui for a headless run')
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    output = ROOT/'results'/f'{timestamp}_{args.world}_seed{args.seed}'
    output.mkdir(parents=True)
    container_output = '/workspace/'+str(output.relative_to(ROOT))
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    manifest = {'config': vars(args), 'runtime_source_sha256': digest, 'image_id': identity['Id'],
                'git_commit': commit, 'git_dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT)),
                'created_utc': timestamp, 'source_note': 'new simulation reference; historical hardware code was not in source repository'}
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    container_name = 'go2-occmap-'+timestamp.lower().replace('_', '-')
    command = [*COMPOSE, 'run', '--rm', '-T', '--name', container_name, 'go2-sim', 'ros2', 'launch', 'bringup_go2', 'simulation.launch.py',
        f'world:={args.world}', f'duration:={args.duration}', f'seed:={args.seed}', f'speed:={args.speed}',
        f'dose_budget:={args.dose_budget}', f'radiation_weight:={args.radiation_weight}',
        f'gui:={str(args.gui).lower()}', f'output:={container_output}']
    print(f'Running {args.world}; output: {output}', flush=True)
    with (output/'ros.log').open('w') as log:
        try:
            process = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                     timeout=args.duration/max(.1, args.speed)+180)
            launch_code = process.returncode
        except (KeyboardInterrupt, subprocess.TimeoutExpired):
            launch_code = 130
        finally:
            # A killed Docker client must not leave physics/control running behind it.
            subprocess.run(['docker', 'stop', '--time', '15', container_name], capture_output=True)
    (output/'launch_exit.json').write_text(json.dumps({'exit_code': launch_code})+'\n')
    report = [*COMPOSE, 'run', '--rm', '-T', '-e', 'MUJOCO_GL=egl', 'go2-sim', 'python3',
              'scripts/summarize_run.py', container_output]
    if args.require_return:
        report.append('--require-return')
    result = subprocess.call(report, cwd=ROOT)
    print(f'Artifacts: {output}\nView with ./go2 results', flush=True)
    raise SystemExit(result or launch_code)


if __name__ == '__main__':
    main()
