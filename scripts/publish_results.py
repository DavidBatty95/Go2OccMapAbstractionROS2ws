#!/usr/bin/env python3
"""Publish selected measured summaries and one replay bundle; retain development failures."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]


def record(path):
    return dict(run=path.name, manifest=json.loads((path/'manifest.json').read_text()),
                summary=json.loads((path/'summary.json').read_text()),
                artifact_sha256=json.loads((path/'artifact_hashes.json').read_text()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', nargs='+', type=Path, required=True)
    parser.add_argument('--development-runs', nargs='*', type=Path, default=[])
    parser.add_argument('--media-run', type=Path, required=True)
    args = parser.parse_args()
    runs = [record(path) for path in args.runs]
    if len({r['manifest']['runtime_source_sha256'] for r in runs}) != 1:
        raise SystemExit('Published current cohort must share one runtime source identity')
    results, media = ROOT/'docs/results', ROOT/'docs/media'
    results.mkdir(parents=True, exist_ok=True)
    media.mkdir(parents=True, exist_ok=True)
    report = {'checkpoint': '2026-10-07', 'scope': 'simulation integration; ideal pose/rays, synthetic radiation',
              'current': runs, 'development': [record(path) for path in args.development_runs]}
    (results/'initial_runs.json').write_text(json.dumps(report, indent=2)+'\n')
    provenance = {'run': args.media_run.name, 'files': {}}
    for name, target in [('walking.gif', 'go2.gif'), ('mapping.gif', 'mapping.gif'),
                         ('report.png', 'report.png'), ('go2.png', 'go2.png')]:
        shutil.copyfile(args.media_run/name, media/target)
        provenance['files'][target] = {'source_artifact': name,
            'sha256': hashlib.sha256((media/target).read_bytes()).hexdigest(), 'bytes': (media/target).stat().st_size}
    # Complete numeric inputs for the displayed run, excluding bulky rendered duplicates.
    replay = results/'representative_run'
    replay.mkdir(exist_ok=True)
    files = ['manifest.json', 'summary.json', 'simulation.json', 'mapping.json', 'exploration.json',
             'ros_interfaces.json', 'launch_exit.json', 'trace.json', 'physics.npz', 'mapping.npz',
             'measurements.npz', 'field_samples.npz', 'ros.log']
    provenance['replay_sha256'] = {}
    for name in files:
        shutil.copyfile(args.media_run/name, replay/name)
        provenance['replay_sha256'][name] = hashlib.sha256((replay/name).read_bytes()).hexdigest()
    (media/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    print(f'Published {len(runs)} current runs, {len(args.development_runs)} development attempts and one replay bundle')


if __name__ == '__main__':
    main()
