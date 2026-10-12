#!/usr/bin/env python3
"""Validate active local documentation, media hashes and recorded replay identities."""
import hashlib
import json
from pathlib import Path
import re
from urllib.parse import unquote
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    errors = []
    files = [ROOT/'README.md', *sorted((ROOT/'docs').glob('*.md'))]
    for path in files:
        for target in re.findall(r'\]\(([^)]+)\)', path.read_text()):
            if '://' in target or target.startswith('mailto:'):
                continue
            destination, _, fragment = target.partition('#')
            resolved = (path.parent/unquote(destination)).resolve() if destination else path
            if not resolved.exists():
                errors.append(f'{path.name}: missing {target}')
            elif fragment and resolved.suffix == '.md':
                text = re.sub(r'```.*?```', '', resolved.read_text(), flags=re.S)
                headings = re.findall(r'^#+\s+(.+)$', text, re.M)
                slugs = {re.sub(r'[^\w\- ]', '', h.lower()).replace(' ', '-') for h in headings}
                if fragment not in slugs:
                    errors.append(f'{path.name}: missing anchor {target}')
    for svg in (ROOT/'docs/media').glob('*.svg'):
        ET.parse(svg)
    media = json.loads((ROOT/'docs/media/provenance.json').read_text())
    hashes = {f'docs/media/{name}': record['sha256'] for name, record in media['files'].items()}
    hashes.update({f'docs/results/representative_run/{name}': sha for name, sha in media['replay_sha256'].items()})
    for name, sha in hashes.items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != sha:
            errors.append(f'Media/replay hash mismatch: {name}')
    report = json.loads((ROOT/'docs/results/initial_runs.json').read_text())
    if len({run['manifest']['runtime_source_sha256'] for run in report['current']}) != 1:
        errors.append('Current measured cohort mixes runtime source identities')
    if not any(run['summary']['outcome'] == 'FAIL' for run in report['development']):
        errors.append('Expected retained development failure is missing')
    if errors:
        raise SystemExit('\n'.join(errors))
    print(f'Checked {len(files)} Markdown files, SVG XML, {len(hashes)} media/replay hashes and cohort provenance')


if __name__ == '__main__':
    main()
