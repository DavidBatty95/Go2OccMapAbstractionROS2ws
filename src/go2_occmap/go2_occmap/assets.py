"""Download public, immutable Go2 assets and retain their upstream licences."""
import hashlib
import json
import os
from pathlib import Path
from urllib.request import urlopen

ASSETS = Path(os.environ.get('GO2_ASSET_DIR', '/workspace/.cache/go2'))
MANIFEST = Path(__file__).with_name('assets.json')


def ensure_assets():
    config = json.loads(MANIFEST.read_text())
    model = f'https://raw.githubusercontent.com/unitreerobotics/unitree_mujoco/{config["model_commit"]}'
    policy = f'https://raw.githubusercontent.com/fan-ziqi/rl_sar/{config["policy_commit"]}'
    files = [(f'go2/{name}', f'{model}/unitree_robots/go2/{name}', sha)
             for name, sha in config['model_files'].items()]
    files += [('policy.pt', f'{policy}/policy/go2/robot_lab/policy.pt', config['policy_sha256']),
              ('policy.yaml', f'{policy}/policy/go2/robot_lab/config.yaml', config['policy_config_sha256']),
              ('UNITREE-LICENSE', f'{model}/LICENSE', config['model_license_sha256']),
              ('RL-SAR-LICENSE', f'{policy}/LICENSE', config['policy_license_sha256'])]
    for name, url, sha in files:
        path = ASSETS/name
        if path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == sha:
            continue
        print(f'Downloading {name}', flush=True)
        with urlopen(url, timeout=90) as response:
            raw = response.read()
        if hashlib.sha256(raw).hexdigest() != sha:
            raise RuntimeError(f'Checksum mismatch: {name}')
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix+'.download')
        temporary.write_bytes(raw)
        temporary.replace(path)
    return ASSETS
