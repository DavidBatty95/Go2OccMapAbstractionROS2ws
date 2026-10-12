#!/usr/bin/env python3
"""Host-side public-asset setup; standard library only."""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('GO2_ASSET_DIR', str(ROOT/'.cache/go2'))
sys.path.insert(0, str(ROOT/'src/go2_occmap'))
from go2_occmap.assets import ensure_assets

if __name__ == '__main__':
    print(ensure_assets())
