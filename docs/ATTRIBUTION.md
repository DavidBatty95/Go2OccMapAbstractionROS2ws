# Attribution and asset provenance

[Project home](../README.md) · [Media](MEDIA.md) · [Hardware history](HARDWARE_STATUS.md)

## Project source

The ROS 2 simulation workspace was added for Dr David Batty's occupancy-map
abstraction research repository. Its new mapper, graph, radiation estimator,
mission logic and recording tools are documented as a reference baseline.
The published repository had no original hardware-package source to port.

The named-joint Go2 policy adapter follows the approach used in David Batty's
Dougie-Sim integration, with the deployment observation/action contract from
the public `rl_sar` configuration. Runtime code here is self-contained; neither
setup nor execution needs access to Dougie-Sim or any private submodule.

The repository's pre-existing usage notice said that licensing remained to be
finalised and requested contact before redistribution/reuse. That decision is
preserved. No new blanket project licence is imposed by this refresh.

## Public runtime assets

The [asset manifest](../src/go2_occmap/go2_occmap/assets.json) contains file
SHA-256 checksums and immutable revisions:

| Asset | Source / revision | Upstream terms |
|---|---|---|
| Go2 XML and meshes | [Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco), `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d` | BSD-3-Clause; downloaded `UNITREE-LICENSE` |
| Go2 `robot_lab` policy and configuration | [rl_sar](https://github.com/fan-ziqi/rl_sar), `376d42c9b128f963ab08579762d5a216a976ce39` | Apache-2.0; downloaded `RL-SAR-LICENSE`; copyright 2024–2025 Ziqi Fan |

Assets and complete upstream notices are cached under `.cache/go2/` and not
committed as binary copies. The downloader verifies cached files as well as new
downloads. The core policy observation and action settings remain attributable
to the upstream deployment configuration.

MuJoCo, PyTorch, ROS 2, Cyclone DDS, NumPy, SciPy, Matplotlib and Pillow retain
their own respective licences. Docker base/package distribution terms are
separate from the repository's project-source notice.

## Scientific references

The README links the author's thesis and occupancy-map/radiation-emulation
publications for research context. They are not descriptions of this new
regular-lattice planner or evidence of its simulated return performance.
New performance claims should cite the exact run/source identities in the
[results report](RESULTS.md), including failed attempts and approximation boundaries.
