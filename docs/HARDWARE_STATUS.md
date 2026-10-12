# Hardware history and migration boundary

[Project home](../README.md) · [Architecture](ARCHITECTURE.md) · [Attribution](ATTRIBUTION.md)

## What was available

At the start of this refresh, the GitHub repository's `main` branch at
`4a41bad` contained **only `README.md`**. That document described a Unitree Go2
hardware workspace and named `bringup_go2`, an SLLIDAR S2 driver, a Marvelmind
driver and radiation-emulation nodes. No package source, workspace manifests,
launch files, robot interfaces or hardware test results accompanied it.

Consequently, the new runnable workspace is not represented as a containerized
copy of unavailable hardware code. The package name `bringup_go2` is used for
the newly supplied **simulation** launch package.

## Implemented now

- Installed ROS 2 Humble packages in Docker.
- Public checksum-verified Go2 model and pretrained locomotion policy.
- MuJoCo dynamics, ideal pose/ranges and simulated count observations.
- Observed occupancy, regular-lattice graph abstraction and dose-aware exploration.
- DDS monitoring, physical/state recording, independent evaluation and replay reports.

## Still needed for hardware restoration

- The actual hardware workspace source and exact revisions from the original experiments.
- Robot command/state transport and verified SDK/message contracts.
- SLLIDAR S2 acquisition, calibration, frame transforms and scan quality checks.
- Marvelmind transport and the radiation-emulation measurement contract.
- Estimated localization, timing alignment and handling of sensor faults.
- Hardware-specific velocity ownership, stopping behavior and validation evidence.
- Migration of the intended research algorithm, if it differs from this reference baseline.

No hardware mode is exposed by `./go2`. The default loopback-only DDS domain 47
serves the simulated nodes. It should not be interpreted as a hardware bringup
command or a claim of equivalence to the experiments cited in the thesis.

## Related research

- [Radiation-aware PhD thesis](https://livrepository.liverpool.ac.uk/3199094/)
- [ROS 1 occupancy abstraction research](https://github.com/DavidBatty95/OccMapAbstractionROS1Pkg)
- [Python occupancy abstraction research](https://github.com/DavidBatty95/OccMapAbstraction-Python-Based)
- [Dougie-Sim](https://github.com/DavidBatty95/Dougie-Sim), the sibling Go2 integration baseline

Results and capabilities of those projects are not relabelled as results of this
new ROS 2 simulation workspace.
