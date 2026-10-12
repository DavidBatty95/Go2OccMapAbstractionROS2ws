# Installation and running

[Project home](../README.md) · [Architecture](ARCHITECTURE.md) · [Results](RESULTS.md)

## Supported reference environment

- x86-64 Linux with Docker Engine and Compose v2+.
- Python 3 on the host for the standard-library command wrapper and asset fetcher.
- Internet during first setup, a writable checkout and several GB of disk space.
- CPU-only headless execution is the default. Interactive windows require X11
  or XWayland; recording replays use Mesa/EGL offscreen rendering.

The container has ROS 2 Humble, Python 3.10, MuJoCo 3.3.6, CPU PyTorch 2.6.0,
NumPy 2.2.6 and SciPy 1.15.3. Core Python dependencies are specified in
[docker/requirements.txt](../docker/requirements.txt). The base image is pinned
by digest. Asset downloads use immutable commits and SHA-256 hashes. Apt and
transitive package resolution can still change future build bytes; each run
records the resulting image identity.

## Build and launch

```bash
./go2 setup
./go2 demo --world partitioned --duration 120 --speed 2 --seed 1 --require-return
./go2 results
```

Setup downloads public meshes, model, policy, deployment configuration and
licence notices. Assets are cached under `.cache/go2/`. Interrupted downloads
are only installed after their checksum matches. The Docker build runs colcon
and installs both packages under `/opt/go2_ws/install`.

The launcher maps the host UID/GID, so retained reports are owned by the caller.
The checkout is mounted at `/workspace` for assets, scripts and results. Runtime
Python nodes come from the installed image, not a sibling repository or host ROS.
After changing runtime source files, rebuild with `./go2 setup`; stale images
are rejected by a runtime-source digest check.

Every demo creates a unique timestamped run directory and captures all ROS launch
output in `ros.log`. The simulator exits at a return or duration limit. Exiting
a critical node shuts down the other nodes; they save their artifacts in their
shutdown handlers. The launcher also stops its named container on interruption
or wall-clock timeout before attempting the final report.

## Browser reports

`./go2 results` serves only the results directory at <http://127.0.0.1:8088>.
Select a timestamped folder to see the map, inferred field, exposure, replays
and explicit run checks. Use `./go2 results --port 8090` to select another port.
The original NPZ/JSON files and logs are linked or retained beside the page.

The physics GIF replays recorded joint/base coordinates, not a fresh controller
run. The map GIF is reconstructed from observed grids and recorded decisions.
Rendering time is separate from mission wall time.

## Interactive windows

```bash
./go2 demo --gui --duration 120 --require-return
```

This starts the MuJoCo viewer and RViz. Use the graphical session's existing
`DISPLAY` and valid `XAUTHORITY` file. The Compose file passes those through.
Where the X server requires explicit permission, authorize your local user with
`xhost +si:localuser:$(id -un)`. Do not replace a valid cookie with an empty one.

The GUI launcher/configuration is supplied; the published cohort was run
headlessly, with offscreen replay rendering verified. Interactive display
availability depends on the host's graphical session.

## ROS inspection

```bash
./go2 shell
ros2 node list
ros2 topic hz /scan
ros2 topic echo --once /mission/status
ros2 topic info /map --verbose
```

The standard simulation nodes are `go2_simulator`, `observed_grid_mapper`,
`radiation_graph_explorer` and `interface_monitor`. All use loopback DDS domain
47 by default. To run an independent session, set a different domain consistently:

```bash
GO2_ROS_DOMAIN_ID=48 ./go2 demo --duration 120 --require-return
# In another terminal, use that same domain for inspection:
GO2_ROS_DOMAIN_ID=48 ./go2 shell
```

Do not run two command-owning sessions in the same domain. The domain/interface
defaults keep this demo separate from physical Go2 middleware. Hardware bringup
is not implemented by this simulation profile.

## Troubleshooting

| Symptom | Action |
|---|---|
| Docker denied or Compose missing | Check the daemon, user Docker access and Compose v2+ installation |
| Missing/stale image | Run `./go2 setup`; the launcher deliberately checks the image's source label |
| Asset checksum error | Retry setup; inspect network/proxy access to the pinned public URLs |
| Demo reports missing artifacts | Read the run's `ros.log`; a critical node likely failed during initialization |
| No topics in a host shell | Use `./go2 shell` or match the RMW, DDS configuration and domain on the host |
| No graphical display | Omit `--gui`; browser reports do not require an X server |
| Early return | Time and dose reserves can reject further frontiers; inspect `trace.json` rather than assuming a fault |
| Failed return / no graph attachment | Inspect the state/reason, map and recorded path. The runner retains and reports failures |
| Lower-than-requested speed | `--speed` is pacing, not a hard real-time guarantee; reduce it on constrained hosts |
| Matplotlib `Axes3D` warning | The image includes distro and pip Matplotlib components; the report uses 2-D plots and tested rendering remains available |
| Short run fails movement check | A pipeline trial needs actual motion and map/graph messages; the 120-second reference command is the validated entry point |

The CLI rejects durations below 15 s, nonpositive budgets and invalid speed
ranges. Passing the smoke checks establishes the listed integration conditions,
not general navigation reliability or dose safety.
