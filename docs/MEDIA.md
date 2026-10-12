# Recorded media and reproducibility

[Project home](../README.md) · [Results](RESULTS.md) · [Attribution](ATTRIBUTION.md)

## Gallery source

The README gallery uses run
`20261007T152519_192061Z_partitioned_seed1`, recorded on 7 October 2026 with
the final-source image and runtime identities in [the measured report](RESULTS.md).
Both animations describe the **same physical/ROS mission**.

| File | Meaning |
|---|---|
| [go2.gif](media/go2.gif) | Sixty rendered frames from recorded MuJoCo free-base and joint coordinates. The following camera is applied during replay, not during control. Roughly twelve simulated seconds are shown over six seconds. |
| [mapping.gif](media/mapping.gif) | Sixty-frame accelerated replay across the mission. Observed maps and actual trajectory are recorded; graph display is rebuilt from each retained observed grid, and route overlays use the latest recorded decision. |
| [report.png](media/report.png) | Final observed map/graph, final sampled-field GP mean and evaluator/count-derived exposure. |
| [go2.png](media/go2.png) | Static frame from the physical-state replay. |
| [banner.svg](media/banner.svg), [architecture.svg](media/architecture.svg) | Original decorative/architectural schematics, not telemetry. |

The physics preview is not a newly simulated trajectory: MuJoCo forward
kinematics renders the states saved during the actual trial. No base position
was set by the controller during that physical run. Map playback speed and
sampling differ from physics-video playback; the panels are not synchronized
real-time camera views.

[provenance.json](media/provenance.json) records source artifact names, SHA-256
hashes and sizes for each displayed asset. It also identifies the representative
numeric replay inputs under [results/representative_run/](results/representative_run/).

## Retained inputs

The representative bundle contains configuration, summaries, independent DDS
counts, physical qpos/time/dose arrays, observed occupancy snapshots, measured
radiation samples, retained GP samples, planner traces and ROS logs. It is small
enough to inspect without the original ignored run directory. The field/geometry
truth in evaluator artifacts is not fed to the planner.

Every new `./go2 demo` run generates its own report through
[scripts/summarize_run.py](../scripts/summarize_run.py). This uses the installed
runtime's model and graph implementation plus retained numeric state. Rendering
library or source changes can alter generated pixels; use matching source/image
identities before expecting exact replay bytes.

To re-render an existing raw run with the matching built image:

```bash
docker compose run --rm -T -e MUJOCO_GL=egl go2-sim \
  python3 scripts/summarize_run.py /workspace/results/YOUR_RUN --require-return
```

Replace `YOUR_RUN` with its actual directory. This re-scores and renders recorded
artifacts; it does not rerun robot control. New experiments should go through
`./go2 demo` so their configuration and image identity are recorded first.

## Static alternatives

Use the [static Go2 view](media/go2.png), [final report](media/report.png) and
[results table](RESULTS.md) to inspect the demonstration without animation.
The full numeric report distinguishes retained development failures from the
final-source cohort; gallery selection is not a substitute for that record.
