# Recorded results and verification

[Project home](../README.md) · [Methods](ARCHITECTURE.md) · [Media](MEDIA.md)

## Final-source integration cohort

Checkpoint **7 October 2026**. The [portable report](results/initial_runs.json)
retains configurations, summaries, image/source identities and artifact hashes.
Four current runs share runtime-source SHA-256:

```text
f1614cf3b95a3426142b6c8b7eaeb7255d7fa9a8b9e23d43462af895c1a9ce2b
```

Docker image:
`sha256:d48769fff412483214796847d4c46fd3e5bbaae19fdcef8de1cca7328c59de5b`.
The working tree was not committed at capture time; the recorded Git commit is
the earlier README-only revision, and `git_dirty=true` plus the runtime digest
identify that distinction explicitly.

| Run | Pacing | Return (s) | Travel (m) | Free-area fraction | Actual dose (µSv) | Home distance (m) | Plan p95 (ms) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Room / seed 1 / weight 1 | 1× | 94.64 | 14.88 | 86.94% | 0.28719 | 0.334 | 57.01 |
| Partitioned / seed 1 / weight 1 | 2× | 93.65 | 13.39 | 78.46% | 0.28275 | 0.263 | 39.71 |
| Partitioned / seed 2 / weight 1 | 2× | 98.68 | 14.23 | 78.41% | 0.29054 | 0.338 | 76.94 |
| Partitioned / seed 1 / weight 0 | 2× | 89.65 | 14.01 | 76.51% | 0.49488 | 0.271 | 41.28 |

All four runs returned, stayed upright, recorded zero robot/obstacle contact
steps and remained below the 2 µSv synthetic budget. The initial observed-free
fraction was 43.50% in the room and 41.46% in the partitioned scene, before most
travel. Each mission had a 120 s maximum.

The weight-0 run trades different coverage and dose against the weight-1 run.
It is **not fully radiation-blind**: GP-based candidate admission and return
reserves remain enabled. One pair with different achieved coverage cannot
establish a general causal dose-reduction claim.

## What the measurements mean

- **Free-area fraction:** number of true-free raster cells labelled free by the
  final observed map, divided by total true-free cells in the configured 10 × 8 m
  domain. The evaluator gets truth; navigation does not.
- **Travel:** XY distance accumulated over saved 10 Hz physical base states.
  Body/stance drift contributes; this is not graph-route length.
- **Dose:** hidden-field integration along physical motion. µSv/µSv/h are
  synthetic dose-equivalent proxies, not calibrated hardware damage units.
- **Contacts:** MuJoCo contact steps involving robot and obstacle geometries.
  Expected feet/floor contacts are excluded. This is not a near-miss metric.
- **Planning p95:** measured wall-clock time for successful graph/field/route
  planning calls. Wait-only branches are not included, and host timing varies.
- **Return:** planner reports `returned`, with independent final home distance
  below 0.4 m. The planner itself uses 0.35 m.

The current cohort's graph peaks ranged from 198 to 323 nodes, versus thousands
of observed free raster cells. The implemented abstraction is a regular spatial
sample of inflated free space, not a recovered adaptive historical algorithm.

## ROS and physical checks

`./go2 test` passed **15 tests** on the final revision. Coverage includes:

- stale and nonfinite velocity rejection;
- forward scan-stop gating;
- occupancy free/hit/unknown update behavior;
- graph disconnection across unknown space and reconnection through a checked gap;
- radiation-weighted route detours;
- positive, bounded field estimates and unseen-location uncertainty;
- heading-first route following and coordinate conversion;
- route-dose integration;
- local graph attachment with minimum clearance and unknown-cell rejection;
- named twelve-joint policy mapping and untethered physical walking;
- room scans, invalid physical commands and finite radiation falloff.
- actual DDS delivery of the odometry angular velocity and twelve-joint state;
- bounded failure after an unsupported-motion wait, without resetting the failure state.

The run acceptance checks additionally require healthy physics, actual travel,
command receipt, map updates, graph construction, sensor observations, all
twelve monitored ROS topics, no checked invalid messages, no obstacle contacts
and no actual budget exceedance. `--require-return` adds returned-home acceptance.
These are local executed checks; hosted CI status is separate.

A separate intentional-interruption check sent Ctrl+C to the launcher after
simulator readiness. The launcher exited nonzero and left no running Go2
container. The loopback browser server and README image rendering were also
checked. That deliberate abort is a lifecycle test, not an exploration trial
in either results cohort.

## Retained development failures

An earlier runtime (`cc0d6ede…`) produced a seed-2 partitioned-scene run that
reached the 120 s deadline **3.244 m from home**. It travelled 9.51 m, observed
80.74% of free area, accumulated 0.28796 µSv and had no obstacle contacts, but
**failed return acceptance**.

The physical policy had drifted outside the graph's nominal clearance mask.
The planner lost an attachment and waited too long before reconnecting. A first
change added a bounded, known-free local attachment check and explicit wait
trace records, and several runs passed.

A subsequent repeat (`a1488f99…`) exposed a second failure: an unsupported wait
near an obstacle allowed further zero-command gait drift. At 120 s it was
5.622 m from home, with 13,799 obstacle-contact physics steps and 0.93791 µSv
actual dose. These are repeated contact steps, not 13,799 separate collision events.

The final revision increases global graph clearance to 0.60 m and local
reconnection clearance to 0.50 m, and turns four-second unsupported-motion waits
into explicit terminal failures. It does not relax contact or return acceptance.
The final seed-2 trial returned in 98.68 s without obstacle contacts. These
measured reruns do not prove general navigation or recovery safety.

The portable report keeps all **eleven development attempts**, including both
failures, separate from the four current-source trials. Earlier successful runs
are not silently mixed into the final cohort.

## Reproduce new trials

```bash
./go2 setup
./go2 test
./go2 demo --world room --duration 120 --seed 1 --require-return
./go2 demo --world partitioned --duration 120 --speed 2 --seed 1 --require-return
./go2 demo --world partitioned --duration 120 --speed 2 --seed 2 --require-return
./go2 demo --world partitioned --duration 120 --speed 2 --seed 1 --radiation-weight 0 --require-return
```

Each command creates a fresh directory. Detector seed, ROS scheduling and machine
timing influence the trajectory; fresh runs need not match table values exactly.
Retain failures and identify any changed code/settings when publishing new results.

To create a new portable report from explicit retained directories:

```bash
python3 scripts/publish_results.py \
  --runs results/RUN_A results/RUN_B \
  --development-runs results/EARLIER_ATTEMPT \
  --media-run results/RUN_A
```

Replace the example paths with actual run directories. The publisher requires
the current cohort to share a runtime-source identity and keeps the selected
development attempts under their own configurations. Publishing overwrites the
documentation snapshot; it does not rerun simulation.
