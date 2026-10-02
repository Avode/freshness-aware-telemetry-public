# Reproducibility and evidence guide

This companion separates three activities: rebuilding the **source environment**, running **new local simulations**, and regenerating **figures from recorded observations**. Passing source-code tests does not reproduce a field result, and rebuilding a figure from supplied records does not rerun the original experiment.

## System boundary

The Linux computer owns the single Gazebo physics world, Husky's sensor simulation, ROS bridge, wheel/IMU estimation, AMCL, Nav2, the edge outbox, and local safety. In distributed mode a TLS/MQTT gateway transports allowlisted telemetry, status, acknowledgments, and commands to a separately operated Mac receiver. The public repository contains the Linux source and wire-contract examples; it contains neither the Mac receiver nor private broker settings. The local web command center is an alternative operator interface for reproduction on one Linux computer.

The estate is a declared, 200 × 190 m surveyed site. `config/scene.json` is the full simulation manifest, including its generation seed and initial robot placements. `network/scene.json` is the exact preloaded site-prior map prepared for the remote command center: it preserves the surveyed site geometry but intentionally omits the seed and robot placements. Its SHA-256 matches `map_sha256` in both `network/site.json` and the illustrative `network/STATUS_SCHEMA.json` heartbeat. This shared map identity is neither live simulator truth nor robot-discovered occupancy. The operational twin uses the prior plus robot-reported pose, LiDAR, camera, IMU, health, plan, and mission observations. Independent 2-D SLAM is available for comparison. Gazebo ground truth enters `milestone/evaluator.py` and `autonomy/evaluator.py` for offline scoring only. Go2 and X500 are static imports in this milestone. The Husky arm remains in its transport pose; manipulation is not evaluated.

## Prepare and verify the scene

The implementation was developed on Ubuntu 22.04 with ROS 2 Humble and Gazebo Fortress. Start with a functioning ROS installation that provides Nav2, AMCL, SLAM Toolbox, `ros_gz_bridge`, RViz, and the usual Python ROS bindings. The model importer uses NumPy, SciPy, ROS `xacro`, and upstream robot descriptions; Gazebo scene validation uses `ign sdf`. Python gateway tests need `paho-mqtt` 2.1.0; figure regeneration needs Matplotlib. Install those packages into the interpreter used by the ROS launchers (`/usr/bin/python3` on the original system). A compiler and Assimp development library are needed for the optional mesh conversion/Blender authoring route.

Run the following from the cloned repository root. The three environment variables are optional but make output locations explicit; if unset, the project defaults to its local `assets/`, `runs/`, and `assets/ros-overlay` directories.

```bash
export FLEETSCOPE_ASSETS="$PWD/assets"
export FLEETSCOPE_RUNS="$PWD/runs"
export FLEETSCOPE_OVERLAY="$FLEETSCOPE_ASSETS/ros-overlay"
python3 scripts/fetch_assets.py
python3 scripts/import_robots.py
python3 scripts/build_world.py
python3 scripts/verify_world.py
./milestone/setup-deps.sh
python3 autonomy/build.py
```

`scripts/fetch_assets.py` checks out the exact commits in `config/robot_sources.json` from Unitree, Clearpath, Kinova, and PX4. `scripts/import_robots.py` creates locally linked parked SDF models from those upstream descriptions. The external meshes are not redistributed in this repository. The public `scripts/build_world.py` recreates the included original `worlds/plantation.sdf` snapshot and full `config/scene.json`, including the world's sensors and optional player scene entity. `network/scene.json` is a separately packaged site-prior subset, not a second simulator and not an output of the world builder. `scripts/verify_world.py` checks the world and local model references and runs Gazebo's SDF check. The optional dependency helper extracts ROS `robot_localization` packages into the specified user-owned overlay. `python3 autonomy/build.py` generates the dynamic Husky world, surveyed occupancy prior, and navigation configuration; the full test suite expects these generated files. If upstream repositories, ROS packages, or graphics drivers differ, record their versions and any failure before comparing outputs with the original run.

The packaged remote-map identity can be checked without launching Gazebo:

```bash
python3 - <<'PY'
import hashlib, json
from pathlib import Path
digest = hashlib.sha256(Path('network/scene.json').read_bytes()).hexdigest()
for file in ('network/site.json', 'network/STATUS_SCHEMA.json'):
    assert json.loads(Path(file).read_text())['map_sha256'] == digest, file
print(digest)
PY
```

The static exhibit can be opened with `./launch.sh`. For the paper-relevant local stack:

```bash
./run-command-center.sh
# Open http://127.0.0.1:8765/ in a browser.
```

It launches one Gazebo server and a local command-center dashboard. The backend reconstructs only received state; the web view is not another simulator. Use the dashboard's Forecourt circuit for a short inspection. `./run-autonomy.sh` runs the autonomy stack with RViz; `./run-milestone.sh` runs the earlier, manual-sensor demonstration. Do not start multiple stack launchers at once. Ctrl-C stops a launcher's owned processes but preserves its recording in `$FLEETSCOPE_RUNS`.

## Software checks and optional new experiments

```bash
source autonomy/env.sh
python3 -m unittest milestone.test_core autonomy.test_navigation \
  command_center.test_center network.test_protocol network.test_gateway -q
```

These are post-build regression tests for timestamp watermarking, bounded retry, mapping, command handling, and gateway behavior. Run model fetch/import and `python3 autonomy/build.py` first; `network.test_gateway` imports `paho-mqtt`. Their result depends on the installed ROS/Python environment; they are not repeated mission trials. The author-draft evaluation reported 53 passing tests on its original workstation.

With a fresh, healthy `./run-command-center.sh` stack running, the following **active** validations intentionally issue commands or inject simulation faults:

```bash
source autonomy/env.sh
python3 -m command_center.validate
# On a separate fresh run, for the longer inspection baseline:
python3 -m command_center.validate --baseline
```

The default scenario exercises camera, IMU, LiDAR, position and link faults, explicit safety holds, replay, and history projection. The baseline starts a full inspection and records evaluator-measured position and checkpoint errors; it can take several minutes. Use separate fresh runs for these commands. The offline evaluator has access to simulator truth for scoring; the operational dashboard and navigation do not. A new run is new evidence and should not be silently substituted for the included recorded values.

`./run-remote-center.sh` requires a separately provisioned TLS broker and a private edge configuration path supplied as `FLEETSCOPE_MQTT_CONFIG`; keep that file outside the repository. Its wire protocol is in `network/protocol.py` and `network/STATUS_SCHEMA.json`. This release cannot by itself reproduce the Mac receiver journal, Mac dashboard, or end-to-end command acknowledgement at the Mac. A broker-level connection is not evidence that the Mac has durably ingested or displayed an observation.

## Regenerate the supplied figures

From the repository root, with NumPy and Matplotlib available:

```bash
python3 research/figures/build_evidence_figures.py
```

The script writes `incident_vs_later_queue.{png,svg}`, `mac_ack_and_rviz_age_trace.{png,svg}`, and `full_route_accuracy.{png,svg}` to `research/figures/`. It checks key source counts and metrics while generating the charts. Figure data are selected, recorded observations, not synthetic replications.

| Figure | Included input | Interpretation |
|---|---|---|
| `incident_vs_later_queue` | `data/reports/stale-telemetry-diagnosis.json`, `data/reports/telemetry-freshness-validation.json` | Endpoint queue values in an older incident and 15 sampled values from a later, different session. There are no intermediate old-incident queue values. Conditions and interventions were unmatched. |
| `mac_ack_and_rviz_age_trace` | `data/reports/telemetry-freshness-validation.json` | A 28 s diagnostic with 97 additional gateway-visible application ACK **messages**, zero reported drops, and 15 locally LIVE RViz samples. ACK count is not a unique-packet count or an audit of Mac UI rendering. Pose age uses **simulation seconds**. |
| `full_route_accuracy` | `data/reports/command-center-baseline.json`, `data/route/pose-errors.jsonl`, `data/route/mission-events.jsonl` | One simulated full-route run, 1,742 evaluator samples. RMS position error 1.975 m, empirical p95 3.673 m, maximum 3.875 m; physical checkpoint errors 0.527, 3.636, and 0.486 m. No confidence interval or physical-robot inference. |

The other included compact reports are `data/reports/command-center-validation.json` (one scripted local fault scenario), `data/reports/mqtt-validation.json` (one Ubuntu-host local TLS/MQTT stand-in receiver test), and `data/reports/long-run-summary.json` (a derived summary from one actual Mac-connected Ubuntu run). The MQTT stand-in report does not verify the Mac implementation. The long-run summary lists cumulative gateway publication and application-ACK messages plus a received quick-route command and mission completion; it does not contain the full Mac receipt journal. These reports retain original scope and run identifiers. They are evidence summaries and selected evaluator samples, not the complete SQLite/camera/log archive. The Mac's raw receipt journal was not independently audited.

## Interpretation limits

The stale incident and later trace were collected in separate sessions. The capture-through timestamp correction and retry scheduling change were introduced together, and viewing conditions differed. Their individual or combined causal effects cannot be estimated from these records. The 12 repeatedly delivered oldest packets in the incident had no observed application ACK; the Mac-side rejection reason was not directly inspected. The later trace supports a narrower claim: fresh valid envelopes, a small observed edge queue, gateway-visible Mac ACK traffic, and a locally fresh RViz view during the sampled interval.

The long Mac-connected run described in the manuscript has gateway-side publication, ACK, and mission summaries, but its complete raw Mac journal and browser state are outside this repository. The estate route is a single simulated trial. Functional waypoint completion coexists with metre-scale localization and shelter-arrival error. Results do not establish reliable physical localization, quadruped/drone operation, fleet scheduling, or performance across different sites. A controlled repeated comparison and independent Mac timing audit are required for stronger performance claims.
