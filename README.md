# Freshness-Aware Telemetry for an Observation-Driven Mobile Robot Digital Twin

Research code accompanying the manuscript of the same title by **Nik Umar bin Nik Mohamed Nizan** (Independent Researcher, Malaysia). This repository contains the Linux-side simulation, robot telemetry, command center, and selected evidence used in the author draft. It is a research demonstrator, not a validated fleet product.

The experiment uses **one active wheeled robot** in a plantation and logistics estate. A Clearpath Husky-derived mobile base supplies simulated wheel, IMU, LiDAR, and camera observations; AMCL, Nav2, and an edge agent estimate its state and perform inspections. A Unitree Go2 quadruped and PX4 X500 drone are parked scene imports without active telemetry or missions. The operator's digital twin draws the surveyed site prior and *received* robot observations. It does not run a second Gazebo world or receive privileged simulator pose. An offline evaluator uses simulator truth only to score localization and physical arrival.

## What is here

| Path | Role |
|---|---|
| `scripts/`, `config/`, `worlds/`, `models/` | Deterministic estate builder, pinned external model importer, full simulation manifest, Gazebo world, and original estate geometry |
| `milestone/` | Sensor adapter, time alignment, capture-through timestamp watermark, bounded SQLite outbox, initial received-state twin, and tests |
| `autonomy/` | Surveyed planning prior, localization, SLAM comparison, Nav2 inspection missions, evaluator, and tests |
| `command_center/` | Sensor-health observer, local safety hold, command handling, durable history projection, loopback web dashboard, and tests |
| `network/` | Linux MQTT gateway, wire protocol, exact preloaded site-prior map and map identity, non-secret schema example, and protocol/gateway tests |
| `data/reports/`, `data/route/` | Selected recorded summaries and evaluator samples supporting the supplied figures; these are not a full raw run archive |
| `research/figures/` | Figure-generation code and selected rendered plots |
| `plugins/player_controller/` | Optional Gazebo first-person scene-controller source; not part of the reported telemetry evaluation |

The source tree does **not** include manufacturer mesh copies, compiled plugins, a Mac receiver or its private connection configuration, broker credentials, raw camera journals, or a live robot dataset. `config/robot_sources.json` pins the four upstream model revisions; `docs/licenses/` preserves their notices. The external model importer reconstructs local assets from those sources.

`config/scene.json` is the complete simulation layout manifest, including its generation seed and parked robot placements. `network/scene.json` is the exact surveyed site-prior map shared with the remote command center: it retains the site geometry while intentionally omitting the seed and robot placements. Its SHA-256 is recorded in `network/site.json` and `network/STATUS_SCHEMA.json` as a shared map identity. This is a preloaded map, not simulator ground truth or a record of what a robot has observed.

## Environment and a local demonstration

The recorded implementation ran on **Ubuntu 22.04, ROS 2 Humble, and Gazebo Fortress** (the `ign gazebo` executable). It also needs the ROS packages used by this stack, including Nav2, AMCL, SLAM Toolbox, `ros_gz_bridge`, RViz, and `robot_localization`; Python 3 with NumPy, SciPy, OpenCV, PyYAML, `paho-mqtt` 2.1.0 for gateway tests, and Matplotlib for figure generation; and Git, `xacro`, a C++ compiler, and Assimp development libraries for asset rebuilding. Install Python packages into the interpreter used by the ROS launchers (`/usr/bin/python3` on the original system). The launcher checks required ROS executables. Package installation differs between systems and is not automated as a clean-room installation here.

From the repository root, choose writable directories for assets, run records, and the user-owned ROS overlay. The defaults are `./assets`, `./runs`, and `./assets/ros-overlay`:

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
./run-command-center.sh
```

The dashboard is at <http://127.0.0.1:8765/>. Wait for the sensors and navigation processes to initialize, then dispatch the short **Forecourt circuit** inspection. The dashboard is local to the Linux host; close it with Ctrl-C in the launch terminal. Run directories and logs are retained under `$FLEETSCOPE_RUNS`. The first build downloads pinned upstream robot descriptions and may take time. The public `scripts/build_world.py` regenerates the included original world snapshot, including its sensor configuration and optional player scene entity. `./launch.sh` opens the static estate in Gazebo; `./run-autonomy.sh` launches the autonomy stack with RViz; `./run-milestone.sh` runs the earlier sensor-telemetry demonstration. Use one stack at a time.

The `./run-remote-center.sh` entrypoint starts the Linux simulation and MQTT gateway **only when an independently provisioned TLS broker and private edge configuration are supplied** through `FLEETSCOPE_MQTT_CONFIG`. Keep that JSON file outside the repository. The Mac receiver, its durable database, and its configuration are deliberately absent from this public code package. The local dashboard above is the supported way to explore the included source without that separate system.

## Tests and evidence

After fetching/importing the pinned models and running `python3 autonomy/build.py` as above, source ROS Humble (the helper sets the project environment) and run the software regression tests from the repository root. The gateway tests additionally import `paho-mqtt`:

```bash
source autonomy/env.sh
python3 -m unittest milestone.test_core autonomy.test_navigation \
  command_center.test_center network.test_protocol network.test_gateway -q
```

For the recorded plots, install NumPy and Matplotlib in the Python environment and run:

```bash
python3 research/figures/build_evidence_figures.py
```

The script reads the selected files in `data/` and writes PNG/SVG plots under `research/figures/`. [REPRODUCIBILITY.md](REPRODUCIBILITY.md) maps each figure and reported value to its inputs, gives optional live validation commands, and separates source-code tests from simulation evidence.

The earlier stale-queue incident and the later Mac-connected trace are **different runs under different viewing conditions**. Two fixes were made together. The plots are descriptive observations, not a controlled before/after experiment or an estimate of either fix's causal effect. The route-error plot is one completed simulated inspection; mission success does not establish precise localization. No physical-robot result or heterogeneous-fleet evaluation is claimed. The Mac journal and browser rendering were not independently audited for the manuscript.

## Citation and rights

Use [CITATION.cff](CITATION.cff) for software citation metadata. No repository-wide license is supplied in this release. Third-party robot models retain their upstream terms; see `docs/licenses/` and `config/robot_sources.json`.
