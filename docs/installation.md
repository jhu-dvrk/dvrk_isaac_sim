# Installation and runtime environment

## Supported baseline

The initial target environment is:

- Ubuntu 24.04;
- ROS 2 Jazzy;
- Isaac Sim 6.0;
- Python 3.12;
- `dvrk_model` available in the ROS 2 workspace or through `DVRK_MODEL_PATH`.

Isaac Sim and ROS 2 must use compatible Python and middleware libraries. ROS 2 custom-message packages used from Isaac Sim Python must be built with Python 3.12.

## Single ROS 2 workspace

Use one overlay workspace for this project. Place `crtk_msgs`, `dvrk_model`, and `dvrk_isaac_sim` together under its `src` directory. This gives Isaac Sim one consistent `install` space containing the custom-message Python modules and the dVRK model package metadata.

```bash
mkdir -p /path/to/isaac_sim_ws/src
cd /path/to/isaac_sim_ws/src

git clone https://github.com/collaborative-robotics/crtk_msgs.git
git clone https://github.com/jhu-dvrk/dvrk_model.git
git clone /path/to/dvrk_isaac_sim
```

Do not create a separate overlay just for `crtk_msgs` or `dvrk_model` for the normal workflow.

## ROS 2 workspace dependencies

The sourced ROS 2 environment must provide:

- `rclpy`;
- `crtk_msgs`;
- `geometry_msgs`;
- `sensor_msgs`;
- `std_msgs`;
- `image_transport` for endoscope image transport;
- `dvrk_model`, or an explicit `DVRK_MODEL_PATH`.

Build the workspace with the same Python version used by Isaac Sim:

```bash
python3 --version
# Expected: Python 3.12.x

cd /path/to/isaac_sim_ws
colcon build --symlink-install
```

The Python package setup validates and saves the Isaac Sim installation during `colcon build`. If `ISAAC_SIM_DIR` has not yet been set, the package still builds with a warning, but its installed scripts report that the workspace must be rebuilt with `ISAAC_SIM_DIR`. Set it to either a direct installation directory or a source/build root containing `_build/linux-x86_64/release/python.sh`, then rebuild:

```bash
export ISAAC_SIM_DIR=/path/to/isaac-sim
source /opt/ros/jazzy/setup.bash
cd /path/to/isaac_sim_ws
colcon build --symlink-install
```

The setup step expands `~`, searches both locations, resolves the directory containing the executable `python.sh`, and saves it in the git-ignored `share/isaac_sim.yaml`. Later builds can omit `ISAAC_SIM_DIR` and use the saved path. If the variable is set to a different installation, setup warns and updates the saved path. The repository only tracks the machine-independent `share/isaac_sim.yaml.example` template.
Source ROS 2 Jazzy before building. The package setup invokes the validation using the Python interpreter selected by colcon.

### Forcing Python 3.12

If `python3` resolves to another version, do not change the system-wide `python3` symlink with `update-alternatives`; that can break ROS 2 and other system tools. Invoke the 3.12 interpreter explicitly instead:

```bash
source /opt/ros/jazzy/setup.bash

/usr/bin/python3.12 --version
# Expected: Python 3.12.x

cd /path/to/isaac_sim_ws
/usr/bin/python3.12 -m colcon build --symlink-install
```

If `colcon` is not installed as a Python module for 3.12, create a dedicated 3.12 environment that can still see the system ROS packages:

```bash
python3.12 -m venv --system-site-packages /path/to/ros2_py312
source /path/to/ros2_py312/bin/activate
python --version
# Expected: Python 3.12.x

python -m pip install colcon-common-extensions
source /opt/ros/jazzy/setup.bash
cd /path/to/isaac_sim_ws
python -m colcon build --symlink-install
```

For CMake packages in a mixed workspace, the Python executable can also be made explicit:

```bash
python -m colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3.12
```

After building, verify the generated Python message package with the same interpreter:

```bash
source /path/to/isaac_sim_ws/install/setup.bash
/usr/bin/python3.12 -c "from crtk_msgs.msg import OperatingState; print(OperatingState)"
```

The interpreter used to build `crtk_msgs` and other custom ROS 2 messages must match the Python interpreter used by Isaac Sim. Isaac Sim 6.0 uses Python 3.12.

## Required shell setup

Every shell that launches Isaac Sim or communicates with it must source the system ROS 2 installation and the single project overlay:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/isaac_sim_ws/install/setup.bash
```

When using Isaac Sim's bundled ROS 2 libraries, set the ROS distribution and middleware before launching Isaac Sim:

```bash
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

If the simulator is launched from a native ROS 2 installation, use the corresponding ROS 2 environment consistently for all processes. Do not mix incompatible ROS 2 distributions or Python versions.

## dVRK model assets

If `dvrk_model` is in the sourced workspace, the asset pipeline can discover it through the ROS package index. Otherwise configure the repository explicitly:

```bash
export DVRK_MODEL_PATH=/path/to/dvrk_model
```

The simulator uses the Xacro/URDF and mesh files from this repository as the source of truth. Generated USD assets are cached separately; see the [design specification](design.md#51-usd-conversion-and-caching).

## Custom CRTK messages in Isaac Sim Python

`crtk_msgs/msg/OperatingState` is a ROS 2 custom message rather than an Isaac Sim built-in message. When Python code running inside Isaac Sim imports it, the ROS 2 workspace containing `crtk_msgs` must have been sourced before Isaac Sim starts:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/isaac_sim_ws/install/setup.bash
export ROS_DISTRO=jazzy

cd /path/to/isaac-sim
./python.sh /path/to/dvrk_isaac_sim/scripts/simulator.py
```

The same rule applies to any future project-specific ROS 2 messages. They must be built for Python 3.12 and be visible through the sourced workspace.

## ROS 2 adapter integration test

The current backend-independent ROS adapter can be run for one configured component:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/isaac_sim_ws/install/setup.bash

ros2 run dvrk_isaac_sim dvrk_isaac_sim_ros \
  --ros-args \
  -r __ns:=/PSM1 \
  -p robot_config:="$(ros2 pkg prefix dvrk_arm_description)/share/dvrk_arm_description/arms/PSM1.yaml"
```

The Isaac Sim integration test below uses the same sourced environment to launch the simulator and its in-process CRTK ROS adapter.

Run the package unit tests with `colcon test` or `python3 -m pytest -q`.
Run a bounded headless test of one configured scene with:

```bash
ros2 launch dvrk_isaac_sim test_scene.launch.py \
  scene:=ECM_PSM1_PSM2_PSM3_stereo.yaml
```

## Interactive full-cart benchmark

Use the opt-in benchmark on the target GPU to exercise the full ECM/PSM1/PSM2/
PSM3 stereo-RTSP scene through an external native ROS 2 client.  It sends safe,
limit-clamped `servo_jp` trajectories to every arm, checks state and `/clock`
timestamps, probes the local RTSP stream, and writes a JSON report when asked.

```bash
python3.12 scripts/benchmark_interactive.py \
  --duration 60 \
  --output /tmp/dvrk-isaac-interactive.json
```

This benchmark is intentionally separate from the normal test suite: its rate
thresholds are hardware- and renderer-dependent.  Tune `--min-state-rate-hz`
and `--min-rtsp-fps` for the target workstation, retain the JSON output with
performance results, and use it as the interactive regression gate.

For renderer and camera-resolution A/B measurements, the benchmark can override
the renderer and create a temporary scaled scene without changing the checked-in
scene YAML:

```bash
python3.12 scripts/benchmark_interactive.py --renderer MinimalRendering
python3.12 scripts/benchmark_interactive.py --camera-scale 0.5
```

To separate the RTSP server/encoder path from the camera scene, compare the
normal benchmark with `--no-rtsp-probe` (server graph, no local decoder) and
`--disable-rtsp` (temporary scene with no RTSP graph).  The local probe uses
the same GPU for H.264 decoding by default, so its result intentionally
measures the end-to-end single-GPU case rather than server rendering alone.
Use `--rtsp-decoder avdec_h264` to move only the local measurement client's
H.264 decode to the CPU; it is useful for diagnosing same-GPU contention, not
as a replacement for Isaac Sim's server-side encoder.

## USD asset conversion

The repository does not commit generated USD files. The source of truth remains
the virtual Xacro/URDF and meshes from dvrk_model; conversion is an explicit,
repeatable build step and generated assets are cached under `~/.cache/dvrk_isaac_sim/`.

Source the ROS 2 workspace first so Xacro can resolve dvrk_model, then invoke
the converter with Isaac Sim's Python:

    source /opt/ros/jazzy/setup.bash
    source /path/to/isaac_sim_ws/install/setup.bash
    export DVRK_MODEL_PATH=/path/to/dvrk_model  # optional when the package is sourced

    ${ISAAC_SIM_DIR}/python.sh \
      /path/to/isaac_sim_ws/install/dvrk_isaac_sim/share/dvrk_isaac_sim/scripts/convert_dvrk_model.py \
      --model PSM1 --instrument 420006

Use --model PSM2, --model PSM3, or --model ECM for the other virtual
components. The output directory can be overridden with --output-dir; the
default is the `~/.cache/dvrk_isaac_sim/` directory. This
keeps conversion separate from the runtime ROS interface and makes it possible to review or regenerate assets
without committing generated files.

By default, the converter removes importer-authored Physics schemas because
the project uses kinematic motion and does not need PhysX rigid bodies. Use
--keep-physics only when experimenting with dynamic simulation.

## Isaac Sim ROS 2 simulation

The simulator is configured from `share/isaac_sim.yaml`. This file is installed
with the package and is the recommended place to save researcher-specific
settings. It contains the Isaac Sim path, generated-asset cache, renderer,
startup mode, duration, fixed simulation rate, and ROS environment. Scene-specific robots, instruments,
endoscope, and camera settings belong in the selected scene YAML.
It intentionally does not select a default scene.

Scenes are stored under `share/scenes/*.yaml`. The repository includes
individual PSM examples (`PSM1_420006_mono.yaml`, `PSM2_420093_mono.yaml`, and
`PSM3_420006_mono.yaml`) plus two- and three-PSM cart scenes. Scene files select the
robots, frames, and instrument variants.

The simulator launch command has two user-facing options:

- `config:=...` selects a saved config file;
- `scene:=...` selects a scene filename under `share/scenes` or an explicit YAML path;

`test_scene.launch.py` accepts those options and `timeout:=...` for the
bounded test window. Runtime settings stay in the backend configuration.

If neither `scene:=...` nor `scene` in the config is provided, startup stops and
prints every YAML scene found in the config file's `scenes` directory.

From the sourced ROS 2 environment:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/isaac_sim_ws/install/setup.bash
ros2 launch dvrk_isaac_sim simulator.launch.py scene:=PSM1_420006_mono.yaml
```

To save settings, copy the example config and edit it:

```bash
cp /path/to/isaac_sim_ws/install/dvrk_isaac_sim/share/dvrk_isaac_sim/share/isaac_sim.yaml \
   /path/to/my-dvrk-isaac.yaml
$EDITOR /path/to/my-dvrk-isaac.yaml
ros2 launch dvrk_isaac_sim simulator.launch.py \
  config:=/path/to/my-dvrk-isaac.yaml scene:=ECM_PSM1_PSM2_PSM3_mono.yaml
```

Set `camera.mode` to `mono`, `stereo`, or `off` in the scene YAML. Set
`camera.transports` to any combination of `ros_raw`, `ros_compressed`, and
`rtsp`. `ros_compressed` publishes standard JPEG on `image_raw/compressed`,
which works with `rqt_image_view`; the ROS image paths are subscriber-gated.
RTSP uses Isaac Sim's NVENC-backed server and is enabled continuously. Configure all outputs with:
```yaml
scene:
  camera:
    transports: [ros_raw, ros_compressed, rtsp]
    ros_compressed: {quality: 85}
    rtsp:
      port: 8554
      mount_path: /ECM
      encoding: h264
```
Set `renderer` to the desired Isaac Sim renderer in the config YAML. Set
`simulation_rate_hz` to the fixed ROS/kinematic update rate and
`render_rate_hz` to the independent wall-clock render target; they default to
120 Hz and 30 Hz respectively.
Mono publishes `/ECM/image_raw` and `/ECM/camera_info`. Stereo publishes one
synchronized side-by-side image on `/ECM/image_raw`, with twice the configured
width; RTSP streams that same tiled image on the configured `/ECM` mount path.
Missing USD/URDF conversion artifacts are generated automatically in the
configured `generated_dir`.

The Isaac runner services ROS callbacks on their own executor thread and
advances kinematic models on a separate fixed-rate loop with
`dt = 1 / simulation_rate_hz`. A shared component lock makes command
application and state publication atomic. The main Isaac thread renders only
the newest state at a wall-clock-controlled
`render_rate_hz`, so a slow render never queues stale intermediate poses. The
runner publishes `/clock` from simulation time and reports real-time factor,
actual control/render/camera rates, average/maximum render time, control-state
age, and per-arm command counters once per second.

While Isaac Sim is paused, `/clock` stops and robot positions do not advance.
Periodic CRTK messages continue so clients can detect that the process is alive,
but their header timestamps are set to zero to indicate invalid/stale data.
Operating-state events remain latched and are not replaced by a pause event.
 Users normally do not need to pass conversion paths
or individual asset paths on the launch command line.

In a second shell, source the same environments and inspect the topics:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/isaac_sim_ws/install/setup.bash
ros2 topic list | grep -E '^/(PSM1|ECM)/'
ros2 topic echo /PSM1/measured_js
ros2 topic echo /ECM/measured_cp
```

Send a joint command from the second shell:

```bash
ros2 topic pub --once /PSM1/move_jp sensor_msgs/msg/JointState \
  "{name: [yaw, pitch, insertion, roll, wrist_pitch, wrist_yaw], position: [0.2, -0.1, 0.12, 0.0, 0.0, 0.0]}"
```

The corresponding `measured_js` and `measured_cp` values should move over
subsequent simulation steps.
