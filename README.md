# dVRK Isaac Sim

`dvrk_isaac_sim` is a lightweight simulated patient-side robot environment for Isaac Sim 6.0 and ROS 2.

The project focuses on software integration and kinematic simulation rather than hardware or dynamic fidelity. It provides configurable virtual PSMs and an ECM, dVRK/CRTK-style interfaces, selectable dVRK instrument visuals, and a rendered endoscope view.

## Current scope

- ROS 2 only.
- Isaac Sim 6.0.
- Kinematic PSM control with `yaw`, `pitch`, `insertion`, `roll`, `wrist_pitch`, and `wrist_yaw`; jaw control is provided as a separate logical one-joint interface.
- Kinematic ECM control with `yaw`, `pitch`, `insertion`, and `roll`.
- Startup profiles for two or three PSMs plus an ECM.
- Reuse of visual assets from [`dvrk_model`](https://github.com/jhu-dvrk/dvrk_model).
- YAML-defined robot base frames, with future TF support planned.
- ROS image transport for the ECM view.

Full patient-cart CAD, dynamics, contact simulation, and hardware-runtime dependencies are outside the project scope.

## Documentation

- [Installation and runtime environment](docs/installation.md)
- [Design specification](docs/design.md)
- [Frames and conventions](docs/frames.md)
- [ROS 2 interface](docs/ros_interface.md)
- [Implementation roadmap](docs/implementation_plan.md)
- [Simulator-base migration plan](docs/simulator_base_migration_plan.md)
- Shared PSM/ECM configuration is provided by `dvrk_simulator_base`.
- [Two-PSM scene](share/scenes/ECM_PSM1_PSM2_mono.yaml)
- [Three-PSM mono scene](share/scenes/ECM_PSM1_PSM2_PSM3_mono.yaml)
- [Three-PSM stereo scene](share/scenes/ECM_PSM1_PSM2_PSM3_stereo.yaml)
- [PSM1 + ECM example](share/scenes/PSM1_420006_mono.yaml)
- [PSM2 + ECM example](share/scenes/PSM2_420093_mono.yaml)
- [PSM3 + ECM example](share/scenes/PSM3_420006_mono.yaml)
- [Haply MTML/MTMR + ROS patient-cart system](share/dvrk_systems/system-MTML-MTMR-Haply-patient-cart-ROS.json)
- [Cart frame generator](scripts/generate_cart_frames.py)

## dVRK resources

- [Official dVRK documentation](https://dvrk.readthedocs.io/)
- [dVRK ROS and software documentation](https://dvrk.readthedocs.io/main/)
- [`sawIntuitiveResearchKit`](https://github.com/jhu-dvrk/sawIntuitiveResearchKit)
- [`dvrk_model`](https://github.com/jhu-dvrk/dvrk_model)
- [dVRK ROS 2 software documentation](https://dvrk.readthedocs.io/main/pages/software/ros-2.html)

The simulator reuses dVRK model assets and interface conventions but does not require the dVRK runtime, cisst, or SAW.

The recommended workflow is to build and launch through ROS 2:

```bash
mkdir -p /path/to/isaac_sim_ws/src
cd /path/to/isaac_sim_ws/src
git clone https://github.com/collaborative-robotics/crtk_msgs.git
git clone https://github.com/jhu-dvrk/dvrk_model.git
git clone /path/to/dvrk_isaac_sim

cd /path/to/isaac_sim_ws/src/dvrk_isaac_sim
export ISAAC_SIM_DIR=/path/to/isaac-sim
source /opt/ros/jazzy/setup.bash
cd /path/to/isaac_sim_ws
colcon build --symlink-install

source /opt/ros/jazzy/setup.bash
source /path/to/isaac_sim_ws/install/setup.bash
ros2 launch dvrk_isaac_sim simulator.launch.py
```

Start the full virtual cart with the default three PSMs and kinematic ECM, or select the two-PSM profile:

```bash
ros2 launch dvrk_isaac_sim simulator.launch.py config:=/path/to/my-dvrk-isaac.yaml scene:=PSM1_420006_mono.yaml
```

The public launch interface has only `scene:=` and optional `config:=`.
Renderer, headless mode, duration, rates, ROS middleware, and generated asset
location belong in the backend configuration; robots, instruments, camera, and
transport settings belong in the scene YAML.

The default renderer is `RaytracedLighting`, which provides a visible viewport on the supported Isaac Sim setup. Instruments, endoscopes, and camera settings are defined by each scene. The simulator adds neutral lighting and a gray environment so dark instruments remain visible. `simulation_rate_hz` controls the ROS/kinematics loop and `render_rate_hz` independently controls wall-clock rendering; their defaults are 120 Hz and 30 Hz. A once-per-second performance line reports real-time factor, actual control/render/camera rates, and render time.

The CRTK monitor is provided by the external PyQt5 rqt plugins, never by an
Isaac Kit window. After starting the simulator, launch one Arm panel for each
configured robot:

```bash
rqt --standalone rqt_crtk/Arm --args --arm PSM1
```

It displays state in degrees/mm and sends normal `state_command`, `move_jp`,
`jaw/move_jp`, and `move_cp` CRTK messages. Use `rqt_crtk/Diagnostics` for
the simulator health and timing data published on `/diagnostics`.

The ECM has no mesh in the full-cart scene. Its kinematic optical frame drives the Isaac camera. Mono publishes `/ECM/image_raw` and `/ECM/camera_info`. Stereo publishes one synchronized side-by-side image on `/ECM/image_raw`; its tiled RTSP stream uses `rtsp://<host>:8554/ECM`.

To regenerate the default cart frame snippet after changing the constants, run:

```bash
./scripts/generate_cart_frames.py
```

The initial ROS 2 adapter can be run for one configured component:

```bash
ros2 run dvrk_isaac_sim dvrk_isaac_sim_ros \
  --ros-args -r __ns:=/PSM1 \
  -p robot_config:="$(ros2 pkg prefix dvrk_arm_description)/share/dvrk_arm_description/arms/PSM1.yaml"
```

## Tested teleoperation

### 3Dconnexion

The ROS 2 interface has been tested with the dVRK system configuration using an old Logitech 3Dconnexion SpaceBall 5000 as the MTMR input and the simulated `/PSM1` as the puppet. The corresponding dVRK configuration is `system-MTMR-3Dconnexion-PSM1_from_ROS.json` in `saw3Dconnexion/share`.

Start Isaac Sim first, source the Isaac Sim ROS 2 workspace, and launch the PSM/ECM simulation. Then start the dVRK system with the SpaceBall configuration. The PSM should report `ENABLED` and `is_homed=true`; MTMR motion should teleoperate the simulated PSM Cartesian pose and jaw.

### Haply

Start the Haply service first and run each command in its own terminal. The Isaac scene publishes the PSM1, PSM2, PSM3, and ECM ROS interfaces; the dVRK system configuration connects Haply MTML/MTMR devices to those interfaces. The ECM JPEG topic `/ECM/image_raw/compressed` works with `rqt_image_view`; RTSP remains available for low-latency viewing.

Terminal 1, Isaac Sim:

```bash
source /opt/ros/jazzy/setup.bash
source ~/wss/isaac/install/setup.bash
ros2 launch dvrk_isaac_sim simulator.launch.py scene:=ECM_PSM1_PSM2_PSM3_mono.yaml
```

Terminal 2, camera view:

```bash
gst-launch-1.0  \
  rtspsrc location=rtsp://localhost:8554/ECM \
    protocols=udp buffer-mode=none latency=0 drop-on-latency=true \
    ntp-sync=false do-retransmission=false \
  ! rtph264depay wait-for-keyframe=true \
  ! h264parse \
  ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream \
  ! nvh264dec max-display-delay=0 \
  ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream \
  ! videoconvert \
  ! autovideosink sync=false
```

Terminal 3, dVRK system:

```bash
source /opt/ros/jazzy/setup.bash
source ~/wss/dvrk/install/setup.bash
cd ~/wss/isaac/src/dvrk_isaac_sim/share/dvrk_systems
ros2 run dvrk_robot dvrk_system -j system-MTML-MTMR-Haply-patient-cart-ROS.json
```

The Haply configuration expects the Haply service at `ws://localhost:10001`. The console input mode is simulated; Haply MTML/MTMR provide the teleoperation devices, while PSM and ECM state comes from Isaac Sim over ROS 2.

### OpenXR

The OpenXR patient-cart launch uses the full three-PSM/ECM stereo RTSP scene:

```bash
ros2 launch dvrk_isaac_sim open_xr.launch.py
```

See [the OpenXR configuration](share/open-xr/README.md) for prerequisites and
the optional `config:=` and `console:=` arguments.

## Testing

The current automated tests are pure-Python tests for configuration, scene
loading, kinematics, frame conversion, and operating-state behavior. Run them
from the package directory after installing the ROS 2 workspace dependencies:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/isaac_sim_ws/install/setup.bash
cd /path/to/isaac_sim_ws/src/dvrk_isaac_sim
python3.12 -m pytest -q
```

To also verify ROS 2 packaging and installation:

```bash
cd /path/to/isaac_sim_ws
colcon build --symlink-install --packages-select dvrk_isaac_sim
```

To run one bounded headless Isaac Sim scene smoke test:

```bash
ros2 launch dvrk_isaac_sim test_scene.launch.py \
  scene:=ECM_PSM1_PSM2_PSM3_stereo.yaml
```

Pass `timeout:=5.0` only when a longer smoke-test window is needed.

The repository also provides a configuration-only validation command. From the
workspace root:

```bash
cd /path/to/isaac_sim_ws
python3.12 src/dvrk_isaac_sim/scripts/validate_config.py
```

Or, from the package directory:

```bash
python3.12 scripts/validate_config.py
```

It validates the simulator config and every scene without starting Isaac Sim.
