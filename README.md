# dVRK Isaac Sim

`dvrk_isaac_sim` provides a ROS 2 patient-cart simulation for Isaac Sim 6.0.
It publishes CRTK-style PSM and ECM interfaces, renders an endoscope camera,
and uses the canonical PSM/ECM descriptions from `dvrk_arm_description`.

It is intended for software integration, visualization, and kinematic
simulation. It does not model full patient-cart mechanics, contacts, or
hardware I/O.

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, and Isaac Sim 6.0.
- `ISAAC_SIM_DIR` set to the Isaac Sim installation while first building.
- The dVRK workspace packages, including `dvrk_arm_description`, `dvrk_model`,
  `dvrk_simulator_base`, and `rqt_dvrk` when using the monitor.

See [installation.md](docs/installation.md) for the workspace and Isaac Python
setup.

## Build

```bash
source /opt/ros/jazzy/setup.bash
export ISAAC_SIM_DIR=/path/to/isaac-sim
cd ~/wss/dvrk
colcon build --symlink-install --packages-select dvrk_isaac_sim
source install/setup.bash
```

The first build records the Isaac Sim path in the ignored
`share/isaac_sim.yaml`. The tracked
[`share/isaac_sim.yaml.example`](share/isaac_sim.yaml.example) is the portable
template.

## Run a scene

```bash
ros2 launch dvrk_isaac_sim simulator.launch.py \
  scene:=ECM_PSM1_PSM2_PSM3_mono.yaml
```

The launch interface is deliberately small:

- `scene:=...` selects a scene in `share/scenes` or an absolute scene YAML.
- `config:=...` selects the backend runtime YAML.
- `rqt:=true` starts the dockable dVRK monitor: tabbed Arms and Diagnostics.
- `rqt_console:=true` additionally includes the dVRK Console widget.

Renderer, `headless`, rates, middleware, and generated assets belong in the
backend configuration. Robots, instruments, world objects, camera, and
transport settings belong in the scene YAML. `headless: false` opens the Isaac
Kit viewport; `headless: true` runs without it.

Useful shipped scenes include:

- [`ECM_PSM1_PSM2_mono.yaml`](share/scenes/ECM_PSM1_PSM2_mono.yaml)
- [`ECM_PSM1_PSM2_PSM3_mono.yaml`](share/scenes/ECM_PSM1_PSM2_PSM3_mono.yaml)
- [`ECM_PSM1_PSM2_PSM3_stereo.yaml`](share/scenes/ECM_PSM1_PSM2_PSM3_stereo.yaml)
- [`ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml`](share/scenes/ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml)

## Monitoring and camera output

The simulator does not embed monitoring controls in Isaac Kit. Use the ROS
monitor launched with `rqt:=true`, or run a panel explicitly:

```bash
rqt --standalone rqt_dvrk/Arm --args --arm PSM1
rqt --standalone rqt_dvrk/Arms --args --arm PSM1 --arm ECM
rqt --standalone rqt_dvrk/Diagnostics
```

The ECM optical frame drives the simulated camera. Mono scenes publish
`/ECM/image_raw` and `/ECM/camera_info`; stereo scenes produce a synchronized
side-by-side image. RTSP settings and endpoints are defined by the selected
scene.

For a scene with RTSP enabled, view the ECM stream with GStreamer:

```bash
gst-launch-1.0 \
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

## Teleoperation and OpenXR

The repository includes dVRK system configurations for Haply and OpenXR
workflows. The OpenXR example uses the three-PSM/ECM stereo RTSP scene and
forces Isaac headless mode:

```bash
ros2 launch dvrk_isaac_sim open_xr.launch.py rqt:=true
```

See [share/open-xr/README.md](share/open-xr/README.md) for OpenXR prerequisites
and [docs/ros_interface.md](docs/ros_interface.md) for CRTK topics and command
semantics.

## Testing

Run the fast configuration and kinematics tests after sourcing the workspace:

```bash
cd ~/wss/dvrk
source install/setup.bash
pytest -q src/dvrk/dvrk_isaac_sim/test
```

Run a bounded Isaac smoke test when Isaac Sim is available:

```bash
ros2 launch dvrk_isaac_sim test_scene.launch.py \
  scene:=ECM_PSM1_PSM2_PSM3_stereo.yaml timeout:=5.0
```

## Further documentation

- [Installation and runtime environment](docs/installation.md)
- [Design](docs/design.md)
- [Frames and conventions](docs/frames.md)
- [ROS interface](docs/ros_interface.md)
- [Implementation roadmap](docs/implementation_plan.md)
