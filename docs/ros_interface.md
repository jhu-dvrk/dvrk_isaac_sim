# ROS 2 interface contract

This document defines the supported ROS 2 interface surface for the kinematic simulator.

The same names are used by the internal Python interfaces. The ROS adapter is a transport layer, not a naming translation layer.

The first ROS 2 adapter is a Python node installed as `dvrk_isaac_sim_ros`. It accepts one `robot_config` parameter and is intended to be namespace-remapped per device:

```bash
ros2 run dvrk_isaac_sim dvrk_isaac_sim_ros \
  --ros-args -r __ns:=/PSM1 \
  -p robot_config:="$(ros2 pkg prefix dvrk_arm_description)/share/dvrk_arm_description/arms/PSM1.yaml"
```

## 1. Namespaces

The supported arm namespaces are:

```text
/PSM1
/PSM2
/PSM3
/ECM
```

The namespace must be configurable so multiple PSMs can be launched independently.

The supported multi-PSM launch profiles are:

```text
PSM1 + PSM2 + ECM
PSM1 + PSM2 + PSM3 + ECM
```

Each PSM publishes under its own namespace. Base-frame configuration is currently loaded from the selected scene YAML profile. TF-based base-frame lookup is reserved for a future release.

PSMs publish their six-digit instrument identifier once on `/<PSM>/tool_type` as
`std_msgs/String` when the simulation starts. The topic uses reliable,
transient-local QoS with depth 1, so late subscribers receive the retained
identifier. ECM components do not publish this topic.

## 2. Supported topics

### PSM

```text
/PSM1/measured_js
/PSM1/measured_cp
/PSM1/measured_cv
/PSM1/move_jp
/PSM1/servo_jp
/PSM1/move_cp
/PSM1/servo_cp
```

### ECM

```text
/ECM/measured_js
/ECM/measured_cp
/ECM/measured_cv
/ECM/move_jp
/ECM/servo_jp
/ECM/move_cp
/ECM/servo_cp
```

The initial payloads follow CRTK 1.0:

```text
measured_js   sensor_msgs/JointState (PSM order: yaw, pitch, insertion, roll, wrist_pitch, wrist_yaw)
measured_cp   geometry_msgs/PoseStamped
measured_cv   geometry_msgs/TwistStamped
move_jp       sensor_msgs/JointState
servo_jp      sensor_msgs/JointState
move_cp       geometry_msgs/PoseStamped
servo_cp      geometry_msgs/PoseStamped
operating_state crtk_msgs/OperatingState
state         crtk_msgs/StringStamped
```

The current adapter publishes `measured_js`, `measured_cp`, `measured_cv`, `setpoint_js`, `operating_state`, and `state`, and subscribes to `move_jp`, `servo_jp`, `move_cp`, and `servo_cp`. Six-DOF PSM Cartesian commands use position-and-orientation IK. ECM Cartesian commands remain position-only because its four joints cannot generally satisfy a full six-axis pose. Both `move_cp` and `servo_cp` use `geometry_msgs/PoseStamped`, matching the dVRK ROS bridge and CRTK Python client.

Each arm also publishes diagnostic events using `crtk_msgs/msg/StringStamped`: `/<arm>/info`, `/<arm>/warning`, and `/<arm>/error`. Initialization milestones and accepted operating-state changes are published on `info`; rejected commands and IK failures are published on `warning`. A rejected Cartesian move still completes its move handle with a busy-start/busy-end pair so CRTK clients do not block indefinitely.

### Endoscope view

Mono scenes publish a rendered endoscope view through ROS 2 image transport:

```text
/ECM/image_raw
/ECM/image_raw/compressed
/ECM/camera_info
```

`camera.transports` independently enables `ros_raw`, `ros_compressed`, and `rtsp`. `ros_compressed` publishes a standard JPEG `sensor_msgs/CompressedImage` on `/ECM/image_raw/compressed`, compatible with `rqt_image_view` and other `image_transport` consumers. Raw and JPEG capture/encoding are skipped until their respective ROS topic has a subscriber. RTSP remains an explicitly enabled continuous stream.

The ROS image topics are published at the configured camera rate (default 30 Hz), independently of the simulation update rate. Mono image and camera-info messages use the same simulation timestamp and the configured ECM optical frame ID. Set `scene.camera.mode` to `mono`, `stereo`, or `off` in the selected scene YAML. Stereo publishes one synchronized side-by-side image at `/ECM/image_raw` and its JPEG form at `/ECM/image_raw/compressed`, with per-eye calibration on `/ECM/left/camera_info` and `/ECM/right/camera_info`.

For ROS and RTSP output together, configure:

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

The stream URL is `rtsp://SIMULATOR_HOST:8554/ECM`. Mono scenes stream the mono camera render product; stereo scenes stream one synchronized 1x2 tiled render product, with the same `/ECM` mount path. No separate per-eye RTSP streams are created.

The current adapter also publishes state topics:

```text
/PSM1/state
/PSM1/operating_state
/ECM/state
/ECM/operating_state
```

## GUI monitor and controls

`rqt_crtk/Arm` is an external PyQt5 ROS 2 panel, not an Isaac Kit window. Start
it after the simulator, for example:

```bash
rqt --standalone rqt_crtk/Arm --args --arm PSM1
```

Each configured PSM or ECM has a panel showing its CRTK operating state, homed status, and measured joints. Revolute joints are displayed in degrees; insertion joints are displayed in millimetres.

The joint fields are editable target values. `Apply` publishes `move_jp` (and
`jaw/move_jp` for PSMs); Cartesian Apply publishes `move_cp` in the most
recently received CRTK Cartesian frame. The operating-state selector publishes
the same `state_command` interface used by any other ROS client.

## 3. Time and pause semantics

The Isaac Sim runner uses a dedicated fixed-rate ROS/kinematics loop configured
by `simulation_rate_hz` in `share/isaac_sim.yaml` (default: 120 Hz). Rendering
independently samples the newest state at the wall-clock `render_rate_hz`
(default: 30 Hz). It publishes `/clock` from simulation time. All normal CRTK
and camera timestamps use the same source.

ROS subscription callbacks run on a separate executor thread. The periodic
performance line includes cumulative `rx`, `apply`, `coalesce`, and `reject`
counts for each arm that has received commands. `age` is the most recent
callback-to-application delay; `control-state-age` shows the age of the
kinematic snapshot at render submission and completion. These values separate
command-path problems from a slow camera/render path.

When the Isaac timeline is paused, `/clock` stops and robot state does not
advance. Periodic CRTK state messages continue with a zero timestamp, which
means the simulator process is alive but the reported state is not currently
valid. Operating-state and other event messages retain their latched event
semantics and are not emitted merely because the timeline was paused.

## 4. State semantics

`measured_js` reports the current simulated joint positions and velocities, in the configured joint order. Effort is not physically simulated and should either be omitted or clearly reported as unavailable according to the selected message contract.

`measured_cp` reports the active tool pose using the configured Cartesian reference frame.

`measured_cv` reports the corresponding spatial velocity computed from the joint state and Jacobian, not from noisy physics sensors.

Simulated PSMs and ECM start in `ENABLED` with `is_homed=true` and the insertion joint initialized to `0.12 m`. Motion commands are accepted only in `ENABLED`; `DISABLED`, `PAUSED`, and `FAULT` hold the current joint position. The `operating_state.state` and `state.string` fields are kept synchronized. `move_*` commands publish two busy-edge events: `operating_state.is_busy=true` when a move starts and `false` when the target is reached. `servo_*` commands do not change `is_busy`.

`state_command` uses `crtk_msgs/msg/StringStamped` and the command is carried in its `string` field. The state publishers use reliable, transient-local QoS with a keep-last depth of 10, so late subscribers receive the retained recent state events. State is published at startup and after each accepted state command; motion publishes only its busy-start and busy-end events.

The supported commands are:

```text
enable       -> ENABLED
disable      -> DISABLED
pause        -> PAUSED
resume       -> ENABLED
home         -> is_homed=true
unhome       -> is_homed=false
fault        -> FAULT
clear_fault  -> DISABLED
```

`home` and `unhome` are logical operations in this kinematic simulator; no physical homing motion is performed. Invalid commands are rejected and leave the state unchanged.

## 5. Command semantics

`move_jp` is a target command. The simulator interpolates to the target.

`servo_jp` and `servo_cp` update the current target without changing `operating_state.is_busy`. The simulator retains the latest Cartesian servo command until it is processed by the next simulation update.

Cartesian commands are implemented through the configured FK/Jacobian and kinematic IK. PSM commands use position and orientation; ECM commands use the reachable position component.

## 6. Compatibility policy

The ROS adapter must keep message conversion separate from robot logic. Each supported interface version should have a named converter so future dVRK/CRTK changes do not require modifying the kinematic core.

Internal classes and methods should use the same CRTK vocabulary: `measured_js`, `measured_cp`, `measured_cv`, `move_jp`, `servo_jp`, `move_cp`, `servo_cp`, `state`, and `operating_state`. Isaac-specific names belong only in the Isaac backend.

Image transport is a separate sensor interface and does not need a CRTK state or command name. Its camera pose is owned by the ECM optical frame.

### PSM jaw interface

PSMs expose a logical one-joint `jaw` interface in radians. Both `jaw/move_jp` and `jaw/servo_jp` accept a `sensor_msgs/msg/JointState` with one position value. The value drives the two generated instrument jaw links using their URDF mimic ratios (+0.5 and -0.5). State is reported on `jaw/measured_js` and `jaw/setpoint_js`.

The default 420006 limits are -0.349066 to 1.39626 radians; commands outside the configured limits are rejected.

### PSM Cartesian reference frame

When an ECM is present, PSM `measured_cp`, `setpoint_cp`, and `measured_cv` are published in the current `ECM_view` frame. The conversion is explicitly FK-based: PSM world FK is transformed into the PSM base frame, then into the current dVRK view frame derived from ECM optical FK (X-left, Y-up, Z-away). Incoming PSM `move_cp` and `servo_cp` commands follow the reverse path—current ECM view frame to PSM base frame to world—before inverse kinematics. This keeps Cartesian teleoperation aligned while the ECM moves. A command with `header.frame_id: world` is accepted as an explicit world-frame diagnostic command.

The dVRK system configuration should therefore use identity PSM base transforms with `reference_frame: ECM_view`; the patient-cart poses belong to Isaac Sim, not to the teleoperation system configuration. MTML and MTMR retain explicit `HRSV` base transforms so Haply motion follows the dVRK surgeon-display convention.

Select `/ECM/image_raw` or `/ECM/image_raw/compressed` in `rqt_image_view`.
The compressed topic is JPEG image_transport-compatible and needs no separate
republisher.
