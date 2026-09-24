# dVRK Isaac Sim design specification

## 1. Purpose

`dvrk_isaac_sim` provides a lightweight simulated patient cart in Isaac Sim 6.0 for software integration and surgical robotics research.

The initial simulator is kinematic. It does not depend on dVRK Classic, dVRK Si, cisst, SAW, or a dynamic model. It exposes a ROS 2 interface modeled after the dVRK/CRTK interface.

The first supported devices are:

- PSM: yaw, pitch, insertion, roll, wrist pitch, and wrist yaw control with a selectable dVRK instrument visual; jaw is a separate logical CRTK joint driving URDF-mimic jaw visuals.
- ECM: yaw, pitch, insertion, and roll control with a minimal endoscope visual and rendered endoscope view.

The simulator can start with either two or three PSM instances. PSM instances are independently configured and use namespaces `/PSM1`, `/PSM2`, and `/PSM3`.

The first release prioritizes predictable installation, a stable ROS 2 interface, deterministic behavior, and testability over physical realism.

## 2. Design principles

1. The robot model is independent of ROS 2 and Isaac Sim.
2. ROS 2 and Isaac Sim are adapters around the robot model.
3. Kinematic state is the source of truth; USD transforms are a rendered representation.
4. Configuration, not Python code, selects robots, instruments, frames, and limits.
5. Existing assets from `dvrk_model` are reused without requiring the dVRK runtime.
6. The public interface is stable even if the internal IK implementation changes.
7. Every pose and velocity has an explicitly documented frame and unit convention.
8. Internal interfaces preserve CRTK/dVRK naming wherever the concept has a CRTK equivalent.
9. Robot base transforms are configuration-driven; a future TF-backed provider can use the same interface.

## 2.1 CRTK naming policy

CRTK names are the canonical names for internal robot capabilities, not merely aliases used by the ROS adapter.

Examples:

```text
measured_js       measured joint state
measured_cp       measured Cartesian pose
measured_cv       measured Cartesian velocity
move_jp           joint-position move command
servo_jp          joint-position servo command
move_cp           Cartesian-pose move command
servo_cp          Cartesian-pose servo command
state             component state
operating_state   operating-state interface
```

Internal classes should be component-oriented rather than Isaac-oriented. Preferred conceptual names are:

```text
CRTKComponent
CRTKArm
CRTKPSM
CRTKECM
JointStateInterface
CartesianPoseInterface
```

The exact Python class naming style may use normal Python `PascalCase`, but public methods and interface attributes should retain CRTK `snake_case` names. Device acronyms remain all caps. Isaac-specific classes should be limited to the backend layer, for example `IsaacPSMBackend` and `IsaacECMBackend`.

Do not introduce parallel names such as `get_pose`, `send_joint_target`, or `cartesian_state` when the corresponding CRTK name is `measured_cp`, `move_jp`, or `measured_cv`. Non-CRTK internal helpers may use descriptive implementation names when no CRTK concept exists.

## 3. Runtime architecture

```text
ROS 2 publishers/subscribers
            |
            v
      ROS 2 adapter
            |
            v
      Robot manager
       |          |
       v          v
  Kinematic    Command/
    models      state logic
            |
            v
     Isaac Sim backend
            |
            v
       USD scene
```

The kinematic model owns the current joint state, target state, limits, interpolation, and measured-state generation. The Isaac backend applies the resulting state to USD. The ROS adapter translates messages into model commands and publishes model state.

The ECM camera/rendering component owns the rendered view. Its pose is derived from the ECM optical frame. Mono uses one render product; stereo packs two baseline-separated cameras into a single tiled render product for the side-by-side ROS image and RTSP stream.

## 4. Initial degrees of freedom

### PSM

```text
[yaw, pitch, insertion, roll, wrist_pitch, wrist_yaw]
```

The virtual PSM model provides three geometry-free joints at the RCM and attaches the instrument to `PSM1_adaptor_link` or its equivalent for another prefix. Instrument roll, wrist pitch, and wrist yaw are now kinematic DOFs. Jaw remains a separate logical ROS interface because it is not part of the arm Cartesian joint vector.

The joint names and limits must remain configurable even though the default names match the dVRK model.

### ECM

```text
[yaw, pitch, insertion, roll]
```

The existing virtual ECM model provides these four joints and attaches the endoscope to `ECM_adaptor_link`. The project intentionally does not target full patient-cart CAD. The rendered endoscope, RCM reference, and optical frame are sufficient for view control.

## 5. Asset strategy

The simulator consumes the `dvrk_model` repository as an asset source. It must support both:

- discovering `dvrk_model` through the active ROS 2 workspace; and
- an explicit `DVRK_MODEL_PATH` configuration.

The initial PSM instrument defaults to model `420006` (Si large needle driver), but the model number is configurable. Instrument metadata comes from `urdf/common/instruments/instruments.yaml`.

The asset pipeline is responsible for:

1. locating the requested Xacro entry point;
2. expanding Xacro to URDF;
3. importing or converting the URDF to USD;
4. validating expected joint and frame names;
5. attaching the generated visual to the configured parent frame.

The simulator must not require a running dVRK process to load or display these assets.

### 5.1 USD conversion and caching

USD assets are generated from the `dvrk_model` Xacro/URDF files by a dedicated Isaac Sim conversion tool. The Xacro/URDF files and source meshes remain the source of truth; generated USD files are build artifacts.

The conversion tool should support:

```text
load      require an existing converted USD
convert   explicitly regenerate the USD asset
auto      load a valid cache entry or convert when missing/stale
```

`auto` is the convenient default for researchers. `load` is recommended for reproducible experiments and continuous integration.

Generated assets belong under the user cache `~/.cache/dvrk_isaac_sim`, outside `src`, `build`, and `install`. Generated assets should be cached using a key derived from the source Xacro/URDF and mesh content, instrument or endoscope selection, Isaac Sim version, conversion settings, and asset schema version.

A representative cache layout is:

```text
~/.cache/dvrk_isaac_sim/
├── PSM1_420006/
│   └── kinematics.json
└── ECM_Si_straight/
    └── kinematics.json
```

Multiple PSM instances should reference the same cached USD asset with different base transforms. Conversion must preserve joint names, link/frame names, `adaptor_link`, instrument hierarchy, materials where practical, units, limits, and source metadata.

The ECM camera is configured at runtime rather than embedded in the endoscope asset. Camera properties belong to the selected scene YAML; ROS transport and publication behavior belong to the sensor implementation. Generated USD files are not committed.

## 6. Scene and base-frame configuration

Robot YAML files are installed by `dvrk_simulator_base`. They support relative
`include` keys and deterministic deep merging; child mappings override shared
values while lists are replaced as a whole. Isaac scene profiles reference
these canonical files with `package://dvrk_arm_description/arms/...` URIs.


Scene profiles select the devices launched in a simulation. The initial profiles are:

```text
share/scenes/ECM_PSM1_PSM2_mono.yaml
share/scenes/ECM_PSM1_PSM2_PSM3_mono.yaml
share/scenes/ECM_PSM1_PSM2_PSM3_stereo.yaml
```

Each profile lists the PSM and ECM configuration files and their base-frame definitions. Every device has:

- a parent/reference frame;
- a unique base frame;
- an initial base pose relative to that parent frame;
- a device namespace.

The current implementation uses YAML poses. A future transform provider may resolve the same base frames through TF without changing the robot configurations:

```text
BaseFrameProvider
├── YamlBaseFrameProvider       current implementation
└── TfBaseFrameProvider         future option
```

The selected provider must be explicit in the scene configuration. A TF provider must define behavior for missing, stale, or inconsistent transforms.

## 7. Kinematic API

The backend-independent robot API should provide CRTK-named interfaces:

```python
measured_js() -> JointState
measured_cp(frame="tool") -> Pose
measured_cv(frame="tool") -> Twist

move_jp(joint_position) -> None
servo_jp(joint_position) -> None
move_cp(pose) -> IKResult
servo_cp(pose) -> IKResult

compute_fk(q=None, frame="tool") -> Pose
compute_jacobian(q=None, frame="tool") -> ndarray
compute_ik(target, seed=None) -> IKResult
```

`measured_js`, `measured_cp`, and `measured_cv` are state interfaces. `move_*` and `servo_*` are command interfaces. The test suite includes analytical FK/Jacobian coverage for the simplified chains. Isaac Sim 6.0 kinematics APIs may be used by the Isaac backend, but they must not leak into the public model API.

## 8. Command behavior

Position commands are time-continuous by default. The simulator interpolates from the current state to the target while respecting configured velocity limits.

The command engine supports:

- joint-position targets;
- joint and Cartesian servo targets;
- Cartesian pose targets through IK;
- explicit rejection of invalid commands.

The initial safe default is to reject invalid joint positions and report the failure. Clamping may be added as a configuration option, but should not be silent.

## 9. Time and determinism

Simulation time is the source of timestamps when Isaac Sim is running. A dedicated control thread services ROS and advances kinematic state at the fixed, user-configurable `simulation_rate_hz` (default 120 Hz), independently of the blocking Isaac render call. The main thread samples only the newest control state and renders it at the wall-clock-controlled `render_rate_hz` (default 30 Hz); intermediate visual states are intentionally dropped instead of queued. The runner publishes `/clock`; when paused, `/clock` stops and periodic CRTK messages use zero timestamps to indicate invalid/stale data. Once per second it reports real-time factor, actual control/simulation/render/camera rates, and render duration.

Reset must restore:

- joint positions;
- joint velocities;
- command queues;
- motion-event state;
- robot operating state;
- camera pose derived from the ECM.

## 10. Non-goals

The following are outside the project scope, including as long-term goals:

- dynamic simulation;
- contact forces;
- collision response;
- full patient-cart CAD;
- physical jaw dynamics;
- force/torque sensing;
- complete dVRK topic coverage;
- surgical anatomy models.
