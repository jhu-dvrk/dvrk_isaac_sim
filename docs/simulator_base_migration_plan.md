# `dvrk_simulator_base` migration plan

Status: in progress on `feat-simulator-base`.

## Current progress

Completed in the initial migration slice:

- Isaac depends on and directly imports the base configuration, quaternion,
  Cartesian-frame, command-validation, operating-state, ROS-message, and QoS
  modules.
- Isaac scene robot entries resolve canonical arm YAML through
  `package://dvrk_simulator_base/share/arms/...`; duplicate Isaac arm YAML and
  their installation entries are removed.
- The Isaac-only generated URDF kinematics manifest is passed explicitly to
  the Isaac kinematic model and USD visual synchronizer, rather than being
  stored in the backend-neutral `RobotConfig`.
- The Isaac CRTK adapter now uses base `CommandMailboxes` and `ArmSnapshot`:
  ROS callbacks enqueue validated commands, the Isaac control loop applies
  them, and periodic ROS publication consumes an immutable snapshot.

Remaining phases below are intentionally not complete.

## Goal and scope

Make Isaac Sim a backend of `dvrk_simulator_base`, sharing the same ROS 2 and
CRTK behavior as `dvrk_pybullet`. Isaac will continue to own USD conversion,
articulation/FK/IK, rendering, camera acquisition, video transport, and the
Isaac application lifecycle. The base package will own configuration, frame
semantics, command handling, operating state, snapshots, ROS message/QoS
behavior. Desktop monitoring is supplied by external PyQt5 rqt plugins.

This branch excludes exercises, task objects, contacts, grasping, collision
behavior, and physics-fidelity work. Its target is PSM1, PSM2, PSM3, and ECM,
including PSM Cartesian control in the live ECM view frame and an OpenXR
teleoperation example equivalent to the PyBullet example.

## Current duplication to remove

| Isaac module/data | Base replacement | Action |
| --- | --- | --- |
| `config.py` | `dvrk_simulator_base.config` | Use base `RobotConfig`, `JointConfig`, and loaders; then remove local code. |
| `scene.py` | `dvrk_simulator_base.scene` | Use `SceneResolver`/`load_scene_config`; retain only Isaac runtime and camera settings. |
| `rotations.py`, `cartesian_frames.py` | Base equivalents | Replace imports; remove local modules. |
| `command_validation.py`, `operating_state.py` | Base equivalents | Replace imports; remove local modules. |
| `ros_messages.py`, `ros_qos.py` | Base equivalents | Replace imports; remove local modules. |
| `ros_interface.py`, `ros_node.py` | Base mailbox/snapshot/ROS helpers | Refactor as an Isaac adapter; do not retain an independent CRTK state machine. |
| `isaac_ui.py`, `monitor.py` | `rqt_crtk/Arm`, `rqt_crtk/Diagnostics` | Removed; use external generic rqt plugins. |
| `share/arms/*.yaml` | Base installed arm YAML | Remove after all scenes resolve package-owned base arm files. |

Retain Isaac-specific code: `camera.py`, `kinematics.py` while it is the
temporary backend implementation, `usd_visual.py`, USD conversion tools, the
Isaac application loop, and Isaac-only runtime configuration.

## Phase 0 — Establish the compatibility baseline

1. Add `dvrk_simulator_base` as an Isaac runtime dependency.
2. Add tests for PSM/ECM base-config loading, joint limits, CRTK topic/QoS
   behavior, state transitions, command rejection, and frame conversion.
3. Record a ROS graph for a PSM+ECM Isaac scene and compare it with PyBullet.
   Any difference must be intentional and documented.
4. Keep asset conversion, rendering settings, and scene geometry unchanged.

Exit criterion: the existing behavior has executable tests before duplicate
code is removed.

## Phase 1 — Adopt base configuration and static frame semantics

1. Replace local configuration, scene, rotation, and frame imports with base
   imports.
2. Introduce `runtime_config.py` for only `isaac_sim_dir`, generated USD cache,
   renderer, headless mode, rate settings, and camera/video options. It must
   not redefine robot or scene data classes.
3. Resolve scenes in this order: supplied path, Isaac-specific scenes, then
   base exercises when those are later enabled. No exercise asset is loaded in
   this branch.
4. Load all PSM/ECM definitions from the base package and apply Isaac scene
   overrides with `with_base_pose`, not by mutating YAML documents.
5. Remove Isaac `share/arms` and its `setup.py` data-file entries only after
   installed-package scene tests pass.

Exit criterion: base configuration, quaternion convention, and frame names
have one source of truth.

## Phase 2 — Isaac backend and shared CRTK runtime

1. Implement `IsaacArmBackend` for PSM and ECM under the base kinematics
   contract. It owns simulator-thread-only joint application/measurement,
   Isaac FK/IK/Jacobian calls, USD prim/link mapping, and rendering sync.
2. Move command acceptance, trajectory timing, state machine, bounded command
   mailbox, and snapshot construction to base components. ROS callbacks enqueue
   commands only; they never call Isaac APIs.
3. Publish `ArmSnapshot` objects from the simulator owner thread. ROS and GUI
   consume snapshots and therefore never access Isaac state directly.
4. Apply one dynamic transform chain consistently:

   ```text
   world -> PSMi_base -> PSMi_tool
   world -> ECM_base  -> ECM_optical -> ECM_view
   ```

   PSM Cartesian data uses live `ECM_view` if an ECM is present; explicit world
   commands remain valid. Joint commands remain base-local. Use the base
   optical-to-view helper and validate PSM1, PSM2, and PSM3 independently.
5. Keep the current kinematic model only as the temporary implementation of
   `IsaacArmBackend`, never as a second CRTK model.

Exit criterion: matching joint states produce backend FK reported using the
same CRTK frame semantics in Isaac and PyBullet.

## Phase 3 — Use external rqt monitoring

1. Remove all Isaac Qt monitor code. Do not start a Qt event loop in Isaac
   Kit's process: Kit and Qt have independent UI/event-loop ownership.
2. Use `rqt_crtk/Arm` for standard CRTK state, joint, jaw, and Cartesian
   controls over ROS 2. It has no Isaac imports or runtime references.
3. Publish simulator health/timing as `diagnostic_msgs/DiagnosticArray` on
   `/diagnostics`, for display by `rqt_crtk/Diagnostics`.

Exit criterion: Isaac and PyBullet expose the same monitor controls, units,
state display, and command behavior.

Implementation status: complete. `isaac_ui.py` and the PyQt6 monitor were
removed. `rqt_crtk/Arm` supplies the ROS CRTK monitor and
`rqt_crtk/Diagnostics` consumes `dvrk_isaac_sim/runtime` diagnostics. Video
and OpenXR remain backend-local.

## Phase 4 — OpenXR patient-cart example

The current Isaac implementation uses native RTSP only. Unix-FD video is
postponed; Newton and PyBullet keep their socket transports. The shared base
owns the IPC worker loop, ROS interfaces, and common diagnostics. Isaac owns
USD assets, kinematics, reference-frame conversion, rendering, and RTSP.

`launch/open_xr.launch.py` combines the patient cart and an exercise scene.
JHU system launches expose only `exercise`, `rqt`, and `headless`. When a
control panel is launched, operators home the system and enable teleoperation
from that panel; launch files do not send those commands automatically.
