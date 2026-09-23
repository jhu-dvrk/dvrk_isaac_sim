# dVRK Isaac Sim implementation plan

This is the active version-1 plan for the ROS 2 / Isaac Sim kinematic patient-cart
simulator. Completed refactors are summarized briefly below; the actionable
sections contain only remaining work.

## Current state

The repository now has:

- typed simulator and scene YAML loading with shared launch/runtime resolution;
- scene-owned PSM/ECM frames, instruments, endoscope, and camera settings;
- kinematic PSM and ECM models with FK, Jacobian, IK, Cartesian view-frame conversion, and jaws;
- CRTK ROS 2 topics, operating-state events, QoS, `/clock`, and paused zero-stamped periodic data;
- manifest-driven visual-only USD updates, mono/stereo ECM image publication, and GUI monitoring;
- workspace-root `.generated/isaacsim-6.0` asset caching;
- `simulator.launch.py`, `test_scene.launch.py`, `simulator.py`, and configuration validation;
- 16 pure-Python tests plus headless Isaac Sim scene integration tests.

Version 1 does not require backward-compatible code or YAML aliases. Acronyms
remain uppercase in APIs and filenames: `CRTK`, `PSM`, `ECM`, `USD`, and `URDF`.

## Priority 1 — Kinematics and asset consistency

### 1.1 Establish the kinematics source of truth

- Define URDF-manifest kinematics as authoritative when a valid manifest exists.
- Report clearly whether analytical or URDF-manifest kinematics are active.
- Reject incompatible joint names, ordering, or mimic-joint definitions before startup.
- Add random-sample FK comparisons between analytical and URDF paths.
- Add Jacobian and IK regression cases near joint limits and singularities.
- Verify PSM/ECM DH and URDF frame conventions against the dVRK reference data.

### 1.2 Improve asset conversion and caching

- Store source model, instrument/endoscope, converter options, Isaac Sim version, and source metadata in the manifest.
- Detect stale generated assets from manifest metadata instead of relying only on directory names.
- Add a cache inspection command and a dry-run mode showing expected output paths.
- Separate Xacro expansion, manifest generation, USD import, and cleanup into testable functions.
- Add an explicit converter compatibility check for Isaac Sim 6.0.

## Priority 2 — ROS/CRTK behavior and integration

### 2.1 Define interface behavior precisely

- Document accepted joint-name ordering and partial-name behavior.
- Add a servo timeout/watchdog only if a downstream controller requires one.
- Define all operating-state transitions and rejected-transition behavior.
- Define jaw limits, units, and mimic behavior in the robot schema.
- Verify transient-local state delivery to late subscribers.
- Verify state, event, `/clock`, camera, and command timestamps with ROS simulation time clients.

### 2.2 Cartesian verification

- Test PSM commands while ECM yaw, pitch, insertion, and roll move.
- Test `measured_cp`, `setpoint_cp`, `measured_cv`, `move_cp`, and `servo_cp` in the moving view frame.
- Test explicit `world` frame commands.
- Test orientation-only, translation-only, and unreachable commands.
- Verify IK failures publish a warning and complete the corresponding move handle.

### 2.3 Native ROS 2 integration tests

- Add a native ROS 2 node test for joint command round trips.
- Add Cartesian command round-trip tests.
- Add operating-state QoS tests with late subscriptions.
- Add jaw command and state tests.
- Add tests for zero timestamps while Isaac Sim is paused.
- Keep these tests optional when the ROS 2 environment is unavailable.

## Priority 3 — Camera and sensor interface

- Use the configured camera frequency for Isaac camera creation and publication.
- Verify mono and stereo calibration values against the scene YAML.
- Verify tiled stereo image dimensions, calibration topics, and RTSP output.
- Provide JPEG `image_transport` for interoperable ROS video; use RTSP for high-rate video.
- Add image timestamp and frame-consistency tests.
- Add a low-rate/headless camera test profile that checks image dimensions, encoding, and non-empty frames.
- Add a diagnostic mode that reports camera pose and optical/view axes when the image is blank.

## Priority 4 — GUI and usability

- Display measured and setpoint Cartesian poses.
- Display jaw position and jaw command controls for PSMs.
- Show current scene name, renderer, simulation rate, and pause state.
- Show command rejection and IK error messages in the GUI.
- Add reset and home controls with clearly defined semantics.
- Add a compact scene/robot status summary for multi-PSM scenes.

## Priority 5 — Configuration and frame providers

- Add schema version fields and validation for simulator, scene, and robot YAML files.
- Validate camera dimensions, frame, rate, stereo baseline, instrument, and endoscope fields.
- Validate that every YAML-configured base frame has a complete pose.
- Define a frame-provider interface.
- Keep YAML frame configuration as the deterministic default.
- Add a TF-based provider with timeout and stale-transform handling.
- Document precedence when both YAML and TF are configured.

## Priority 6 — Testing and package quality

- Add malformed, incomplete, and schema-version configuration tests.
- Add URDF-manifest and mimic-joint tests.
- Add asset-cache naming and invalidation tests.
- Add Cartesian ECM-view conversion tests independent of ROS middleware.
- Add package import checks under Python 3.12.
- Add formatting/linting configuration.
- Add Apache-2.0 license and replace placeholder maintainer metadata.
- Decide whether documentation should be installed with the package.
- Keep repository automation deferred for now; use `colcon test` for local validation.

## Priority 7 — Additional devices and scene content

- Add support for additional PSM instruments through scene-only changes where possible.
- Add optional anatomy/task geometry without coupling it to patient-cart robot models.
- Add tiled stereo-camera image-content and RTSP behavior tests.
- Keep dynamics and full patient-cart CAD outside the core scope unless separately approved.

## Recommended next sequence

1. Complete the manifest-driven USD visual update and kinematics comparison work.
2. Add native ROS 2 round-trip tests for joint, Cartesian, jaw, and operating-state interfaces.
3. Add camera image-content and frame-consistency diagnostics.
4. Improve the GUI after the runtime and interface behavior are test-covered.
5. Add TF frame support and further instruments.

## Decisions already made

- URDF-manifest kinematics are preferred when valid; analytical kinematics remain a fallback.
- No backward-compatible aliases are required for version 1.
- Scene files own instruments, endoscopes, cameras, robot lists, and frames.
- Simulator config owns Isaac Sim path, renderer, generated-asset path, duration, simulation rate, and ROS environment.
- Compressed image output remains available through the ROS image transport interface.
- `/clock` is published from the Isaac Sim runtime.
- Missing or unavailable scene assets fail clearly or are generated by the launch path.
- Full patient-cart CAD and dynamics are not project goals.
