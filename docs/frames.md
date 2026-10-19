# Frames and conventions

## 1. Coordinate tree

The initial scene uses this logical frame tree:

```text
world
├── PSM1_base
│   └── PSM1_RCM
│       └── PSM1_tool
└── ECM_base
    └── ECM_RCM
        └── ECM_optical
```

The exact USD prim names may differ, but the logical names are part of the simulator interface.

Each robot's base transform is defined relative to its configured `parent_frame`. The initial source is YAML:

```yaml
base_frame: PSM1_base
parent_frame: world
base_pose:
  position: [0.0, 0.0, 0.0]
  orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
```

Future releases may obtain the same transform from TF. The frame names and logical tree remain unchanged.

## 2. Units

- Translation: meters.
- Rotation: radians in ROS joint messages and Python APIs.
- Quaternion order: ROS convention `(x, y, z, w)` at the ROS boundary.
- Isaac Sim internal quaternion ordering must be converted explicitly at adapter boundaries.
- Velocities: meters/second and radians/second.

## 3. RCM and insertion

Yaw and pitch are applied at the RCM. Insertion translates along the configured instrument/endoscope shaft axis after the configured adaptor offset.

The positive insertion direction must be verified against the imported `dvrk_model` URDF and recorded in the implementation tests. It must not be inferred from visual appearance.

## 4. Cartesian pose frames

`measured_cp` represents the pose of the active tool frame:

- PSM: the instrument/tool frame;
- ECM: the endoscope optical/view frame.

PSM Cartesian topics use the current `ECM_view` frame when an ECM is present. The simulator derives that frame from the ECM optical FK using the dVRK view axes (X left, Y up, Z away). A command with `header.frame_id: world` is treated as an explicit world-frame command.

The ECM camera must be attached to `ECM_optical`, not directly to the mechanical adaptor frame. This keeps the rendered camera orientation separate from the mechanical endoscope model.

### Haply orientation matching

The single-master `system-MTMR-PSM1-Haply-ROS.json` configuration enables
`match_MTM_orientation` in the dVRK PSM teleoperation controller. This requires
rebuilding `sawIntuitiveResearchKitCore` and restarting `dvrk_system`.
The PSM commanded rotation matches the Haply `measured_cp` rotation numerically
in the configured teleoperation axes instead of retaining an initial rotation
offset. Translation remains relative with scale 0.4 and re-anchors on enable
and clutch release. While clutched, the PSM holds; matching resumes on release.
A calibrated orientation reset reported by Haply is followed while teleoperation
is enabled. If calibration disables teleoperation, enable it again afterward.

This option defaults to false for other configurations. Rotation lock takes
precedence. It does not register physical stylus mounting axes to the tool,
change the Haply base transform, or override PSM IK and joint limits. Both
devices must use compatible teleoperation reference axes. Large orientation
changes can be commanded on enable, clutch release, or recalibration; use this
configuration only with the simulated PSM and validate clear-space motion first.

## 5. Endoscope view

The simulator renders the endoscope perspective from a camera attached to `ECM_optical`. Camera pose follows the kinematic ECM state, including yaw, pitch, insertion, and roll.

The rendering pipeline provides configurable image dimensions, field of view, clipping planes, simulation-time timestamps, and camera calibration information. Stereo uses two cameras with a configurable baseline, packed into one 1x2 tiled render product.

ROS image publication uses `image_transport` conventions. Raw image, JPEG compressed image, and camera information are the ROS interfaces; RTSP is an optional network-video transport.

The simulator publishes these base topics:

```text
/ECM/image_raw
/ECM/camera_info
```

For stereo, `/ECM/image_raw` contains a synchronized side-by-side image with twice the configured width. Per-eye calibration remains available on `/ECM/left/camera_info` and `/ECM/right/camera_info`. With RTSP enabled, the same tiled image is streamed at `rtsp://<host>:8554/ECM`.

## 6. Required tests

Frame tests must verify:

- home pose values;
- zero yaw/pitch orientation;
- positive and negative yaw/pitch directions;
- positive insertion direction;
- PSM and ECM Cartesian poses against known analytical results;
- ROS quaternion conversion;
- camera pose equals the ECM optical-frame pose.
- rendered image timestamp follows simulation time;
- camera intrinsics match `camera_info`;
- image orientation and optical axis match `ECM_optical`;
- image publication has defined behavior when rendering is unavailable.
