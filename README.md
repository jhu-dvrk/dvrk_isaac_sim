# dVRK Isaac Sim

Isaac Sim renders a configured dVRK patient cart while a separate ROS 2 process
handles CRTK subscriptions, publications, and the common simulator diagnostics.
The processes use the same Unix-socket IPC as Newton and PyBullet. FK, IK, and
reference-frame conversion run in the Isaac worker.

Build and source the dVRK workspace, then select an installed Isaac interpreter:

```bash
export ISAAC_SIM_DIR=~/devel/isaac
colcon build --symlink-install --packages-select dvrk_isaac_sim
source install/setup.bash
ros2 launch dvrk_isaac_sim simulator.launch.py scene:=ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml headless:=true
```

`DVRK_ISAAC_SIM_PYTHON` can select an interpreter explicitly. Otherwise the
runtime uses its cached selection, a discovered Isaac installation, a workspace
virtual environment, or the current interpreter. Nothing is selected at build
time. Generated assets and interpreter selection are cached under
`$XDG_CACHE_HOME/dvrk_isaac_sim` (normally `~/.cache/dvrk_isaac_sim`).

The generic launch accepts `config`, `scene`, `headless`, `rqt`, and
`rqt_console`. Runtime YAML controls the renderer, simulation/render/publication
rates, command capacity, and optional `generated_root`. Scene YAML controls
robots, instruments, base poses, exercise objects, and camera settings. The CLI
accepts repeated `--scene` arguments to combine a patient cart with an exercise.

Missing PSM assets are converted inside the worker's existing Isaac application.
The ECM remains a kinematic camera frame without a visible endoscope mesh.
Exercise URDF objects are imported with their declared fixed/dynamic behavior;
USD objects retain their authored physics. PSMs remain visual kinematic robots;
this backend does not implement Newton/PyBullet's grasp attachment behavior.

Control and rendering are scheduled independently, defaulting to 120 Hz and
30 Hz. Camera poses are applied before rendering and images are extracted
afterwards. Common `/diagnostics` metrics report simulation, camera, publication,
and snapshot reception rates plus snapshot age.

Isaac currently uses native RTSP video only, defaulting to
`rtsp://localhost:8554/ECM`; stereo uses a side-by-side tiled image.
Newton and PyBullet continue to support Unix-FD video.

```bash
ros2 launch dvrk_isaac_sim open_xr.launch.py scene:=tray_cubes.yaml rqt:=true
ros2 run dvrk_isaac_sim clean_cache
python3 src/dvrk/dvrk_isaac_sim/scripts/validate_config.py
python3 src/dvrk/dvrk_isaac_sim/scripts/generate_cart_frames.py
```

System launches that include a control panel leave homing and teleoperation to
the operator. The optional interactive benchmark remains an explicit ROS client
for checking joint response and RTSP decoding; it does not start physical robots.
