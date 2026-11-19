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
scene primitives and instrument collision proxies are simulated by Newton in a
separate Python worker. Newton is the default collision backend; set
`collision_backend: none` in the simulator configuration to disable physics.

The `test_cube`, `ball_tray`, and `match_row` scenes provide contact test setups.
Instrument collision proxies are generated from PSM visual meshes using the
profile in `share/collision_primitives/instrument_collision.yaml`; V-HACD runs
during conversion, not during simulation. Install the isolated worker and
collision-generation dependencies with `pip install -r requirements-collision.txt`,
then set `newton_python` in the simulator config or `DVRK_NEWTON_PYTHON` to that
environment's interpreter. See [installation](docs/installation.md) for details.

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

For two Haply devices and PCsensor foot pedals with the cisst dVRK system GUI:

```bash
ros2 launch dvrk_isaac_sim haply.launch.py
```

This starts the Isaac patient cart (`ECM_PSM1_PSM2_PSM3_haply.yaml`), the
`test_cube_objects.yaml` exercise, and the Haply dVRK system from the
`haply_config` configuration (devices, pedals, teleoperation). Start Haply
Service, configure device serial numbers and pedal permissions, and run only
one simulator profile at a time; see the
[Haply setup](../../cisst-saw/sawHaplyService/share/dvrk_simulation/README.md).
Home and enable teleoperation using the dVRK system GUI, then hold the
operator-present pedal. The Isaac and Newton collision worker dependencies
described above are also required.

There is no separate camera viewer window. The cart streams the mono ECM
camera over RTSP with h264 encoding; the raw RTSP encoding is avoided because
Isaac's raw RTSP path horizontally mirrors the image. To look through the
endoscope in the Isaac viewport, use the viewport's camera selector (the
camera icon in the viewport toolbar) and pick `Camera` under `World/ECM`; it
follows the ECM as it moves. The stream also remains available at
`rtsp://localhost:8554/ECM` for an optional external viewer.

Use `exercise:=ball_tray_objects.yaml` or `exercise:=match_row_objects.yaml`
for the other collision test scenes, or pass an exercise overlay path.
`headless:=true` disables the Isaac desktop viewer.

The first launch for a new instrument or exercise converts assets inside the
Isaac worker before the simulator reports ready. If that exceeds the frontend's
180-second startup limit, the session stops with "simulation process stopped
responding"; relaunching reuses the cached assets under
`~/.cache/dvrk_isaac_sim` and starts much faster.

For the full daVinci-Si setup with physical MTMs, MTML DQLA foot pedals, and the
control panel, run from this workspace:

```bash
bash src/dvrk/dvrk_isaac_sim/scripts/run_dvrk_full.sh --env test_cube
```

The runner sources this checkout's workspace and uses the existing
`dvrk_config_jhu davinci_si_isaac_sim.launch.py` profile without modifying its
system configuration. It does not start Haply or automatically home/enable
teleoperation. Use `--env ball_tray`, `--env match_row`, or an exercise YAML path
to select another scene.

The runner maps the three built-in exercise names to their `*_objects.yaml`
overlays, so the patient-cart profile supplies robots and camera settings only
once. The original scene files remain standalone single-PSM tests. Explicit YAML
paths passed to the runner must be overlays without duplicate robot definitions.
