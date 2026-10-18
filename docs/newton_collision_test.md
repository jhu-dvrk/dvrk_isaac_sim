# Experimental Newton collision backend

The simulator uses Newton for instrument/environment contacts and dynamic scene
props. Isaac still renders the scene and cameras. Newton is the contact backend
for both the ROS launch path and the existing full-cart bash launcher.

This first version retains the current position-driven arm motion. Jaw commands
are filtered through a bounded logical PID before updating the kinematic jaw
colliders, which avoids instant jaw teleports and reduces bounce/chatter during
closure. It does not yet add compliant Newton-owned jaw articulations or arm
contact back-reaction. Existing table floor and optional kinematic motion guards
remain separate from Newton contacts.

## Run in this workspace

Newton 1.6.0, its simulation/importer dependencies, and USD support have been
installed in the package's isolated `.venv-1` environment. Isaac's bundled
Python/Warp packages were not replaced. After building the package and sourcing
the normal ROS environment, launch:

```bash
bash ~/dvrk_ws/src/dvrk/dvrk_isaac_sim/scripts/run_dvrk_full.sh --env test_cube
```

The same command opens the full Newton cart and the existing MTMR/PSM1 Haply
system. `--env ball_tray` selects the ball tray, and `--env match_row` selects
the three match-like sticks on a table. Interpreter discovery uses
`DVRK_NEWTON_PYTHON` when set, then the local/workspace virtual environments.
Device selection is automatic: CUDA when available, CPU otherwise.

For a single-PSM test:

```bash
ros2 launch dvrk_isaac_sim simulator.launch.py \
  scene:=PSM1_420006_mono.yaml env:=test_cube.yaml
```

Newton remains the default `collision_backend`, but the simulator can be
started without the Newton worker by passing `collision_backend:=none`. Passing
`collision_backend:=kinematic_guard` enables the existing kinematic motion guard
without starting Newton contacts.

Use the existing joint/cartesian/jaw controls to move the instrument against
the cube. In the monitor's bottom `Newton Collisions` panel, `Collision meshes`
toggles the overlays; `Instrument visuals` can be disabled to inspect only the
colliders. Both are enabled initially.
`Frame colliders` fits the viewport to the instrument collision geometry.
Collision copies use renderable purpose/visibility and translucent debug materials,
independent of CAD styling.
Flattened instrument collision geometry is red and turns green when
its Newton shape participates in a generated contact during the latest physics
interval. The performance log includes `newton-contacts`. This count includes
all generated contacts, including environment contacts; it is not a grip-force
measurement. Dynamic prop poses are rendered from Newton's output.
Overlapping proxies belonging to the same instrument are filtered; cross-arm
and instrument/environment pairs are retained.

No generated robot assets or source manifests are modified by changing
environments. Debug collision copies are made editable only in the runtime
stage.

For a direct source-script test, from the package directory with ROS sourced.
Unlike `simulator.launch.py`, this bypasses asset generation, so the generated
assets must already contain the desired collision geometry:

```bash
/home/yzhan874/isaacsim/python.sh scripts/simulator.py \
  --scene PSM1_420006_mono.yaml --env test_cube.yaml
```

## Implementation and limits

The ROS launch path requests the hybrid instrument collision profile when
generating PSM assets for Newton. A mesh-only or stale cached PSM asset is
regenerated; launch fails rather than silently starting Newton unless roll,
wrist pitch, wrist yaw, and both jaw links all contain generated collision
coverage. The default profile fits one cylinder to the roll shaft and uses
V-HACD convex decomposition for the wrist and jaws. Edit
`share/collision_primitives/instrument_hybrid.yaml` to tune the active Newton
profile. The converter rewrites only the generated cache, not source meshes.

Newton imports the resulting flattened `PhysicsLinks` subtrees and environment
USD geometry. Importer tests cover primitive dimensions and offsets, including
convex hull dimensions and offsets. Contact generation and response need not
match PhysX exactly. No source mesh, manifest, or generated asset is rewritten
for display. A separate Python
worker avoids incompatible Newton/Warp versions loaded by Isaac extensions. It
uses Newton's `CollisionPipeline` with
`SolverMuJoCo(use_mujoco_contacts=False)` and physics substeps no larger than
1/240 second. Instrument roots interpolate between rendered control snapshots;
dynamic props stay solver-owned. The bridge synchronously exchanges poses and
contact shape IDs, so this is a comparison prototype, not a performance-tuned
or hard-real-time backend.
Newton advances at most one render interval per update to avoid unbounded CPU
catch-up after kernel compilation or rendering stalls. Under load its physics
can run slower than the ROS/control clock; this is not a timing-faithful
benchmark. Instrument motion still follows the latest control snapshot.

PhysX collision and rigid-body participation are disabled in the runtime stage
after Newton imports the scene. The simulator does not create a PhysX physics
scene or PhysX prop-specific attributes. Isaac may still load its PhysX
extensions; those do not resolve the Newton-owned contacts. Static environment
geometry and kinematic tools cannot physically move in response to forces. No
contact-position or force-arrow overlay is implemented yet.

The driver mismatch on the tested machine has been resolved. Automatic device
selection now recognizes the RTX 4080, and the default one-PSM Isaac launch
completed on `cuda:0`, importing 36 shapes and stepping contacts. The first GPU
launch compiles kernels and can take tens of seconds; later launches reuse the
kernel cache. A headless smoke test does not establish interactive frame rate.

On another machine, create a separate Python 3.12 environment and install
`newton[sim,importers]==1.6.0` and `usd-core` there, then pass its Python executable
as `newton_python`. Do not install a different Warp over Isaac's bundled runtime.

## Validation

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv-1/bin/python -m pytest test -q
```

Tests cover primitive dimensions/offsets, inherited guide/hidden visibility,
debug materials and CAD visibility independence, interpreter discovery,
a prop resting on a table, a moving prescribed box and contact color,
isolated-worker pose transfer/scale preservation/cleanup, and instanced collision
copy highlighting. The existing kinematic and collider tests are unchanged.
