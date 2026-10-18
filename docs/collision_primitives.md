# Instrument Collision Generation

## Default Hybrid Profile

Newton ROS launches select `share/collision_primitives/instrument_hybrid.yaml`.
It fits one cylinder to the shaft and uses V-HACD through `vhacdx==0.0.10`
to decompose each wrist and jaw visual mesh independently. `trimesh==5.1.1`
loads the OBJ triangles. Install `requirements-collision.txt` in the isolated
Newton Python environment, or select it using `DVRK_COLLISION_PYTHON`.
This uses the standalone V-HACD algorithm, not Unity's VHACD integration.

```bash
ros2 run dvrk_isaac_sim convert_dvrk_model.py --model PSM2 --instrument 420093 --collision-profile /path/to/dvrk_isaac_sim/share/collision_primitives/instrument_hybrid.yaml --require-instrument-collisions --force
```

For `fit: vhacd`, `max_hulls` bounds the hull count per visual mesh,
`max_vertices` bounds vertices per hull, `resolution` controls voxelization,
and `volume_error_percent` controls the decomposition error target. Defaults
use at most three hulls per wrist visual and four per jaw visual, with 32
vertices per hull. These are initial budgets, not certified grasp tolerances.
Check jaw recess clearance at open and closed poses before relying on grasping.

Each hull is stored as an OBJ under `collision_hulls/` and as explicit points
and triangle indices in manifest format 7. The runtime authors separate USD
meshes with `convexHull` approximation; it never decomposes meshes. Mesh scale
is baked into vertices, and the visual origin is applied once through link FK.
Input hashes, backend version, and settings are recorded. ROS launch invalidates
stale profile/mesh/Xacro inputs. Conversion stages the entire asset, publishing
only after the USD import succeeds; failures retain the previous cache.

Newton prints first-contact and new-maximum summaries with top proxy pairs and
self/interinstrument/environment counts. Newton and MJWarp share an 8192-contact
capacity, with explicit overflow failure before solver stepping. Startup contacts
can be interinstrument overlaps: a larger buffer does not fix intersecting poses.

Each rule matches one instrument part, then applies one action:

- `replace`: remove that part's mesh hull entries and fit either the shaft
  cylinder or V-HACD convex hulls from visual OBJ vertices. Matching visual-only
  links are fitted too, even when the instrument URDF supplies no collision
  elements for them.

Useful rule fields:

- `enabled`: set false to ignore the rule.
- `match.source_link_suffix`: target a link such as `roll_link`,
  `wrist_pitch_link`, `jaw_1_link`, or `jaw_2_link`.
- `fit`: `cylinder` for the roll shaft, or `vhacd` for wrist and jaw meshes.
- `axis`: `x`, `y`, `z`, or `auto`.
- `slices`: split a cylinder fit along the fit axis.
- `margin`: inflate the cylinder in meters.

To tune visually in Isaac Sim, edit the hybrid profile, regenerate the asset with
`--force`, and inspect the red collision prims under
`/World/<PSM>/PhysicsLinks`. Adjust `margin` for shaft contact clearance or the
V-HACD hull budgets for wrist and jaw contact coverage.

Newton ROS launches require valid generated coverage of all five instrument
links, whether primitive or convex hull. Direct runs of `scripts/simulator.py`
do not generate assets and use the geometry already in the cache.
