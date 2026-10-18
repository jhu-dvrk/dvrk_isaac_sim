"""Optional Newton contact world using the existing flattened USD colliders."""

from __future__ import annotations

import math
import operator
from collections import Counter

import numpy as np


class CollisionDisplay:
    """Render the imported collider geometry independently of source styling."""

    def __init__(self, stage, shape_paths):
        from pxr import Sdf, UsdGeom, UsdShade

        self.stage = stage
        self.shapes = {}
        self.materials = {}
        self.visible = True
        self.visuals_visible = True
        self.visual_roots = {
            path.split("/PhysicsLinks/")[0] + "/Geometry"
            for path in shape_paths if "/PhysicsLinks/" in path
        }
        self.previous_active = None
        for active, color in ((False, (1.0, 0.1, 0.1)), (True, (0.1, 1.0, 0.1))):
            path = "/World/NewtonCollisionMaterials/" + ("Contact" if active else "Idle")
            material = UsdShade.Material.Define(stage, path)
            shader = UsdShade.Shader.Define(stage, path + "/Surface")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
            shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(tuple(value * 0.25 for value in color))
            shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.45)
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            self.materials[active] = material
        for path, shape in shape_paths.items():
            if "/PhysicsLinks/" not in path:
                continue
            prim = stage.GetPrimAtPath(path)
            if not prim.IsA(UsdGeom.Gprim):
                continue
            ancestor = prim
            while ancestor.IsValid() and str(ancestor.GetPath()) != "/":
                imageable = UsdGeom.Imageable(ancestor)
                if imageable:
                    imageable.CreatePurposeAttr().Set(UsdGeom.Tokens.default_)
                    imageable.CreateVisibilityAttr().Set(UsdGeom.Tokens.inherited)
                ancestor = ancestor.GetParent()
            self.shapes[path] = shape
        self.update(set())

    def update(self, active_shapes):
        from pxr import UsdGeom, UsdShade

        active_shapes = frozenset(active_shapes)
        if active_shapes == self.previous_active:
            return
        for path, shape in self.shapes.items():
            active = shape in active_shapes
            if self.previous_active is not None and active == (shape in self.previous_active):
                continue
            prim = self.stage.GetPrimAtPath(path)
            gprim = UsdGeom.Gprim(prim)
            gprim.CreateDisplayColorAttr().Set([(0.1, 1.0, 0.1) if active else (1.0, 0.1, 0.1)])
            gprim.CreateDisplayOpacityAttr().Set([0.45])
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                self.materials[active], bindingStrength=UsdShade.Tokens.strongerThanDescendants
            )
        self.previous_active = active_shapes

    def set_visible(self, visible):
        from pxr import UsdGeom

        self.visible = bool(visible)
        for path in self.shapes:
            UsdGeom.Imageable(self.stage.GetPrimAtPath(path)).CreateVisibilityAttr().Set(
                UsdGeom.Tokens.inherited if self.visible else UsdGeom.Tokens.invisible
            )

    def set_visuals_visible(self, visible):
        from pxr import UsdGeom

        self.visuals_visible = bool(visible)
        for path in self.visual_roots:
            prim = self.stage.GetPrimAtPath(path)
            if prim.IsValid():
                UsdGeom.Imageable(prim).CreateVisibilityAttr().Set(
                    UsdGeom.Tokens.inherited if self.visuals_visible else UsdGeom.Tokens.invisible
                )

    def frame_colliders(self):
        from omni.kit.viewport.utility import frame_viewport_prims

        return frame_viewport_prims(prims=list(self.shapes))


class NewtonContactWorld:
    def __init__(self, stage, roots, *, device="cuda:0", max_step=1.0 / 240.0, nconmax=8192):
        if isinstance(nconmax, bool):
            raise ValueError("nconmax must be a positive integer")
        try:
            nconmax = operator.index(nconmax)
        except TypeError as error:
            raise ValueError("nconmax must be a positive integer") from error
        if nconmax <= 0:
            raise ValueError("nconmax must be a positive integer")
        try:
            import newton
            import warp as wp
            from newton.solvers import SolverMuJoCo
        except ImportError as error:
            raise RuntimeError(
                "The isolated worker requires newton[sim,importers]==1.6.0 and usd-core."
            ) from error
        from pxr import Usd, UsdGeom, UsdPhysics

        self.newton = newton
        self.wp = wp
        self.stage = stage
        self.max_step = max_step
        self.body_paths = {}
        self.shape_paths = {}
        self.kinematic = {}
        self.dynamic = {}
        self.nconmax = nconmax
        self._contact_high_water = 0
        builder = newton.ModelBuilder()
        SolverMuJoCo.register_custom_attributes(builder)
        builder.begin_world()
        for root in roots:
            result = builder.add_usd(
                stage, root_path=root, load_visual_shapes=False,
                load_static_visual_shapes=False, load_sites=False,
            )
            self.body_paths.update(result["path_body_map"])
            self.shape_paths.update(result["path_shape_map"])
        builder.end_world()
        if not self.shape_paths:
            raise RuntimeError("Newton imported no collision shapes")
        instrument_shapes = {}
        for path, shape in self.shape_paths.items():
            if "/PhysicsLinks/" in path:
                instrument_shapes.setdefault(path.split("/PhysicsLinks/")[0], []).append(shape)
        for shapes in instrument_shapes.values():
            for index, shape in enumerate(shapes):
                for other in shapes[index + 1:]:
                    builder.add_shape_collision_filter_pair(shape, other)
        self.model = builder.finalize(device)
        self.solver = SolverMuJoCo(
            self.model, use_mujoco_contacts=False, integrator="implicitfast",
            iterations=30, nconmax=nconmax, njmax=4 * nconmax,
        )
        self.state = self.model.state()
        self.next_state = self.model.state()
        self.control = self.model.control()
        self.pipeline = newton.CollisionPipeline(
            self.model, rigid_contact_max=nconmax, broad_phase="explicit",
            shape_pairs_filtered=self.model.shape_contact_pairs,
            include_static_kinematic_pairs=True,
        )
        self.contacts = self.pipeline.contacts()
        self._shape_sources = {}
        for path, shape in self.shape_paths.items():
            if "/PhysicsLinks/" in path:
                instrument, relative = path.split("/PhysicsLinks/", 1)
                source = instrument + "/PhysicsLinks/" + relative.split("/", 1)[0]
                source_link = stage.GetPrimAtPath(source).GetCustomDataByKey("dvrk:source_link")
                if source_link:
                    source = instrument + "/" + str(source_link)
                self._shape_sources[shape] = (instrument, source)
            else:
                self._shape_sources[shape] = (None, path)
        self.contact_count = 0
        self.active_shapes = set()
        child = self.model.joint_child.numpy()
        starts = self.model.joint_q_start.numpy()
        velocity_starts = self.model.joint_qd_start.numpy()
        for path, body in self.body_paths.items():
            prim = stage.GetPrimAtPath(path)
            rigid = UsdPhysics.RigidBodyAPI(prim)
            if rigid.GetKinematicEnabledAttr().Get():
                joint = int(np.flatnonzero(child == body)[0])
                if int(starts[joint + 1] - starts[joint]) != 7:
                    raise RuntimeError(f"Newton proxy {path} must have a free root")
                self.kinematic[path] = (body, int(starts[joint]), int(velocity_starts[joint]))
            else:
                if not np.allclose(
                    UsdGeom.Xformable(prim.GetParent()).ComputeLocalToWorldTransform(Usd.TimeCode.Default()),
                    np.eye(4),
                ):
                    raise RuntimeError(f"Newton dynamic prop {path} requires an identity parent transform")
                scale = self._world_scale(prim, UsdGeom, Usd)
                xform = UsdGeom.Xformable(prim)
                xform.ClearXformOpOrder()
                translation = xform.AddTranslateOp()
                orientation = xform.AddOrientOp()
                if not np.allclose(scale, 1.0):
                    from pxr import Gf

                    xform.AddScaleOp().Set(Gf.Vec3f(*[float(value) for value in scale]))
                self.dynamic[path] = (body, translation, orientation)
        for root in roots:
            for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr().Set(False)
        newton.eval_fk(self.model, self.state.joint_q, self.state.joint_qd, self.state)
        shape_types = Counter(newton.GeoType(int(value)).name for value in self.model.shape_type.numpy())
        print(
            f"Newton contacts: {len(self.shape_paths)} shapes, "
            f"{len(self.kinematic)} prescribed bodies, {len(self.dynamic)} dynamic bodies; "
            f"types={dict(shape_types)}; device={self.model.device}; "
            f"nconmax={nconmax}, njmax={self.solver.mjw_data.njmax}",
            flush=True,
        )

    def _record_contacts(self):
        active = int(self.contacts.rigid_contact_count.numpy()[0])
        capacity = min(self.contacts.rigid_contact_max, self.nconmax)
        stored = min(active, capacity)
        shape0 = self.contacts.rigid_contact_shape0.numpy()[:stored]
        shape1 = self.contacts.rigid_contact_shape1.numpy()[:stored]
        if active > self._contact_high_water:
            self._contact_high_water = active
            categories = Counter({"self": 0, "interinstrument": 0, "environment": 0})
            pairs = Counter()
            for first, second in zip(shape0, shape1):
                owner0, path0 = self._shape_sources.get(int(first), (None, f"shape[{first}]"))
                owner1, path1 = self._shape_sources.get(int(second), (None, f"shape[{second}]"))
                category = (
                    "self" if owner0 is not None and owner0 == owner1 else
                    "interinstrument" if owner0 is not None and owner1 is not None else
                    "environment"
                )
                categories[category] += 1
                pairs[(category, *sorted((path0, path1)))] += 1
            top = "; ".join(
                f"{category} {path0} <-> {path1}: {total}"
                for (category, path0, path1), total in sorted(
                    pairs.items(), key=lambda item: (-item[1], item[0])
                )[:8]
            )
            print(
                f"Newton contact peak: count={active}, capacity={capacity}, sampled={stored}; "
                f"categories={dict(categories)}; top source-link pairs=[{top}]",
                flush=True,
            )
        if active > capacity:
            raise RuntimeError(
                f"Newton contact overflow: count={active} exceeds capacity={capacity} "
                f"(pipeline={self.contacts.rigid_contact_max}, solver nconmax={self.nconmax}); "
                f"pair summary covers only {stored} stored contacts; solver step refused"
            )
        self.contact_count = max(self.contact_count, active)
        self.active_shapes.update(shape0.tolist())
        self.active_shapes.update(shape1.tolist())

    @staticmethod
    def _world_scale(prim, UsdGeom, Usd):
        matrix = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        return np.linalg.norm(matrix[:3, :3], axis=1)

    def advance(self, elapsed, targets=None):
        from pxr import Gf, Usd, UsdGeom

        if not math.isfinite(elapsed) or elapsed <= 0.0:
            return
        count = max(1, math.ceil(elapsed / self.max_step))
        step = elapsed / count
        initial = self.state.joint_q.numpy()
        if targets is None:
            targets = {}
            for path in self.kinematic:
                matrix = UsdGeom.Xformable(self.stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                transform = Gf.Transform(matrix)
                quaternion = transform.GetRotation().GetQuat()
                targets[path] = [*transform.GetTranslation(), *quaternion.GetImaginary(), quaternion.GetReal()]
        targets = {path: np.asarray(pose, dtype=float) for path, pose in targets.items()}
        self.active_shapes = set()
        self.contact_count = 0
        for index in range(count):
            positions = self.state.joint_q.numpy()
            velocities = self.state.joint_qd.numpy()
            fraction = (index + 1) / count
            for path, (_, position_start, velocity_start) in self.kinematic.items():
                previous = initial[position_start:position_start + 7]
                target = targets[path]
                rotation = self.wp.quat_slerp(self.wp.quat(*previous[3:]), self.wp.quat(*target[3:]), fraction)
                positions[position_start:position_start + 3] = previous[:3] + fraction * (target[:3] - previous[:3])
                positions[position_start + 3:position_start + 7] = rotation
                velocities[velocity_start:velocity_start + 3] = (target[:3] - previous[:3]) / elapsed
                difference = self.wp.quat(*target[3:]) * self.wp.quat_inverse(self.wp.quat(*previous[3:]))
                axis, angle = self.wp.quat_to_axis_angle(difference)
                if angle > math.pi:
                    angle -= 2.0 * math.pi
                velocities[velocity_start + 3:velocity_start + 6] = np.asarray(axis) * angle / elapsed
            self.state.joint_q.assign(positions)
            self.state.joint_qd.assign(velocities)
            self.newton.eval_fk(self.model, self.state.joint_q, self.state.joint_qd, self.state)
            self.state.clear_forces()
            self.pipeline.collide(self.state, self.contacts)
            self._record_contacts()
            self.solver.step(self.state, self.next_state, self.control, self.contacts, step)
            self.state, self.next_state = self.next_state, self.state

    def render(self):
        from pxr import Gf

        poses = self.state.body_q.numpy()
        for body, translation, orientation in self.dynamic.values():
            pose = poses[body]
            translation.Set(Gf.Vec3d(*[float(value) for value in pose[:3]]))
            orientation.Set(Gf.Quatf(float(pose[6]), Gf.Vec3f(*[float(value) for value in pose[3:6]])))
        if not hasattr(self, "display"):
            self.display = CollisionDisplay(self.stage, self.shape_paths)
        self.display.update(self.active_shapes)


class NewtonContactProcess:
    """Run Newton outside Isaac's bundled Warp/USD Python environment."""

    def __init__(self, stage, roots, *, python, device="cpu"):
        import os
        from pathlib import Path
        import subprocess
        import tempfile
        from pxr import Usd, UsdGeom, UsdPhysics

        self.stage = stage
        self.directory = tempfile.TemporaryDirectory(prefix="dvrk-newton-")
        for root in roots:
            while True:
                instances = [prim for prim in Usd.PrimRange(stage.GetPrimAtPath(root)) if prim.IsInstance()]
                if not instances:
                    break
                for prim in instances:
                    prim.SetInstanceable(False)
        scene_path = str(Path(self.directory.name) / "contacts.usda")
        stage.Flatten().Export(scene_path)
        environment = dict(os.environ)
        environment.pop("LD_PRELOAD", None)
        environment.pop("LD_LIBRARY_PATH", None)
        environment["PYTHONPATH"] = str(Path(__file__).resolve().parents[1])
        self.process = subprocess.Popen(
            [str(python), "-u", "-m", "dvrk_isaac_sim.newton_contact_worker"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=environment,
            text=True,
        )
        self.dynamic = {}
        self.contact_count = 0
        self.active_shapes = set()
        self.poses = {}
        try:
            result = self._request({"scene": scene_path, "roots": roots, "device": device})
            self.kinematic = result["kinematic"]
            self.shape_paths = result["shape_paths"]
            for path in result["dynamic"]:
                prim = stage.GetPrimAtPath(path)
                scale = NewtonContactWorld._world_scale(prim, UsdGeom, Usd)
                xform = UsdGeom.Xformable(prim)
                xform.ClearXformOpOrder()
                self.dynamic[path] = (xform.AddTranslateOp(), xform.AddOrientOp())
                if not np.allclose(scale, 1.0):
                    from pxr import Gf

                    xform.AddScaleOp().Set(Gf.Vec3f(*[float(value) for value in scale]))
            for prim in stage.Traverse():
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr().Set(False)
            self.poses = result["poses"]
            self.display = CollisionDisplay(stage, self.shape_paths)
        except BaseException:
            self.close()
            raise

    def _request(self, message):
        import json
        import select

        self.process.stdin.write(json.dumps(message) + "\n")
        self.process.stdin.flush()
        if not select.select([self.process.stdout], [], [], 120.0)[0]:
            raise RuntimeError("Newton worker did not respond within 120 seconds")
        response = self.process.stdout.readline()
        if not response:
            raise RuntimeError("Newton worker exited; check its terminal error output")
        result = json.loads(response)
        if "error" in result:
            raise RuntimeError(f"Newton worker: {result['error']}")
        return result

    def advance(self, elapsed):
        from pxr import Gf, Usd, UsdGeom

        targets = {}
        for path in self.kinematic:
            matrix = UsdGeom.Xformable(self.stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            transform = Gf.Transform(matrix)
            quaternion = transform.GetRotation().GetQuat()
            targets[path] = [*transform.GetTranslation(), *quaternion.GetImaginary(), quaternion.GetReal()]
        result = self._request({"elapsed": elapsed, "targets": targets})
        self.poses = result["poses"]
        self.contact_count = result["contact_count"]
        self.active_shapes = set(result["active_shapes"])

    def render(self):
        from pxr import Gf

        for path, (translation, orientation) in self.dynamic.items():
            pose = self.poses[path]
            translation.Set(Gf.Vec3d(*pose[:3]))
            orientation.Set(Gf.Quatf(pose[6], Gf.Vec3f(*pose[3:6])))
        self.display.update(self.active_shapes)

    def close(self):
        import subprocess

        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.process.stdin.close()
        self.process.stdout.close()
        self.directory.cleanup()