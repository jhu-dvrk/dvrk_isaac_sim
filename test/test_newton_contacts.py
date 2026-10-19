import numpy as np
import pytest

pytest.importorskip("newton")
pytest.importorskip("mujoco")
pytest.importorskip("pxr")

from pxr import Usd, UsdGeom, UsdPhysics

from dvrk_isaac_sim.newton_contacts import NewtonContactWorld


@pytest.mark.parametrize("nconmax", [0, -1, True, 1.5, "8192"])
def test_contact_capacity_requires_positive_integer(nconmax):
    with pytest.raises(ValueError, match="nconmax must be a positive integer"):
        NewtonContactWorld(None, [], nconmax=nconmax)


@pytest.mark.parametrize("nconmax", [None, 32, 1])
def test_pipeline_filters_self_contacts_but_preserves_interinstrument_contacts(nconmax, capsys, monkeypatch):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    roots = []
    for instrument, links in (("PSM1", ("shaft", "jaw")), ("PSM2", ("shaft",))):
        root = f"/World/{instrument}/PhysicsLinks"
        UsdGeom.Xform.Define(stage, root)
        roots.append(root)
        for link in links:
            body = UsdGeom.Xform.Define(stage, f"{root}/{link}")
            UsdPhysics.RigidBodyAPI.Apply(body.GetPrim()).CreateKinematicEnabledAttr().Set(True)
            shape = UsdGeom.Sphere.Define(stage, f"{root}/{link}/Collision")
            shape.CreateRadiusAttr().Set(0.05)
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    kwargs = {} if nconmax is None else {"nconmax": nconmax}
    world = NewtonContactWorld(stage, roots, device="cpu", **kwargs)
    capacity = 8192 if nconmax is None else nconmax
    assert world.nconmax == capacity
    assert world.pipeline.rigid_contact_max == capacity
    assert world.contacts.rigid_contact_max == capacity
    assert world.solver.mjw_data.naconmax == capacity
    assert world.solver.mjw_data.njmax >= 4 * capacity
    assert np.max(world.solver.mj_model.geom_condim) == 3
    assert int(world.solver.mj_model.opt.cone) == 0
    pairs = {tuple(sorted(map(int, pair))) for pair in world.pipeline.shape_pairs_filtered.numpy()}
    own_shapes = [shape for path, shape in world.shape_paths.items() if "/PSM1/" in path]
    assert tuple(sorted(own_shapes)) not in pairs
    assert len(pairs) == 2
    if capacity == 1:
        monkeypatch.setattr(world.solver, "step", lambda *args: pytest.fail("overflow reached solver"))
        with pytest.raises(RuntimeError, match="count=2 exceeds capacity=1"):
            world.advance(world.max_step)
        assert "count=2, capacity=1, sampled=1" in capsys.readouterr().out
        return
    world.advance(world.max_step)
    assert world.contact_count == 2
    assert world.active_shapes == set(world.shape_paths.values())
    output = capsys.readouterr().out
    assert "'self': 0, 'interinstrument': 2, 'environment': 0" in output
    assert "/World/PSM1/PhysicsLinks/jaw <-> /World/PSM2/PhysicsLinks/shaft: 1" in output


def _diagnostic_world(shape_pairs, *, capacity=32):
    from types import SimpleNamespace

    class Array:
        def __init__(self, values):
            self.values = np.asarray(values, dtype=np.int32)

        def numpy(self):
            return self.values

    world = NewtonContactWorld.__new__(NewtonContactWorld)
    world.nconmax = capacity
    world._contact_high_water = 0
    world.contact_count = 0
    world.active_shapes = set()
    world._shape_sources = {
        0: ("/World/PSM1", "/World/PSM1/PhysicsLinks/jaw"),
        1: ("/World/PSM1", "/World/PSM1/PhysicsLinks/shaft"),
        2: ("/World/PSM2", "/World/PSM2/PhysicsLinks/jaw"),
        3: (None, "/World/Environment/Table"),
        4: ("/World/PSM1", "/World/PSM1/PhysicsLinks/jaw"),
    }
    world.contacts = SimpleNamespace(
        rigid_contact_max=capacity,
        rigid_contact_count=Array([len(shape_pairs)]),
        rigid_contact_shape0=Array([pair[0] for pair in shape_pairs]),
        rigid_contact_shape1=Array([pair[1] for pair in shape_pairs]),
    )
    return world


def test_contact_diagnostics_aggregate_sources_and_only_report_new_peaks(capsys):
    world = _diagnostic_world([(0, 1), (0, 2), (2, 4), (0, 3)])
    world.contacts.rigid_contact_count.values[0] = 0
    world._record_contacts()
    assert capsys.readouterr().out == ""
    world.contacts.rigid_contact_count.values[0] = 4
    world._record_contacts()
    output = capsys.readouterr().out
    assert "categories={'self': 1, 'interinstrument': 2, 'environment': 1}" in output
    assert "/World/PSM1/PhysicsLinks/jaw <-> /World/PSM2/PhysicsLinks/jaw: 2" in output
    assert "/World/Environment/Table" in output
    assert world.active_shapes == {0, 1, 2, 3, 4}
    world._record_contacts()
    world.contacts.rigid_contact_count.values[0] = 2
    world._record_contacts()
    assert capsys.readouterr().out == ""
    world.contacts.rigid_contact_count.values[0] = 5
    world.contacts.rigid_contact_shape0.values = np.append(world.contacts.rigid_contact_shape0.values, 0)
    world.contacts.rigid_contact_shape1.values = np.append(world.contacts.rigid_contact_shape1.values, 3)
    world._record_contacts()
    assert "count=5" in capsys.readouterr().out
    assert world.contact_count == 5


def test_contact_diagnostics_bound_top_pairs(capsys):
    world = _diagnostic_world([(index, index + 1) for index in range(20)])
    world._record_contacts()
    assert capsys.readouterr().out.count(" <-> ") == 8


def test_contact_capacity_accepts_full_buffer_and_rejects_overflow(capsys):
    world = _diagnostic_world([(0, 2), (0, 3)], capacity=2)
    world._record_contacts()
    assert world.contact_count == 2
    world.contacts.rigid_contact_count.values[0] = 3
    with pytest.raises(RuntimeError, match="count=3 exceeds capacity=2.*solver step refused"):
        world._record_contacts()
    output = capsys.readouterr().out
    assert "count=3, capacity=2, sampled=2" in output
    assert world.contact_count == 2


def test_newton_usd_prop_rests_on_table():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/Environment")
    table = UsdGeom.Cube.Define(stage, "/World/Environment/Table")
    table.CreateSizeAttr().Set(1.0)
    UsdGeom.Xformable(table).AddScaleOp().Set((1.0, 1.0, 0.1))
    UsdPhysics.CollisionAPI.Apply(table.GetPrim())
    sphere = UsdGeom.Sphere.Define(stage, "/World/Environment/Ball")
    sphere.CreateRadiusAttr().Set(0.05)
    UsdGeom.Xformable(sphere).AddTranslateOp().Set((0.0, 0.0, 0.3))
    UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())
    world = NewtonContactWorld(stage, ["/World/Environment"], device="cpu")
    for _ in range(120):
        world.advance(1.0 / 120.0)
    world.render()
    pose = world.state.body_q.numpy()[world.body_paths[str(sphere.GetPath())]]
    assert pose[2] == pytest.approx(0.1, abs=0.005)
    assert world.contact_count > 0
    assert not UsdPhysics.CollisionAPI(sphere).GetCollisionEnabledAttr().Get()
    assert not UsdPhysics.RigidBodyAPI(sphere).GetRigidBodyEnabledAttr().Get()
    np.testing.assert_allclose(UsdGeom.Xformable(table).GetOrderedXformOps()[0].Get(), (1.0, 1.0, 0.1))


def test_newton_prescribed_box_pose_and_contact_highlight():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/PSM1")
    UsdGeom.Xform.Define(stage, "/World/PSM1/PhysicsLinks")
    body = UsdGeom.Xform.Define(stage, "/World/PSM1/PhysicsLinks/shaft")
    translation = body.AddTranslateOp()
    translation.Set((0.0, 0.0, 0.3))
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim()).CreateKinematicEnabledAttr().Set(True)
    cube = UsdGeom.Cube.Define(stage, str(body.GetPath()) + "/Collision")
    cube.CreateSizeAttr().Set(0.1)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdGeom.Xform.Define(stage, "/World/Environment")
    ball = UsdGeom.Sphere.Define(stage, "/World/Environment/Ball")
    ball.CreateRadiusAttr().Set(0.05)
    UsdGeom.Xformable(ball).AddTranslateOp().Set((0.08, 0.0, 0.2))
    UsdPhysics.CollisionAPI.Apply(ball.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(ball.GetPrim())
    world = NewtonContactWorld(stage, ["/World/PSM1/PhysicsLinks", "/World/Environment"], device="cpu")
    translation.Set((0.0, 0.0, 0.2))
    world.advance(1.0 / 120.0)
    world.render()
    body_pose = world.state.body_q.numpy()[world.body_paths[str(body.GetPath())]]
    np.testing.assert_allclose(body_pose[:3], (0.0, 0.0, 0.2), atol=1e-6)
    assert world.contact_count > 0
    np.testing.assert_allclose(cube.GetDisplayColorAttr().Get()[0], (0.1, 1.0, 0.1), atol=1e-7)


def test_isolated_worker_updates_scaled_dynamic_prop():
    import sys
    from dvrk_isaac_sim.newton_contacts import NewtonContactProcess

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/Environment")
    cube = UsdGeom.Cube.Define(stage, "/World/Environment/Cube")
    cube.CreateSizeAttr().Set(1.0)
    xform = UsdGeom.Xformable(cube)
    xform.AddTranslateOp().Set((0.0, 0.0, 0.3))
    xform.AddScaleOp().Set((0.03, 0.04, 0.05))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    worker = NewtonContactProcess(stage, ["/World/Environment"], python=sys.executable)
    try:
        worker.advance(0.05)
        worker.render()
        assert worker.poses[str(cube.GetPath())][2] < 0.3
        np.testing.assert_allclose(xform.GetOrderedXformOps()[-1].Get(), (0.03, 0.04, 0.05), atol=1e-7)
    finally:
        worker.close()
    assert worker.process.poll() is not None


def test_worker_can_highlight_instanced_collision_copy():
    import sys
    from dvrk_isaac_sim.newton_contacts import NewtonContactProcess

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    source = UsdGeom.Xform.Define(stage, "/Source")
    shape = UsdGeom.Sphere.Define(stage, "/Source/Sphere")
    shape.CreateRadiusAttr().Set(0.05)
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/PSM1")
    UsdGeom.Xform.Define(stage, "/World/PSM1/PhysicsLinks")
    body = UsdGeom.Xform.Define(stage, "/World/PSM1/PhysicsLinks/jaw")
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim()).CreateKinematicEnabledAttr().Set(True)
    copy = stage.DefinePrim(str(body.GetPath()) + "/Collision", "Xform")
    copy.GetReferences().AddInternalReference(str(source.GetPath()))
    copy.SetInstanceable(True)
    assert stage.GetPrimAtPath(str(copy.GetPath()) + "/Sphere").IsInstanceProxy()
    worker = NewtonContactProcess(stage, ["/World/PSM1/PhysicsLinks"], python=sys.executable)
    try:
        worker.render()
        child = stage.GetPrimAtPath(str(copy.GetPath()) + "/Sphere")
        assert not child.IsInstanceProxy()
        assert UsdGeom.Imageable(child).ComputeVisibility() == UsdGeom.Tokens.invisible
    finally:
        worker.close()


def test_collision_display_overrides_hidden_guide_and_source_material():
    from pxr import UsdShade
    from dvrk_isaac_sim.newton_contacts import CollisionDisplay

    stage = Usd.Stage.CreateInMemory()
    visuals = UsdGeom.Xform.Define(stage, "/World/PSM1/Geometry")
    root = UsdGeom.Xform.Define(stage, "/World/PSM1/PhysicsLinks")
    root.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
    shape = UsdGeom.Cube.Define(stage, str(root.GetPath()) + "/Jaw/Collision")
    shape.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
    shape.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
    source = UsdShade.Material.Define(stage, "/SourceMaterial")
    UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(source)
    display = CollisionDisplay(stage, {str(shape.GetPath()): 3})
    assert shape.ComputePurpose() == UsdGeom.Tokens.default_
    assert shape.ComputeVisibility() == UsdGeom.Tokens.invisible
    material, _ = UsdShade.MaterialBindingAPI(shape).ComputeBoundMaterial()
    assert str(material.GetPath()) == "/SourceMaterial"
    display.update({3})
    assert shape.ComputeVisibility() == UsdGeom.Tokens.inherited
    material, _ = UsdShade.MaterialBindingAPI(shape).ComputeBoundMaterial()
    assert str(material.GetPath()).endswith("/Contact")
    display.set_visible(False)
    assert shape.ComputeVisibility() == UsdGeom.Tokens.invisible
    display.set_visible(True)
    assert shape.ComputeVisibility() == UsdGeom.Tokens.inherited
    display.update(set())
    assert shape.ComputeVisibility() == UsdGeom.Tokens.invisible
    display.set_visuals_visible(False)
    assert visuals.ComputeVisibility() == UsdGeom.Tokens.invisible
    assert shape.ComputeVisibility() == UsdGeom.Tokens.invisible
    display.set_visuals_visible(True)
    assert visuals.ComputeVisibility() == UsdGeom.Tokens.inherited


@pytest.mark.parametrize("kind", ["box", "capsule", "cylinder", "sphere"])
def test_newton_preserves_usd_primitive_dimensions_and_offset(kind):
    from pxr import Gf
    import newton

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/Environment")
    path = "/World/Environment/Primitive"
    if kind == "box":
        shape = UsdGeom.Cube.Define(stage, path)
        shape.CreateSizeAttr().Set(1.0)
        expected = (0.01, 0.02, 0.03)
        geometry_type = newton.GeoType.BOX
    elif kind in {"capsule", "cylinder"}:
        if kind == "capsule":
            shape = UsdGeom.Capsule.Define(stage, path)
            geometry_type = newton.GeoType.CAPSULE
        else:
            shape = UsdGeom.Cylinder.Define(stage, path)
            geometry_type = newton.GeoType.CYLINDER
        shape.CreateAxisAttr().Set(UsdGeom.Tokens.z)
        shape.CreateRadiusAttr().Set(0.01)
        shape.CreateHeightAttr().Set(0.06)
        expected = (0.01, 0.03, 0.0)
    else:
        shape = UsdGeom.Sphere.Define(stage, path)
        shape.CreateRadiusAttr().Set(0.01)
        expected = (0.01, 0.0, 0.0)
        geometry_type = newton.GeoType.SPHERE
    xform = UsdGeom.Xformable(shape)
    xform.AddTranslateOp().Set(Gf.Vec3d(0.1, 0.2, 0.3))
    if kind == "box":
        xform.AddScaleOp().Set(Gf.Vec3f(0.02, 0.04, 0.06))
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    builder = newton.ModelBuilder()
    result = builder.add_usd(stage, root_path="/World/Environment", load_visual_shapes=False)
    model = builder.finalize("cpu")
    index = result["path_shape_map"][path]
    assert model.shape_type.numpy()[index] == geometry_type
    np.testing.assert_allclose(model.shape_scale.numpy()[index], expected, atol=1e-7)
    np.testing.assert_allclose(model.shape_transform.numpy()[index][:3], (0.1, 0.2, 0.3), atol=1e-7)