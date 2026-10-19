import numpy as np
import pytest

from dvrk_isaac_sim.environment_geometry import cone_bowl_segments, spawn_cone_bowl


def test_cone_walls_are_closed_outward_facing_and_leave_rim_open():
    walls = cone_bowl_segments((0.20, 0.20, 0.12))
    assert len(walls) == 24
    for index, (vertices, faces) in enumerate(walls):
        points = np.asarray(vertices)
        np.testing.assert_allclose(points[0], [0, 0, 0])
        np.testing.assert_allclose(points[3], [0, 0, -0.003])
        np.testing.assert_allclose(points[2], walls[(index + 1) % len(walls)][0][1], atol=1e-15)
        center = points.mean(axis=0)
        edges = {}
        for face in faces:
            triangle = points[list(face)]
            normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
            assert np.dot(normal, triangle.mean(axis=0) - center) > 0
            for start, end in zip(face, (*face[1:], face[0])):
                edge = tuple(sorted((start, end)))
                edges[edge] = edges.get(edge, 0) + 1
            if np.allclose(triangle[:, 2], 0.12):
                assert np.all(np.linalg.norm(triangle[:, :2], axis=1) >= 0.099)
        assert set(edges.values()) == {2}


def test_cone_usd_has_separate_convex_wall_colliders():
    pytest.importorskip("pxr")
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.CreateInMemory()
    root = spawn_cone_bowl(stage, Sdf.Path("/Bowl"), (0.20, 0.20, 0.12), (0.36, 0.40, 0.42, 1.0))
    assert not root.GetPrim().HasAPI(UsdPhysics.CollisionAPI)
    walls = root.GetPrim().GetChildren()
    assert len(walls) == 24
    for wall in walls:
        assert wall.IsA(UsdGeom.Mesh)
        assert wall.HasAPI(UsdPhysics.CollisionAPI)
        assert UsdPhysics.MeshCollisionAPI(wall).GetApproximationAttr().Get() == "convexHull"


def test_newton_ball_falls_into_cone_and_stays_inside():
    pytest.importorskip("newton")
    pytest.importorskip("mujoco")
    pytest.importorskip("pxr")
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
    from dvrk_isaac_sim.newton_contacts import NewtonContactWorld

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    spawn_cone_bowl(stage, Sdf.Path("/World/Environment/Bowl"),
                    (0.20, 0.20, 0.12), (0.36, 0.40, 0.42, 1.0))
    ball = UsdGeom.Sphere.Define(stage, "/World/Environment/Ball")
    ball.CreateRadiusAttr(0.01)
    ball.AddTranslateOp().Set((0.03, 0.0, 0.08))
    UsdPhysics.CollisionAPI.Apply(ball.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(ball.GetPrim())
    UsdPhysics.MassAPI.Apply(ball.GetPrim()).CreateMassAttr(0.0042)
    world = NewtonContactWorld(stage, ["/World/Environment"], device="cpu")
    assert len(world.shape_paths) == 25
    body = world.body_paths["/World/Environment/Ball"]
    for step in range(240):
        world.advance(1 / 120)
        center = world.state.body_q.numpy()[body, :3]
        assert center[2] > 0.0, f"Ball escaped through cone at step {step}"
    assert center[2] < 0.05
    radius = 0.10 * np.cos(np.pi / 24)
    slope = radius / 0.12
    assert slope * center[2] - np.linalg.norm(center[:2]) > 0.01 * np.sqrt(1 + slope**2) - 0.003