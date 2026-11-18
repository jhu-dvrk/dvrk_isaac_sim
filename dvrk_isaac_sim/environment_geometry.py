"""Convex wall segments for an open, downward-pointing conical bowl."""

import math


def cone_bowl_segments(size, segments=24):
    radius_x, radius_y, height = float(size[0]) / 2, float(size[1]) / 2, float(size[2])
    thickness = 0.003
    faces = [(0, 1, 2), (3, 5, 4), (0, 3, 4), (0, 4, 1),
             (1, 4, 5), (1, 5, 2), (2, 5, 3), (2, 3, 0)]
    walls = []
    for index in range(segments):
        angles = [2 * math.pi * index / segments, 2 * math.pi * (index + 1) / segments]
        inner = [(radius_x * math.cos(angle), radius_y * math.sin(angle), height)
                 for angle in angles]
        outer = [((radius_x + thickness) * math.cos(angle),
                  (radius_y + thickness) * math.sin(angle), height) for angle in angles]
        walls.append(([(0.0, 0.0, 0.0), *inner, (0.0, 0.0, -thickness), *outer], faces))
    return walls


def spawn_cone_bowl(stage, prim_path, size, color):
    from pxr import Gf, UsdGeom, UsdPhysics

    root = UsdGeom.Xform.Define(stage, prim_path)
    for index, (points, faces) in enumerate(cone_bowl_segments(size)):
        mesh = UsdGeom.Mesh.Define(stage, prim_path.AppendChild(f"wall_{index:02d}"))
        mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
        mesh.CreateFaceVertexCountsAttr([3] * len(faces))
        mesh.CreateFaceVertexIndicesAttr([vertex for face in faces for vertex in face])
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*color[:3])])
        mesh.CreateDisplayOpacityAttr([float(color[3])])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    return root