from dvrk_isaac_sim.usd_collision import _apply_collision_api, _collision_approximation, _collision_targets


class _FakePrim:
    def __init__(self, name, parent=None):
        self._name = name
        self._parent = parent

    def GetName(self):
        return self._name

    def GetParent(self):
        return self._parent or _InvalidPrim()

    def IsValid(self):
        return True


class _InvalidPrim:
    def IsValid(self):
        return False


def test_collision_approximation_uses_psm_link_tiers_for_descendant_geometry():
    tiers = {
        "PSM1_insertion_link": "convexDecomposition",
        "PSM1_roll_link": "convexDecomposition",
        "PSM1_wrist_yaw_link": "convexDecomposition",
        "PSM1_wrist_pitch_link": "sdf",
        "PSM1_jaw_1_link": "sdf",
        "PSM2_jaw_2_link": "sdf",
    }

    for link_name, approximation in tiers.items():
        assert _collision_approximation(_FakePrim("mesh", _FakePrim(link_name))) == approximation


def test_collision_approximation_ignores_non_tier_links():
    assert _collision_approximation(_FakePrim("PSM1_pitch_link")) is None


class _FakeApproximationAttr:
    def __init__(self):
        self.value = None

    def Set(self, value):
        self.value = value


class _FakeMeshCollision:
    def __init__(self):
        self.approximation = _FakeApproximationAttr()

    def CreateApproximationAttr(self):
        return self.approximation


class _FakeCollisionAPI:
    @staticmethod
    def Apply(prim):
        prim.apis.add(_FakeCollisionAPI)


class _FakeMeshCollisionAPI:
    @staticmethod
    def Apply(prim):
        return prim.mesh_collision


class _FakeUsdPhysics:
    CollisionAPI = _FakeCollisionAPI
    MeshCollisionAPI = _FakeMeshCollisionAPI


class _FakeCollisionPrim(_FakePrim):
    def __init__(self, name, parent=None):
        super().__init__(name, parent)
        self.apis = set()
        self.mesh_collision = _FakeMeshCollision()
        self.display_color = _FakeApproximationAttr()
        self.display_opacity = _FakeApproximationAttr()

    def HasAPI(self, api):
        return api in self.apis


def test_apply_collision_api_authors_mesh_tier():
    prim = _FakeCollisionPrim("mesh", _FakePrim("PSM1_wrist_pitch_link"))

    assert _apply_collision_api(prim, _FakeUsdPhysics)
    assert _FakeCollisionAPI in prim.apis
    assert prim.mesh_collision.approximation.value == "sdf"


class _FakeUsd:
    @staticmethod
    def PrimRange(prim):
        return prim.children


class _FakeUsdGeom:
    class Gprim:
        def __init__(self, prim):
            self.prim = prim

        def CreateDisplayColorAttr(self):
            return self.prim.display_color

        def CreateDisplayOpacityAttr(self):
            return self.prim.display_opacity

    class Mesh:
        pass


def test_apply_collision_api_authors_red_debug_color():
    prim = _FakeCollisionPrim("mesh", _FakePrim("PSM1_wrist_pitch_link"))
    prim.IsA = lambda schema: schema is _FakeUsdGeom.Mesh

    assert _apply_collision_api(prim, _FakeUsdPhysics, _FakeUsdGeom)
    assert prim.display_color.value == [(1.0, 0.0, 0.0)]
    assert prim.display_opacity.value == [1.0]


class _FakeGeometryChild:
    def __init__(self, is_geometry):
        self.is_geometry = is_geometry

    def IsA(self, schema):
        return schema is _FakeUsdGeom.Gprim and self.is_geometry


class _FakeCollisionCandidate:
    def __init__(self, children):
        self.children = children


def test_collision_targets_returns_every_geometry_child():
    outer_grip = _FakeGeometryChild(is_geometry=True)
    gear_region = _FakeGeometryChild(is_geometry=True)
    candidate = _FakeCollisionCandidate([outer_grip, gear_region, _FakeGeometryChild(is_geometry=False)])

    assert _collision_targets(candidate, _FakeUsd, _FakeUsdGeom) == [outer_grip, gear_region]
