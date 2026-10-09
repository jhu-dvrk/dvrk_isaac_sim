from pathlib import Path
import pytest

from dvrk_isaac_sim.camera import CameraOptions
from dvrk_simulator_base.scene import SceneCamera


def test_camera_options_defaults():
    options = CameraOptions()
    assert options.enabled is True
    assert options.mode == "stereo"
    assert options.width == 1280
    assert options.height == 720
    assert options.rate_hz == 30.0
    assert options.transport_width == 2560


def test_camera_options_validation():
    with pytest.raises(ValueError, match="camera mode must be"):
        CameraOptions(mode="invalid")
    with pytest.raises(ValueError, match="camera width and height must be positive"):
        CameraOptions(width=-10)
    with pytest.raises(ValueError, match="camera rate must be finite"):
        CameraOptions(rate_hz=0.0)


def test_camera_options_from_scene():
    scene_camera = SceneCamera(
        mode="stereo",
        settings={
            "width": 1920,
            "height": 1080,
            "publish_rate_hz": 60.0,
            "transports": ["rtsp"],
            "rtsp": {"port": 8554, "mount_path": "/ECM", "encoding": "h264"},
        },
    )
    options = CameraOptions.from_scene(scene_camera)
    assert options.enabled is True
    assert options.mode == "stereo"
    assert options.width == 1920
    assert options.height == 1080
    assert options.rate_hz == 60.0
    assert options.transports == ("rtsp",)
    assert options.rtsp_port == 8554
    assert options.rtsp_mount_path == "/ECM"
    assert options.rtsp_encoding == "h264"


def test_isaac_camera_rejects_unixfd_transport():
    with pytest.raises(ValueError, match="only the rtsp"):
        CameraOptions(transports=("unixfd",))


def test_stereo_eye_positions_use_optical_left_axis():
    import numpy as np
    from dvrk_isaac_sim.camera import IsaacCamera
    from dvrk_simulator_base.types import Pose

    class Camera:
        def set_world_pose(self, **kwargs):
            self.pose = kwargs

    camera = IsaacCamera.__new__(IsaacCamera)
    camera.options = CameraOptions(baseline_m=0.006)
    camera._cameras = [Camera(), Camera()]
    # Optical +X points down; +Y points left along world +X.
    rotation = np.array([[0., 1., 0.], [0., 0., -1.], [-1., 0., 0.]])
    pose = Pose(np.array([0.1, 0.2, 0.3]), rotation)
    camera.set_pose(pose)
    left, right = [eye.pose for eye in camera._cameras]
    np.testing.assert_allclose(left['position'], pose.position + rotation[:, 1] * 0.003)
    np.testing.assert_allclose(right['position'], pose.position - rotation[:, 1] * 0.003)
    assert left['camera_axes'] == right['camera_axes'] == 'world'
    np.testing.assert_allclose(left['orientation'], right['orientation'])
    assert camera._pose_ready
