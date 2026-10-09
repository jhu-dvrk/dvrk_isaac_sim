"""Isaac Sim camera options and video streaming for the virtual ECM."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any

import numpy as np

from dvrk_simulator_base.rotations import rotation_to_quaternion_wxyz
from dvrk_simulator_base.scene import SceneCamera
from dvrk_simulator_base.types import Pose


def _quaternion_wxyz(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a rotation matrix to Isaac's scalar-first quaternion."""
    return rotation_to_quaternion_wxyz(rotation)


@dataclass(frozen=True)
class CameraOptions:
    enabled: bool = True
    mode: str = "stereo"
    width: int = 1280
    height: int = 720
    rate_hz: float = 30.0
    horizontal_fov_degrees: float = 60.0
    near_m: float = 0.005
    far_m: float = 10.0
    baseline_m: float = 0.006
    transports: tuple[str, ...] = ("rtsp",)
    rtsp_port: int = 8554
    rtsp_mount_path: str = "/ECM"
    rtsp_encoding: str = "h264"

    def __post_init__(self) -> None:
        mode = str(self.mode).lower()
        if mode not in {"mono", "stereo"}:
            raise ValueError("camera mode must be 'mono' or 'stereo'")
        if any(transport != "rtsp" for transport in self.transports):
            raise ValueError("Isaac Sim supports only the rtsp camera transport")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera width and height must be positive")
        if not np.isfinite(self.rate_hz) or self.rate_hz <= 0.0:
            raise ValueError("camera rate must be finite and positive")
        if not 0.0 < self.horizontal_fov_degrees < 180.0:
            raise ValueError("camera horizontal FOV must be between 0 and 180 degrees")
        if self.near_m <= 0.0 or self.far_m <= self.near_m:
            raise ValueError("camera clipping planes must satisfy 0 < near < far")
        if not np.isfinite(self.baseline_m) or self.baseline_m <= 0.0:
            raise ValueError("camera baseline must be finite and positive")
        if not 1 <= self.rtsp_port <= 65535:
            raise ValueError("camera.rtsp.port must be between 1 and 65535")
        if not self.rtsp_mount_path.startswith("/"):
            raise ValueError("camera.rtsp.mount_path must start with '/'")
        if str(self.rtsp_encoding).lower() not in {"h264", "raw"}:
            raise ValueError("camera.rtsp.encoding must be h264 or raw")
        object.__setattr__(self, "mode", mode)
        object.__setattr__(self, "rtsp_encoding", str(self.rtsp_encoding).lower())

    @property
    def transport_width(self) -> int:
        return self.width * (2 if self.mode == "stereo" else 1)

    @classmethod
    def from_scene(cls, camera: SceneCamera | dict[str, Any] | None) -> CameraOptions:
        if camera is None:
            return cls(enabled=False)
        if isinstance(camera, dict):
            settings = dict(camera)
            mode = str(settings.get("mode", "mono")).lower()
        else:
            settings = camera.as_dict()
            mode = camera.mode
        if mode not in {"off", "mono", "stereo"}:
            raise ValueError("Isaac Sim supports off, mono, or stereo scene cameras")
        transports = tuple(settings.get("transports", ["rtsp"]))
        rtsp = settings.get("rtsp", {}) or {}
        return cls(
            enabled=mode != "off" and bool(transports),
            mode="mono" if mode == "off" else mode,
            width=int(settings.get("width", 1280)),
            height=int(settings.get("height", 720)),
            rate_hz=float(settings.get("publish_rate_hz", 30.0)),
            horizontal_fov_degrees=float(settings.get("horizontal_fov_deg", 60.0)),
            near_m=float(settings.get("near_clip_m", 0.005)),
            far_m=float(settings.get("far_clip_m", 10.0)),
            baseline_m=float(settings.get("baseline_m", 0.006)),
            transports=transports,
            rtsp_port=int(rtsp.get("port", 8554)),
            rtsp_mount_path=str(rtsp.get("mount_path", "/ECM")),
            rtsp_encoding=str(rtsp.get("encoding", "h264")),
        )


class IsaacCamera:
    """Manage Isaac camera prims and native RTSP streaming."""

    def __init__(self, options: CameraOptions) -> None:
        self.options = options
        self._cameras = []
        self._tiled_sensor = None
        self._rtsp_graphs = []
        self._pose_ready = False
        self._frame_count = 0
        self._last_metrics_time = time.monotonic()

        if not options.enabled:
            return

        from isaacsim.sensors.camera import Camera

        camera_prim_paths = []
        names = ["mono"] if options.mode == "mono" else ["left", "right"]
        fov_rad = math.radians(options.horizontal_fov_degrees)
        for index, name in enumerate(names):
            camera_prim_path = f"/World/ECM/Camera{name.title() if options.mode == 'stereo' else ''}"
            camera = Camera(
                prim_path=camera_prim_path,
                name=f"ECM_camera_{name}",
                frequency=options.rate_hz,
                resolution=(options.width, options.height),
                orientation=np.asarray([1.0, 0.0, 0.0, 0.0]),
            )
            camera_prim_paths.append(camera_prim_path)
            aperture = camera.get_horizontal_aperture()
            camera.set_focal_length(aperture / (2.0 * math.tan(fov_rad / 2.0)))
            camera.set_clipping_range(options.near_m, options.far_m)
            self._cameras.append(camera)

        if options.mode == "stereo":
            from isaacsim.sensors.experimental.rtx import TiledCameraSensor

            self._tiled_sensor = TiledCameraSensor(
                camera_prim_paths,
                resolution=(options.height, options.width),
                annotators=[],
            )
            tiled_render_product = str(self._tiled_sensor.render_product.GetPath())
            if "rtsp" in options.transports:
                self._rtsp_graphs.append(
                    self._create_rtsp_graph(tiled_render_product, "stereo", 0, existing_render_product=True)
                )
        elif options.mode == "mono":
            if "rtsp" in options.transports:
                self._rtsp_graphs.append(
                    self._create_rtsp_graph(camera_prim_paths[0], "mono", 0, existing_render_product=False)
                )

    def _create_rtsp_graph(self, camera_prim: str, name: str, index: int, existing_render_product: bool = False):
        import omni.graph.core as og

        port = self.options.rtsp_port + index
        mount_path = self.options.rtsp_mount_path.rstrip("/")
        graph_path = f"/World/CRTKROS/ECM_{name}_RTSP"
        keys = og.Controller.Keys
        create_nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("RTSPPublish", "isaacsim.streaming.rtsp.RTSPCameraHelper"),
        ]
        connect = [("OnPlaybackTick.outputs:tick", "RTSPPublish.inputs:execIn")]
        values = [("RTSPPublish.inputs:renderProductPath", camera_prim)]
        if not existing_render_product:
            create_nodes.insert(1, ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"))
            connect = [
                ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "RTSPPublish.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "RTSPPublish.inputs:renderProductPath"),
            ]
            values.insert(0, ("RenderProduct.inputs:cameraPrim", camera_prim))
        graph, _, _, _ = og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution",
            },
            {
                keys.CREATE_NODES: create_nodes,
                keys.CONNECT: connect,
                keys.SET_VALUES: [
                    *values,
                    *([] if existing_render_product else [
                        ("RenderProduct.inputs:width", self.options.width),
                        ("RenderProduct.inputs:height", self.options.height),
                    ]),
                    ("RTSPPublish.inputs:port", port),
                    ("RTSPPublish.inputs:mountPath", mount_path),
                    ("RTSPPublish.inputs:useRawEncoding", self.options.rtsp_encoding == "raw"),
                ],
            },
        )
        return graph

    def set_pose(self, pose: Pose) -> None:
        orientation = _quaternion_wxyz(pose.orientation)
        for index, camera in enumerate(self._cameras):
            position = pose.position.copy()
            if self.options.mode == "stereo":
                position += pose.orientation[:, 1] * (self.options.baseline_m / 2.0) * (1.0 if index == 0 else -1.0)
            camera.set_world_pose(position=position, orientation=orientation, camera_axes="world")
        self._pose_ready = True

    def start_streaming(self) -> None:
        """Attach RTSP writers once before the first render, including their AOVs."""
        import omni.graph.core as og
        for graph in self._rtsp_graphs:
            og.Controller.evaluate_sync(graph)

    def capture(self, simulation_time: float) -> None:
        """Record a rendered camera frame; the native RTSP writer sends it."""
        if self.options.enabled and self._pose_ready:
            self._frame_count += 1

    def take_camera_rate_hz(self) -> float:
        now = time.monotonic()
        elapsed = max(now - self._last_metrics_time, 1e-6)
        rate = self._frame_count / elapsed
        self._frame_count = 0
        self._last_metrics_time = now
        return rate

    def close(self) -> None:
        self._pose_ready = False
        self._rtsp_graphs.clear()
