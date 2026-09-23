"""Isaac Sim camera sensors and ROS 2 image publishers for the virtual ECM."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from dvrk_arm_description import RobotConfig
from dvrk_simulator_base.types import Pose


from dvrk_simulator_base.rotations import rotation_to_quaternion_wxyz


def _quaternion_wxyz(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a rotation matrix to Isaac's scalar-first quaternion."""
    return rotation_to_quaternion_wxyz(rotation)


class IsaacCameraPublisher:
    """Create an Isaac camera and publish its rendered frames as ROS images.

    ``mono`` publishes ``/<ECM>/image_raw`` and ``/<ECM>/camera_info``.
    ``stereo`` publishes per-eye camera information plus a synchronized
    side-by-side image on ``image_raw``.
    Scene camera ``transports`` independently select ``ros_raw``,
    ``ros_compressed`` (standard JPEG image_transport), and ``rtsp``.
    ROS image encoding is only performed when the corresponding topic has a
    subscriber; RTSP uses Isaac Sim's built-in RTSPCameraHelper.
    The camera pose follows the ECM measured optical pose and is independent of
    any ECM mesh, so the ECM can remain a kinematic-only component.
    """

    def __init__(self, node: Any, config: RobotConfig, mode: str,
                 scene_camera: dict[str, Any] | None = None) -> None:
        if mode not in {"mono", "stereo"}:
            raise ValueError(f"unsupported camera mode: {mode}")
        from isaacsim.sensors.camera import Camera
        from sensor_msgs.msg import CameraInfo, CompressedImage, Image

        camera_config = dict(config.raw.get("robot", {}).get("camera", {}))
        camera_config.update(scene_camera or {})
        camera_config.pop("mode", None)
        camera_config.pop("owner", None)
        self.node = node
        self.mode = mode
        self.frame_id = str(camera_config.get("frame", config.tool_frame))
        self.width = int(camera_config.get("width", 1280))
        self.height = int(camera_config.get("height", 720))
        self.encoding = str(camera_config.get("encoding", "rgba8"))
        self.baseline = float(camera_config.get("baseline_m", 0.006))
        self.fov = math.radians(float(camera_config.get("horizontal_fov_deg", 60.0)))
        self.near = float(camera_config.get("near_clip_m", 0.005))
        self.far = float(camera_config.get("far_clip_m", 10.0))
        self.publish_rate = float(camera_config.get("publish_rate_hz", 30.0))
        if "transport" in camera_config:
            raise ValueError("camera.transport was replaced by camera.transports")
        self.transports = camera_config.get("transports", ["ros_raw"])
        if (not isinstance(self.transports, list)
                or not all(isinstance(item, str) for item in self.transports)):
            raise ValueError("camera.transports must be a list of strings")
        if len(set(self.transports)) != len(self.transports):
            raise ValueError("camera.transports must not contain duplicates")
        unsupported_transports = set(self.transports) - {"ros_raw", "ros_compressed", "rtsp"}
        if unsupported_transports:
            raise ValueError("unsupported camera transport(s): " +
                             ", ".join(sorted(unsupported_transports)))
        compressed_config = camera_config.get("ros_compressed", {}) or {}
        if not isinstance(compressed_config, dict):
            raise ValueError("camera.ros_compressed must be a mapping")
        self.jpeg_quality = int(compressed_config.get("quality", 85))
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("camera.ros_compressed.quality must be between 1 and 100")
        rtsp_config = camera_config.get("rtsp", {}) or {}
        if not isinstance(rtsp_config, dict):
            raise ValueError("camera.rtsp must be a mapping")
        self.rtsp_port = int(rtsp_config.get("port", 8554))
        self.rtsp_mount_path = str(
            rtsp_config.get("mount_path", f"/{config.name}")
        )
        self.rtsp_encoding = str(rtsp_config.get("encoding", "h264")).lower()
        if not 1 <= self.rtsp_port <= 65535:
            raise ValueError("camera.rtsp.port must be between 1 and 65535")
        if not self.rtsp_mount_path.startswith("/"):
            raise ValueError("camera.rtsp.mount_path must start with '/'")
        if self.rtsp_encoding not in {"h264", "raw"}:
            raise ValueError("camera.rtsp.encoding must be h264 or raw")
        if self.publish_rate <= 0.0:
            raise ValueError("camera.publish_rate_hz must be positive")
        self._next_capture: float | None = None
        self._pose_ready = False
        self._Image = Image
        self._CompressedImage = CompressedImage
        self._CameraInfo = CameraInfo
        self._cameras = []
        camera_prim_paths = []
        self._publishers = []
        self._side_by_side_publisher = (
            node.create_publisher(Image, "image_raw", 10)
            if mode == "stereo" and "ros_raw" in self.transports else None
        )
        self._side_by_side_compressed_publisher = (
            node.create_publisher(CompressedImage, "image_raw/compressed", 10)
            if mode == "stereo" and "ros_compressed" in self.transports else None
        )
        self._rtsp_graphs = []
        names = ["mono"] if mode == "mono" else ["left", "right"]
        for name in names:
            suffix = "" if mode == "mono" else f"/{name}"
            camera_prim_path = f"/World/ECM/Camera{name.title() if mode == 'stereo' else ''}"
            camera = Camera(
                prim_path=camera_prim_path,
                name=f"ECM_camera_{name}", frequency=self.publish_rate,
                resolution=(self.width, self.height),
                orientation=np.asarray([1.0, 0.0, 0.0, 0.0]),
            )

            # Stereo output is rendered by the single tiled render product
            # below.  Initializing each legacy Camera here would create two
            # additional per-eye render products which are never read, but
            # still consume a full render on every Isaac update.
            needs_mono_pixels = (
                mode == "mono"
                and ("ros_raw" in self.transports
                     or "ros_compressed" in self.transports)
            )

            if needs_mono_pixels:
                camera.initialize()

            camera_prim_paths.append(camera_prim_path)
            # Isaac expresses focal length and aperture in the same stage
            # units. Preserve the camera's sensor width while selecting the
            # requested horizontal field of view.
            aperture = camera.get_horizontal_aperture()
            camera.set_focal_length(aperture / (2.0 * math.tan(self.fov / 2.0)))
            camera.set_clipping_range(self.near, self.far)
            self._cameras.append(camera)
            self._publishers.append((
                node.create_publisher(Image, f"image_raw{suffix}", 10)
                if mode == "mono" and "ros_raw" in self.transports else None,
                node.create_publisher(CompressedImage, f"image_raw{suffix}/compressed", 10)
                if mode == "mono" and "ros_compressed" in self.transports else None,
                node.create_publisher(CameraInfo, f"camera_info{suffix}", 10),
            ))
            if mode == "mono" and "rtsp" in self.transports:
                self._rtsp_graphs.append(
                    self._create_rtsp_graph(camera_prim_path, name, index=len(self._rtsp_graphs))
                )
        self._tiled_sensor = None
        self._tiled_render_product_path = None
        if mode == "stereo":
            from isaacsim.sensors.experimental.rtx import TiledCameraSensor

            tiled_annotators = (
                ["rgb"]
                if ("ros_raw" in self.transports
                    or "ros_compressed" in self.transports)
                else []
            )

            self._tiled_sensor = TiledCameraSensor(
                camera_prim_paths,
                resolution=(self.height, self.width),
                annotators=tiled_annotators,
            )
            self._tiled_render_product_path = str(
                self._tiled_sensor.render_product.GetPath()
            )
            if "rtsp" in self.transports:
                self._rtsp_graphs.append(
                    self._create_rtsp_graph(
                        self._tiled_render_product_path, "stereo", 0, True
                    )
                )
        namespace = node.get_namespace().strip("/")
        topic_prefix = f"/{namespace}" if namespace else ""
        image_width = self.width * 2 if mode == "stereo" else self.width
        node.get_logger().info(
            "ECM ROS 2 image publisher: "
            f"topic={topic_prefix}/image_raw, "
            f"encoding={self.encoding}, resolution={image_width}x{self.height}, "
            f"frame_id={self.frame_id}, rate={self.publish_rate:g} Hz, "
            f"mode={mode}{', synchronized side-by-side' if mode == 'stereo' else ''}"
        )
        if mode == "stereo":
            node.get_logger().info(
                f"ECM tiled camera: render_product={self._tiled_render_product_path}, "
                f"tiled_resolution={self.width * 2}x{self.height}"
            )
        if "rtsp" in self.transports:
            node.get_logger().info(
                "ECM GStreamer RTSP: "
                f"url=rtsp://<host>:{self.rtsp_port}{self.rtsp_mount_path}, "
                f"encoding={self.rtsp_encoding}, source="
                f"{'tiled render product' if mode == 'stereo' else 'camera render product'}"
            )
        elif mode == "mono":
            node.get_logger().info(
                f"ECM GStreamer RTSP: disabled (transports={self.transports})"
            )

    def _create_rtsp_graph(self, camera_prim: str, name: str, index: int,
                           existing_render_product: bool = False):
        """Create Isaac Sim's built-in RTSP camera graph."""
        import omni.graph.core as og

        port = self.rtsp_port + index
        mount_path = self.rtsp_mount_path.rstrip("/")
        graph_path = f"/World/CRTKROS/{self.node.get_name()}_{name}_RTSP"
        keys = og.Controller.Keys
        create_nodes = [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("RTSPPublish", "isaacsim.streaming.rtsp.RTSPCameraHelper")]
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
                        ("RenderProduct.inputs:width", self.width),
                        ("RenderProduct.inputs:height", self.height),
                    ]),
                    ("RTSPPublish.inputs:port", port),
                    ("RTSPPublish.inputs:mountPath", mount_path),
                    ("RTSPPublish.inputs:useRawEncoding",
                     self.rtsp_encoding == "raw"),
                ],
            },
        )
        # The helper attaches its writer on the first playback tick and is
        # idempotent thereafter. Do not also evaluate this graph synchronously
        # from publish(); doing so duplicates work on every rendered frame.
        self.node.get_logger().info(
            f"ECM RTSP stream: rtsp://<host>:{port}{mount_path}"
        )
        return graph

    def _image_message(self, data: np.ndarray, stamp, camera_name: str):
        message = self._Image()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        data = np.asarray(data, dtype=np.uint8)
        if data.ndim == 2:
            data = data[:, :, None]
        if self.encoding == "rgb8" and data.shape[2] >= 3:
            data = data[:, :, :3]
        elif self.encoding == "rgba8" and data.shape[2] == 3:
            alpha = np.full(data.shape[:2] + (1,), 255, dtype=np.uint8)
            data = np.concatenate((data, alpha), axis=2)
        message.height, message.width = data.shape[:2]
        message.encoding = self.encoding
        message.is_bigendian = False
        message.step = int(data.shape[1] * data.shape[2])
        message.data = data.tobytes()
        return message

    def _compressed_image_message(self, data: np.ndarray, stamp):
        """Encode a standard JPEG CompressedImage for image_transport users."""
        from io import BytesIO
        from PIL import Image as PILImage

        rgb = np.asarray(data, dtype=np.uint8)
        if rgb.ndim == 2:
            rgb = np.repeat(rgb[:, :, None], 3, axis=2)
        elif rgb.shape[2] >= 3:
            rgb = rgb[:, :, :3]
        encoded = BytesIO()
        PILImage.fromarray(rgb, mode="RGB").save(
            encoded, format="JPEG", quality=self.jpeg_quality
        )
        message = self._CompressedImage()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.format = "rgb8; jpeg compressed rgb8"
        message.data = encoded.getvalue()
        return message

    def _camera_info(self, stamp):
        info = self._CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width, info.height = self.width, self.height
        fx = (self.width / 2.0) / math.tan(self.fov / 2.0)
        fy = fx
        cx, cy = (self.width - 1) / 2.0, (self.height - 1) / 2.0
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def set_pose(self, pose: Pose) -> None:
        """Apply the ECM pose before Isaac renders the next frame."""
        orientation = _quaternion_wxyz(pose.orientation)
        for index, camera in enumerate(self._cameras):
            position = pose.position.copy()
            if self.mode == "stereo":
                position += pose.orientation[:, 0] * (self.baseline / 2.0) * (-1.0 if index == 0 else 1.0)
            # ECM_optical uses local +X as the viewing direction. Isaac's
            # world-axis camera mode also defines +X as forward.
            camera.set_world_pose(position=position, orientation=orientation, camera_axes="world")
        self._pose_ready = True

    def publish(self, seconds: float, pose: Pose | None = None) -> None:
        """Publish the frame rendered with the latest pose."""
        capture_period = 1.0 / self.publish_rate

        if (self._next_capture is None
                or seconds + capture_period < self._next_capture):
            self._next_capture = seconds

        if seconds + 1.0e-12 < self._next_capture:
            return

        elapsed_periods = max(
            1,
            math.floor(
                (seconds + 1.0e-12 - self._next_capture) / capture_period
            ) + 1,
        )
        self._next_capture += elapsed_periods * capture_period

        if pose is not None:
            self.set_pose(pose)

        if not self._pose_ready:
            return

        from builtin_interfaces.msg import Time

        stamp = Time(sec=int(seconds), nanosec=int((seconds - int(seconds)) * 1e9))

        for index, (camera, (image_publisher, compressed_publisher, info_publisher)) in enumerate(
                zip(self._cameras, self._publishers)):
            needs_image = (
                (image_publisher is not None and image_publisher.get_subscription_count() > 0)
                or (compressed_publisher is not None
                    and compressed_publisher.get_subscription_count() > 0)
            )
            data = camera.get_rgba() if self.mode == "mono" and needs_image else None
            if image_publisher is not None and data is not None and image_publisher.get_subscription_count() > 0:
                image_publisher.publish(self._image_message(data, stamp, str(index)))
            if (compressed_publisher is not None and data is not None
                    and compressed_publisher.get_subscription_count() > 0):
                compressed_publisher.publish(self._compressed_image_message(data, stamp))
            if info_publisher.get_subscription_count() > 0:
                info_publisher.publish(self._camera_info(stamp))
        side_by_side_needed = (
            (self._side_by_side_publisher is not None
             and self._side_by_side_publisher.get_subscription_count() > 0)
            or (self._side_by_side_compressed_publisher is not None
                and self._side_by_side_compressed_publisher.get_subscription_count() > 0)
        )
        if side_by_side_needed and self._tiled_sensor is not None:
            tiled_data, _ = self._tiled_sensor.get_data("rgb", tiled=True)
            if tiled_data is None:
                return
            side_by_side = np.asarray(tiled_data.numpy(), dtype=np.uint8)
            if (self._side_by_side_publisher is not None
                    and self._side_by_side_publisher.get_subscription_count() > 0):
                self._side_by_side_publisher.publish(
                    self._image_message(side_by_side, stamp, "stereo")
                )
            if (self._side_by_side_compressed_publisher is not None
                    and self._side_by_side_compressed_publisher.get_subscription_count() > 0):
                self._side_by_side_compressed_publisher.publish(
                    self._compressed_image_message(side_by_side, stamp)
                )
