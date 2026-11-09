"""Ability-side object perception orchestration for the YOLO tasks."""
from __future__ import annotations

from typing import Any, Mapping
import os
import numpy as np

from .yolo_client import YoloClient
from .camera_geometry import merge_camera_input, resolve_geometry, frame_name


class ObjectPerceptionHandler:
    def __init__(self, sdk: Any, models: Any = None, cameras: Mapping[str, Any] | None = None):
        if cameras is not None and not isinstance(cameras, Mapping):
            raise ValueError("cameras 必须是按 sensor_id 索引的对象")
        self.cameras = dict(cameras or {})
        self.sdk = sdk
        self.models = models
        self._clients: dict[str, YoloClient] = {}

    def close(self) -> None:
        for client in self._clients.values():
            client.close()
        self._clients.clear()

    def _client(self, data: Mapping[str, Any]) -> YoloClient:
        profile_name = str(data.get("model_profile") or "default")
        settings = {}
        if self.models is not None:
            try:
                profile = self.models.resolve("yolo_detect", data.get("model_profile"))
                profile_name, settings = profile.name, dict(profile.settings)
            except Exception:
                if data.get("model_profile"):
                    raise
        checkpoint = data.get("checkpoint_path") or os.environ.get("YOLO_CHECKPOINT_PATH")
        if checkpoint and "checkpoint_path" not in settings:
            settings["checkpoint_path"] = str(checkpoint)
        client = self._clients.get(profile_name)
        if client is None:
            client = self._clients[profile_name] = YoloClient(settings)
        return client

    def capture(self, sensor_id: str):
        rgb = self.sdk.sensors.capture_rgb(sensor_id)
        depth = self.sdk.sensors.capture_depth(sensor_id)
        return rgb, depth

    def prepare(self, data):
        return merge_camera_input(self.cameras, data)

    def segment(self, data: Mapping[str, Any]):
        return self._segment(self.prepare(data), geometry=False)[:3]

    def segment_with_geometry(self, data):
        return self._segment(data, geometry=True)

    def _segment(self, data, *, geometry):
        sensor_id = str(data.get("sensor_id") or "head_camera")
        rgb_frame, depth_frame = self.capture(sensor_id)
        color = np.asarray(rgb_frame.data)
        depth = np.asarray(depth_frame.data)
        if color.ndim != 3 or color.shape[2] != 3:
            raise ValueError("RGB 帧必须是 H×W×3 图像")
        if depth.ndim == 3:
            depth = depth[..., 0]
        if depth.ndim != 2 or depth.shape != color.shape[:2]:
            raise ValueError(f"RGB 和深度图尺寸不一致: rgb={color.shape[:2]}, depth={depth.shape}")
        rgb_meta = getattr(rgb_frame, 'metadata', {}) or {}
        depth_meta = getattr(depth_frame, 'metadata', {}) or {}
        def stamp(meta):
            value = meta.get('ros_stamp') or {}
            seconds = float(value.get('sec', value.get('secs', 0))) + float(value.get('nanosec', value.get('nsecs', 0))) * 1e-9
            return seconds if seconds > 0 else None
        stamps = (stamp(rgb_meta), stamp(depth_meta))
        if None in stamps:
            stamps = (rgb_meta.get('received_monotonic'), depth_meta.get('received_monotonic'))
        if None not in stamps and abs(stamps[0] - stamps[1]) > 0.1:
            raise ValueError('RGB 与深度图时间差超过 0.1 秒，请停稳后重新采集')
        calibration = resolve_geometry(self.sdk, data, rgb_frame.frame_id) if geometry else None
        if geometry and frame_name(depth_frame.frame_id) and frame_name(depth_frame.frame_id) != calibration.camera_frame:
            raise ValueError('深度图坐标系与 RGB 光学坐标系不一致，请提供对齐到 RGB 的深度图')
        client = self._client(data)
        objects = client.segment_image(
            color,
            conf_threshold=data.get("conf_threshold"),
            iou_threshold=data.get("iou_threshold"),
            imgsz=data.get("imgsz"), device=data.get("device"),
            class_names=data.get("class_names"),
        )
        return rgb_frame, depth_frame, objects, calibration

    def segment_image(self, image: np.ndarray, data: Mapping[str, Any]):
        return self._client(data).segment_image(
            image, conf_threshold=data.get("conf_threshold"),
            iou_threshold=data.get("iou_threshold"), imgsz=data.get("imgsz"),
            device=data.get("device"), class_names=data.get("class_names"),
        )
