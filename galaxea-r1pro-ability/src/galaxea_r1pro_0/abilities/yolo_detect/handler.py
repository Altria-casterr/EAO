"""Ability-side object perception orchestration for the YOLO tasks."""
from __future__ import annotations

from typing import Any, Mapping
import os
import numpy as np
import time

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

    @staticmethod
    def _stamp(frame):
        meta = getattr(frame, "metadata", {}) or {}
        value = meta.get("ros_stamp") or {}
        seconds = (float(value.get("sec", value.get("secs", 0)))
                   + float(value.get("nanosec", value.get("nsecs", 0))) * 1e-9)
        if seconds > 0:
            return seconds
        received = meta.get("received_monotonic")
        return float(received) if received is not None else None

    def capture(self, sensor_id: str, *, attempts: int = 30,
                capture_error_attempts: int = 3, max_skew_s: float = 0.1,
                retry_interval_s: float = 0.05):
        """等待一对同步 RGB-D 帧；低频相机下不因首对帧错相而立即失败。"""
        if (attempts < 1 or capture_error_attempts < 1 or max_skew_s <= 0
                or retry_interval_s < 0):
            raise ValueError("RGB-D 同步参数非法")
        capture_errors = 0
        last_skew = None
        for attempt in range(attempts):
            try:
                rgb = self.sdk.sensors.capture_rgb(sensor_id)
                depth = self.sdk.sensors.capture_depth(sensor_id)
            except Exception:
                capture_errors += 1
                if capture_errors >= capture_error_attempts:
                    raise
                time.sleep(retry_interval_s)
                continue
            stamps = self._stamp(rgb), self._stamp(depth)
            if None in stamps:
                return rgb, depth
            last_skew = abs(stamps[0] - stamps[1])
            if last_skew <= max_skew_s:
                return rgb, depth
            if attempt + 1 < attempts:
                time.sleep(retry_interval_s)
        raise ValueError(
            f"RGB 与深度图连续 {attempts} 次未同步，最末时间差 "
            f"{last_skew:.3f} 秒，要求不超过 {max_skew_s:.3f} 秒"
        )

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
