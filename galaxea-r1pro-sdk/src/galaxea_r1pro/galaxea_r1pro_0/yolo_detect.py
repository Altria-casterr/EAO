"""YOLO detection client that only captures images from SDK and delegates perception logic."""

from __future__ import annotations

from typing import Any

from .backends.base import RobotBackend
from .utils import utils_yolo


class YoloDetect:
    def __init__(self, backend: RobotBackend, sensors) -> None:
        self._backend = backend
        self._sensors = sensors

    def _capture_images(self, camera_id: str) -> tuple[Any, Any]:
        color_frame = self._sensors.capture_rgb(camera_id)
        depth_frame = self._sensors.capture_depth(camera_id)
        return color_frame.data, depth_frame.data

    def yolo_seg_with_point_cloud(
        self,
        *,
        camera_id: str = "head_camera",
        point_cloud_top_k: int = 3,
        model_path: str | None = None,
        class_names: list[str] | None = None,
        confidence: float = 0.5,
        iou_threshold: float | None = None,
        imgsz: tuple[int, int] | None = None,
        device: int | str = 0,
        image_scale: float = 0.765,
        target_coordinate_system: str = "base",
        pcd_downsample_scale: float = 1.0,
        **kwargs: Any,
    ) -> dict[str, Any]:
        color_image, depth_image = self._capture_images(camera_id)
        return utils_yolo.yolo_seg_with_point_cloud(
            color_image=color_image,
            depth_image=depth_image,
            model_path=model_path,
            point_cloud_top_k=point_cloud_top_k,
            class_names=class_names,
            confidence=confidence,
            iou_threshold=iou_threshold,
            imgsz=imgsz,
            device=device,
            image_scale=image_scale,
            target_coordinate_system=target_coordinate_system,
            pcd_downsample_scale=pcd_downsample_scale,
            **kwargs,
        )
