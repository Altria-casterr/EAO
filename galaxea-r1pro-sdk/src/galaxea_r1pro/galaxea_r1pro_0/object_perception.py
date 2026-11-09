"""Object perception client for YOLO RGB-D segmentation and point cloud generation."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from typing import Any, Mapping

from .backends.base import RobotBackend


def _legacy_main_path() -> Path:
    return (
        Path(__file__).resolve().parents[4]
        / "skill_library-v2.x.x"
        / "Atom"
        / "Object_Detect_Yolo"
        / "Yolo_Seg"
        / "scripts"
        / "main.py"
    )


def _load_legacy_detector() -> type:
    legacy_main = _legacy_main_path()
    if not legacy_main.is_file():
        raise RuntimeError(f"YOLO implementation not found: {legacy_main}")
    spec = importlib.util.spec_from_file_location("legacy_yolo_main", legacy_main)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load YOLO implementation: {legacy_main}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Object_Detect_Yolo_Seg


class ObjectPerception:
    """Client for YOLO segmentation and point-cloud generation."""

    def __init__(self, backend: RobotBackend) -> None:
        self._backend = backend

    def yolo_seg_with_point_cloud(
        self,
        *,
        color_img_path: str,
        depth_img_path: str,
        model_path: str,
        conf_threshold: float = 0.5,
        iou_threshold: float | None = None,
        imgsz: tuple[int, int] | None = None,
        device: int = 0,
        class_names: list[str] | None = None,
        pcd_downsample_scale: float = 1.0,
        image_scale: float = 0.765,
        target_coordinate_system: str = "base",
        point_cloud_top_k: int = 3,
        **kwargs: Any,
    ) -> Mapping[str, Any]:
        detector_cls = _load_legacy_detector()
        detector = detector_cls(model_path=model_path, **kwargs)
        return detector.yolo_seg_with_point_cloud(
            color_img_path=color_img_path,
            depth_img_path=depth_img_path,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            imgsz=imgsz,
            device=device,
            class_names=class_names,
            pcd_downsample_scale=pcd_downsample_scale,
            image_scale=image_scale,
            mode="seg_with_pcd",
            target_coordinate_system=target_coordinate_system,
            point_cloud_top_k=point_cloud_top_k,
        )
