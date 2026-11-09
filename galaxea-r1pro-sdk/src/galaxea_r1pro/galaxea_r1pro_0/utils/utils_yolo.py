"""YOLO detection and point-cloud utilities."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np

DEFAULT_CAMERA_INTRINSICS = {
    "fx": 320.28,
    "fy": 320.82,
    "cx": 320.79,
    "cy": 241.47,
}
DEFAULT_CAMERA_TO_TCP = (0.0, 0.0, 0.0, -1.5708, 0.0, -1.5708)
DEFAULT_TCP_TO_BASE = (0.0, 0.0, 450.0, 0.0, 1.0472, 0.0)


def default_model_path() -> Path:
    return Path(__file__).resolve().parent / "yolo_model.pt"


def _to_mm(depth_values: np.ndarray) -> np.ndarray:
    depth = depth_values.astype(np.float32)
    if depth.size == 0:
        return depth
    if float(np.nanmax(depth)) <= 20.0:
        return depth * 1000.0
    return depth


def _rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _transform_point(point: tuple[float, float, float], transform: tuple[float, float, float, float, float, float]) -> tuple[float, float, float]:
    tx, ty, tz, roll, pitch, yaw = transform
    rotated = _rotation_matrix(roll, pitch, yaw) @ np.asarray(point, dtype=np.float32)
    translated = rotated + np.asarray((tx, ty, tz), dtype=np.float32)
    return tuple(float(v) for v in translated)


def _camera_to_base(point_camera_mm: tuple[float, float, float]) -> tuple[float, float, float]:
    point_tcp = _transform_point(point_camera_mm, DEFAULT_CAMERA_TO_TCP)
    return _transform_point(point_tcp, DEFAULT_TCP_TO_BASE)


def _build_point_cloud_center(
    depth_image: np.ndarray,
    mask: np.ndarray | None,
    box_xyxy: tuple[float, float, float, float],
    camera_intrinsics: dict[str, float],
    pcd_downsample_scale: float,
) -> tuple[float, float, float]:
    h, w = depth_image.shape[:2]
    x1, y1, x2, y2 = [int(round(v)) for v in box_xyxy]
    x1 = max(0, min(w - 1, x1))
    x2 = max(0, min(w, x2))
    y1 = max(0, min(h - 1, y1))
    y2 = max(0, min(h, y2))
    if x2 <= x1 or y2 <= y1:
        return 0.0, 0.0, 0.0

    region = depth_image[y1:y2, x1:x2]
    valid_mask = np.isfinite(region) & (region > 0)
    if mask is not None:
        region_mask = mask[y1:y2, x1:x2] > 0
        if np.any(region_mask):
            valid_mask &= region_mask
    if not np.any(valid_mask):
        return 0.0, 0.0, 0.0

    ys, xs = np.nonzero(valid_mask)
    stride = max(1, int(round(1.0 / pcd_downsample_scale))) if 0 < pcd_downsample_scale < 1.0 else 1
    ys = ys[::stride]
    xs = xs[::stride]
    z = _to_mm(region[ys, xs])
    u = xs.astype(np.float32) + float(x1)
    v = ys.astype(np.float32) + float(y1)
    fx = float(camera_intrinsics["fx"])
    fy = float(camera_intrinsics["fy"])
    cx = float(camera_intrinsics["cx"])
    cy = float(camera_intrinsics["cy"])
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return float(np.median(x)), float(np.median(y)), float(np.median(z))


def yolo_seg_with_point_cloud(
    *,
    color_image: Any,
    depth_image: Any,
    model_path: str | None = None,
    point_cloud_top_k: int = 3,
    class_names: list[str] | None = None,
    confidence: float = 0.5,
    iou_threshold: float | None = None,
    imgsz: tuple[int, int] | None = None,
    device: int | str = 0,
    image_scale: float = 0.765,
    target_coordinate_system: str = "base",
    pcd_downsample_scale: float = 1.0,
    camera_intrinsics: dict[str, float] | None = None,
    **_: Any,
) -> dict[str, Any]:
    """执行 YOLO 分割并输出前 K 个目标中心点。"""

    if color_image is None or depth_image is None:
        return {
            "centers": [],
            "objects": [],
            "point_cloud_top_k": point_cloud_top_k,
            "coordinate_system": target_coordinate_system,
            "image_shape": None,
        }

    try:
        from ultralytics import YOLO
    except Exception as exc:  # pragma: no cover
        raise RuntimeError("ultralytics 未安装，无法执行 YOLO 推理") from exc

    model = YOLO(model_path or str(default_model_path()))
    predict_kwargs: dict[str, Any] = {
        "conf": confidence,
        "device": device,
        "verbose": False,
    }
    if imgsz is not None:
        predict_kwargs["imgsz"] = imgsz
    if iou_threshold is not None:
        predict_kwargs["iou"] = iou_threshold

    result = model(color_image, **predict_kwargs)[0]
    objects: list[dict[str, Any]] = []
    if result.boxes is None:
        return {
            "centers": [],
            "objects": objects,
            "point_cloud_top_k": point_cloud_top_k,
            "coordinate_system": target_coordinate_system,
            "image_shape": tuple(color_image.shape),
        }

    intrinsics = dict(DEFAULT_CAMERA_INTRINSICS)
    if camera_intrinsics is not None:
        intrinsics.update(camera_intrinsics)

    boxes = result.boxes.xyxy.cpu().numpy()
    cls_ids = result.boxes.cls.cpu().numpy().astype(int)
    confs = result.boxes.conf.cpu().numpy()
    masks = result.masks.data.cpu().numpy() if result.masks is not None else None

    if class_names is None:
        class_names = []

    candidates: list[dict[str, Any]] = []
    for idx, box in enumerate(boxes):
        cls_id = int(cls_ids[idx])
        class_name = class_names[cls_id] if 0 <= cls_id < len(class_names) else f"class_{cls_id}"
        mask = None
        if masks is not None and idx < len(masks):
            mask = masks[idx]
            if mask.shape[:2] != depth_image.shape[:2]:
                import cv2
                mask = cv2.resize(mask.astype(np.float32), (depth_image.shape[1], depth_image.shape[0]), interpolation=cv2.INTER_NEAREST)
            mask = (mask > 0.5).astype(np.uint8)
        center_x, center_y, center_z = _build_point_cloud_center(
            depth_image,
            mask,
            tuple(box.tolist()),
            intrinsics,
            pcd_downsample_scale,
        )
        center_camera_mm = (center_x, center_y, center_z)
        center_base_mm = _camera_to_base(center_camera_mm) if target_coordinate_system == "base" else None
        candidates.append(
            {
                "class_id": cls_id,
                "class_name": class_name,
                "confidence": float(confs[idx]),
                "box_xyxy": box.tolist(),
                "mask": mask,
                "center_camera_mm": center_camera_mm,
                "center_base_mm": center_base_mm,
            }
        )

    candidates.sort(key=lambda item: item["confidence"], reverse=True)
    objects = candidates[: max(0, int(point_cloud_top_k))]
    centers = [item["center_base_mm"] or item["center_camera_mm"] for item in objects]

    return {
        "centers": centers,
        "objects": objects,
        "point_cloud_top_k": point_cloud_top_k,
        "coordinate_system": target_coordinate_system,
        "image_shape": tuple(color_image.shape),
        "pcd_downsample_scale": pcd_downsample_scale,
        "image_scale": image_scale,
    }
