"""YOLO、掩码深度处理、点云匹配和旋转估计工具。

实现保持 Object_Detect_Yolo/Utils/Utils.py 的坐标和搜索约定：
- 欧拉角旋转顺序为 Rz @ Ry @ Rx；
- 旧工程 6D 位姿的平移单位为毫米；
- 旋转匹配使用体素 IoU，单位转换为米后搜索。
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any, Mapping, Sequence

import cv2
import numpy as np
from scipy.optimize import minimize_scalar
from .point_cloud_filter import filter_point_cloud


class YoloUtils:
    """旧 YOLO 工具库中与 Ability 运行时相关的无状态工具。"""

    @staticmethod
    def reference_image_paths(base_dir: str | Path | None = None) -> tuple[Path, Path]:
        root = Path(base_dir) if base_dir is not None else Path(__file__).resolve().parent / "reference"
        return root / "zero_color_s_box.png", root / "zero_depth_s_box.png"

    @staticmethod
    def rotation_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
        cx, sx = np.cos(rx), np.sin(rx)
        cy, sy = np.cos(ry), np.sin(ry)
        cz, sz = np.cos(rz), np.sin(rz)
        rx_matrix = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        ry_matrix = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        rz_matrix = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        return rz_matrix @ ry_matrix @ rx_matrix

    @classmethod
    def pose_to_matrix(cls, pose: Sequence[float]) -> np.ndarray:
        if len(pose) != 6:
            raise ValueError("位姿必须是长度为6的列表：[x, y, z, rx, ry, rz]")
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = cls.rotation_matrix(*map(float, pose[3:]))
        matrix[:3, 3] = np.asarray(pose[:3], dtype=np.float64)
        return matrix

    @staticmethod
    def refine_mask_by_depth(
        depth_image: np.ndarray,
        mask: np.ndarray,
        *,
        mask_erode_iterations: int = 2,
        depth_band_tolerance: float = 200.0,
        mask_keep_fill_ratio: float = 0.5,
        mask_keep_depth_ratio: float = 0.5,
        depth_relax_fill_ratio: float = 0.62,
        depth_relax_solidity: float = 0.90,
        depth_relax_width_stability: float = 0.35,
        depth_relax_multiplier: float = 3.0,
    ) -> np.ndarray:
        """按旧工程规则用掩码区域的深度中位数清除背景点。"""
        depth = np.asarray(depth_image)
        binary = np.asarray(mask) > 0
        if binary.shape != depth.shape:
            binary = cv2.resize(binary.astype(np.uint8), (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_NEAREST).astype(bool)
        valid = binary & np.isfinite(depth) & (depth > 0)
        if not np.any(valid):
            return np.zeros(depth.shape, dtype=np.uint8)

        values = depth[valid].astype(np.float32)
        median = float(np.median(values))
        ys, xs = np.where(binary)
        area = max(1, (xs.max()-xs.min()+1)*(ys.max()-ys.min()+1))
        fill = float(binary.sum()) / area
        inliers = float(np.mean(np.abs(values-median) <= depth_band_tolerance))
        if fill >= mask_keep_fill_ratio and inliers >= mask_keep_depth_ratio:
            mask_erode_iterations = 0
        if mask_erode_iterations > 0:
            binary = cv2.erode(binary.astype(np.uint8), np.ones((3, 3), np.uint8),
                               iterations=mask_erode_iterations).astype(bool)
        valid = binary & np.isfinite(depth) & (depth > 0)
        if not valid.any():
            return np.zeros(depth.shape, dtype=np.uint8)
        values = depth[valid].astype(np.float32)
        median = float(np.median(values))
        tolerance = max(float(depth_band_tolerance), 1e-12)
        ys, xs = np.where(binary)
        area = max(1, (xs.max()-xs.min()+1)*(ys.max()-ys.min()+1))
        fill = float(binary.sum()) / area
        contours, _ = cv2.findContours(binary.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        solidity, width_cv = 0.0, 1.0
        if contours:
            contour = max(contours, key=cv2.contourArea)
            solidity = max(cv2.contourArea(contour), 1.0) / max(cv2.contourArea(cv2.convexHull(contour)), 1.0)
            widths = np.count_nonzero(binary[ys.min():ys.max()+1, xs.min():xs.max()+1], axis=1).astype(np.float32)
            widths = widths[widths > 0]
            width_cv = float(widths.std()/max(widths.mean(), 1.0))
        if fill >= depth_relax_fill_ratio and solidity >= depth_relax_solidity and width_cv <= depth_relax_width_stability:
            tolerance *= depth_relax_multiplier
        elif np.mean(np.abs(values-median) <= tolerance) >= 0.85:
            tolerance *= 1.5
        refined = binary & np.isfinite(depth) & (depth > 0) & (np.abs(depth.astype(np.float32)-median) <= tolerance)
        return (refined if refined.any() else valid).astype(np.uint8)

    @staticmethod
    def generate_point_cloud(
        depth_image: np.ndarray,
        mask: np.ndarray,
        intrinsics: Mapping[str, float],
        depth_scale: float,
        transform: tuple[np.ndarray, np.ndarray],
        max_points: int = 5000,
        refine_mask: bool = True,
        filter_config: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        if not np.isfinite(depth_scale) or depth_scale <= 0:
            raise ValueError("depth_scale 必须为正的有限数")
        if isinstance(max_points, bool) or not isinstance(max_points, int) or max_points < 0:
            raise ValueError("max_points 必须为非负整数，0 表示不限制")
        depth = np.asarray(depth_image)
        if depth.ndim == 3:
            depth = depth[..., 0]
        binary = np.asarray(mask) > 0
        if binary.shape != depth.shape:
            raise ValueError(f"掩码和深度图尺寸不一致: mask={binary.shape}, depth={depth.shape}")
        if refine_mask:
            binary = YoloUtils.refine_mask_by_depth(depth, binary, depth_band_tolerance=0.2 / depth_scale) > 0
        valid = binary & np.isfinite(depth) & (depth > 0)
        v, u = np.where(valid)
        if len(u) == 0:
            raise ValueError("掩码区域没有有效深度")
        fx, fy = float(intrinsics["fx"]), float(intrinsics["fy"])
        cx, cy = float(intrinsics["cx"]), float(intrinsics["cy"])
        if not np.isfinite([fx, fy, cx, cy]).all() or fx <= 0 or fy <= 0:
            raise ValueError("fx 和 fy 必须大于 0")
        z = depth[v, u].astype(np.float64) * float(depth_scale)
        points_camera = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))
        rotation, translation = transform
        points_robot = points_camera @ rotation.T + translation
        keep, stats = filter_point_cloud(points_robot, filter_config)
        points_camera, points_robot = points_camera[keep], points_robot[keep]
        low, high = points_robot.min(axis=0), points_robot.max(axis=0)
        center = (low + high) / 2.0
        if max_points > 0 and len(points_robot) > max_points:
            indices = np.linspace(0, len(points_robot)-1, max_points, dtype=np.int64)
            points_camera, points_robot = points_camera[indices], points_robot[indices]
        return {
            "points_camera": points_camera, "points_robot": points_robot,
            "center_robot": center, "center_method": "bounding_box",
            "bounds_robot": {"min": low, "max": high}, "filter_stats": stats,
        }

    @staticmethod
    def match_target(objects: Sequence[Mapping[str, Any]], target_point: Sequence[float], thresholds: Sequence[float]) -> dict[str, Any]:
        target = np.asarray(target_point, dtype=np.float64)
        candidates = []
        for obj in objects:
            center = obj.get("center_robot")
            if center is None:
                continue
            delta = np.abs(np.asarray(center, dtype=np.float64) - target)
            candidates.append({**dict(obj), "delta": delta.tolist(), "distance": float(np.linalg.norm(delta))})
        candidates.sort(key=lambda item: item["distance"])
        best = next((item for item in candidates
                     if all(d <= t for d, t in zip(item["delta"], thresholds))), None)
        success = best is not None
        return {"success": success, "best_match": best if success else None, "candidates": candidates, "target_point": target.tolist(), "thresholds": list(thresholds)}


class PointCloudRotationMatcher:
    """旧工程的单轴体素 IoU 旋转搜索实现，不产生文件或进程副作用。"""

    def __init__(self, *, yaw_range: tuple[float, float] = (-90.0, 90.0), search_step: float = 2.0, focus_step: float = 0.5, max_points: int = 15000, shape_bin_size: float = 0.05, enable_roll: bool = False, enable_pitch: bool = False, enable_yaw: bool = True):
        if len(yaw_range) != 2 or not np.isfinite(yaw_range).all() or yaw_range[0] >= yaw_range[1]:
            raise ValueError("yaw_range 必须为有限且递增的二元范围")
        if not np.isfinite([search_step, focus_step, shape_bin_size]).all() or min(search_step, focus_step, shape_bin_size) <= 0 or max_points < 3:
            raise ValueError("旋转搜索步长、容差、体素边长必须为正，max_points 至少为 3")
        self.ranges = {"roll": (-90.0, 90.0), "pitch": (-90.0, 90.0), "yaw": yaw_range}
        self.search_step = search_step
        self.focus_step = focus_step
        self.max_points = max_points
        self.shape_bin_size = shape_bin_size
        self.enabled = {"roll": enable_roll, "pitch": enable_pitch, "yaw": enable_yaw}
        self.reference = None
        self.target_voxels = None

    def load_clouds(self, ref_cloud_mm: np.ndarray, target_cloud_mm: np.ndarray) -> "PointCloudRotationMatcher":
        reference = self._prepare(ref_cloud_mm)
        target = self._prepare(target_cloud_mm)
        if len(reference) < 3 or len(target) < 3:
            raise ValueError("点云点数不足，至少需要 3 个点")
        self.reference = reference / 1000.0
        self.target_voxels = self._voxels(target / 1000.0)
        return self

    def estimate_pose(self) -> dict[str, Any]:
        if self.reference is None or self.target_voxels is None:
            raise RuntimeError("先调用 load_clouds 加载点云")
        enabled = [axis for axis, value in self.enabled.items() if value]
        if not enabled:
            raise ValueError("至少需要使能一个旋转轴")
        result = {"roll_angle": 0.0, "pitch_angle": 0.0, "yaw_angle": 0.0, "roll_iou": 0.0, "pitch_iou": 0.0, "yaw_iou": 0.0}
        for axis in enabled:
            low, high = self.ranges[axis]
            angles = np.unique(np.append(np.arange(low, high, self.search_step), high))
            coarse = max(angles, key=lambda angle: self._score(float(angle), axis))
            refined = minimize_scalar(lambda angle: 1.0 - self._score(float(angle), axis), bounds=(max(low, coarse - 5.0), min(high, coarse + 5.0)), method="bounded", options={"xatol": self.focus_step})
            # 体素 IoU 并不光滑，细搜可能比粗搜差，不能无条件覆盖。
            angle = float(coarse)
            if refined.success and self._score(float(refined.x), axis) > self._score(angle, axis):
                angle = float(refined.x)
            score = self._score(angle, axis)
            result[f"{axis}_angle"] = angle
            result[f"{axis}_iou"] = score
        best_axis = max(enabled, key=lambda axis: result[f"{axis}_iou"])
        best_iou = result[f"{best_axis}_iou"]
        result.update({"best_axis": best_axis, "best_angle": result[f"{best_axis}_angle"], "best_iou": best_iou, "success": best_iou > 0.5, "timestamp": datetime.now().isoformat()})
        return result

    def _score(self, angle: float, axis: str) -> float:
        rotation = self._single_axis_rotation(angle, axis)
        rotated = self.reference @ rotation.T
        source = self._voxels(rotated)
        union = source | self.target_voxels
        return len(source & self.target_voxels) / len(union) if union else 0.0

    def _prepare(self, cloud: np.ndarray) -> np.ndarray:
        points = np.asarray(cloud, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError("点云格式错误，必须是 N×3")
        if len(points) > self.max_points:
            points = points[np.linspace(0, len(points) - 1, self.max_points, dtype=np.int64)]
        return points

    def _voxels(self, cloud_m: np.ndarray) -> set[tuple[int, int, int]]:
        return set(map(tuple, np.round(cloud_m / self.shape_bin_size).astype(np.int32)))

    @staticmethod
    def _single_axis_rotation(angle: float, axis: str) -> np.ndarray:
        return YoloUtils.rotation_matrix(np.deg2rad(angle) if axis == "roll" else 0.0, np.deg2rad(angle) if axis == "pitch" else 0.0, np.deg2rad(angle) if axis == "yaw" else 0.0)


__all__ = ["YoloUtils", "PointCloudRotationMatcher"]
