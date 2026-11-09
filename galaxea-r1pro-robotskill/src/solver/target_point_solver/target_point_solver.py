"""目标点坐标系转换求解器（借鉴 Yolo_Seg/scripts/main_v2.py L607-L625）。

纯规划模块：解析 SDK 返回的两路 CommandResult，将世界坐标系目标点转换到机器人相对坐标系。
两路 result 独立输入，内部按 _floating_base 模式提取 data：
    data = (result.output or {}).get("data")
- transformer_martix_result: sdk.state.get_transformer_martix() 的返回
- floating_base_result: sdk.state.get_floating_base() 的返回

data 格式（与 utils_JointEEFPoseControl.transform_pose 一致）：
    {"position": {"x","y","z"}, "orientation": {"x","y","z","w"}}

trans_flag=True 执行坐标系转换；trans_flag=False 仅做米→毫米缩放。
"""

from __future__ import annotations

import logging
from typing import Any, Mapping, Sequence

import numpy as np

logger = logging.getLogger(__name__)


class TargetPointSolver:
    """目标点坐标系转换求解器。

    solve() 接收两路 SDK CommandResult 作为独立入参，内部按
    ``data = (result.output or {}).get("data")`` 提取 data（借鉴
    utils_JointEEFPoseControl._floating_base）。
    """

    @staticmethod
    def _extract_data(result: Any) -> dict | None:

        if isinstance(result, Mapping):
            return result if "position" in result else None

        # CommandResult：提取 .output.data
        data = (getattr(result, "output", None) or {}).get("data")
        return data if isinstance(data, dict) else None

    @staticmethod
    def _parse_position(data: Mapping[str, Any]) -> tuple[float, float, float]:

        pos = data["position"]
        if isinstance(pos, Mapping):
            return float(pos["x"]), float(pos["y"]), float(pos["z"])
        return float(pos[0]), float(pos[1]), float(pos[2])

    @staticmethod
    def _parse_orientation(data: Mapping[str, Any]) -> tuple[float, float, float, float]:

        ori = data["orientation"]
        if isinstance(ori, Mapping):
            return float(ori["x"]), float(ori["y"]), float(ori["z"]), float(ori["w"])
        return float(ori[0]), float(ori[1]), float(ori[2]), float(ori[3])

    @staticmethod
    def _quat_to_rotation(quat: Sequence[float]) -> np.ndarray:
        """四元数 (qx, qy, qz, qw) → 3x3 旋转矩阵（纯 numpy 实现）。

        scipy.spatial.transform.Rotation.as_quat() 返回 (x, y, z, w) 顺序，
        与 SDK 一致；此处按 Hamilton 右手系标准公式构造。
        """

        if len(quat) != 4:
            raise ValueError("四元数必须是长度为4的列表：[qx, qy, qz, qw]")

        qx, qy, qz, qw = quat
        norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
        if norm == 0.0:
            logger.warning("四元数模长为0，返回单位旋转矩阵")
            return np.eye(3, dtype=np.float64)
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        return np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
        ], dtype=np.float64)

    @classmethod
    def _pose_to_matrix(
        cls,
        position: Sequence[float],
        orientation: Sequence[float],
    ) -> np.ndarray:
        """position + orientation(四元数) → 4x4 齐次变换矩阵。"""

        if len(position) != 3:
            raise ValueError("position 必须是长度为3的列表：[x, y, z]")

        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = cls._quat_to_rotation(orientation)
        matrix[:3, 3] = position
        return matrix

    def solve(
        self,
        target_point: Sequence[float] | None,
        transformer_martix_result: Any | None = None,
        floating_base_result: Any | None = None,
        trans_flag: bool = False,
        mm_scale: float = 1.0,
    ) -> list[float] | None:
        """转换目标点坐标。

        Args:
            target_point: 目标点 [x, y, z]（米）；为 None 时返回 None。
        Returns:
            转换后的点列表（毫米）；输入为 None 或 data 缺失时返回 None。
        """

        if target_point is None:
            raise ValueError("target_point 不能为空")

        if not trans_flag:
            points_world = [item * mm_scale for item in target_point]
            logger.warning("初始点和经过坐标系转换后的点分别是%s, %s", target_point, points_world)
            return points_world

        # trans_flag=True: 世界坐标系 → 机器人相对坐标系
        logger.warning("当前进行坐标转换，将世界坐标系转换为机器人相对坐标系")

        # 借鉴 _floating_base：data = (result.output or {}).get("data")
        tm_data = self._extract_data(transformer_martix_result)
        
        if tm_data is None:
            logger.error("trans_flag=True 但未能从 transformer_martix_result 提取 data")
            raise ValueError("未能从 transformer_martix_result 提取 data")

        fb_data = self._extract_data(floating_base_result)
        if fb_data is None:
            logger.error("trans_flag=True 但未能从 floating_base_result 提取 data")
            raise ValueError("未能从 floating_base_result 提取 data")

        target_point_arr = np.array(target_point, dtype=np.float64)

        # map2body 转换坐标的路线（map-odom-body）
        base_to_world_position = self._parse_position(tm_data)
        base_to_world_orientation = self._parse_orientation(tm_data)
        base_to_world_matrix = self._pose_to_matrix(
            base_to_world_position, base_to_world_orientation
        )

        # floating_base 的 z 分量用于目标点 z 轴偏移
        _, _, floating_base_z = self._parse_position(fb_data)

        points_world = (
            target_point_arr @ base_to_world_matrix[:3, :3].T
            + base_to_world_matrix[:3, 3]
        )
        points_world[2] = target_point_arr[2] - abs(floating_base_z)
        points_world = (points_world * mm_scale).tolist()

        logger.warning("初始点和经过坐标系转换后的点分别是%s, %s", target_point, points_world)
        return points_world
