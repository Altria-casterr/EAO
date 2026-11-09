"""功能函数层：Ability 层工具函数。"""

from __future__ import annotations

import logging
import os
import time as _time
from pathlib import Path

import numpy as np
import yaml
from typing import Any
from scipy.spatial.transform import Rotation as R

logger = logging.getLogger(__name__)


def check_reached(current: list[float], target: list[float], tol: list[float], description: str = "到位检测") -> bool:
    """检查当前值是否在目标容差范围内。"""
    if len(current) != len(target) or len(current) != len(tol):
        logger.warning(f"{description} 长度不匹配: current={len(current)}, target={len(target)}, tol={len(tol)}")
        return False
    
    for i, (c, t, tolerance) in enumerate(zip(current, target, tol)):
        if abs(c - t) > tolerance:
            logger.debug(f"{description} 第 {i} 个未到位: current={c}, target={t}, tol={tolerance}, diff={abs(c-t)}")
            return False
    
    logger.debug(f"{description} 已到位")
    return True

def find_cr(ability_name: str, ability_path: str | None = None) -> str | None:
    """查找能力的 CR 配置文件。"""
    if ability_path is not None:
        search_root = os.path.join(ability_path, "crs")
        candidates = (
            os.path.join(search_root, filename)
            for filename in os.listdir(search_root)
            if filename.endswith(".cr.yaml")
        ) if os.path.isdir(search_root) else ()
    else:
        search_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "abilities"))
        candidates = (
            os.path.join(root, filename)
            for root, _, filenames in os.walk(search_root)
            for filename in filenames
            if filename.endswith(".cr.yaml")
        )

    for candidate in candidates:
        try:
            data = yaml.safe_load(Path(candidate).read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError):
            continue
        spec = data.get("spec") or {}
        if spec.get("abilityName") == ability_name:
            return candidate
    return None


def check_reached(
    current_values: Sequence[Any],
    target_values: Sequence[Any],
    tolerances: Sequence[Any],
    description: str = "位置到达",
) -> bool:
    """判断当前值是否全部到达目标值（误差在容差内，借鉴 Atom Func.check_stop）。

    Args:
        current_values: 当前值列表（如夹爪实际开度、关节实际位置）
        target_values: 目标值列表
        tolerances: 容差列表（与 target_values 一一对应）
        description: 描述信息（用于日志）

    Returns:
        bool: 是否全部到达目标位置
    """

    if len(current_values) != len(target_values) or len(tolerances) != len(target_values):
        logger.error(
            "%s：数量不匹配（当前=%d 目标=%d 容差=%d）",
            description,
            len(current_values),
            len(target_values),
            len(tolerances),
        )
        return False

    errors = [abs(float(current) - float(target)) for current, target in zip(current_values, target_values)]
    error_str = ", ".join(f"{i + 1}={error:.4f}" for i, error in enumerate(errors))
    logger.info("%s误差：%s", description, error_str)

    all_reached = all(error <= tolerance for error, tolerance in zip(errors, tolerances))
    if all_reached:
        logger.info("%s已到达目标位置", description)
    return all_reached

def euler_to_quaternionx(euler_angles: list[float],seq: str = "xyz") -> "np.ndarray":
    """欧拉角（度）转四元数（对齐 Atom Utils.euler_to_quaternion，scipy 实现）。

    输入 [roll, pitch, yaw]（度），按 seq 顺序（默认 xyz，EEF 使用 yxz）转四元数；
    返回 numpy 数组 (qx, qy, qz, qw)，并做与目标四元数一致的符号归一化。
    """

    euler_angles = np.asarray(euler_angles, dtype=np.float64)
    euler_rad = np.radians(euler_angles)
    rot = R.from_euler(seq.upper(), euler_rad, degrees=False)
    quat = rot.as_quat()

    logger.warning("欧拉角转四元数，输入：%s，输出：%s", euler_angles, quat)

    target_quat = np.array([0.0, -0.70682518, 0.0, 0.70738827], dtype=np.float64)
    if np.dot(quat, target_quat) < 0:
        quat = -quat
    return quat
