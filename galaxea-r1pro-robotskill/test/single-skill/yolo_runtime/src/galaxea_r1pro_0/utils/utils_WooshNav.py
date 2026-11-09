from typing import Any

from scipy.spatial.transform import Rotation as R


def quat_to_yaw_deg(orientation_xyzw: Any) -> float:
    """semantic 四元数(xyzw) → 绕 Z 轴 yaw(度)，供底盘导航使用。"""

    try:
        values = [float(value) for value in orientation_xyzw]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"orientation_xyzw 必须是四元数: {orientation_xyzw}") from exc
    if len(values) != 4:
        raise ValueError(f"orientation_xyzw 必须包含 4 个元素: {orientation_xyzw}")
    _, _, yaw = R.from_quat(values).as_euler("xyz", degrees=True)
    return float(yaw)
