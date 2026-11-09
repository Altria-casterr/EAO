"""功能函数层：跨模块复用的通用工具函数。"""

from __future__ import annotations

import logging
import time as _time
import numpy as np

from scipy.spatial.transform import Rotation as R
from typing import Any
from .errors import ProfileError

logger = logging.getLogger(__name__)

DEFAULT_ROS_PORT = 9091


def parse_endpoint(endpoint: str | None,default_port: int = DEFAULT_ROS_PORT) -> tuple[str, int]:
    """把机器人 endpoint 解析为 (host, port)。

    支持 "192.168.1.100:8080"、"ros2://192.168.1.100:9091" 等格式；
    未带端口时使用 default_port（默认 9091）。

    Raises:
        ProfileError: endpoint 为空、缺少主机或端口非法。
    """

    if not endpoint:
        raise ProfileError(
            field="endpoint",
            reason=ProfileError.INVALID_ENDPOINT,
            hint="RobotProfile.endpoint 不能为空",
        )

    text = str(endpoint).strip()
    for scheme in ("ros2://", "ros://", "ws://", "wss://", "http://", "https://", "tcp://"):
        if text.startswith(scheme):
            text = text[len(scheme):]
            break

    if ":" in text:
        host, port_text = text.rsplit(":", 1)
        host = host.strip()
        if not host:
            raise ProfileError(
                field="endpoint",
                value=endpoint,
                reason=ProfileError.INVALID_ENDPOINT,
                hint="endpoint 缺少主机地址",
            )
        try:
            port = int(port_text)
        except ValueError:
            raise ProfileError(
                field="endpoint",
                value=endpoint,
                reason=ProfileError.INVALID_ENDPOINT,
                hint=f"endpoint 端口不是整数: {port_text!r}",
            ) from None
        if not (0 < port < 65536):
            raise ProfileError(
                field="endpoint",
                value=endpoint,
                reason=ProfileError.INVALID_ENDPOINT,
                hint=f"endpoint 端口越界: {port}",
            )
        return host, port

    return text, default_port

def create_timestamp_message(frame_id: str = "") -> dict:
    """构造 ROS header 时间戳消息（不依赖 roslibpy）。"""

    now = _time.time()
    seconds = int(now)
    return {
        "stamp": {"sec": seconds, "nanosec": int((now - seconds) * 1e9)},
        "frame_id": frame_id,
    }

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

def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float, seq: str = "xyz") -> tuple[float, float, float]:
    """四元数（xyzw）转欧拉角，返回 (roll, pitch, yaw)，单位为弧度（scipy 实现）。"""

    roll, pitch, yaw = R.from_quat([float(qx), float(qy), float(qz), float(qw)]).as_euler(seq, degrees=False)
    return float(roll), float(pitch), float(yaw)

def normalize_frame_name(frame_name: str | None) -> str | None:
    """统一坐标系名称，消除前导/尾随斜杠差异（对齐 Atom Utils.normalize_frame_name）。"""

    if frame_name is None:
        return None
    return str(frame_name).strip().strip("/")

def inverse_transform(transform: dict) -> dict:
    """计算 TF 变换（translation/rotation 字典）的逆变换（对齐 Atom Utils.inverse_transform）。"""

    rotation = transform["rotation"]
    translation = transform["translation"]
    inv_rotation = R.from_quat([
        float(rotation["x"]),
        float(rotation["y"]),
        float(rotation["z"]),
        float(rotation["w"]),
    ]).inv()
    inv_translation = -inv_rotation.apply([
        float(translation["x"]),
        float(translation["y"]),
        float(translation["z"]),
    ])
    inv_quat = inv_rotation.as_quat()
    return {
        "translation": {
            "x": float(inv_translation[0]),
            "y": float(inv_translation[1]),
            "z": float(inv_translation[2]),
        },
        "rotation": {
            "x": float(inv_quat[0]),
            "y": float(inv_quat[1]),
            "z": float(inv_quat[2]),
            "w": float(inv_quat[3]),
        },
    }


def compute_chain_transform(first_transform: dict, second_transform: dict) -> dict:
    """组合两个连续变换 A->B 与 B->C，得到 A->C（对齐 Atom Utils.compute_chain_transform）。"""

    first_rotation = R.from_quat([
        float(first_transform["rotation"]["x"]),
        float(first_transform["rotation"]["y"]),
        float(first_transform["rotation"]["z"]),
        float(first_transform["rotation"]["w"]),
    ])
    first_translation = np.asarray([
        float(first_transform["translation"]["x"]),
        float(first_transform["translation"]["y"]),
        float(first_transform["translation"]["z"]),
    ], dtype=np.float64)
    second_rotation = R.from_quat([
        float(second_transform["rotation"]["x"]),
        float(second_transform["rotation"]["y"]),
        float(second_transform["rotation"]["z"]),
        float(second_transform["rotation"]["w"]),
    ])
    second_translation = np.asarray([
        float(second_transform["translation"]["x"]),
        float(second_transform["translation"]["y"]),
        float(second_transform["translation"]["z"]),
    ], dtype=np.float64)

    composed_rotation = second_rotation * first_rotation
    composed_translation = second_rotation.apply(first_translation) + second_translation
    composed_quat = composed_rotation.as_quat()
    return {
        "translation": {
            "x": float(composed_translation[0]),
            "y": float(composed_translation[1]),
            "z": float(composed_translation[2]),
        },
        "rotation": {
            "x": float(composed_quat[0]),
            "y": float(composed_quat[1]),
            "z": float(composed_quat[2]),
            "w": float(composed_quat[3]),
        },
    }

def find_transform_path(source_frame: str, tf_cache: dict) -> dict | None:
    """基于 TF 缓存查找 source_frame 到根系的变换路径（对齐 Atom Utils.find_transform_path）。"""

    current = source_frame
    visited: set[str] = set()
    total_transform = {
        "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }
    while True:
        if current in visited:
            return None
        visited.add(current)
        if current in tf_cache:
            parent = tf_cache[current]["parent_frame"]
            edge_transform = {
                "translation": tf_cache[current]["translation"],
                "rotation": tf_cache[current]["rotation"],
            }
            total_transform = compute_chain_transform(total_transform, edge_transform)
            current = parent
            continue
        break
    total_transform["root_frame"] = current
    return total_transform


def safe_close(connection: Any, method: str = "close", name: str = "连接") -> bool:
    """安全调用连接对象的关闭方法；异常只记录不抛出。"""

    closer = getattr(connection, method, None)
    if closer is None:
        logger.warning("%s 没有 %s 方法", name, method)
        return False
    try:
        result = closer()
        logger.info("%s 已关闭", name)
        return result is not False
    except Exception as exc:
        logger.error("%s 关闭失败: %s", name, exc)
        return False
