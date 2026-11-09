"""Joint_EEFPose_Control 能力：末端执行器位姿控制。

输入参数仅保留与 publisher 消息构造相关的字段；
roll/pitch/yaw 按 Atom 转换为四元数写入 PoseStamped。
"""

from __future__ import annotations

from .backends.base import RobotBackend
from .utils.types import CommandHandle, Pose
from .utils.utils_common import euler_to_quaternionx


class EEFControl:
    """R1 Pro 末端执行器位姿控制。"""

    def __init__(self, backend: RobotBackend) -> None:
        self._backend = backend

    def control_single_arm(
        self,
        *,
        x: float, y: float, z: float,
        roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
        frame_id: str = "torso_link4",
        axis: str = "left",
    ) -> CommandHandle:
        quat = tuple(float(value) for value in euler_to_quaternionx([roll, pitch, yaw], seq="yxz"))
        return self._backend.start_command(
            "eef_control", "control_single_arm",
            {
                "axis": axis,
                "target": Pose(
                    frame_id=frame_id,
                    position=(x, y, z),
                    orientation_xyzw=quat,
                ),
            },
        )

    def control_both_arm(
        self,
        *,
        left_x: float, left_y: float, left_z: float,
        left_roll: float, left_pitch: float, left_yaw: float,
        right_x: float, right_y: float, right_z: float,
        right_roll: float, right_pitch: float, right_yaw: float,
        left_frame_id: str = "1", right_frame_id: str = "1",
    ) -> list:
        """双臂末端控制：左右臂异步多线程并行提交（借鉴 Atom）。"""

        import threading

        results: dict[str, CommandHandle] = {}

        def _run(side: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float, frame_id: str) -> None:
            results[side] = self.control_single_arm(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, frame_id=frame_id, axis=side)

        left_thread = threading.Thread(
            target=_run, args=("left", left_x, left_y, left_z, left_roll, left_pitch, left_yaw, left_frame_id), daemon=True
        )
        right_thread = threading.Thread(
            target=_run, args=("right", right_x, right_y, right_z, right_roll, right_pitch, right_yaw, right_frame_id), daemon=True
        )
        left_thread.start()
        right_thread.start()
        left_thread.join()
        right_thread.join()

        return [results["left"], results["right"]]

