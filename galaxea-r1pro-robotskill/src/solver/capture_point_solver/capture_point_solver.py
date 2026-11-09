"""抓取点求解器（借鉴 Grab_Up/Galaxea_R1Pro/scripts/main.py::get_capture_point）。

纯规划模块：输入箱子中心点与目标手臂，输出抓取点 (x, y, z)。
不依赖 Robot SDK，可独立单测；双手模式额外返回 right_y。
另含 move_box_to_center：根据当前双臂抓取位姿计算箱子移至机器人中心的终点位姿。
get_capture_point_place：放置预定位点求解（借鉴 Put_Down/Agibot_G2），
分左手、右手、双手三种 axis；双手默认按左手计算，另一手按 box_long 推出。
"""

from __future__ import annotations

import logging
import random
from typing import Any, List, Mapping

import numpy as np
from scipy.spatial.transform import Rotation as R

logger = logging.getLogger(__name__)

# 抓取点固定姿态（roll, pitch, yaw，单位：度）：左臂 (-90, -30, 0)、右臂 (-90, 30, 0)。
# 欧拉顺序与 EEF 控制保持一致（yxz），保证转出的四元数与底层下发口径相同。
LEFT_CAPTURE_RPY = (-90.0, -30.0, 0.0)
RIGHT_CAPTURE_RPY = (-90.0, 30.0, 0.0)
CAPTURE_EULER_SEQ = "yxz"


class CapturePointSolver:
    """抓取点求解器：根据箱子中心点与手臂侧别计算抓取点。

    左右臂 y 方向界限基于 x=0.3、z=0.0 位置标定；
    box_long 为夹爪末端量程内的箱体长度（米）。
    box_height / box_width 用于 move_box_to_center 计算终点位姿。
    """

    def __init__(
        self,
        box_long: float,
        box_height: float | None = None,
        box_width: float | None = None,
        randomize_capture_y: bool = True,
        left_leftmost_limit: float = 0.5,
        left_rightmost_limit: float = 0.0,
        right_leftmost_limit: float = 0.0,
        right_rightmost_limit: float = -0.5) -> None:
        self.box_long = box_long
        self.box_height = box_height
        self.box_width = box_width
        self.randomize_capture_y = randomize_capture_y
        self.left_leftmost_limit = left_leftmost_limit
        self.left_rightmost_limit = left_rightmost_limit
        self.right_leftmost_limit = right_leftmost_limit
        self.right_rightmost_limit = right_rightmost_limit

    @staticmethod
    def _biased_random(min: float, max: float) -> float:
        """三角分布随机数（两个均匀分布取平均后映射到 [min, max]）。

        中间值概率最高，适合在可达范围内偏置选取抓取点。
        """

        u1 = random.random()
        u2 = random.random()
        normalized_value = (u1 + u2) / 2.0
        return min + normalized_value * (max - min)

    def _solve_left_y(
        self,
        box_middle_point: Mapping[str, float],
        safe_offset: float) -> float | None:
        """左臂抓取 y 坐标：箱体左缘不能超过左臂最左界限。"""

        left_left = self.left_leftmost_limit
        left_right = self.left_rightmost_limit
        box_left_y = box_middle_point["y"] + (self.box_long / 2) + safe_offset

        if box_left_y > left_left:
            logger.warning(
                "The box left position %s exceeds left arm leftmost limit %s, "
                "please check box middle position",
                box_left_y,
                self.left_leftmost_limit,
            )
            return None

        if box_left_y > left_right:
            left_right = box_left_y

        # 真机双手预备点需要与后续 force_align 的最大行程一致。调用方可关闭
        # 随机采样，确定性地选取最靠近箱侧的安全点。
        y = round(
            self._biased_random(min=left_right, max=left_left)
            if self.randomize_capture_y else left_right,
            2,
        )
        logger.info("left_right: %s, left_left: %s", left_right, left_left)
        return y

    def _solve_right_y(
        self,
        box_middle_point: Mapping[str, float],
        safe_offset: float) -> float | None:
        """右臂抓取 y 坐标：箱体右缘不能超过右臂最右界限。"""

        right_left = self.right_leftmost_limit
        right_right = self.right_rightmost_limit
        box_right_y = box_middle_point["y"] - (self.box_long / 2) - safe_offset

        if box_right_y < right_right:
            logger.warning(
                "The box right position %s exceeds right arm rightmost limit %s, "
                "please check box middle position",
                box_right_y,
                self.right_rightmost_limit,
            )
            return None

        if box_right_y < right_left:
            right_left = box_right_y

        y = round(
            self._biased_random(min=right_right, max=right_left)
            if self.randomize_capture_y else right_left,
            2,
        )
        logger.info("right_left: %s, right_right: %s", right_left, right_right)
        return y

    @staticmethod
    def _parse_current_pose(data: Mapping[str, Any] | None) -> dict[str, Any] | None:

        if not isinstance(data, Mapping):
            return None

        pose = data.get("pose") or {}
        position = pose.get("position") or {}
        orientation = pose.get("orientation") or {}
        try:
            values: dict[str, Any] = {
                "x": float(position["x"]),
                "y": float(position["y"]),
                "z": float(position["z"]),
                "qx": float(orientation.get("x", 0.0)),
                "qy": float(orientation.get("y", 0.0)),
                "qz": float(orientation.get("z", 0.0)),
                "qw": float(orientation.get("w", 1.0)),
            }
        except (KeyError, TypeError, ValueError):
            return None

        values["frame_id"] = str(data.get("header", {}).get("frame_id", "")).strip() or "torso_link4"
        return values

    def euler_to_quaternion(self, euler_angles: List[float], seq :str ='xyz') -> np.ndarray:
        """欧拉角转四元数（与标准逻辑对齐）"""
        euler_angles = np.asarray(euler_angles, dtype=np.float64)
        euler_rad = np.radians(euler_angles)
        rot = R.from_euler(seq.upper(), euler_rad, degrees=False)
        quat = rot.as_quat()

        logging.warning(f"欧拉角转四元数，输入：{euler_angles}，输出：{quat}")

        target_quat = np.array([0.0, -0.70682518, 0.0, 0.70738827], dtype=np.float64)

        if np.dot(quat, target_quat) < 0:
            quat = -quat

        return quat

    @staticmethod
    def _quat_fields(quat: np.ndarray) -> dict[str, float]:
        qx, qy, qz, qw = (float(value) for value in quat)
        return {"qx": qx, "qy": qy, "qz": qz, "qw": qw}

    def get_capture_point(
        self,
        left_data: Mapping[str, Any] | None = None,
        right_data: Mapping[str, Any] | None = None,
        box_middle_point: Mapping[str, float] | None = None,
        axis: str = "left",
        z_lift: float = 0.05,
        safe_offset: float = 0.07) -> dict[str, Any] | None:
        """计算抓取点，输出 target 的 xyz + 固定姿态四元数。

        姿态不再沿用当前末端位姿，而是固定的 roll/pitch/yaw（度）转四元数：
        左臂 (-90, -30, 0)、右臂 (-90, 30, 0)，欧拉顺序与 EEF 控制一致（yxz）。

        Args:
            left_data: 左臂 ee_pose data（保留参数；姿态已固定，不再使用）。
            right_data: 右臂 ee_pose data（保留参数；姿态已固定，不再使用）。
            box_middle_point: 箱子中心点 {"x", "y", "z"}（米）。
            axis: "left" / "right" / "both"。
            z_lift: 抓取点 z 方向抬升量（米）。
            safe_offset: 箱体侧边安全余量（米）。

        Returns:
            left:  {"x", "y", "z", "qx", "qy", "qz", "qw"}
            right: 同上
            both:  {"left": {...}, "right": {...}}
            越界或参数非法时返回 None。
        """

        if box_middle_point is None:
            raise ValueError("box_middle_point is None")

        x = box_middle_point["x"]
        z = box_middle_point["z"] + z_lift
        left_quat = self._quat_fields(self.euler_to_quaternion(list(LEFT_CAPTURE_RPY), CAPTURE_EULER_SEQ))
        right_quat = self._quat_fields(self.euler_to_quaternion(list(RIGHT_CAPTURE_RPY), CAPTURE_EULER_SEQ))

        if axis == "left":
            y = self._solve_left_y(box_middle_point, safe_offset)
            if y is None:
                raise ValueError("left arm y-axis position is None")

            logger.info("Current left grasp Y-axis position: %s", y)
            return {"x": x, "y": y, "z": z, **left_quat}

        if axis == "right":
            y = self._solve_right_y(box_middle_point, safe_offset)
            if y is None:
                raise ValueError("right arm y-axis position is None")

            logger.info("Current right grasp Y-axis position: %s", y)
            return {"x": x, "y": y, "z": z, **right_quat}

        if axis == "both":
            left_y = self._solve_left_y(box_middle_point, safe_offset)
            if left_y is None:
                raise ValueError("left arm y-axis position is None")
            right_y = self._solve_right_y(box_middle_point, safe_offset)
            if right_y is None:
                raise ValueError("right arm y-axis position is None")

            logger.info("Current left grasp Y-axis position: %s", left_y)
            logger.info("Current right grasp Y-axis position: %s", right_y)
            return {
                "left": {"x": x, "y": left_y, "z": z, **left_quat},
                "right": {"x": x, "y": right_y, "z": z, **right_quat},
            }

        logger.warning("Invalid axis parameter: %s", axis)
        return None

    def get_capture_point_place(
        self,
        box_middle_point: Mapping[str, float] | None = None,
        axis: str = "both",
        put_mode: str = "left",
        safe_offset_x: float = 0.1,
        safe_offset_z: float = 0.1,
        safe_offset_y: float = 0.0) -> dict[str, Any] | None:
        """计算放置预定位点（借鉴 Put_Down/Agibot_G2/scripts/main.py 的
        get_capture_point 与 move_to_capture_point）。

        预定位：x = 箱心 x - 0.03（后退），z = 箱心 z + 0.05（抬高）；
        计算手 y 取箱缘外 0.1~0.2 m 安全带（randomize_capture_y=False 时取
        最靠近箱缘的安全点），y_offset = 箱缘 y - 计算手 y，即后续推箱行程；
        x_offset=0.05 / z_offset=-0.02 沿用参考实现，供双手 X/Z 卡位使用。
        """

        if box_middle_point is None:
            raise ValueError("box_middle_point is None")

        if axis not in ("left", "right", "both"):
            logger.warning("Invalid axis parameter: %s", axis)
            return None

        if axis == "both" and put_mode not in ("left", "right"):
            logger.warning("Invalid put_mode parameter: %s", put_mode)
            return None

        safe_offset = safe_offset_y
        x = box_middle_point["x"] + safe_offset_x
        z = box_middle_point["z"] + safe_offset_z
        left_quat = self._quat_fields(self.euler_to_quaternion(list(LEFT_CAPTURE_RPY), CAPTURE_EULER_SEQ))
        right_quat = self._quat_fields(self.euler_to_quaternion(list(RIGHT_CAPTURE_RPY), CAPTURE_EULER_SEQ))

        def solve(put: str) -> tuple[float, float] | None:
            """单手 y 与 y_offset（箱缘 y - 抓取 y）；无可行安全带时返回 None。"""

            if put == "left":
                edge_y = box_middle_point["y"] + self.box_long / 2
                y_near = max(self.left_rightmost_limit, edge_y + safe_offset)
                y_far = min(self.left_leftmost_limit, edge_y + safe_offset * 2)
                low, high = y_near, y_far

            else:
                edge_y = box_middle_point["y"] - self.box_long / 2
                y_near = min(self.right_leftmost_limit, edge_y - safe_offset)
                y_far = max(self.right_rightmost_limit, edge_y - safe_offset * 2)
                low, high = y_far, y_near

            if low > high:
                logger.warning(
                    "The box %s edge %s leaves no safe band within arm limits, "
                    "please check box middle position",
                    put,
                    edge_y,
                )
                return None

            y = round(
                self._biased_random(min=low, max=high)
                if self.randomize_capture_y else y_near,
                2,
            )
            logger.info("Current %s place grasp Y-axis position: %s", put, y)
            return y, edge_y - y

        if axis in ("left", "right"):
            solved = solve(axis)

            if solved is None:
                return None

            y, y_offset = solved

            quat = left_quat if axis == "left" else right_quat
            logger.warning("current place x ,y ,z is %s, %s, %s", x, y, z)
            return {"x": x, "y": y, "z": z, **quat,
                    "x_offset": 0.03, "y_offset": y_offset, "z_offset": -0.05}

        solved = solve(put_mode)

        if solved is None:
            return None

        primary_y, y_offset = solved

        if put_mode == "left":
            left_y, right_y = primary_y, primary_y - self.box_long
        else:
            left_y, right_y = primary_y + self.box_long, primary_y
        logger.info("Current both place Y-axis positions: left=%s, right=%s", left_y, right_y)
        logger.warning("current place x ,y ,z is %s, %s, %s", x, left_y, z)
        return {
            "left": {"x": x, "y": left_y, "z": z, **left_quat},
            "right": {"x": x, "y": right_y, "z": z, **right_quat},
            "x_offset": 0.03, "y_offset": y_offset, "z_offset": -0.05,
        }

    def move_to_retract_point(
        self,
        current_data: Mapping[str, Any] | None,
        box_middle_y: float,
        axis: str = "left",
        offset_z: float = 0.02,
        save_offset_y: float = 0.03,
        save_offset_x: float = 0.1,
        safe_offset_z: float = 0.1,
    ) -> dict[str, dict[str, float]] | None:
        """借鉴 Put_Down/Agibot_G2/scripts/main.py 的 move_to_retract_point，
        计算单手撤出的三个目标点：z（下降脱离）、y（沿箱缘外撤）、x（向后回撤）。

        沿用参考实现的外撤距离公式：y_offset = abs(箱缘 Y) - abs(当前 Y)，
        target_y_offset = max(abs(y_offset)/2, save_offset_y)；
        左手沿 +Y、右手沿 -Y 外撤。姿态保持当前四元数不变。

        第三步 x_point 的目标高度为撤手开始高度 + safe_offset_z，X 沿负方向
        回撤 save_offset_x（离开箱体前侧），Y 保持第二步外撤后的状态。

        Args:
            current_data: 当前手 ee_pose data（含 pose.position / orientation）。
            box_middle_y: 箱体中心 Y（torso_link4，米）。
            axis: "left" / "right"。
            offset_z: 第一段下降量（米），默认 0.02。
            save_offset_y: Y 外撤最小安全距离（米），默认 0.03。
            save_offset_x: 第三步 X 向后回撤量（米），默认 0.1。
            safe_offset_z: 第三步目标 Z 相对撤手开始高度的增量（米），默认 0.1。

        Returns:
            {"z": pose, "y": pose, "x": pose}，每个 pose 含
            x/y/z/qx/qy/qz/qw；参数非法或位姿无法解析时返回 None。
        """

        if axis not in ("left", "right"):
            logger.warning("Invalid axis parameter: %s", axis)
            return None

        current = self._parse_current_pose(current_data)
        if current is None:
            logger.error("未能从 current_data 解析当前位姿")
            return None

        sign = 1 if axis == "left" else -1
        edge_y = box_middle_y + sign * self.box_long / 2
        y_offset = abs(edge_y) - abs(current["y"])
        target_y_offset = max(abs(y_offset) / 2.0, save_offset_y)

        current_x, current_y, current_z = current["x"], current["y"], current["z"]
        target_z = current_z - offset_z
        target_y = current_y + sign * save_offset_y
        exit_x = current_x - save_offset_x
        exit_z = current_z + safe_offset_z

        quat = {k: current[k] for k in ("qx", "qy", "qz", "qw")}

        z_point = {"x": current_x, "y": current_y, "z": target_z, **quat}
        y_point = {"x": current_x, "y": target_y, "z": target_z, **quat}
        x_point = {"x": exit_x, "y": target_y, "z": exit_z, **quat}

        logger.info(
            "Move to retract point: %s, z=%s -> y=%s -> x(back)=%s",
            axis, target_z, target_y, exit_x,
        )
        return {"z": z_point, "y": y_point, "x": x_point}

    def move_box_to_center(
        self,
        left_data: Mapping[str, Any] | None,
        right_data: Mapping[str, Any] | None,
        end_pose_z_offset: float = 0.2,
        end_pose_x_offset: float = 0.2) -> dict[str, dict[str, float]] | None:

        left = self._parse_current_pose(left_data)
        if left is None:
            logger.error("未能从 left_data 解析当前位姿")
            return None
        right = self._parse_current_pose(right_data)
        if right is None:
            logger.error("未能从 right_data 解析当前位姿")
            return None

        left_y = left["y"]
        right_y = right["y"]

        # 箱子目标位置 z 坐标（默认 0.3 = box_height/2 + 0.2）
        # end_pose_z = (self.box_height or 0.1) / 2 + end_pose_z_offset
        end_pose_z = left["z"]
        # 箱子目标位置 x 坐标（默认 0.4 = box_width/2 + 0.2）
        # end_pose_x = (self.box_width or 0.4) / 2 + end_pose_x_offset
        end_pose_x = left["x"] - ( (self.box_width or 0.4) / 4 )
        # 通过左右臂间距计算箱子目标位置 y 坐标（中心）
        # end_pose_y = (left_y - right_y) / 2
        end_pose_y = (left_y - right_y) / 2

        # 左臂：保留当前姿态四元数
        left_target = {
            "x": end_pose_x, "y": end_pose_y, "z": end_pose_z,
            "qx": left["qx"], "qy": left["qy"], "qz": left["qz"], "qw": left["qw"],
        }
        # 右臂：保留当前姿态四元数
        right_target = {
            "x": end_pose_x, "y": -end_pose_y, "z": end_pose_z,
            "qx": right["qx"], "qy": right["qy"], "qz": right["qz"], "qw": right["qw"],
        }

        logger.info("Left target=%s", left_target)
        logger.info("Right target=%s", right_target)

        return {
            "left": {"target": left_target},
            "right": {"target": right_target},
        }

    def get_other_arm_position(
        self,
        axis: str,
        grasp_data: Mapping[str, Any] | None,
        other_data: Mapping[str, Any] | None,
        init_x: float,
        safe_y_move: float = 0.1,
        z_lift: float = 0.1) -> dict[str, Any] | None:
        """
        Args:
            axis: 抓取侧 "left" / "right"。
            grasp_data: 抓取侧手当前 ee_pose data（取当前 x/y/z）。
            other_data: 另一只手 ee_pose data（提供当前四元数）。
            init_x: 另一只手 y 向预定位的 x。
            safe_y_move: y 方向安全余量（米），默认 0.1。
            z_lift: 抓取点相对抓取手当前 z 的下降量（米），默认 0.1。

        Returns:
            {"axis", "y_move", "target"}，位姿为 xyz + 四元数；
            参数非法或 data 无法解析时返回 None。
        """

        if axis not in ("left", "right"):
            logger.warning("Invalid axis parameter: %s", axis)
            return None

        grasp = self._parse_current_pose(grasp_data)
        if grasp is None:
            logger.error("未能从 grasp_data 解析当前位姿")
            return None

        grasp_y = grasp["y"]

        if axis == "left":
            other_axis = "right"
            other_y = grasp_y - self.box_long - safe_y_move
            quat_ = self._quat_fields(self.euler_to_quaternion(list(RIGHT_CAPTURE_RPY), CAPTURE_EULER_SEQ))

        else:

            other_axis = "left"
            other_y = grasp_y + self.box_long + safe_y_move
            quat_ = self._quat_fields(self.euler_to_quaternion(list(LEFT_CAPTURE_RPY), CAPTURE_EULER_SEQ))

        # other = self._parse_current_pose(other_data)
        # if other is None:
        #     logger.error("未能从 other_data 解析当前位姿")
        #     return None

        quat = {"qx": quat_["qx"], "qy": quat_["qy"], "qz": quat_["qz"], "qw": quat_["qw"]}

        # target 与抓取手当前 x 相同，z 为当前 z 减去抬升量
        capture_x = grasp["x"]
        capture_z = grasp["z"] - z_lift

        # 第一步：y 向预定位（保持抓取高度）
        y_move = {"x": init_x, "y": other_y, "z": capture_z, **quat}

        # 第二步：移动到抓取点
        target = {"x": capture_x, "y": other_y, "z": capture_z, **quat}

        logger.info("%s arm other position y_move=%s, target=%s",
                    other_axis, y_move, target)

        return {"axis": other_axis, "y_move": y_move, "target": target}
