"""R1 Pro 真机后端入口。

真机后端承载真机专属逻辑：
- ``connect``：从 profile.endpoint 解析 ROS 地址，按 connect_flag 检查连接，
  成功后按当前 ability 裁剪并注册对应话题；
- ``start_command``：只处理执行型命令（publisher=话题发布 / action=Action goal）；
- ``get_command``：统一读取入口——传入 command_id 查执行型结果；
  传入 (resource, operation) 读取型反馈（委托 transport/process_sub.py）；
- test_mode 走内存模拟（InMemoryBackend）。
"""

from __future__ import annotations

import logging
import copy
import math
import time
from typing import Any
from uuid import uuid4

import roslibpy

from ..transport.process_pub import GALAXEA_R1PRO_PUBLISH_COMMANDS
from ..transport.process_sub import GALAXEA_R1PRO_GET_COMMANDS
from ..utils.Connect_Ros import GALAXEA_R1PRO_ABILITY_TOPICS, GALAXEA_R1PRO_TOPICS, ConnectRos
from ..utils.errors import BackendUnavailableError, CommandNotFoundError, MessageNotFoundError
from ..utils.profile import RobotProfile
from ..utils.types import (
    CommandFeedback,
    CommandHandle,
    CommandResult,
    CommandStatus,
    SensorFrame,
    utc_now,
)
from ..utils.utils_ImageCapture import decode_color_message, decode_depth_message
from ..utils.utils_common import parse_endpoint
from .memory import InMemoryBackend, _CommandRecord

logger = logging.getLogger(__name__)


def _json_safe(value: Any) -> Any:
    """把 roslibpy/自定义对象递归转换为可 JSON 序列化的基础类型。"""

    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_safe(item) for item in value]

    name = getattr(value, "name", None)
    enum_value = getattr(value, "value", None)
    if name is not None and enum_value is not None:
        return name

    object_dict = getattr(value, "__dict__", None)
    if isinstance(object_dict, dict) and object_dict:
        return {str(key): _json_safe(item) for key, item in object_dict.items()}

    return str(value)

# 相机 → 图像话题映射（参考 Atom Image_Capture Realsense：CompressedImage 压缩传输）
CAMERA_IMAGE_TOPICS: dict[str, dict[str, str]] = {
    "head_camera": {
        "rgb8": "/camera/camera/color/image_raw/compressed",
        "32FC1": "/camera/camera/aligned_depth_to_color/image_raw/compressedDepth",
    },
}

IMAGE_WAIT_TIMEOUT = 10.0


class RealBackend(InMemoryBackend):
    """R1 Pro 真机后端；未接入的操作会明确失败而不是假装执行。"""

    def __init__(self, profile: RobotProfile) -> None:

        super().__init__(profile)

        self.ros_connection: ConnectRos | None = None
        self.camera_topics = copy.deepcopy(CAMERA_IMAGE_TOPICS)
        self.camera_topic_types = dict(profile.options.get("camera_topic_types") or {})
        configured = profile.options.get("camera_topics", {})
        if not isinstance(configured, dict):
            raise ValueError("camera_topics 必须为 sensor_id 到话题映射的对象")
        for sensor, topics in configured.items():
            if not isinstance(topics, dict) or set(topics) != {"rgb8", "32FC1"} or not all(isinstance(v, str) and v.startswith('/') for v in topics.values()):
                raise ValueError(f"camera_topics.{sensor} 需要 rgb8 和 32FC1 两个完整话题名")
            self.camera_topics[sensor] = dict(topics)

    # ---------- 连接 ----------

    def connect(self, ability_name: str | None = None) -> None:
        """建立真机连接；test_mode 走内存模拟，生产模式直接走 ROS。

        ability_name 用于裁剪需要注册的话题（None / 未登记则注册全部话题）。
        复用场景：已有连接但 ROS 实际已断时，先清理旧连接再重新连接。
        """

        if self.profile.options.get("test_mode", False):
            self._connected = True
            return

        if self._connected:
            if self._ros_alive():
                return
            logger.warning("检测到 ROS 连接已断开，清理旧连接并重新连接")
            self.close()

        host, port = parse_endpoint(self.profile.endpoint)

        try:
            # Step1、先建立ros连接
            self.ros_connection = ConnectRos(ros_host=host, ros_port=port)
            if not self.ros_connection.connect_flag:
                raise BackendUnavailableError(
                    backend="real",
                    reason=BackendUnavailableError.ENDPOINT_UNREACHABLE,
                    operation="RealBackend.connect",
                    hint=f"连接 {host}:{port} 失败或超时（{self.ros_connection.timeout}s）",
                )
            
            # Step2、按当前 ability 裁剪话题注册表：只注册该能力实际用到的话题
            registry = copy.deepcopy(GALAXEA_R1PRO_TOPICS)

            ability_topics = GALAXEA_R1PRO_ABILITY_TOPICS.get(ability_name)
            if ability_topics is None:
                # 未登记的 ability 保持旧行为（注册全部话题），避免误配把能力拉不起来
                logger.warning(
                    "ability %s 未在 GALAXEA_R1PRO_ABILITY_TOPICS 登记，将注册全部话题",
                    ability_name,
                )
            else:
                for category in registry:
                    registry[category] = {
                        topic: message_type
                        for topic, message_type in registry[category].items()
                        if topic in ability_topics
                    }

            # Step3、注册裁剪后的话题
            self.ros_connection.register_topics(registry)

        except BackendUnavailableError:
            # 话题注册等已带明确语义的失败原样抛出，避免被误报成“连接失败”
            raise
        except Exception as exc:
            raise BackendUnavailableError(
                backend="real",
                reason=BackendUnavailableError.ENDPOINT_UNREACHABLE,
                operation="RealBackend.connect",
                hint=f"连接 {host}:{port} 失败",
            ) from exc
        self._connected = True

    def _ros_alive(self) -> bool:
        """检测当前 rosbridge 连接是否真实存活（复用 SDK 时对端断连会变 False）。"""

        connection = self.ros_connection
        if connection is None:
            return False
        try:
            return bool(connection.connect_flag) and bool(connection.client.is_connected)
        except Exception:
            return False

    def is_connected(self) -> bool:
        """当前是否处于真实连接状态（供 Ability 在复用 SDK 时检测断连）。"""

        return self._connected and self._ros_alive()

    # ---------- 执行型命令 ----------

    def start_command(
        self,
        resource: str,
        operation: str,
        target: dict[str, Any],
    ) -> CommandHandle:

        # TODO 根据_require_connected判别当前是否存在xx异常
        self._require_connected("start_command")

        if self.profile.options.get("test_mode", False):
            return super().start_command(resource, operation, target)

        command_id = str(uuid4())
        handle = CommandHandle(command_id, self.profile.robot_id, resource, operation)

        spec = GALAXEA_R1PRO_PUBLISH_COMMANDS.get((resource, operation))
        if spec is None:
            raise BackendUnavailableError(
                backend="real",
                reason=BackendUnavailableError.NOT_IMPLEMENTED,
                operation=f"{resource}.{operation}",
                hint=f"操作 {resource}.{operation} 尚未映射到 Galaxea_R1Pro 的 ROS 消息",
            )

        kind = spec.get("kind", "publisher")
        topic: Any = spec.get("topic")

        # 先登记记录，保证 action 回调（可能同步触发）也能更新记录
        deferred = bool(self.profile.options.get("defer_commands", False))
        status = CommandStatus.RUNNING if deferred else CommandStatus.SUCCEEDED

        feedback = CommandFeedback(
            command_id=command_id,
            status=status,
            progress=0.25 if deferred else 1.0,
            phase="executing" if deferred else "completed",
            metrics={"backend": self.profile.backend.value},
        )
        result = CommandResult(command_id, status, feedback, output={"target": dict(target)})

        with self._lock:
            self._commands[command_id] = _CommandRecord(handle, dict(target), result)
            self._held_resources.discard(resource)

        if kind == "action":

            action = self.ros_connection.actions.get(topic)

            if action is None:
                raise BackendUnavailableError(
                    backend="real",
                    reason=BackendUnavailableError.ENDPOINT_UNREACHABLE,
                    operation=f"{resource}.{operation}",
                    hint=f"Action {topic} 未注册，请先 connect()",
                )

            if self.ros_connection.navigation_state.get("is_navigating"):
                raise BackendUnavailableError(
                    backend="real",
                    reason=BackendUnavailableError.NAVIGATION_BUSY,
                    operation=f"{resource}.{operation}",
                    hint="已有导航任务正在执行，无法启动新的导航",
                )

            goal = roslibpy.Goal(spec["builder"](target))

            state: dict[str, Any] = {
                "is_navigating": True,
                "goal": goal,
                "goal_id": None,
                "result": None,
                "feedback": None,
                "error": None,
            }
            self.ros_connection.navigation_state = state

            def _set_result(status: CommandStatus, phase: str, output: dict[str, Any]) -> None:
                """Action 回调推进登记记录，保证 defer_commands=true 时状态可收敛。"""

                with self._lock:
                    record = self._commands.get(command_id)
                    if record is None:
                        return
                    feedback = CommandFeedback(command_id, status, 1.0, phase)
                    record.result = CommandResult(command_id, status, feedback, output=output)

            def _on_result(result: dict) -> None:
                state["result"] = result
                state["is_navigating"] = False

                action_status = result.get("status")
                status_value = getattr(action_status, "value", action_status)
                status_name = getattr(action_status, "name", None)
                message = result.get("message") or result.get("detail") or result.get("error")

                if status_value == 4 or status_name == "SUCCEEDED":
                    command_status = CommandStatus.SUCCEEDED
                    phase = "completed"
                    state["error"] = None
                elif status_value == 2 or status_name == "CANCELED":
                    command_status = CommandStatus.CANCELLED
                    phase = "cancelled"
                    state["error"] = message or "navigation cancelled"
                else:
                    success = result.get("success", False)
                    if success:
                        command_status = CommandStatus.SUCCEEDED
                        phase = "completed"
                        state["error"] = None
                    else:
                        command_status = CommandStatus.FAILED
                        phase = "failed"
                        state["error"] = message or "未知错误"

                _set_result(command_status, phase, {"navigation": _json_safe(result), "state": _json_safe(state)})

            def _on_feedback(feedback: dict) -> None:
                state["feedback"] = feedback

            def _on_error(error: Any) -> None:
                state["error"] = str(error)
                state["is_navigating"] = False
                _set_result(CommandStatus.FAILED, "failed", {"error": str(error), "state": _json_safe(state)})

            goal_id = action.send_goal(goal, _on_result, _on_feedback, _on_error)
            if goal_id is None:
                logger.error("导航目标发送失败：send_goal 返回 None")
                state["is_navigating"] = False
                state["error"] = "导航目标发送失败：send_goal 返回 None"
                _set_result(CommandStatus.FAILED,"failed",{"error": state["error"], "state": _json_safe(state)})
            else:
                state["goal_id"] = goal_id

        else:

            topic = spec["topic"](target) if callable(spec["topic"]) else spec["topic"]

            message = spec["builder"](target)

            publisher = self.ros_connection.publishers.get(topic)

            if publisher is None:
                raise BackendUnavailableError(
                    backend="real",
                    reason=BackendUnavailableError.ENDPOINT_UNREACHABLE,
                    operation=f"{resource}.{operation}",
                    hint=f"话题 {topic} 未注册，请先 connect()",
                )
            publisher.publish(message)

        return handle

    # ---------- 统一读取入口（执行型查询 + 读取型反馈） ----------

    def get_command(
        self,
        command_id: str | None = None,
        *,
        resource: str | None = None,
        operation: str | None = None,
        target: dict[str, Any] | None = None,
    ) -> CommandResult:
    
        if self.profile.options.get("test_mode", False):
            return super().get_command(command_id, resource=resource, operation=operation, target=target)

        if command_id is not None:

            with self._lock:
                record = self._commands.get(command_id)

            if record is None:
                raise CommandNotFoundError(command_id=command_id, hint=f"未知命令: {command_id}")

            return record.result

        # 读取型：transport/process_sub.py 处理
        if resource is None or operation is None:
            raise BackendUnavailableError(
                backend="real",
                reason=BackendUnavailableError.NOT_IMPLEMENTED,
                operation="RealBackend.get_command",
                hint="读取型查询必须提供 resource 与 operation",
            )
        spec = GALAXEA_R1PRO_GET_COMMANDS.get((resource, operation))
        if spec is None:
            raise BackendUnavailableError(
                backend="real",
                reason=BackendUnavailableError.NOT_IMPLEMENTED,
                operation=f"{resource}.{operation}",
                hint=f"读取操作 {resource}.{operation} 尚未映射到 Galaxea_R1Pro 的反馈话题",
            )

        handler = spec["handler"]
        if spec.get("accepts_target"):
            data = handler(self.ros_connection, **dict(target or {}))
        else:
            data = handler(self.ros_connection)
        has_data = data not in (None, {}, [], ())
        status = CommandStatus.SUCCEEDED if has_data else CommandStatus.FAILED
        return CommandResult(
            command_id="",
            status=status,
            feedback=CommandFeedback(
                command_id="",
                status=status,
                progress=1.0 if has_data else 0.0,
                phase="completed" if has_data else "no_feedback",
            ),
            output={"data": data},
        )

    def capture(self, sensor_id: str, encoding: str) -> SensorFrame:
        """采集彩色/深度图像（借鉴 Atom Image_Capture Realsense main.py）。

        - ``rgb8``：JPEG CompressedImage → BGR ndarray
        - ``32FC1``：PNG compressedDepth → 解码数组（R/G 通道编码 16-bit 深度）

        通过 ConnectRos 的 latest_messages 等待最新消息最多 IMAGE_WAIT_TIMEOUT 秒；
        未收到消息或解码失败抛 MessageNotFoundError（与读取型反馈一致）。
        """

        self._require_connected("capture")

        if self.profile.options.get("test_mode", False):
            return super().capture(sensor_id, encoding)

        topics = self.camera_topics.get(sensor_id)
        if topics is None:
            raise BackendUnavailableError(
                backend="real",
                reason=BackendUnavailableError.NOT_IMPLEMENTED,
                operation=f"capture.{sensor_id}",
                hint=f"相机 {sensor_id} 未映射图像话题（可选: {sorted(self.camera_topics)}）",
            )
        topic = topics.get(encoding)
        if topic is None:
            raise BackendUnavailableError(
                backend="real",
                reason=BackendUnavailableError.NOT_IMPLEMENTED,
                operation=f"capture.{sensor_id}.{encoding}",
                hint=f"编码 {encoding} 未映射（可选: {sorted(topics)}）",
            )

        msg = None
        max_age = float(self.profile.options.get("image_max_age_s", 1.0))
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError("image_max_age_s 必须为正的有限数")
        received_at = None
        receipts = getattr(self.ros_connection, "message_received_at", None)
        deadline = time.time() + IMAGE_WAIT_TIMEOUT
        while time.time() < deadline:
            msg = self.ros_connection.latest_messages.get(topic)
            if isinstance(receipts, dict):
                received_at = receipts.get(topic)
                if received_at is None or time.monotonic() - received_at > max_age:
                    msg = None
            if msg is not None:
                break
            time.sleep(0.05)

        if msg is None:
            raise MessageNotFoundError(
                topic=topic,
                reason=MessageNotFoundError.MESSAGE_TIMEOUT,
                timeout=IMAGE_WAIT_TIMEOUT,
                hint=f"等待图像话题 {topic} 消息超时（{IMAGE_WAIT_TIMEOUT}s）",
            )

        data = decode_color_message(msg) if encoding == "rgb8" else decode_depth_message(msg)
        if data is None:
            raise MessageNotFoundError(
                topic=topic,
                reason=MessageNotFoundError.MESSAGE_NOT_FOUND,
                timeout=IMAGE_WAIT_TIMEOUT,
                hint=f"图像话题 {topic} 消息解码失败",
            )

        return SensorFrame(
            sensor_id=sensor_id,
            frame_id=str((msg.get("header") or {}).get("frame_id") or ""),
            timestamp=utc_now(),
            encoding=encoding,
            data=data,
            metadata={"topic": topic, "transport": "compressed_image", "ros_stamp": (msg.get("header") or {}).get("stamp"), "timestamp_source": "capture_receipt", "received_monotonic": received_at},
        )

    def stop_command(self, command_id: str, reason: str) -> CommandResult:

        if self.profile.options.get("test_mode", False):
            return super().stop_command(command_id, reason)

        with self._lock:
            record = self._commands.get(command_id)

            if record is None:
                raise CommandNotFoundError(command_id=command_id, hint=f"未知命令: {command_id}")

            # Step1、ROS 侧停止：action 导航先取消（刹车由上层 stop/unlock 命令发布）
            spec = GALAXEA_R1PRO_PUBLISH_COMMANDS.get((record.handle.resource, record.handle.operation))

            if spec is not None and spec.get("kind") == "action":
                state = self.ros_connection.navigation_state
                state["is_navigating"] = False
                action = self.ros_connection.actions.get(spec.get("topic"))
                goal_id = state.get("goal_id")
                if action is not None and goal_id:
                    try:
                        action.cancel_goal(goal_id)
                    except Exception as exc:
                        logger.error("取消导航任务失败: %s", exc)

            # Step2、按 stop_confirmed 判定终态，并更新登记记录
            if not bool(self.profile.options.get("stop_confirmed", True)):
                status = CommandStatus.UNKNOWN
                phase = "stop_unconfirmed"
            else:
                status = CommandStatus.CANCELLED
                phase = "held"
                self._held_resources.add(record.handle.resource)

            feedback = CommandFeedback(
                command_id=command_id,
                status=status,
                progress=record.result.feedback.progress,
                phase=phase,
                metrics={"stop_reason": reason},
            )
            record.result = CommandResult(command_id, status, feedback, output=record.result.output)
            return record.result

    def close(self) -> None:
        """关闭真机传输连接并复位状态。"""

        if self.ros_connection is not None:
            self.ros_connection.close()
            self.ros_connection = None
            self._commands.clear()
            self._connected = False
        super().close()
