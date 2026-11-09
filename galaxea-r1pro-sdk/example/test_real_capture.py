"""RealBackend.capture 图像采集测试（fake rosbridge 消息 + 真实 cv2 解码）。"""

from __future__ import annotations

import base64
import unittest
from unittest import mock

import cv2
import numpy as np

from galaxea_r1pro.galaxea_r1pro_0.backends.real import (
    CAMERA_IMAGE_TOPICS,
    RealBackend,
)
from galaxea_r1pro.galaxea_r1pro_0.utils.errors import BackendUnavailableError, MessageNotFoundError
from galaxea_r1pro.galaxea_r1pro_0.utils.profile import RobotProfile
from galaxea_r1pro.galaxea_r1pro_0.utils.types import BackendKind


class _FakeRos:
    def __init__(self, latest_messages: dict | None = None) -> None:
        self.latest_messages = latest_messages or {}
        self.connect_flag = True
        self.client = mock.Mock()
        self.client.is_connected = True


def _backend(latest_messages: dict | None = None) -> RealBackend:
    backend = RealBackend(
        RobotProfile(
            robot_id="r1pro-capture",
            backend=BackendKind.REAL,
            firmware_profile="test-v1",
        )
    )
    backend._connected = True
    backend.ros_connection = _FakeRos(latest_messages)
    return backend


def _jpeg_message(pixel=(200, 100, 50)) -> dict:
    img = np.full((8, 8, 3), pixel, dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", img)
    assert ok
    return {"data": base64.b64encode(buf.tobytes()).decode("ascii"), "format": "jpeg"}


def _depth_message(value: int = 0x1234) -> dict:
    # PNG 深度编码：R=低 8 位，G=高 8 位，B=0（cv2 按 BGR 处理，数组序为 B/G/R）
    depth = np.full((8, 8, 3), (0, (value >> 8) & 0xFF, value & 0xFF), dtype=np.uint8)
    ok, buf = cv2.imencode(".png", depth)
    assert ok
    return {"data": base64.b64encode(buf.tobytes()).decode("ascii"), "format": "png"}


class RealCaptureTest(unittest.TestCase):
    def test_capture_rgb_decodes_jpeg(self) -> None:
        backend = _backend({CAMERA_IMAGE_TOPICS["head_camera"]["rgb8"]: _jpeg_message()})
        frame = backend.capture("head_camera", "rgb8")
        self.assertEqual(frame.sensor_id, "head_camera")
        self.assertEqual(frame.encoding, "rgb8")
        self.assertEqual(frame.data.shape, (8, 8, 3))
        # JPEG 为有损压缩，允许 ±2 容差
        actual = tuple(int(v) for v in frame.data[0, 0])
        self.assertTrue(all(abs(a - b) <= 2 for a, b in zip(actual, (200, 100, 50))), actual)

    def test_capture_depth_decodes_png(self) -> None:
        backend = _backend({CAMERA_IMAGE_TOPICS["head_camera"]["32FC1"]: _depth_message()})
        frame = backend.capture("head_camera", "32FC1")
        self.assertEqual(frame.encoding, "32FC1")
        self.assertEqual(frame.data.shape, (8, 8, 3))
        # R/G 通道恢复 16-bit 深度值
        b, g, r = (int(v) for v in frame.data[0, 0])
        self.assertEqual(r | (g << 8), 0x1234)
        self.assertEqual(b, 0)

    def test_unknown_sensor_raises(self) -> None:
        backend = _backend()
        with self.assertRaises(BackendUnavailableError):
            backend.capture("side_camera", "rgb8")

    def test_unknown_encoding_raises(self) -> None:
        backend = _backend()
        with self.assertRaises(BackendUnavailableError):
            backend.capture("head_camera", "bgr8")

    def test_timeout_without_message_raises(self) -> None:
        backend = _backend()
        with mock.patch("galaxea_r1pro.galaxea_r1pro_0.backends.real.IMAGE_WAIT_TIMEOUT", 0.05):
            with self.assertRaises(MessageNotFoundError) as ctx:
                backend.capture("head_camera", "rgb8")
        self.assertEqual(ctx.exception.topic, CAMERA_IMAGE_TOPICS["head_camera"]["rgb8"])

    def test_decode_failure_raises(self) -> None:
        backend = _backend({CAMERA_IMAGE_TOPICS["head_camera"]["rgb8"]: {"data": "bm90LWltYWdl"}})
        with mock.patch("galaxea_r1pro.galaxea_r1pro_0.backends.real.IMAGE_WAIT_TIMEOUT", 0.05):
            with self.assertRaises(MessageNotFoundError):
                backend.capture("head_camera", "rgb8")


if __name__ == "__main__":
    unittest.main()
