"""验证 R1 Pro 真机 RGB-D 采集和 YOLO 分割能力。"""

from __future__ import annotations

import sys
from pathlib import Path

# 允许直接执行本脚本时从 skill_library/src 导入 SDK。
SRC_ROOT = Path(__file__).resolve().parents[1] / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from galaxea_r1pro import BackendKind, R1ProSDK, RobotProfile


ENDPOINT = "172.16.70.147:9091"
SENSOR_ID = "head_camera"
OUTPUT_IMAGE = Path(__file__).resolve().parent / "yolo_result_overlay.png"


def print_frame(name: str, frame) -> None:
    data = frame.data
    shape = getattr(data, "shape", None)
    dtype = getattr(data, "dtype", None)
    print(f"[PASS] {name}: encoding={frame.encoding}, shape={shape}, dtype={dtype}")


def save_visualization(result, output_path: Path) -> None:
    import cv2
    import numpy as np

    image = result.rgb_frame.data.copy()
    overlay = image.copy()
    colors = [
        (0, 0, 255),
        (0, 255, 0),
        (255, 0, 0),
        (0, 255, 255),
        (255, 0, 255),
        (255, 255, 0),
    ]

    for index, obj in enumerate(result.objects, start=1):
        color = colors[(index - 1) % len(colors)]
        x1, y1, x2, y2 = [int(round(v)) for v in obj.box_xyxy]
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            image,
            f"{index}:{obj.class_name} {obj.confidence:.2f}",
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )

        if obj.mask is not None:
            mask = np.asarray(obj.mask)
            overlay[mask > 0] = color

    visualized = cv2.addWeighted(overlay, 0.25, image, 0.75, 0.0)
    cv2.imwrite(str(output_path), visualized)
    print(f"[PASS] 可视化结果已保存: {output_path}")


def main() -> int:
    profile = RobotProfile(
        robot_id="r1pro-yolo-check",
        backend=BackendKind.REAL,
        firmware_profile="vendor-2026.08",
        endpoint=ENDPOINT,
        options={
            "test_mode": False,
            "yolo_conf": 0.5,
            "yolo_iou": 0.7,
            "yolo_device": "cuda" if _cuda_available() else "cpu",
        },
    )

    sdk = None
    try:
        print(f"连接 ROS: {ENDPOINT}")
        sdk = R1ProSDK.open(profile)
        print("[PASS] SDK 连接成功")

        print(f"读取彩色图: {SENSOR_ID}")
        rgb_frame = sdk.sensors.capture_rgb(SENSOR_ID)
        print_frame("capture_rgb", rgb_frame)

        print(f"读取深度图: {SENSOR_ID}")
        depth_frame = sdk.sensors.capture_depth(SENSOR_ID)
        print_frame("capture_depth", depth_frame)

        print("执行 YOLO 分割")
        result = sdk.vision.segment(SENSOR_ID)
        print(f"[PASS] YOLO 分割完成: objects={len(result.objects)}, image_shape={result.image_shape}")

        for index, obj in enumerate(result.objects, start=1):
            mask_shape = getattr(obj.mask, "shape", None)
            print(
                f"  object[{index}]: class_id={obj.class_id}, "
                f"class_name={obj.class_name}, confidence={obj.confidence:.4f}, "
                f"box={obj.box_xyxy}, mask_shape={mask_shape}"
            )

        save_visualization(result, OUTPUT_IMAGE)
        return 0
    except Exception as exc:
        print(f"[FAIL] YOLO 能力验证失败: {type(exc).__name__}: {exc}")
        return 1
    finally:
        if sdk is not None:
            sdk.close()
            print("SDK 连接已关闭")


def _cuda_available() -> bool:
    try:
        import torch

        return bool(torch.cuda.is_available())
    except Exception:
        return False


if __name__ == "__main__":
    raise SystemExit(main())
