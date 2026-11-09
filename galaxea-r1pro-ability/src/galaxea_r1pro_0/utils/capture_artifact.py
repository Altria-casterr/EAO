"""Publish captured RGB frames to the Ability/Pilot exchange directory."""
from __future__ import annotations

import os
from pathlib import Path
import tempfile
from uuid import uuid4

import cv2
import numpy as np


def rgb_exchange_root() -> Path:
    value = os.environ.get("SEMANTIC_ABILITY_ARTIFACT_ROOT", "").strip()
    if not value:
        raise RuntimeError("RGB Artifact 保存失败：未配置 SEMANTIC_ABILITY_ARTIFACT_ROOT")
    root = Path(value).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    return root


def publish_rgb(frame, root: Path) -> dict:
    # RealBackend.capture('rgb8') actually returns decoded OpenCV BGR pixels.
    pixels = np.asarray(frame.data)
    if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] != 3:
        raise ValueError("RGB Artifact 需要 uint8 H×W×3 图像数组")
    ok, encoded = cv2.imencode(".jpg", pixels)
    if not ok:
        raise RuntimeError("RGB Artifact JPEG 编码失败")
    directory = root / "captures"
    directory.mkdir(parents=True, exist_ok=True)
    directory = directory.resolve()
    if not directory.is_relative_to(root):
        raise ValueError("RGB Artifact 目录超出交换目录")
    key = uuid4().hex
    target = directory / (key + ".jpg")
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=directory, suffix=".partial", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(encoded.tobytes())
        os.replace(temporary, target)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return {
        "candidate_id": "rgb:" + key,
        "sensor_id": frame.sensor_id,
        "frame_id": frame.frame_id,
        "encoding": "jpeg",
        "source_encoding": frame.encoding,
        "media_type": "image/jpeg",
        "width": int(pixels.shape[1]),
        "height": int(pixels.shape[0]),
        "observed_at": frame.timestamp.isoformat(),
        "payload_size": int(encoded.nbytes),
        "exchange_path": target.relative_to(root).as_posix(),
        "publication_state": "ready_for_pilot",
    }
