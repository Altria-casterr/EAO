"""图像采集解码工具（借鉴 Atom Image_Capture Realsense 的 Func.decode_image/decode_depth）。

- ``decode_color_message``：压缩图或原始 RGB/BGR → BGR ndarray
- ``decode_depth_message``：compressedDepth(PNG) 或原始 16UC1/32FC1 → 解码数组

依赖 numpy/cv2 采用函数内 lazy import，未安装 opencv 也能导入本模块。
"""

from __future__ import annotations

import base64
from typing import Any


def decode_color_message(msg: dict[str, Any] | None) -> Any:
    """CompressedImage 或原始 rgb8/bgr8 Image → BGR ndarray。

    参考 Atom Func.decode_image：rosbridge JSON 消息 data 字段为 base64 的
    JPEG 二进制，`cv2.imdecode` 直接解码为 BGR 图像。
    """

    import cv2
    import numpy as np

    data_b64 = (msg or {}).get("data", "")
    if not data_b64:
        return None
    raw = base64.b64decode(data_b64) if isinstance(data_b64, str) else bytes(data_b64)
    encoding = (msg or {}).get("encoding")
    if encoding in ("rgb8", "bgr8"):
        height, width, step = int(msg["height"]), int(msg["width"]), int(msg["step"])
        if height <= 0 or width <= 0 or step < width * 3 or len(raw) < height * step:
            raise ValueError("原始彩色图尺寸、行跨度或数据长度不合法")
        img = np.ndarray((height, width, 3), dtype=np.uint8, buffer=raw,
                         strides=(step, 3, 1)).copy()
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if encoding == "rgb8" else img
    img = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    return img if img is not None else None


def decode_depth_message(msg: dict[str, Any] | None) -> Any:
    """compressedDepth(PNG) 或原始 16UC1/32FC1 → 解码数组。

    参考 Atom Func.decode_depth：rosbridge 的 base64 数据内含 PNG 头，
    先定位 PNG 头再解码；PNG 将 16-bit 深度编码到 RGB（R=低 8 位，G=高 8 位，B=0）。
    """

    import cv2
    import numpy as np

    data_b64 = (msg or {}).get("data", "")
    if not data_b64:
        return None
    raw = base64.b64decode(data_b64) if isinstance(data_b64, str) else bytes(data_b64)
    if (msg or {}).get('encoding') in ('16UC1', '32FC1'):
        # 原始 sensor_msgs/Image：保留真实深度单位，正确处理字节序和行填充。
        encoding = msg['encoding']
        dtype = np.dtype(('>' if msg.get('is_bigendian') else '<') + ('u2' if encoding == '16UC1' else 'f4'))
        height, width, step = int(msg['height']), int(msg['width']), int(msg['step'])
        if height <= 0 or width <= 0 or step < width*dtype.itemsize or len(raw) < height*step:
            raise ValueError('原始深度图尺寸、行跨度或数据长度不合法')
        return np.ndarray((height,width), dtype=dtype, buffer=raw,
                          strides=(step,dtype.itemsize)).astype(dtype.newbyteorder('='),copy=True)
    png_header = b"\x89PNG\r\n\x1a\n"
    start = raw.find(png_header)
    if start == -1:
        return None
    img = cv2.imdecode(np.frombuffer(raw[start:], dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    return img if img is not None else None
