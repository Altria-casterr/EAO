"""Ability-owned YOLO segmentation client."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np


@dataclass(frozen=True)
class DetectedObject:
    class_id: int
    class_name: str
    confidence: float
    box_xyxy: list[float]
    mask: np.ndarray | None


class YoloClient:
    """Runs the configured Ability-owned Ultralytics YOLO model."""

    def __init__(self, settings: Mapping[str, Any] | None = None):
        self.settings = dict(settings or {})
        self._model = None

    def close(self) -> None:
        self._model = None

    def _load_model(self):
        if self._model is not None:
            return self._model
        checkpoint = self.settings.get("checkpoint_path")
        if not checkpoint:
            raise RuntimeError("未配置 Ability 侧 YOLO checkpoint_path；可在 modelRegistryPath 或 YOLO_CHECKPOINT_PATH 中配置")
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError("Ability 侧 YOLO 需要安装 ultralytics") from exc
        self._model = YOLO(str(checkpoint))
        if getattr(self._model, "task", None) not in (None, "segment"):
            raise RuntimeError("checkpoint_path 必须是 YOLO segmentation 模型")
        return self._model

    def segment(self, rgb: np.ndarray, *, sensor_id: str = "head_camera", **kwargs: Any) -> Any:
        rgb = np.asarray(rgb)
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("YOLO 输入必须是 uint8 H×W×3 RGB 图像")
        model = self._load_model()

        results = model.predict(
            rgb,
            conf=kwargs.get("conf_threshold"),
            iou=kwargs.get("iou_threshold"),
            imgsz=kwargs.get("imgsz") or self.settings.get("imgsz", 896),
            device=kwargs.get("device") or self.settings.get("device"),
            verbose=False,
            retina_masks=True,
        )
        return self._convert(results, rgb.shape[:2], kwargs.get("class_names"))

    def segment_image(self, rgb: np.ndarray, **kwargs: Any) -> list[DetectedObject]:
        result = self.segment(rgb, sensor_id="", **kwargs)
        if hasattr(result, "objects"):
            return list(result.objects)
        return list(result)

    @staticmethod
    def _convert(results: Any, shape: tuple[int, int], class_names: Any = None) -> list[DetectedObject]:
        if not results:
            return []
        result = results[0]
        names = getattr(result, "names", {}) or {}
        boxes = getattr(result, "boxes", None)
        masks = getattr(result, "masks", None)
        if boxes is None:
            return []
        cls = boxes.cls.cpu().numpy().astype(int)
        conf = boxes.conf.cpu().numpy().astype(float)
        xyxy = boxes.xyxy.cpu().numpy().astype(float)
        mask_data = masks.data.cpu().numpy() > 0.5 if masks is not None else None
        objects = []
        for index, class_id in enumerate(cls):
            name = names.get(int(class_id), str(class_id)) if isinstance(names, Mapping) else str(class_id)
            if isinstance(class_names, Mapping):
                name = class_names.get(str(class_id), class_names.get(int(class_id), name))
            mask = mask_data[index] if mask_data is not None else None
            if mask is not None and mask.shape != shape:
                import cv2
                mask = cv2.resize(mask.astype(np.uint8), (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST).astype(bool)
            objects.append(DetectedObject(int(class_id), str(name), float(conf[index]), xyxy[index].tolist(), mask))
        return objects
