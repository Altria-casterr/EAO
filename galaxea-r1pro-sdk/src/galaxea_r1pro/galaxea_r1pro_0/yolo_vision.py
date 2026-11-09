"""YOLO 识别与分割能力。"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Mapping, Sequence

from .backends.base import RobotBackend
from .utils.types import SensorFrame, VisionObject, VisionResult

os.environ.setdefault("ULTRALYTICS_OFFLINE", "1")
os.environ.setdefault("YOLO_OFFLINE", "1")


class YoloVision:
    """基于 RGB-D 图像的最小 YOLO 识别与分割能力。"""

    def __init__(self, backend: RobotBackend, options: Mapping[str, Any] | None = None) -> None:
        self._backend = backend
        self._options = dict(options or {})
        self._model = None
        self._class_names_override = self._normalize_class_names(self._options.get("yolo_class_names"))

    def detect(
        self,
        sensor_id: str = "head_camera",
        *,
        conf_threshold: float | None = None,
        iou_threshold: float | None = None,
        imgsz: int | Sequence[int] | None = None,
        device: str | int | None = None,
        class_names: Sequence[str] | Mapping[int, str] | None = None,
    ) -> VisionResult:
        return self._infer(
            sensor_id=sensor_id,
            include_masks=False,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            imgsz=imgsz,
            device=device,
            class_names=class_names,
        )

    def segment(
        self,
        sensor_id: str = "head_camera",
        *,
        conf_threshold: float | None = None,
        iou_threshold: float | None = None,
        imgsz: int | Sequence[int] | None = None,
        device: str | int | None = None,
        class_names: Sequence[str] | Mapping[int, str] | None = None,
    ) -> VisionResult:
        return self._infer(
            sensor_id=sensor_id,
            include_masks=True,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            imgsz=imgsz,
            device=device,
            class_names=class_names,
        )

    def segment_image(
        self,
        color_image: Any,
        *,
        conf_threshold: float | None = None,
        iou_threshold: float | None = None,
        imgsz: int | Sequence[int] | None = None,
        device: str | int | None = None,
        class_names: Sequence[str] | Mapping[int, str] | None = None,
    ) -> tuple[VisionObject, ...]:
        """对外部 RGB 图像执行一次 YOLO 分割，不采集机器人传感器。"""
        model = self._get_model()
        resolved_device = self._resolve_device(device)
        resolved_imgsz = imgsz if imgsz is not None else self._options.get("yolo_imgsz", 640)
        resolved_conf = float(conf_threshold if conf_threshold is not None else self._options.get("yolo_conf", 0.5))
        resolved_iou = float(iou_threshold if iou_threshold is not None else self._options.get("yolo_iou", 0.7))
        names = self._resolve_class_names(class_names)
        result = model(
            color_image,
            conf=resolved_conf,
            iou=resolved_iou,
            imgsz=resolved_imgsz,
            device=resolved_device,
            verbose=False,
        )[0]
        return tuple(
            self._parse_result(
                result=result,
                class_names=names,
                image_shape=color_image.shape,
                include_masks=True,
            )
        )

    def _infer(
        self,
        *,
        sensor_id: str,
        include_masks: bool,
        conf_threshold: float | None,
        iou_threshold: float | None,
        imgsz: int | Sequence[int] | None,
        device: str | int | None,
        class_names: Sequence[str] | Mapping[int, str] | None,
    ) -> VisionResult:
        rgb_frame = self._backend.capture(sensor_id, "rgb8")
        depth_frame = self._backend.capture(sensor_id, "32FC1")
        color_image = rgb_frame.data

        if color_image is None:
            raise ValueError(f"相机 {sensor_id} 未返回彩色图像")

        model = self._get_model()
        resolved_device = self._resolve_device(device)
        resolved_imgsz = imgsz if imgsz is not None else self._options.get("yolo_imgsz", 640)
        resolved_conf = float(conf_threshold if conf_threshold is not None else self._options.get("yolo_conf", 0.5))
        resolved_iou = float(iou_threshold if iou_threshold is not None else self._options.get("yolo_iou", 0.7))
        names = self._resolve_class_names(class_names)

        result = model(
            color_image,
            conf=resolved_conf,
            iou=resolved_iou,
            imgsz=resolved_imgsz,
            device=resolved_device,
            verbose=False,
        )[0]

        objects = self._parse_result(result=result, class_names=names, image_shape=color_image.shape, include_masks=include_masks)
        return VisionResult(
            sensor_id=sensor_id,
            image_shape=tuple(int(v) for v in color_image.shape),
            objects=tuple(objects),
            rgb_frame=rgb_frame,
            depth_frame=depth_frame,
            metadata={
                "model_path": str(self._resolve_model_path()),
                "conf_threshold": resolved_conf,
                "iou_threshold": resolved_iou,
                "imgsz": resolved_imgsz,
                "device": resolved_device,
                "mode": "segment" if include_masks else "detect",
            },
        )

    def _get_model(self):
        if self._model is None:
            from ultralytics import YOLO

            self._model = YOLO(str(self._resolve_model_path()))
        return self._model

    def _resolve_model_path(self) -> Path:
        configured = self._options.get("yolo_model_path")
        if configured:
            return Path(configured)
        return Path(__file__).resolve().parent / "utils" / "yolo11x-seg_for_WAIC_0715.pt"

    def _resolve_device(self, device: str | int | None) -> str | int:
        if device is not None:
            return device
        configured = self._options.get("yolo_device")
        if configured is not None:
            return configured
        try:
            import torch

            return "cuda" if torch.cuda.is_available() else "cpu"
        except Exception:
            return "cpu"

    def _resolve_class_names(self, class_names: Sequence[str] | Mapping[int, str] | None) -> list[str] | None:
        names = class_names if class_names is not None else self._class_names_override
        return self._normalize_class_names(names)

    def _normalize_class_names(self, class_names: Sequence[str] | Mapping[int, str] | None) -> list[str] | None:
        if class_names is None:
            return None
        if isinstance(class_names, Mapping):
            if not class_names:
                return []
            max_index = max(int(index) for index in class_names)
            resolved = [f"unknown_class_{idx}" for idx in range(max_index + 1)]
            for index, name in class_names.items():
                resolved[int(index)] = str(name)
            return resolved
        return [str(name) for name in class_names]

    def _parse_result(
        self,
        *,
        result: Any,
        class_names: list[str] | None,
        image_shape: Sequence[int],
        include_masks: bool,
    ) -> list[VisionObject]:
        if result.boxes is None:
            return []

        import cv2
        import numpy as np

        boxes = result.boxes.xyxy.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy().astype(int)
        confidences = result.boxes.conf.cpu().numpy()
        masks_data = result.masks.data.cpu().numpy() if result.masks is not None else None
        width = int(image_shape[1])
        height = int(image_shape[0])

        objects: list[tuple[int, VisionObject]] = []
        for index, box in enumerate(boxes):
            class_id = int(class_ids[index])
            class_name = self._resolve_class_name(result, class_names, class_id)
            mask = None
            center_x = int(round((float(box[0]) + float(box[2])) / 2.0))

            if include_masks and masks_data is not None:
                resized_mask = cv2.resize(masks_data[index], (width, height), interpolation=cv2.INTER_LINEAR)
                binary_mask = np.where(resized_mask > 0.5, 255, 0).astype(np.uint8)
                mask = binary_mask
                moments = cv2.moments(binary_mask)
                if moments["m00"] != 0:
                    center_x = int(moments["m10"] / moments["m00"])

            objects.append(
                (
                    center_x,
                    VisionObject(
                        class_id=class_id,
                        class_name=class_name,
                        confidence=float(confidences[index]),
                        box_xyxy=tuple(float(v) for v in box.tolist()),
                        mask=mask,
                    ),
                )
            )

        objects.sort(key=lambda item: item[0])
        return [obj for _, obj in objects]

    def _resolve_class_name(self, result: Any, class_names: list[str] | None, class_id: int) -> str:
        if class_names is not None and 0 <= class_id < len(class_names):
            return class_names[class_id]
        names = getattr(result, "names", None)
        if isinstance(names, Mapping):
            value = names.get(class_id)
            if value is not None:
                return str(value)
        return f"unknown_class_{class_id}"
