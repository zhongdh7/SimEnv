"""OpenCV implementation of the red-sphere 2D detector."""

from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    """A contour accepted as a red sphere candidate."""

    x: int
    y: int
    width: int
    height: int
    center_x: float
    center_y: float
    area: float
    circularity: float
    aspect_ratio: float
    touches_border: bool = False
    fitted_radius_px: float = 0.0
    edge_fit_residual: float = 0.0


@dataclass(frozen=True)
class DetectorConfig:
    """Tunable HSV, morphology and contour-filter parameters."""

    hue_low_1: int = 0
    hue_high_1: int = 10
    hue_low_2: int = 170
    hue_high_2: int = 179
    saturation_min: int = 100
    saturation_max: int = 255
    value_min: int = 70
    value_max: int = 255
    blur_kernel_size: int = 3
    morphology_kernel_size: int = 5
    open_iterations: int = 1
    close_iterations: int = 2
    min_area: float = 120.0
    max_area: float = 120000.0
    min_circularity: float = 0.68
    min_aspect_ratio: float = 0.70
    max_aspect_ratio: float = 1.30
    border_margin_px: int = 2
    edge_min_area: float = 45.0
    edge_min_arc_points: int = 5
    edge_min_radius_px: float = 4.0
    edge_max_arc_residual: float = 0.16

    def validate(self) -> None:
        hue_values = (self.hue_low_1, self.hue_high_1, self.hue_low_2, self.hue_high_2)
        if any(value < 0 or value > 179 for value in hue_values):
            raise ValueError("HSV hue thresholds must be in [0, 179]")
        if self.hue_low_1 > self.hue_high_1 or self.hue_low_2 > self.hue_high_2:
            raise ValueError("Each HSV hue lower bound must not exceed its upper bound")
        for name, lower, upper in (
            ("saturation", self.saturation_min, self.saturation_max),
            ("value", self.value_min, self.value_max),
        ):
            if lower < 0 or upper > 255 or lower > upper:
                raise ValueError(f"Invalid {name} range [{lower}, {upper}]")
        for name, size in (
            ("blur_kernel_size", self.blur_kernel_size),
            ("morphology_kernel_size", self.morphology_kernel_size),
        ):
            if size < 1 or size % 2 == 0:
                raise ValueError(f"{name} must be a positive odd integer")
        if self.open_iterations < 0 or self.close_iterations < 0:
            raise ValueError("Morphology iteration counts cannot be negative")
        if self.min_area < 0 or self.max_area < self.min_area:
            raise ValueError("Invalid contour area range")
        if not 0.0 <= self.min_circularity <= 1.0:
            raise ValueError("min_circularity must be in [0, 1]")
        if self.min_aspect_ratio <= 0 or self.max_aspect_ratio < self.min_aspect_ratio:
            raise ValueError("Invalid aspect-ratio range")
        if self.border_margin_px < 0 or self.edge_min_arc_points < 3:
            raise ValueError("Invalid border margin or edge arc point count")
        if self.edge_min_area < 0.0 or self.edge_min_radius_px <= 0.0:
            raise ValueError("Invalid edge target size thresholds")
        if self.edge_max_arc_residual <= 0.0:
            raise ValueError("edge_max_arc_residual must be positive")


class RedSphereDetector:
    """Segment red pixels and filter contours using simple 2D geometry."""

    def __init__(self, config: DetectorConfig):
        config.validate()
        self.config = config

    def _fit_border_arc(self, contour, image_width, image_height):
        points = contour.reshape((-1, 2)).astype(np.float64)
        margin = float(self.config.border_margin_px)
        keep = (
            (points[:, 0] > margin)
            & (points[:, 0] < image_width - 1.0 - margin)
            & (points[:, 1] > margin)
            & (points[:, 1] < image_height - 1.0 - margin)
        )
        arc = points[keep]
        if arc.shape[0] < self.config.edge_min_arc_points:
            return None
        matrix = np.column_stack((2.0 * arc, np.ones(arc.shape[0])))
        target = np.einsum("ij,ij->i", arc, arc)
        try:
            solution, _residuals, rank, _singular = np.linalg.lstsq(
                matrix, target, rcond=None
            )
        except np.linalg.LinAlgError:
            return None
        if rank < 3:
            return None
        center = solution[:2]
        radius_squared = float(np.dot(center, center) + solution[2])
        if not np.isfinite(radius_squared) or radius_squared <= 0.0:
            return None
        radius = float(np.sqrt(radius_squared))
        if radius < self.config.edge_min_radius_px:
            return None
        radial = np.linalg.norm(arc - center[None, :], axis=1)
        normalized_residual = float(
            np.sqrt(np.mean(np.square(radial - radius))) / radius
        )
        if normalized_residual > self.config.edge_max_arc_residual:
            return None
        return float(center[0]), float(center[1]), radius, normalized_residual

    def detect(self, bgr_image: np.ndarray) -> Tuple[np.ndarray, List[Detection]]:
        if bgr_image is None or bgr_image.ndim != 3 or bgr_image.shape[2] != 3:
            raise ValueError("Expected a non-empty BGR image with three channels")

        image = bgr_image
        if self.config.blur_kernel_size > 1:
            kernel = (self.config.blur_kernel_size, self.config.blur_kernel_size)
            image = cv2.GaussianBlur(image, kernel, 0)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_1 = np.array(
            [self.config.hue_low_1, self.config.saturation_min, self.config.value_min],
            dtype=np.uint8,
        )
        upper_1 = np.array(
            [self.config.hue_high_1, self.config.saturation_max, self.config.value_max],
            dtype=np.uint8,
        )
        lower_2 = np.array(
            [self.config.hue_low_2, self.config.saturation_min, self.config.value_min],
            dtype=np.uint8,
        )
        upper_2 = np.array(
            [self.config.hue_high_2, self.config.saturation_max, self.config.value_max],
            dtype=np.uint8,
        )
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_1, upper_1),
            cv2.inRange(hsv, lower_2, upper_2),
        )

        morphology_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.config.morphology_kernel_size, self.config.morphology_kernel_size),
        )
        if self.config.open_iterations:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                morphology_kernel,
                iterations=self.config.open_iterations,
            )
        if self.config.close_iterations:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                morphology_kernel,
                iterations=self.config.close_iterations,
            )

        contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_result[-2]
        detections: List[Detection] = []
        image_height, image_width = mask.shape
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area > self.config.max_area:
                continue

            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0.0:
                continue
            circularity = float(4.0 * np.pi * area / (perimeter * perimeter))

            x, y, width, height = cv2.boundingRect(contour)
            if height == 0:
                continue
            aspect_ratio = float(width) / float(height)
            margin = self.config.border_margin_px
            touches_border = bool(
                x <= margin or y <= margin
                or x + width >= image_width - margin
                or y + height >= image_height - margin
            )

            fitted_radius = 0.25 * (width + height)
            edge_residual = 0.0
            edge_fit = None
            if touches_border:
                if area < self.config.edge_min_area:
                    continue
                edge_fit = self._fit_border_arc(contour, image_width, image_height)
                if edge_fit is None:
                    continue
            else:
                if area < self.config.min_area:
                    continue
                if circularity < self.config.min_circularity:
                    continue
                if not self.config.min_aspect_ratio <= aspect_ratio <= self.config.max_aspect_ratio:
                    continue

            moments = cv2.moments(contour)
            if edge_fit is not None:
                center_x, center_y, fitted_radius, edge_residual = edge_fit
            elif moments["m00"]:
                center_x = float(moments["m10"] / moments["m00"])
                center_y = float(moments["m01"] / moments["m00"])
            else:
                center_x = x + width / 2.0
                center_y = y + height / 2.0
            detections.append(
                Detection(
                    x=x,
                    y=y,
                    width=width,
                    height=height,
                    center_x=center_x,
                    center_y=center_y,
                    area=area,
                    circularity=circularity,
                    aspect_ratio=aspect_ratio,
                    touches_border=touches_border,
                    fitted_radius_px=fitted_radius,
                    edge_fit_residual=edge_residual,
                )
            )

        detections.sort(key=lambda item: item.area, reverse=True)
        return mask, detections

    @staticmethod
    def annotate(bgr_image: np.ndarray, detections: List[Detection]) -> np.ndarray:
        debug_image = bgr_image.copy()
        for index, detection in enumerate(detections):
            top_left = (detection.x, detection.y)
            bottom_right = (
                detection.x + detection.width - 1,
                detection.y + detection.height - 1,
            )
            cv2.rectangle(debug_image, top_left, bottom_right, (0, 255, 0), 2)
            if detection.touches_border:
                label = (
                    f"red edge {index}: A={detection.area:.0f} "
                    f"arc={detection.edge_fit_residual:.2f} r={detection.fitted_radius_px:.1f}"
                )
            else:
                label = (
                    f"red sphere {index}: A={detection.area:.0f} "
                    f"C={detection.circularity:.2f} R={detection.aspect_ratio:.2f}"
                )
            text_origin = (detection.x, max(15, detection.y - 6))
            cv2.putText(
                debug_image,
                label,
                text_origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
        return debug_image
