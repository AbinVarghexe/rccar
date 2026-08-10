"""
Preprocessing pipeline for autonomous lane and junction detection.
"""

import os
import sys
import cv2
import numpy as np

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
root_dir = os.path.dirname(parent_dir)
for p in [curr_dir, parent_dir, root_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from config import PreprocessingConfig
except ImportError:
    from autonomous_car.config import PreprocessingConfig

class ImagePreprocessor:
    def __init__(self, config: PreprocessingConfig) -> None:
        self.config: PreprocessingConfig = config

    def process(self, frame: np.ndarray) -> np.ndarray:
        if frame is None or frame.size == 0:
            raise ValueError("Input frame to ImagePreprocessor cannot be empty")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kernel_size = self.config.blur_kernel_size
        if kernel_size[0] % 2 == 0 or kernel_size[1] % 2 == 0:
            kernel_size = (kernel_size[0] | 1, kernel_size[1] | 1)
        
        blurred = cv2.GaussianBlur(gray, kernel_size, self.config.gaussian_sigma)

        if self.config.use_adaptive_threshold:
            block_size = self.config.adaptive_block_size
            if block_size % 2 == 0:
                block_size += 1
            binary = cv2.adaptiveThreshold(
                blurred,
                self.config.max_binary_value,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                block_size,
                self.config.adaptive_c
            )
        else:
            _, binary = cv2.threshold(
                blurred,
                self.config.binary_threshold,
                self.config.max_binary_value,
                cv2.THRESH_BINARY
            )

        morph_k = cv2.getStructuringElement(
            cv2.MORPH_RECT, self.config.morph_kernel_size
        )
        closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, morph_k)
        cleaned = cv2.morphologyEx(closed, cv2.MORPH_OPEN, morph_k)

        roi_binary = self.apply_roi_mask(cleaned)
        return roi_binary

    def apply_roi_mask(self, binary_img: np.ndarray) -> np.ndarray:
        height, width = binary_img.shape[:2]
        polygon_pts = []
        for norm_x, norm_y in self.config.roi_polygon_normalized:
            px = int(norm_x * width)
            py = int(norm_y * height)
            polygon_pts.append([px, py])
            
        pts = np.array([polygon_pts], dtype=np.int32)
        mask = np.zeros_like(binary_img)
        cv2.fillPoly(mask, pts, 255)
        masked_img = cv2.bitwise_and(binary_img, mask)
        return masked_img
