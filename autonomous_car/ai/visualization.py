import cv2
import numpy as np
from typing import List, Dict, Optional
from autonomous_car.ai.classes import DetectionResult, DecisionResult, ActionEnum

class Visualizer:
    """Renders crisp autonomous driving HUD, corridor lanes, bounding boxes, and decision banners."""

    ACTION_COLORS = {
        ActionEnum.FORWARD: (40, 180, 40),       # Green
        ActionEnum.STOP:    (30, 30, 220),       # Red
        ActionEnum.SLOW:    (0, 140, 255),       # Orange
        ActionEnum.WAIT:    (0, 200, 220),       # Yellow/Gold
        ActionEnum.LEFT:    (220, 120, 0),       # Cyan-Blue
        ActionEnum.RIGHT:   (220, 120, 0),       # Cyan-Blue
        ActionEnum.SCAN:    (200, 50, 180),      # Magenta
        ActionEnum.UNKNOWN: (120, 120, 120),     # Gray
    }

    @staticmethod
    def draw(
        frame: np.ndarray,
        detections: List[DetectionResult],
        risks: Dict[int, Dict[str, float]],
        decision: DecisionResult,
        fps: float = 0.0,
        latency_ms: float = 0.0,
        lane_width_ratio: float = 0.42,
        mode_str: str = "HYBRID",
        auto_enabled: bool = True,
    ) -> np.ndarray:
        if frame is None or frame.size == 0:
            return frame

        h, w = frame.shape[:2]

        # 1. Render Path Corridor Guidelines
        lane_start = int((1.0 - lane_width_ratio) / 2.0 * w)
        lane_end = int((1.0 + lane_width_ratio) / 2.0 * w)

        # Translucent corridor tint
        overlay = frame.copy()
        cv2.rectangle(overlay, (lane_start, 0), (lane_end, h), (255, 230, 0), -1)
        cv2.addWeighted(overlay, 0.07, frame, 0.93, 0, frame)

        # Corridor lane border lines
        cv2.line(frame, (lane_start, 0), (lane_start, h), (255, 220, 0), 2, cv2.LINE_AA)
        cv2.line(frame, (lane_end, 0), (lane_end, h), (255, 220, 0), 2, cv2.LINE_AA)

        # 2. Render Object Detections & Risks
        for d in detections:
            bbox = d.bbox
            tid = d.track_id if d.track_id is not None else id(d)

            x1 = max(0, min(w - 1, int(bbox.x1)))
            y1 = max(0, min(h - 1, int(bbox.y1)))
            x2 = max(0, min(w - 1, int(bbox.x2)))
            y2 = max(0, min(h - 1, int(bbox.y2)))

            risk_info = risks.get(tid, {"risk_score": 0.1, "ttc": float('inf')})
            risk_score = risk_info.get("risk_score", 0.1)
            ttc = risk_info.get("ttc", float('inf'))

            # Bus is highlighted in bold high-visibility red
            is_bus = "bus" in d.class_name.lower()
            if is_bus or risk_score > 0.65:
                color = (0, 0, 240)       # Red (Critical danger)
                box_thickness = 3 if is_bus else 2
            elif risk_score > 0.40:
                color = (0, 160, 255)     # Orange (Caution / Avoidance)
                box_thickness = 2
            else:
                color = (0, 220, 0)       # Green (Safe)
                box_thickness = 2

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, box_thickness)

            # Label text
            label = f"{d.class_name.upper()}"
            if d.track_id is not None:
                label += f" #{d.track_id}"
            label += f" {int(d.confidence * 100)}%"

            if ttc != float('inf') and ttc < 10.0:
                label += f" TTC:{ttc:.1f}s"

            # Label background banner
            (lbl_w, lbl_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            cv2.rectangle(frame, (x1, max(0, y1 - lbl_h - 6)), (x1 + lbl_w + 6, y1), color, -1)
            cv2.putText(
                frame, label, (x1 + 3, max(12, y1 - 4)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA
            )

        # 3. Top Action Decision Banner
        action = decision.action if decision else ActionEnum.UNKNOWN
        reason = decision.reason if decision else "Standby"
        action_col = Visualizer.ACTION_COLORS.get(action, (80, 80, 80))

        # Top banner background bar
        cv2.rectangle(frame, (0, 0), (w, 42), (18, 18, 20), -1)
        cv2.line(frame, (0, 42), (w, 42), action_col, 2)

        # Action Badge (pill)
        action_text = f" {action.value} "
        (aw, ah), _ = cv2.getTextSize(action_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(frame, (10, 8), (10 + aw + 10, 34), action_col, -1)
        cv2.putText(
            frame, action_text, (15, 27),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA
        )

        # Reason text
        clean_reason = reason[:55] + "..." if len(reason) > 55 else reason
        cv2.putText(
            frame, clean_reason, (25 + aw + 10, 27),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 225), 1, cv2.LINE_AA
        )

        # 4. Bottom Diagnostics Bar
        cv2.rectangle(frame, (0, h - 26), (w, h), (15, 15, 18), -1)

        auto_state_text = f"AUTO: {'ACTIVE' if auto_enabled else 'STANDBY'} [{mode_str.upper()}]"
        auto_state_col = (50, 210, 90) if auto_enabled else (160, 160, 160)
        cv2.putText(
            frame, auto_state_text, (12, h - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.42, auto_state_col, 1, cv2.LINE_AA
        )

        # Clearance stats
        cl_l = int(getattr(decision, "clearance_left", 1.0) * 100)
        cl_r = int(getattr(decision, "clearance_right", 1.0) * 100)
        clearance_text = f"L-CLEAR:{cl_l}% | R-CLEAR:{cl_r}%"
        cv2.putText(
            frame, clearance_text, (w // 2 - 60, h - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA
        )

        # FPS & Latency
        diag_text = f"{fps:.1f} FPS | {latency_ms:.0f}ms"
        (dw, dh), _ = cv2.getTextSize(diag_text, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
        cv2.putText(
            frame, diag_text, (w - dw - 12, h - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1, cv2.LINE_AA
        )

        return frame
