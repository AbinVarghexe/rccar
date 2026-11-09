import cv2
import numpy as np
from typing import List, Dict, Optional, Any
from ai.classes import DetectionResult, DecisionResult, ActionEnum

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
        ActionEnum.REVERSE: (0, 140, 255),       # Orange
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
        target_pin: Optional[Dict[str, Any]] = None,
        current_pose: Optional[Dict[str, Any]] = None,
        last_obstacle_pose: Optional[Dict[str, Any]] = None,
    ) -> np.ndarray:
        if frame is None or frame.size == 0:
            return frame

        h, w = frame.shape[:2]

        # 1. Full Camera Feed Optical Coverage Frame (Corner Targeting Brackets)
        has_objects = len(detections) > 0
        bracket_len = 28
        bracket_col = (0, 0, 245) if has_objects else (0, 210, 100)
        
        # Top-left corner
        cv2.line(frame, (16, 52), (16 + bracket_len, 52), bracket_col, 2)
        cv2.line(frame, (16, 52), (16, 52 + bracket_len), bracket_col, 2)
        # Top-right corner
        cv2.line(frame, (w - 16, 52), (w - 16 - bracket_len, 52), bracket_col, 2)
        cv2.line(frame, (w - 16, 52), (w - 16, 52 + bracket_len), bracket_col, 2)
        # Bottom-left corner
        cv2.line(frame, (16, h - 34), (16 + bracket_len, h - 34), bracket_col, 2)
        cv2.line(frame, (16, h - 34), (16, h - 34 - bracket_len), bracket_col, 2)
        # Bottom-right corner
        cv2.line(frame, (w - 16, h - 34), (w - 16 - bracket_len, h - 34), bracket_col, 2)
        cv2.line(frame, (w - 16, h - 34), (w - 16, h - 34 - bracket_len), bracket_col, 2)

        # 2. Render Object Detections Across Full Camera Frame
        for d in detections:
            bbox = d.bbox
            tid = d.track_id if d.track_id is not None else id(d)

            x1 = max(0, min(w - 1, int(bbox.x1)))
            y1 = max(0, min(h - 1, int(bbox.y1)))
            x2 = max(0, min(w - 1, int(bbox.x2)))
            y2 = max(0, min(h - 1, int(bbox.y2)))

            # Any detected object triggers sudden stop -> Bold High-Visibility Red
            color = (0, 0, 245)  # Red Alert
            box_thickness = 3

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, box_thickness)

            # Label text: OBJECT NAME + CONFIDENCE % + SUDDEN STOP
            label = f"STOP: {d.class_name.upper()} {int(d.confidence * 100)}%"
            if d.track_id is not None:
                label += f" #{d.track_id}"

            (lbl_w, lbl_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
            cv2.rectangle(frame, (x1, max(0, y1 - lbl_h - 8)), (x1 + lbl_w + 8, y1), color, -1)
            cv2.putText(
                frame, label, (x1 + 4, max(12, y1 - 4)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA
            )

        # 2b. Render 3D World Destination Pin Reticle & Guideline
        if target_pin and isinstance(target_pin, dict):
            tx = int(target_pin.get("x", -1))
            ty = int(target_pin.get("y", -1))
            dist_cm = float(target_pin.get("dist_cm", 0.0))
            bearing_deg = float(target_pin.get("bearing_deg", 0.0))
            in_fov = bool(target_pin.get("in_fov", True))

            if in_fov and 0 <= tx <= w and 0 <= ty <= h:
                # Target is visible in current camera FOV
                pin_col = (0, 235, 255)  # Cyan-Gold AR reticle
                # Concentric AR target rings
                cv2.circle(frame, (tx, ty), 18, pin_col, 2, cv2.LINE_AA)
                cv2.circle(frame, (tx, ty), 5, pin_col, -1, cv2.LINE_AA)
                cv2.drawMarker(frame, (tx, ty), pin_col, cv2.MARKER_CROSS, 28, 2)
                # Dotted/solid trajectory line from vehicle bumper (bottom-center) to 3D point
                cv2.line(frame, (w // 2, h - 30), (tx, ty), (0, 235, 255), 2, cv2.LINE_AA)

                # Floating distance and bearing badge
                badge_text = f"DEST: {dist_cm:.0f}cm | {bearing_deg:+.1f} deg"
                (tw, th), _ = cv2.getTextSize(badge_text, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
                bx = max(10, min(w - tw - 16, tx - tw // 2))
                by = max(52, ty - 26)
                cv2.rectangle(frame, (bx - 4, by - th - 4), (bx + tw + 4, by + 4), (18, 18, 22), -1)
                cv2.rectangle(frame, (bx - 4, by - th - 4), (bx + tw + 4, by + 4), pin_col, 1)
                cv2.putText(frame, badge_text, (bx, by), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
            elif not in_fov or tx < 0 or tx > w:
                # Target is outside camera FOV -> Render directional off-screen beacon arrow
                arrow_x = 24 if bearing_deg < 0 else w - 24
                arrow_y = h // 2
                arrow_col = (0, 210, 255)
                cv2.circle(frame, (arrow_x, arrow_y), 16, (20, 20, 25), -1)
                cv2.circle(frame, (arrow_x, arrow_y), 16, arrow_col, 2)
                arrow_sym = "<" if bearing_deg < 0 else ">"
                cv2.putText(frame, arrow_sym, (arrow_x - 6, arrow_y + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, arrow_col, 2)
                dir_label = f"DEST {bearing_deg:+.0f} deg ({dist_cm:.0f}cm)"
                (dw, dh), _ = cv2.getTextSize(dir_label, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
                label_x = 44 if bearing_deg < 0 else w - dw - 44
                cv2.putText(frame, dir_label, (label_x, arrow_y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38, arrow_col, 1, cv2.LINE_AA)

        # 3. Top Action Decision Banner
        is_scanning = bool(decision and getattr(decision, "action", None) == ActionEnum.SCAN)
        is_blind = bool(decision and (
            getattr(decision, "source", "") == "vision_blind"
            or "DARK" in getattr(decision, "reason", "")
            or "BLANK" in getattr(decision, "reason", "")
        ) and not is_scanning)

        if is_scanning:
            action = ActionEnum.SCAN
            reason = decision.reason
            action_col = Visualizer.ACTION_COLORS.get(ActionEnum.SCAN, (200, 50, 180))
            action_text = " SCANNING "
        elif is_blind:
            action = ActionEnum.STOP
            reason = decision.reason
            action_col = (0, 0, 240)  # Bright Red
            action_text = " SUDDEN STOP "
        elif has_objects:
            action = ActionEnum.STOP
            prim_obj = max(detections, key=lambda x: x.confidence)
            reason = f"SUDDEN STOP: {prim_obj.class_name.upper()} ({int(prim_obj.confidence*100)}%) in camera feed!"
            action_col = (0, 0, 240)  # Bright Red
            action_text = " SUDDEN STOP "
        else:
            action = decision.action if decision else ActionEnum.FORWARD
            reason = decision.reason if decision else "Clear Path (Full Camera Feed Monitored)"
            action_col = Visualizer.ACTION_COLORS.get(action, (40, 180, 40))
            action_text = f" {action.value} "

        # Top banner background bar
        cv2.rectangle(frame, (0, 0), (w, 42), (18, 18, 20), -1)
        cv2.line(frame, (0, 42), (w, 42), action_col, 3)

        # Action Badge (pill)
        (aw, ah), _ = cv2.getTextSize(action_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(frame, (10, 8), (10 + aw + 10, 34), action_col, -1)
        cv2.putText(
            frame, action_text, (15, 27),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA
        )

        # Reason text
        clean_reason = reason[:62] + "..." if len(reason) > 62 else reason
        cv2.putText(
            frame, clean_reason, (25 + aw + 10, 27),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 235), 1, cv2.LINE_AA
        )

        # Center blind banner if scene is dark/blank
        if is_blind:
            bw, bh = 420, 50
            bx = (w - bw) // 2
            by = (h - bh) // 2
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (15, 15, 20), -1)
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 0, 240), 2)
            cv2.putText(
                frame, "VISION BLIND - DRIVING HALTED", (bx + 20, by + 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA
            )

        # 4. Bottom Diagnostics Bar
        cv2.rectangle(frame, (0, h - 26), (w, h), (15, 15, 18), -1)

        auto_state_text = f"AUTO: {'ACTIVE' if auto_enabled else 'STANDBY'} [{mode_str.upper()}]"
        auto_state_col = (50, 210, 90) if auto_enabled else (160, 160, 160)
        cv2.putText(
            frame, auto_state_text, (12, h - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.40, auto_state_col, 1, cv2.LINE_AA
        )

        # Pose Coordinates Display
        if current_pose and isinstance(current_pose, dict):
            px_val = current_pose.get("x", 0.0)
            py_val = current_pose.get("y", 0.0)
            ph_val = current_pose.get("heading_deg", 0.0)
            pose_text = f"POSE: ({px_val:.0f},{py_val:.0f}) {ph_val:.0f} deg"
            cv2.putText(
                frame, pose_text, (w // 2 - 110, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (100, 215, 255), 1, cv2.LINE_AA
            )

        # Last Obstacle Pose Alert Badge (if recorded)
        if last_obstacle_pose and isinstance(last_obstacle_pose, dict):
            ox = last_obstacle_pose.get("x", 0.0)
            oy = last_obstacle_pose.get("y", 0.0)
            oh = last_obstacle_pose.get("heading_deg", 0.0)
            obs_cls = str(last_obstacle_pose.get("obstacle_class", "OBSTACLE")).upper()
            obs_txt = f"OBSTACLE POSE: ({ox:.0f},{oy:.0f}) {oh:.0f} deg [{obs_cls}]"
            cv2.rectangle(frame, (10, h - 50), (10 + 260, h - 30), (20, 20, 30), -1)
            cv2.rectangle(frame, (10, h - 50), (10 + 260, h - 30), (0, 140, 255), 1)
            cv2.putText(
                frame, obs_txt, (15, h - 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 180, 255), 1, cv2.LINE_AA
            )

        # FPS & Latency
        diag_text = f"{fps:.1f} FPS | {latency_ms:.0f}ms"
        (dw, dh), _ = cv2.getTextSize(diag_text, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)
        cv2.putText(
            frame, diag_text, (w - dw - 12, h - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1, cv2.LINE_AA
        )

        return frame
