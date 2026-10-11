import time
from enum import Enum
from typing import List, Optional, Tuple
from dataclasses import dataclass, field

class ActionEnum(str, Enum):
    FORWARD = "FORWARD"
    STOP = "STOP"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    SLOW = "SLOW"
    WAIT = "WAIT"
    SCAN = "SCAN"
    UNKNOWN = "UNKNOWN"

@dataclass
class BoundingBox:
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def center_x(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def center_y(self) -> float:
        return (self.y1 + self.y2) / 2.0

    @property
    def width(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def area(self) -> float:
        return self.width * self.height

@dataclass
class DetectionResult:
    class_name: str
    confidence: float
    bbox: BoundingBox
    track_id: Optional[int] = None

@dataclass
class DecisionResult:
    action: ActionEnum
    reason: str
    timestamp: float = field(default_factory=time.time)
    hazard_detected: bool = False
    obstacle_side: Optional[str] = None  # "left", "right", "center", None
    clearance_left: float = 1.0          # 0.0 to 1.0 (1.0 = completely open)
    clearance_right: float = 1.0         # 0.0 to 1.0
    suggested_throttle: float = 0.55
    suggested_steering: float = 0.0      # -1.0 (left) to +1.0 (right)
    source: str = "local"                # "local", "server", "cv_corridor"
