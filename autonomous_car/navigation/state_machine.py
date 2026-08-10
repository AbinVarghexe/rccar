"""
Navigation Finite State Machine (FSM).
"""

import os
import sys
import threading
from enum import Enum
from typing import Optional, Callable, List

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
root_dir = os.path.dirname(parent_dir)
for p in [curr_dir, parent_dir, root_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from utils.logger import car_logger
except ImportError:
    from autonomous_car.utils.logger import car_logger

class State(str, Enum):
    IDLE = "IDLE"
    MANUAL_DRIVE = "MANUAL_DRIVE"
    FOLLOW_LANE = "FOLLOW_LANE"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    OBSTACLE = "OBSTACLE"
    SCANNING_ENVIRONMENT = "SCANNING_ENVIRONMENT"
    STOP = "STOP"
    NAVIGATING_WAYPOINTS = "NAVIGATING_WAYPOINTS"
    WAYPOINT_REACHED = "WAYPOINT_REACHED"

class NavigationStateMachine:
    def __init__(self, initial_state: State = State.IDLE) -> None:
        self._current_state: State = initial_state
        self._previous_state: State = initial_state
        self._lock = threading.Lock()
        self._on_state_change_callbacks: List[Callable[[State, State], None]] = []

        car_logger.info(f"Navigation State Machine initialized in [{self._current_state.value}] state.")

    @property
    def current_state(self) -> State:
        with self._lock:
            return self._current_state

    @property
    def previous_state(self) -> State:
        with self._lock:
            return self._previous_state

    def register_callback(self, callback: Callable[[State, State], None]) -> None:
        self._on_state_change_callbacks.append(callback)

    def transition_to(self, new_state: State, reason: str = "") -> bool:
        with self._lock:
            if self._current_state == new_state:
                return False

            old_state = self._current_state
            self._previous_state = old_state
            self._current_state = new_state

            reason_str = f" Reason: {reason}" if reason else ""
            car_logger.info(
                f"[STATE TRANSITION]: [{old_state.value}] -> [{new_state.value}].{reason_str}"
            )

        for cb in self._on_state_change_callbacks:
            try:
                cb(old_state, new_state)
            except Exception as e:
                car_logger.error(f"Error in state change callback: {e}")

        return True

    def reset(self) -> None:
        self.transition_to(State.IDLE, reason="System Reset")
