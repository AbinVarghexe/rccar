"""Offline tests for the sonar obstacle avoider (no hardware, no OpenCV)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control.obstacle_avoider import AvoidConfig, AvoidState, ObstacleAvoider  # noqa: E402

DT = 0.06  # sonar period


class Sim:
    """Feeds the avoider like the dashboard: a fresh reading every DT."""

    def __init__(self, av, left=0.6, right=0.3):
        self.av, self.t, self.left, self.right = av, 1000.0, left, right

    def step(self, dist):
        self.t += DT
        return self.av.update(self.t, dist, self.t, self.left, self.right)

    def run(self, dist_fn, seconds):
        out = []
        for _ in range(int(seconds / DT)):
            out.append(self.step(dist_fn(self.t)))
        return out


def test_open_road_never_overrides():
    sim = Sim(ObstacleAvoider())
    assert all(d.cmd is None for d in sim.run(lambda t: 200, 2.0))


def test_stale_sonar_stops():
    av = ObstacleAvoider()
    sim = Sim(av)
    sim.step(200)
    d = av.update(sim.t + 1.0, 200, sim.t, 0.5, 0.5)  # same timestamp, 1 s later
    assert d.state == AvoidState.SENSOR_FAULT and d.cmd == "STOP"


def test_missing_sonar_stops():
    d = ObstacleAvoider().update(5.0, None, None)
    assert d.cmd == "STOP"


def test_spike_is_rejected_by_median():
    sim = Sim(ObstacleAvoider())
    sim.run(lambda t: 200, 0.5)
    d = sim.step(10)  # lone spike
    assert d.cmd is None


def test_obstacle_brakes_then_turns_to_open_side_then_bypasses():
    av = ObstacleAvoider()
    sim = Sim(av, left=0.7, right=0.2)
    sim.run(lambda t: 200, 0.3)
    blocked_until = [None]

    def dist(t):
        # obstacle ahead until the car has pivoted for ~1 s of TURN time
        if av.state == AvoidState.TURN and blocked_until[0] is None:
            blocked_until[0] = t + 0.8
        if blocked_until[0] is not None and t > blocked_until[0]:
            return 150
        return 25

    out = sim.run(dist, 6.0)
    states = [d.state for d in out]
    assert AvoidState.BRAKE in states
    turn_cmds = {d.cmd for d in out if d.state == AvoidState.TURN}
    assert "LEFT" in turn_cmds and "RIGHT" not in turn_cmds  # left was clearer
    assert AvoidState.BYPASS in states
    assert states[-1] in (AvoidState.CLEAR, AvoidState.CAUTION)
    assert out[-1].cmd is None  # navigation has control again


def test_boxed_in_ends_stuck_with_stop_and_never_spins_forever():
    av = ObstacleAvoider()
    sim = Sim(av)
    out = sim.run(lambda t: 20, 25.0)
    assert out[-1].state == AvoidState.STUCK and out[-1].cmd == "STOP"
    assert AvoidState.BACKUP in [d.state for d in out]


def test_stuck_releases_when_path_opens():
    av = ObstacleAvoider()
    sim = Sim(av)
    sim.run(lambda t: 20, 25.0)
    out = sim.run(lambda t: 200, 1.0)
    assert out[-1].state in (AvoidState.CLEAR, AvoidState.CAUTION)


def test_repeated_brakes_latch_stuck():
    cfg = AvoidConfig(max_attempts=2)
    av = ObstacleAvoider(cfg)
    sim = Sim(av)
    # obstacle returns right after every bypass
    out = sim.run(lambda t: 150 if av.state == AvoidState.TURN else 25, 15.0)
    assert AvoidState.STUCK in [d.state for d in out]


def test_caution_creeps_forward_in_pulses():
    sim = Sim(ObstacleAvoider())
    out = sim.run(lambda t: 55, 2.0)
    gates = {d.gate_forward for d in out}
    assert gates == {True, False} and all(d.cmd is None for d in out)


def test_equal_sides_keep_previous_direction():
    av = ObstacleAvoider()
    assert av._choose_side(10.0, 0.5, 0.5) == "RIGHT"
    av._begin_turn(10.0, "LEFT", False)
    assert av._choose_side(12.0, 0.5, 0.52) == "LEFT"
