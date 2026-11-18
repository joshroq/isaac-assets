"""Homing stall diagnosis, no hardware: a rover that stops answering is a LINK problem
(WiFi drop - its deadman stopped it), one that answers but does not move is a motor/power
problem. 2026-11-03 a WiFi drop was reported as "motors unpowered?".

    python3 -m pytest -q sim2real_calib/loop/tests/test_stall_diagnosis.py
"""
import math
import sys
import time
from pathlib import Path

import pytest

LOOP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LOOP))
import homing  # noqa: E402


class StuckRover:
    """Accepts commands but never moves; rx_age says whether the agent still answers."""

    def __init__(self, replying):
        self.replying = replying
        self.alive = True
        self.t0 = time.time()

    @property
    def rx_age(self):
        return 0.1 if self.replying else time.time() - self.t0

    def velocity(self, lin, ang):
        pass

    def stop(self):
        pass


class StillCam:
    fused = False
    status = "stub camera"

    def pose(self, max_age=0.35):
        return (time.time(), 0.5, 0.2, math.pi / 2)

    def settled_pose(self, duration=0.6, max_age=0.4):
        return self.pose()


def cfg():
    c = homing.load_config()
    c["homing"]["attempts"] = 1
    return c


def test_silent_rover_is_reported_as_link_stall():
    with pytest.raises(homing.HomingError, match="link to the rover stalled"):
        homing.go_to_pose(StuckRover(replying=False), StillCam(), (0.5, 0.5, math.pi / 2),
                          cfg(), log=lambda *a: None)


def test_answering_but_motionless_rover_is_reported_as_motors():
    with pytest.raises(homing.HomingError, match="motors"):
        homing.go_to_pose(StuckRover(replying=True), StillCam(), (0.5, 0.5, math.pi / 2),
                          cfg(), log=lambda *a: None)


class DeadDriverRover(StuckRover):
    """Agent answers, but the base driver behind it died (its ssh dropped)."""
    base_alive = False


def test_dead_base_driver_is_reported_as_such():
    with pytest.raises(homing.HomingError, match="base driver stopped"):
        homing.go_to_pose(DeadDriverRover(replying=True), StillCam(), (0.5, 0.5, math.pi / 2),
                          cfg(), log=lambda *a: None)
