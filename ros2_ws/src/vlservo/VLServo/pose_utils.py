#!/usr/bin/env python3
"""
Lightweight helpers for setting robot start pose and head tilt quickly
from the GUI without spinning up long-running processes.

Notes:
- Uses stretch_body to command the robot directly. If not available or
  running off-robot, functions will raise and the caller should handle.
- Head tilt angles are provided in degrees for user convenience and
  converted to radians for the underlying API.
"""

import math
from typing import Optional

try:
    import stretch_body.robot as rb
except Exception:  # pragma: no cover
    rb = None


def _with_robot():
    if rb is None:
        raise RuntimeError("stretch_body not available on this machine")
    robot = rb.Robot()
    robot.startup()
    return robot


def set_head_tilt_deg(deg: float) -> None:
    """Set head tilt (pitch) angle in degrees immediately; 0 deg looks forward.

    Negative tilts look down on most Stretch robots.
    """
    robot = _with_robot()
    try:
        tilt_rad = float(deg) * math.pi / 180.0
        if hasattr(robot, 'head'):
            robot.head.move_to('head_tilt', tilt_rad)
        robot.push_command(); robot.wait_command()
    finally:
        try:
            robot.stop()
        except Exception:
            pass


def go_to_start_pose(head_tilt_deg: Optional[float] = 0.0) -> None:
    """Move to a navigation start pose: head forward, gripper forward.

    - Head pan -> 0.0 rad (forward)
    - Head tilt -> provided degrees (default 0 deg = forward)
    - Wrist yaw -> 0.0 rad if available (gripper facing forward)
    - Optionally position arm/lift to safe nominal values
    """
    robot = _with_robot()
    try:
        if hasattr(robot, 'head'):
            robot.head.move_to('head_pan', 0.0)
            try:
                tilt_rad = float(head_tilt_deg or 0.0) * math.pi / 180.0
                robot.head.move_to('head_tilt', tilt_rad)
            except Exception:
                pass
        # Align gripper forward if wrist yaw joint exists
        try:
            if hasattr(robot, 'end_of_arm'):
                j = robot.end_of_arm.get_joint('wrist_yaw')
                if j is not None:
                    j.move_to(math.pi / 2.0)
        except Exception:
            pass
        # Nominal arm/lift/gripper
        try:
            if hasattr(robot, 'arm'):
                robot.arm.move_to(0.01)
        except Exception:
            pass
        try:
            if hasattr(robot, 'lift'):
                robot.lift.move_to(0.7)
        except Exception:
            pass
        try:
            if hasattr(robot, 'end_of_arm'):
                robot.end_of_arm.get_joint('stretch_gripper').move_to(10.46)
        except Exception:
            pass
        robot.push_command(); robot.wait_command()
    finally:
        try:
            robot.stop()
        except Exception:
            pass

