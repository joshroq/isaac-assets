#!/usr/bin/env python3
"""cmd_vel watchdog / deadman for remote (WAN) teleoperation of the rover.

Why: the Waveshare driver (ugv_ws/.../ugv_bringup/ugv_driver.py) forwards the
last /cmd_vel to the ESP32 and returns — it has NO timeout. If the WAN link
drops while a nonzero command was in flight, the rover keeps moving. This node
runs ON THE ROVER, so it can stop the base even when the desktop disconnects.

Design (interposition, unambiguous):
    desktop teleop  --publish-->  in_topic  (default /cmd_vel_remote)
                                     |
                                cmd_vel_watchdog  (on the rover)
                                     |
                                out_topic (default /cmd_vel)  --> Waveshare driver

  * Every message on in_topic is passed straight through to out_topic and
    resets the deadman timer.
  * If no in_topic message arrives for `cmd_timeout` s while "armed", the node
    publishes zero-velocity to out_topic at `rate` Hz -> the local driver stops
    the base (this works because the publisher is LOCAL, unaffected by the WAN).
  * The node only arms after it has seen at least one in_topic message, and it
    disarms after `disarm_after` s of continuous silence (robot already stopped),
    releasing out_topic so rover-local nav can drive without interference.

Rover-local Nav2 publishes /cmd_vel directly (no WAN in the loop) and does not
need this; keep remote teleop and local nav from running at the same time.

Params (ROS params, all overridable):
    in_topic (str)      default '/cmd_vel_remote'
    out_topic (str)     default '/cmd_vel'
    cmd_timeout (float) default 0.5   seconds before stop-spam begins
    disarm_after (float)default 3.0   seconds of silence before releasing
    rate (float)        default 20.0  Hz for the stop publisher / check timer
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__("cmd_vel_watchdog")
        self.in_topic = self.declare_parameter("in_topic", "/cmd_vel_remote").value
        self.out_topic = self.declare_parameter("out_topic", "/cmd_vel").value
        self.cmd_timeout = float(self.declare_parameter("cmd_timeout", 0.5).value)
        self.disarm_after = float(self.declare_parameter("disarm_after", 3.0).value)
        self.rate = float(self.declare_parameter("rate", 20.0).value)

        self._last_cmd = None      # monotonic time of last in_topic msg
        self._armed = False
        self._stopping = False     # currently spamming zeros

        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.sub = self.create_subscription(Twist, self.in_topic, self._on_cmd, 10)
        self.timer = self.create_timer(1.0 / max(self.rate, 1.0), self._on_timer)

        self.get_logger().info(
            f"cmd_vel watchdog: {self.in_topic} -> {self.out_topic} | "
            f"stop after {self.cmd_timeout}s, release after {self.disarm_after}s"
        )

    def _on_cmd(self, msg: Twist):
        # pass the command straight through and pet the dog
        self.pub.publish(msg)
        self._last_cmd = time.monotonic()
        self._armed = True
        if self._stopping:
            self._stopping = False
            self.get_logger().info("commands resumed — releasing stop")

    def _on_timer(self):
        if not self._armed or self._last_cmd is None:
            return
        idle = time.monotonic() - self._last_cmd
        if idle > self.disarm_after:
            # robot has been commanded to stop long enough; release the topic
            if self._armed:
                self._armed = False
                self._stopping = False
                self.get_logger().warn(
                    f"no commands for {idle:.1f}s — disarming (out_topic released)"
                )
            return
        if idle > self.cmd_timeout:
            if not self._stopping:
                self._stopping = True
                self.get_logger().warn(
                    f"/cmd_vel stale for {idle:.2f}s — STOPPING rover (deadman)"
                )
            self.pub.publish(Twist())  # all-zero = stop


def main():
    rclpy.init()
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # best-effort final stop
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
