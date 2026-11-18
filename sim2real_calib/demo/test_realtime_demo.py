"""ROS discovery and interruption regressions; no camera, GPU or rover required."""
import importlib.util
import os
from pathlib import Path
import signal
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from contextlib import ExitStack

import realtime_demo as demo


class DemoCleanupTest(unittest.TestCase):
    def test_partial_startup_and_cleanup_failures_still_stop_isaac(self):
        for failure in ("ros_init", "ros_close", "camera_close", "interrupt"):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as folder:
                cam = Mock(latency_s=0.0)
                isaac, ros = Mock(), Mock()
                if failure == "ros_close":
                    ros.close.side_effect = RuntimeError("ros_close")
                if failure == "camera_close":
                    cam.close.side_effect = RuntimeError("camera_close")
                ros_factory = Mock(return_value=ros)
                if failure == "ros_init":
                    ros_factory.side_effect = RuntimeError("ros_init")
                ready_error = KeyboardInterrupt() if failure == "interrupt" else None
                with ExitStack() as stack:
                    stack.enter_context(patch.object(demo, "CameraTrack", return_value=cam))
                    stack.enter_context(patch.object(demo, "Isaac", return_value=isaac))
                    stack.enter_context(patch.object(demo, "Ros", ros_factory))
                    dash = stack.enter_context(patch.object(demo, "Dashboard"))
                    stack.enter_context(patch.object(demo, "load_calib",
                                                     return_value=(None, None, (1.0, 1.0))))
                    stack.enter_context(patch.object(demo, "wait_sim_ready", side_effect=ready_error))
                    stack.enter_context(patch.object(demo, "run_one",
                                                     return_value=(None, None, 0, 0, None)))
                    stack.enter_context(patch.object(demo, "finalize", return_value=None))
                    stack.enter_context(patch("sys.argv", ["realtime_demo.py", "--tests", "T1",
                                                         "--no-real", "--outroot", folder]))
                    if failure == "interrupt":
                        self.assertEqual(demo.main(), 130)
                    else:
                        with self.assertRaisesRegex(RuntimeError, failure):
                            demo.main()
                isaac.stop.assert_called_once()
                cam.close.assert_called_once()
                dash.return_value.close.assert_called_once()

    def test_battery_keeps_one_window_and_returns_to_start_between_tests(self):
        """Every test of a real battery: drive back to its station, run, add a gallery
        tile - all in the SAME dashboard, which is only closed once at the very end."""
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            cam = Mock(latency_s=0.0)
            rover = Mock(voltage=12.0, alive=True)
            stack.enter_context(patch.object(demo, "CameraTrack", return_value=cam))
            stack.enter_context(patch.object(demo, "Isaac"))
            stack.enter_context(patch.object(demo, "Ros"))
            dash_cls = stack.enter_context(patch.object(demo, "Dashboard"))
            stack.enter_context(patch.object(demo, "load_calib",
                                             return_value=(None, None, (1.0, 1.0))))
            stack.enter_context(patch.object(demo, "wait_sim_ready"))
            stack.enter_context(patch("rover_link.RoverLink", return_value=rover))
            stack.enter_context(patch("localizer.FusedTracker"))
            goto = stack.enter_context(patch.object(demo.ST, "goto_station"))
            stack.enter_context(patch.object(demo.ST, "set_guard"))
            stack.enter_context(patch.object(demo.ST, "list_stations", return_value=[]))
            stack.enter_context(patch.object(demo, "go_to_pose"))
            run = stack.enter_context(patch.object(demo, "run_one",
                                                   return_value=("r", "s", 5, 5, (0.5, 0.5, 0))))
            stack.enter_context(patch.object(demo, "finalize", return_value=None))
            show = stack.enter_context(patch.object(demo, "show_result"))
            stack.enter_context(patch("sys.argv", ["realtime_demo.py", "--tests", "T1,T2,T4",
                                                 "--outroot", folder, "--no-countdown",
                                                 "--no-hold"]))
            self.assertEqual(demo.main(), 0)
        dash_cls.assert_called_once()                  # one window for the whole battery
        dash_cls.return_value.close.assert_called_once()
        self.assertEqual([c.args[3] for c in goto.call_args_list],
                         ["T1_straight", "T2_spin", "T4_arc"])
        self.assertEqual(run.call_count, 3)
        self.assertEqual(show.call_count, 3)


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "requires system ROS 2")
class DemoRosTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)
        env = patch.dict(os.environ, ROS_DOMAIN_ID="88", ROS_LOCALHOST_ONLY="1",
                         ROS_DISCOVERY_SERVER="127.0.0.1:11811")
        env.start()
        self.addCleanup(env.stop)

    def test_sigint_keeps_ros_alive_for_stop_and_close_is_repeatable(self):
        with demo.sim_ros_environment():
            ros = demo.Ros()
            try:
                with self.assertRaises(KeyboardInterrupt):
                    os.kill(os.getpid(), signal.SIGINT)
                self.assertTrue(ros.context.ok())
                ros.publish(0, 0)
                # Also tolerate a context already shut down by an external owner.
                ros.context.try_shutdown()
            finally:
                ros.close()
                ros.close()

    def test_discovery_matches_isaac_and_recording_logs_xyz_pose(self):
        launcher = self.work / "fake_isaac"
        launcher.write_text("""#!/usr/bin/env python3
import rclpy, time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
rclpy.init(args=[])
node=rclpy.create_node('fake_isaac')
pub=node.create_publisher(Odometry, '/demo_test_odom', 10)
node.create_subscription(Twist, '/demo_test_cmd', lambda msg: None, 10)
start=time.monotonic()
while True:
    rclpy.spin_once(node, timeout_sec=.01)
    stamp=time.monotonic()-start
    msg=Odometry()
    msg.header.stamp.sec=int(stamp)
    msg.header.stamp.nanosec=int((stamp-int(stamp))*1e9)
    msg.pose.pose.position.x=.1
    msg.pose.pose.position.y=.2
    msg.pose.pose.orientation.w=1.
    pub.publish(msg)
    time.sleep(.01)
""")
        launcher.chmod(0o755)
        with ExitStack() as cleanup:
            cleanup.enter_context(patch.object(demo, "ISAACPY", str(launcher)))
            cleanup.enter_context(demo.sim_ros_environment())
            self.assertEqual(os.environ["ROS_LOCALHOST_ONLY"], "0")
            self.assertNotIn("ROS_DISCOVERY_SERVER", os.environ)
            isaac = demo.Isaac({}, self.work)
            cleanup.callback(isaac.stop)
            ros = demo.Ros(odom_topic="/demo_test_odom", sim_topic="/demo_test_cmd")
            cleanup.callback(ros.close)
            demo.wait_sim_ready(ros, isaac, 8)
            args = SimpleNamespace(real=False, tail=0, flip_sim_y=False)
            result = demo.run_one("smoke", [dict(linear=0, angular=0, duration=1.1)],
                                  self.work / "smoke", ros, None, args)
            self.assertGreater(result[3], 1)
            self.assertGreater(Path(result[1]).stat().st_size, 20)
            cam = Mock(latency_s=0.0, status="fake")
            cam.pose.side_effect = [None, KeyboardInterrupt()]
            cam.stop_recording.return_value = ([], [])
            with self.assertRaises(KeyboardInterrupt):
                demo.run_one("interrupt", [], self.work / "interrupt", ros, cam,
                             SimpleNamespace(real=False, tail=1, flip_sim_y=False))
            cam.stop_recording.assert_called_once()     # camera recording always released
        self.assertIsNotNone(isaac.proc.poll())
        self.assertEqual(os.environ["ROS_LOCALHOST_ONLY"], "1")
        self.assertEqual(os.environ["ROS_DISCOVERY_SERVER"], "127.0.0.1:11811")

    def test_stop_kills_launcher_child(self):
        launcher = self.work / "fake_isaac"
        pidfile = self.work / "child.pid"
        launcher.write_text("#!/usr/bin/env python3\nimport subprocess, pathlib, time\n"
                            "child=subprocess.Popen(['sleep','60'])\n"
                            f"pathlib.Path({str(pidfile)!r}).write_text(str(child.pid))\n"
                            "time.sleep(60)\n")
        launcher.chmod(0o755)
        with patch.object(demo, "ISAACPY", str(launcher)):
            isaac = demo.Isaac({}, self.work)
            try:
                deadline = time.monotonic() + 15   # python start-up is slow under Isaac load
                while not pidfile.exists() and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertTrue(pidfile.exists())
            finally:
                isaac.stop()
        stat = Path(f"/proc/{int(pidfile.read_text())}/stat")
        self.assertTrue(not stat.exists() or stat.read_text().split()[2] == "Z")


if __name__ == "__main__":
    unittest.main()
