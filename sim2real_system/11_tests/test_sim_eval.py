"""ROS lifecycle regressions without a GPU: source ros2env.sh, then run unittest."""
import csv
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import sim_eval


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'requires system ROS 2')
class SimEvalTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)
        self.launcher = self.work / 'fake_isaac'
        self.env_patch = patch.dict(os.environ, ROS_DOMAIN_ID='87', ROS_LOCALHOST_ONLY='1',
                                    ROS_DISCOVERY_SERVER='127.0.0.1:11811')
        self.env_patch.start()
        self.addCleanup(self.env_patch.stop)
        self.launch_patch = patch.object(sim_eval, 'ISAACPY', str(self.launcher))
        self.launch_patch.start()
        self.addCleanup(self.launch_patch.stop)

    def launcher_script(self, code):
        self.launcher.write_text('#!/usr/bin/env python3\n' + code)
        self.launcher.chmod(0o755)

    def run_trial(self, timeout=4):
        return sim_eval.run_trial({}, [dict(linear=.2, angular=0., duration=.4),
                                      dict(linear=0., angular=.5, duration=.4)],
                                  workdir=str(self.work), startup_timeout=timeout)

    def test_exit_reports_log_and_removes_stale_result(self):
        self.launcher_script("print('intentional startup failure', flush=True)\nraise SystemExit(17)\n")
        (self.work / 'sim_run.csv').write_text('stale trajectory must not pass')
        with self.assertRaisesRegex(RuntimeError, 'code 17.*isaac.log'):
            self.run_trial()
        self.assertFalse((self.work / 'sim_run.csv').exists())
        self.assertIn('intentional startup failure', (self.work / 'isaac.log').read_text())
        self.assertEqual(os.environ['ROS_LOCALHOST_ONLY'], '1')
        self.assertEqual(os.environ['ROS_DISCOVERY_SERVER'], '127.0.0.1:11811')

    def test_timeout_stops_launcher_and_child(self):
        self.launcher_script("import subprocess, time, pathlib\n"
                             "child = subprocess.Popen(['sleep', '60'])\n"
                             f"pathlib.Path({str(self.work / 'child.pid')!r}).write_text(str(child.pid))\n"
                             "time.sleep(60)\n")
        with self.assertRaisesRegex(RuntimeError, 'startup_timeout.*isaac.log'):
            self.run_trial(timeout=.7)
        pid = int((self.work / 'child.pid').read_text())
        stat = Path(f'/proc/{pid}/stat')
        # A killed child can briefly remain a zombie until init reaps it.
        self.assertTrue(not stat.exists() or stat.read_text().split()[2] == 'Z')
        self.assertEqual(os.environ['ROS_LOCALHOST_ONLY'], '1')

    def test_sequence_uses_simulation_time_and_custom_topic(self):
        self.launcher_script('''import rclpy, time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
rclpy.init(); n=rclpy.create_node('fake_isaac')
p=n.create_publisher(Odometry, '/audit_odom', 10)
cmd=[0.,0.]
def receive(m): cmd[:]=[m.linear.x,m.angular.z]
n.create_subscription(Twist, '/cmd_vel', receive, 10)
t=0.; x=0.; y=0.
while True:
    rclpy.spin_once(n, timeout_sec=.01)
    t+=.05; x+=cmd[0]*.05; y+=cmd[1]*.05
    m=Odometry(); m.header.stamp.sec=int(t); m.header.stamp.nanosec=int((t-int(t))*1e9)
    m.pose.pose.position.x=x; m.pose.pose.position.y=y; m.pose.pose.orientation.w=1.
    p.publish(m)
    time.sleep(.01)
''')
        result = sim_eval.run_trial({}, [dict(linear=.2, duration=.4),
                                        dict(angular=.5, duration=.4)],
                                    workdir=str(self.work), odom_topic='/audit_odom',
                                    startup_timeout=8)
        with open(result) as f:
            rows = list(csv.DictReader(f))
        times = [float(r['t']) for r in rows]
        self.assertGreaterEqual(times[-1], .8)
        self.assertLess(times[-1], .9)
        self.assertTrue(all(b > a for a, b in zip(times, times[1:])))
        self.assertGreater(float(rows[-1]['x']) - float(rows[0]['x']), .04)
        self.assertGreater(float(rows[-1]['y']) - float(rows[0]['y']), .1)
        self.assertEqual(os.environ['ROS_LOCALHOST_ONLY'], '1')


if __name__ == '__main__':
    unittest.main()
