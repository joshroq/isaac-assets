#!/usr/bin/env python3
"""Record a /cmd_vel time-series to CSV (t,vx,wz). Run on the SIM machine during an
obstacle-avoidance run (subscribe to the sim's /cmd_vel), or on the rover.
  python3 record_cmdvel.py run1.csv --topic /cmd_vel"""
import sys, time, argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
class Rec(Node):
    def __init__(self,f,topic):
        super().__init__('record_cmdvel'); self.f=open(f,'w'); self.f.write('t,vx,wz\n')
        self.t0=None; self.create_subscription(Twist,topic,self.cb,50)
        self.get_logger().info(f'recording {topic} -> {f} (Ctrl-C to stop)')
    def cb(self,m):
        t=time.monotonic()
        if self.t0 is None: self.t0=t
        self.f.write(f'{t-self.t0:.3f},{m.linear.x:.4f},{m.angular.z:.4f}\n'); self.f.flush()
def main():
    p=argparse.ArgumentParser(); p.add_argument('file'); p.add_argument('--topic',default='/cmd_vel'); a=p.parse_args()
    rclpy.init(); n=Rec(a.file,a.topic)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.f.close(); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
