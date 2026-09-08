#!/usr/bin/env python3
"""Replay a recorded cmd_vel CSV so the REAL rover follows the SIM's velocity profile and
ends at the same spot. Publishes to /cmd_vel_raw by default -> run cmd_vel_scaler.py to
apply the calibration -> /cmd_vel. (Use --topic /cmd_vel to skip scaling.)
  python3 replay_cmdvel.py run1.csv                 # through the scaler (recommended)
KEEP CLEAR + spotter; needs batteries."""
import sys, time, csv, argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
def main():
    p=argparse.ArgumentParser(); p.add_argument('file'); p.add_argument('--topic',default='/cmd_vel_raw')
    p.add_argument('--rate',type=float,default=20.0); a=p.parse_args()
    rows=[(float(r['t']),float(r['vx']),float(r['wz'])) for r in csv.DictReader(open(a.file))]
    rclpy.init(); n=rclpy.create_node('replay_cmdvel'); pub=n.create_publisher(Twist,a.topic,10)
    print(f'replaying {len(rows)} samples over {rows[-1][0]:.1f}s to {a.topic}')
    t0=time.monotonic(); i=0; dt=1.0/a.rate
    while rclpy.ok() and i<len(rows):
        now=time.monotonic()-t0
        while i<len(rows)-1 and rows[i+1][0]<=now: i+=1
        m=Twist(); m.linear.x=rows[i][1]; m.angular.z=rows[i][2]; pub.publish(m)
        if now>rows[-1][0]: break
        time.sleep(dt)
    pub.publish(Twist()); print('done'); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
