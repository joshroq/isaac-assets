#!/usr/bin/env python3
"""Measure how far the REAL rover actually moves for a known command, and compute the
scale that makes it match the intended (sim) motion. Drives the rover -> keep on blocks
OFF only when measuring real distance; clear space + spotter. Needs batteries (motors).

  # forward: drive 0.15 m/s for 4 s, then tape-measure the real distance
  python3 calibrate_motion.py --linear 0.15 --time 4
  # turn:    spin 0.5 rad/s for 4 s, then measure the real angle turned
  python3 calibrate_motion.py --angular 0.5 --time 4
It prints commanded vs odom-measured, asks you for the TAPE-measured real value, and
prints the scale to put in calibration.yaml (and can write it with --save)."""
import argparse, math, time, sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
class Cal(Node):
    def __init__(self,a):
        super().__init__('calibrate_motion'); self.a=a
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.buf=Buffer(); TransformListener(self.buf,self)
    def pose(self):
        try:
            t=self.buf.lookup_transform('odom','base_footprint',Time())
            return (t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation))
        except Exception: return None
    def spin(self,s):
        e=time.monotonic()+s
        while rclpy.ok() and time.monotonic()<e: rclpy.spin_once(self,timeout_sec=0.02)
    def drive(self,v,w,T):
        m=Twist(); m.linear.x=float(v); m.angular.z=float(w)
        e=time.monotonic()+T
        while rclpy.ok() and time.monotonic()<e:
            self.pub.publish(m); self.spin(0.05)
        self.pub.publish(Twist()); self.spin(0.3)
    def run(self):
        self.spin(1.0); p0=self.pose()
        if p0 is None: print('No odom TF (is bringup running?)'); return 1
        self.drive(self.a.linear,self.a.angular,self.a.time)
        p1=self.pose()
        d=math.hypot(p1[0]-p0[0],p1[1]-p0[1]); dth=abs((p1[2]-p0[2]+math.pi)%(2*math.pi)-math.pi)
        if self.a.angular!=0:
            cmd=abs(self.a.angular*self.a.time); print(f'\ncommanded angle={math.degrees(cmd):.1f} deg  odom-measured={math.degrees(dth):.1f} deg')
            real=input('Tape/protractor-measured REAL angle turned (deg): ')
            try: r=math.radians(float(real)); print(f'>>> angular_scale = {cmd/r:.4f}  (odom/real={dth/r:.3f})')
            except: print('skip'); r=None
        else:
            cmd=abs(self.a.linear*self.a.time); print(f'\ncommanded dist={cmd:.3f} m  odom-measured={d:.3f} m')
            real=input('Tape-measured REAL distance (m): ')
            try: r=float(real); print(f'>>> linear_scale = {cmd/r:.4f}  (odom/real={d/r:.3f})')
            except: print('skip')
        return 0
def main():
    p=argparse.ArgumentParser()
    p.add_argument('--linear',type=float,default=0.0); p.add_argument('--angular',type=float,default=0.0)
    p.add_argument('--time',type=float,default=4.0); a=p.parse_args()
    rclpy.init(); n=Cal(a)
    try: return n.run()
    finally:
        n.pub.publish(Twist()); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': sys.exit(main())
