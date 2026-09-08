#!/usr/bin/env python3
"""Scale a raw /cmd_vel_raw stream to a calibrated /cmd_vel so the REAL rover reproduces
the SIM rover's motion. Feed sim/replayed commands to /cmd_vel_raw; the driver reads /cmd_vel.
  ros2 run ... OR: python3 cmd_vel_scaler.py --ros-args -p linear_scale:=1.15 -p angular_scale:=0.92
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
class Scaler(Node):
    def __init__(self):
        super().__init__('cmd_vel_scaler')
        self.kl = self.declare_parameter('linear_scale', 1.0).value
        self.ka = self.declare_parameter('angular_scale', 1.0).value
        it = self.declare_parameter('in_topic', '/cmd_vel_raw').value
        ot = self.declare_parameter('out_topic', '/cmd_vel').value
        self.pub = self.create_publisher(Twist, ot, 10)
        self.create_subscription(Twist, it, self.cb, 10)
        self.get_logger().info(f'scaling {it} -> {ot}  (linear*{self.kl}, angular*{self.ka})')
    def cb(self, m):
        o = Twist()
        o.linear.x = m.linear.x * self.kl
        o.linear.y = m.linear.y * self.kl
        o.angular.z = m.angular.z * self.ka
        self.pub.publish(o)
def main():
    rclpy.init(); n = Scaler()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.pub.publish(Twist()); n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
