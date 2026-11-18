#!/usr/bin/env python3
"""Send a Nav2 goal to the rover WITHOUT RViz — over SSH.

Two ways to aim it:
  * ABSOLUTE map coordinates (once you know the map layout):
        python3 send_goal.py --x 1.5 --y 0.0 --yaw 0
  * RELATIVE to where the rover is right now (no map picture needed — ideal blind
    over SSH). Reads the live map->base TF and offsets from the current pose:
        python3 send_goal.py --forward 1.0            # 1 m straight ahead
        python3 send_goal.py --forward 1.0 --left 0.5 # ahead-and-left
        python3 send_goal.py --turn 90                # rotate 90 deg in place
The goal is published on /goal_pose (exactly what RViz's "2D Nav Goal" publishes),
repeated for a couple seconds so Nav2 reliably receives it.
"""
import argparse, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

def yaw_from_quat(x,y,z,w): return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--x", type=float); p.add_argument("--y", type=float)
    p.add_argument("--yaw", type=float, default=0.0, help="absolute goal yaw (rad)")
    p.add_argument("--forward", type=float, default=None, help="relative metres ahead")
    p.add_argument("--left", type=float, default=0.0, help="relative metres to the left")
    p.add_argument("--turn", type=float, default=None, help="relative rotate (deg)")
    p.add_argument("--frame", default="map")
    p.add_argument("--base", default="base_footprint")
    p.add_argument("--topic", default="/goal_pose")
    p.add_argument("--seconds", type=float, default=2.0)
    a = p.parse_args()

    rclpy.init()
    node = rclpy.create_node("send_goal")
    relative = a.forward is not None or a.turn is not None
    gx=gy=gyaw=None
    if a.x is not None and a.y is not None:
        gx,gy,gyaw = a.x, a.y, a.yaw
    if relative:
        buf = Buffer(); TransformListener(buf, node)
        cx=cy=cyaw=None
        for _ in range(50):
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                t = buf.lookup_transform(a.frame, a.base, Time())
                cx=t.transform.translation.x; cy=t.transform.translation.y
                r=t.transform.rotation; cyaw=yaw_from_quat(r.x,r.y,r.z,r.w); break
            except Exception: continue
        if cx is None:
            print(f"ERROR: no {a.frame}->{a.base} TF yet — is SLAM/Nav2 up and tracking?"); return 1
        fwd = a.forward or 0.0; turn = math.radians(a.turn or 0.0)
        gx = cx + fwd*math.cos(cyaw) - a.left*math.sin(cyaw)
        gy = cy + fwd*math.sin(cyaw) + a.left*math.cos(cyaw)
        gyaw = cyaw + turn
        print(f"current pose ({cx:.2f},{cy:.2f},{math.degrees(cyaw):.0f}deg) -> goal ({gx:.2f},{gy:.2f},{math.degrees(gyaw):.0f}deg)")
    if gx is None:
        print("ERROR: give --x/--y (absolute) or --forward/--turn (relative)"); return 1

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(PoseStamped, a.topic, qos)
    end = time.monotonic()+a.seconds; n=0
    while rclpy.ok() and time.monotonic()<end:
        m = PoseStamped(); m.header.frame_id=a.frame
        m.header.stamp = node.get_clock().now().to_msg()
        m.pose.position.x=float(gx); m.pose.position.y=float(gy)
        m.pose.orientation.z=math.sin(gyaw/2); m.pose.orientation.w=math.cos(gyaw/2)
        pub.publish(m); n+=1; rclpy.spin_once(node, timeout_sec=0.0); time.sleep(0.1)
    print(f"published goal x={gx:.2f} y={gy:.2f} yaw={math.degrees(gyaw):.0f}deg to {a.topic} ({n}x)")
    node.destroy_node(); rclpy.shutdown(); return 0

if __name__=="__main__": raise SystemExit(main())
