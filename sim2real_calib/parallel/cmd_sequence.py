#!/usr/bin/env python3
"""Send ONE timed /cmd_vel sequence to the sim and the real rover simultaneously.

The same Twist stream drives both:
  - sim  -> /cmd_vel         (Isaac run_ground_plane.py subscribes to this)
  - real -> /cmd_vel_remote  (the rover-side deadman watchdog relays it to /cmd_vel
                              and zeroes on link loss - NEVER publish straight to the
                              rover's /cmd_vel, per rover_comms/README.md)

Publishes continuously at --rate so the deadman stays fed, then zeroes on finish or
Ctrl-C. Prints the start epoch so the run can be time-aligned with camera/odom logs.

Run from a ROS-sourced terminal (source ros2env.sh) with the discovery server up if
the real rover is involved:
    python3 cmd_sequence.py --sequence sequences/square.yaml        # both
    python3 cmd_sequence.py --sequence sequences/square.yaml --no-real   # sim only
    python3 cmd_sequence.py --sequence sequences/square.yaml --dry-run   # print only

Sequence YAML/JSON:
    name: square
    steps:
      - {linear: 0.0,  angular: 0.0, duration: 1.0}   # settle
      - {linear: 0.15, angular: 0.0, duration: 3.0}   # forward
      - {linear: 0.0,  angular: 0.6, duration: 2.6}   # turn
"""
import argparse
import json
import os
import sys
import time


def load_sequence(path):
    with open(path) as f:
        text = f.read()
    try:
        import yaml
        data = yaml.safe_load(text)
    except Exception:
        data = json.loads(text)
    steps = data["steps"]
    for s in steps:
        s.setdefault("linear", 0.0)
        s.setdefault("angular", 0.0)
        s.setdefault("duration", 0.0)
    return data.get("name", os.path.basename(path)), steps


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sequence", required=True, help="sequence YAML/JSON path")
    ap.add_argument("--rate", type=float, default=20.0, help="publish Hz (default 20)")
    ap.add_argument("--sim-topic", default="/cmd_vel")
    ap.add_argument("--real-topic", default="/cmd_vel_remote")
    ap.add_argument("--no-sim", dest="sim", action="store_false", default=True)
    ap.add_argument("--no-real", dest="real", action="store_false", default=True)
    ap.add_argument("--dry-run", action="store_true", help="print steps, don't publish")
    ap.add_argument("--log", default=None, help="write a timestamped command log here")
    args = ap.parse_args()

    name, steps = load_sequence(args.sequence)
    total = sum(s["duration"] for s in steps)
    print(f"[cmd_sequence] '{name}': {len(steps)} steps, {total:.1f}s total")
    targets = ([args.sim_topic] if args.sim else []) + ([args.real_topic] if args.real else [])
    print(f"[cmd_sequence] publishing to: {targets or '(none)'} at {args.rate} Hz")

    if args.dry_run:
        for i, s in enumerate(steps):
            print(f"  step {i}: lin={s['linear']:+.3f} ang={s['angular']:+.3f} "
                  f"dur={s['duration']:.2f}s")
        return 0

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = Node("cmd_sequence")
    pubs = []
    if args.sim:
        pubs.append(node.create_publisher(Twist, args.sim_topic, 10))
    if args.real:
        pubs.append(node.create_publisher(Twist, args.real_topic, 10))

    # Safety: the real rover has NO cmd_vel watchdog and the Waveshare base has no
    # timeout, so we MUST zero on any termination (SIGTERM/SIGHUP from an SSH drop,
    # not just Ctrl-C). Convert those signals into the same clean shutdown path.
    import signal

    def _sig(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGHUP, _sig)

    logf = open(args.log, "w") if args.log else None
    t0 = time.time()
    print(f"[cmd_sequence] START epoch={t0:.3f}  (align camera/odom logs to this)")

    def publish(lin, ang):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        for p in pubs:
            p.publish(msg)
        if logf:
            logf.write(f"{time.time()-t0:.4f},{lin:.4f},{ang:.4f}\n")

    period = 1.0 / args.rate
    try:
        for i, s in enumerate(steps):
            lin, ang, dur = s["linear"], s["angular"], s["duration"]
            print(f"[cmd_sequence] step {i}: lin={lin:+.3f} ang={ang:+.3f} {dur:.2f}s "
                  f"(t+{time.time()-t0:.1f}s)")
            t_end = time.time() + dur
            while time.time() < t_end:
                publish(lin, ang)
                time.sleep(period)
    except KeyboardInterrupt:
        print("\n[cmd_sequence] interrupted")
    finally:
        # deadman-friendly stop: hammer zero for ~0.5 s
        for _ in range(max(1, int(0.5 * args.rate))):
            publish(0.0, 0.0)
            time.sleep(period)
        print(f"[cmd_sequence] STOP (zeroed). duration {time.time()-t0:.1f}s")
        if logf:
            logf.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
