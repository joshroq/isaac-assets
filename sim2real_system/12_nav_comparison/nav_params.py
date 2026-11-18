#!/usr/bin/env python3
"""Build the ONE Nav2 parameter file both rovers run.

Why this exists: the real rover's `start_lidar_nav.sh` loads `ugv_nav/param/slam_nav.yaml`
(an older ugv_nav on the Jetson) while the Isaac launches load `ugv_nav/params/teb.yaml`,
so "same nav stack" was never actually guaranteed. For a sim-vs-real comparison the stack
must be byte-identical on both sides, so the session generates one params file from the
desktop's `ugv_nav/params/teb.yaml`, ships it to the rover AND uses it for Isaac, and
records its sha256 in every condition's meta.json.

Edits to teb.yaml (the planner, controller, smoother, behaviours and costmap sizes are NOT
touched; anything else needs an explicit `nav_overrides` in the session config):
  * keepout / speed costmap filters and their mask/info servers removed (no masks here);
  * map_server / amcl / map_saver sections removed (live SLAM, nothing to load);
  * behaviour-tree XMLs point at the copies shipped in the bundle (`@NAVCMP_DIR@`), so the
    rover's older ugv_nav share directory is never consulted;
  * stack 'depth': the depth XYZ cloud (/navcmp/depth_points) is added to both costmaps'
    obstacle layer with the same settings rover_nav/inject_depth.py and
    isaac_nav_depth.launch.py already use (0.08-1.5 m, 4 m range).
Topics stay plain (/scan, /odom); the sim side remaps them in the launch file.

    python3 navcmp/nav_params.py --stack lidar --out /tmp/nav2_lidar.yaml
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import shutil
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
UGV_NAV = REPO / "ugv_ws" / "src" / "ugv_main" / "ugv_nav"
SOURCE_PARAMS = UGV_NAV / "params" / "teb.yaml"
SOURCE_BT = UGV_NAV / "behavior_trees"
SLAM_SOURCE = REPO / "ugv_ws" / "src" / "ugv_main" / "ugv_slam" / "config" / "slam_toolbox_online_sync.yaml"
DEPTH_TOPIC = "/navcmp/depth_points"
DEPTH_SOURCE = {
    "topic": DEPTH_TOPIC, "data_type": "PointCloud2", "marking": True, "clearing": True,
    "min_obstacle_height": 0.08, "max_obstacle_height": 1.5,
    "obstacle_max_range": 4.0, "obstacle_min_range": 0.0,
    "raytrace_max_range": 4.5, "raytrace_min_range": 0.0,
}
BT_FILES = ("navigate_to_pose_w_replanning_goal_patience_and_recovery.xml",
            "navigate_through_poses_w_replanning_and_recovery.xml")
DROP_SECTIONS = ("amcl", "amcl_map_client", "amcl_rclcpp_node", "map_server", "map_saver",
                 "keepout_filter_mask_server", "keepout_costmap_filter_info_server",
                 "speed_filter_mask_server", "speed_costmap_filter_info_server",
                 "collision_monitor")   # Humble's navigation_launch.py does not start it


def _costmap(data, name):
    return data[name][name]["ros__parameters"]


def _merge(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _merge(dst[k], v)
        else:
            dst[k] = v


def _apply_overrides(data, overrides):
    """nav_overrides: {node: {param: value}} merged into that node's ros__parameters, e.g.
    {controller_server: {FollowPath: {max_vel_x: 0.2}}, local_costmap: {inflation_layer:
    {inflation_radius: 0.25}}}. Applied identically to both rovers."""
    for node, params in (overrides or {}).items():
        if node in ("local_costmap", "global_costmap"):
            target = _costmap(data, node)
        else:
            target = data.setdefault(node, {}).setdefault("ros__parameters", {})
        _merge(target, params)


def build(stack: str, overrides: dict | None = None, source: Path = SOURCE_PARAMS) -> dict:
    if stack not in ("lidar", "depth"):
        raise ValueError("stack must be 'lidar' or 'depth' (visual stacks are out of scope)")
    data = copy.deepcopy(yaml.safe_load(Path(source).read_text()))
    for sec in DROP_SECTIONS:
        data.pop(sec, None)
    bt = data["bt_navigator"]["ros__parameters"]
    bt["default_nav_to_pose_bt_xml"] = f"@NAVCMP_DIR@/stack/bt/{BT_FILES[0]}"
    bt["default_nav_through_poses_bt_xml"] = f"@NAVCMP_DIR@/stack/bt/{BT_FILES[1]}"
    for name in ("local_costmap", "global_costmap"):
        p = _costmap(data, name)
        p.pop("filters", None)
        p.pop("keepout_filter", None)
        p.pop("speed_filter", None)
        ol = p["obstacle_layer"]
        if ol.get("scan", {}).get("topic") != "/scan":
            raise ValueError(f"{name} scan source is not /scan in {source}")
        if stack == "depth":
            srcs = ol.get("observation_sources", "scan")
            names = srcs.split() if isinstance(srcs, str) else list(srcs)
            if "depth_cloud" not in names:
                names.append("depth_cloud")
            ol["observation_sources"] = " ".join(names)
            ol["depth_cloud"] = dict(DEPTH_SOURCE)
    _apply_overrides(data, overrides)
    return data


def write_stack(dest_dir: Path, stack: str, overrides: dict | None = None) -> dict:
    """Write the bundle's stack/ files; return {file: sha256} for meta.json."""
    dest = Path(dest_dir) / "stack"
    (dest / "bt").mkdir(parents=True, exist_ok=True)
    params = build(stack, overrides)
    text = ("# GENERATED by sim2real_calib/navcmp/nav_params.py from ugv_nav/params/teb.yaml.\n"
            "# Identical on the real rover and in Isaac; @NAVCMP_DIR@ is filled in at launch.\n"
            + yaml.safe_dump(params, sort_keys=False))
    (dest / f"nav2_{stack}.yaml").write_text(text)
    for f in BT_FILES:
        shutil.copyfile(SOURCE_BT / f, dest / "bt" / f)
    shutil.copyfile(SLAM_SOURCE, dest / "slam_toolbox.yaml")
    shutil.copyfile(HERE / "stack" / "navcmp_stack.launch.py", dest / "navcmp_stack.launch.py")
    hashes = {}
    for p in sorted(dest.rglob("*")):
        if p.is_file():
            hashes[str(p.relative_to(dest_dir))] = hashlib.sha256(p.read_bytes()).hexdigest()
    return hashes


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stack", choices=("lidar", "depth"), default="lidar")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    Path(a.out).write_text(yaml.safe_dump(build(a.stack), sort_keys=False))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
