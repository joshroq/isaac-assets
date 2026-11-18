#!/usr/bin/env python3
"""Render a trial as a third-person video INSIDE the real Isaac sim.

Boots a headless Isaac stage with the ACTUAL rover + the ACTUAL full warehouse
(warehouse_basic_obstacles.usd, S3 props and all), then for each recorded trajectory
(paths/<trial>.csv: t,x,y,yaw in the map frame) it kinematically drives the real rover model
along that exact path while a chase camera records the viewport to an MP4 — so you watch the
real simulated robot navigate the real simulated environment, from a free perspective (not
an onboard camera). No physics/nav re-run; deterministic replay of the recorded path.

    isaacpy experiments/render_replay.py --all                 # G4 lidar/depth/rtab, 1 trial each
    isaacpy experiments/render_replay.py <cell_dir> [<cell_dir> ...] [--trials 1] [--fps 24]
"""
from isaacsim import SimulationApp

# res fixed here (SimulationApp needs it before Isaac imports)
WIDTH, HEIGHT = 1280, 720
app = SimulationApp({"headless": True, "renderer": "RayTracedLighting",
                     "width": WIDTH, "height": HEIGHT, "multi_gpu": False, "active_gpu": 0})

import argparse, csv, glob, math, os, sys
import numpy as np
import cv2
import omni.usd
import omni.replicator.core as rep
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Usd, UsdGeom, Gf

HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(HERE)
ROVER_USD = os.path.join(REPO, "jetson_rover_v2.usd")
WAREHOUSE = os.path.join(REPO, "warehouse_basic_obstacles.usd")
ROVER_PRIM = "/World/ugv_rover"; BASE = "/World/ugv_rover/base_footprint"
OUTDIR = os.environ.get("REPLAY_VIDEO_OUTDIR") or os.path.join(HERE, "replay_video")
DEFAULT_CELLS = ["cold_start_lidar_G4_center", "cold_start_depth_G4_center", "cold_start_rtab_G4_center",
                 "known_map_lidar_G4_center", "known_map_depth_G4_center", "known_map_rtab_G4_center"]


def _args():
    p = argparse.ArgumentParser()
    p.add_argument("cells", nargs="*")
    p.add_argument("--all", action="store_true")
    p.add_argument("--trials", type=int, default=1, help="trials per cell to render")
    p.add_argument("--fps", type=int, default=24)
    p.add_argument("--settle", type=int, default=8, help="RTX subframes per frame (denoise)")
    p.add_argument("--max-frames", type=int, default=0, help="cap frames per video (0=uncapped)")
    return p.parse_args()


def open_stage():
    ctx = omni.usd.get_context()
    if not ctx.open_stage(ROVER_USD):
        app.close(); sys.exit(f"cannot open {ROVER_USD}")
    for _ in range(12):
        app.update()
    add_reference_to_stage(usd_path=WAREHOUSE, prim_path="/World/NavEnvironment")
    for _ in range(90):     # let S3 warehouse props download/compose
        app.update()
    return ctx.get_stage()


def main():
    a = _args()
    os.makedirs(OUTDIR, exist_ok=True)
    stage = open_stage()

    # map frame = rover base_footprint start world pose
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    bw = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(BASE))
    bt = bw.ExtractTranslation()
    sx, sy = bt[0], bt[1]; syaw = math.atan2(bw[0][1], bw[0][0])

    rover = stage.GetPrimAtPath(ROVER_PRIM)
    rx = UsdGeom.Xformable(rover); rx.ClearXformOpOrder(); rov_op = rx.AddTransformOp()
    cam = UsdGeom.Camera.Define(stage, "/World/ReplayCam")
    cam.CreateFocalLengthAttr(20.0); cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 2000.0))
    cx = UsdGeom.Xformable(cam.GetPrim()); cx.ClearXformOpOrder(); cam_op = cx.AddTransformOp()
    import tempfile, shutil
    rp = rep.create.render_product("/World/ReplayCam", (WIDTH, HEIGHT))
    writer = rep.WriterRegistry.get("BasicWriter")

    def m2w(mx, my, myaw):
        c, s = math.cos(syaw), math.sin(syaw)
        return sx + mx*c - my*s, sy + mx*s + my*c, myaw + syaw

    def set_rover(wx, wy, wyaw):
        M = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(wyaw)))
        M.SetTranslateOnly(Gf.Vec3d(wx, wy, 0.0)); rov_op.Set(M)

    def set_cam(wx, wy):
        eye = Gf.Vec3d(wx - 3.2, wy - 3.2, 3.4); ctr = Gf.Vec3d(wx, wy, 0.35)
        V = Gf.Matrix4d(); V.SetLookAt(eye, ctr, Gf.Vec3d(0, 0, 1)); cam_op.Set(V.GetInverse())

    def sample(traj, fps):
        t0, t1 = traj[0][0], traj[-1][0]; dur = max(t1 - t0, 0.001)
        n = max(2, int(dur * fps)); out = []
        for i in range(n):
            t = t0 + dur * i / (n - 1); j = 0
            while j < len(traj) - 1 and traj[j+1][0] < t:
                j += 1
            aA, bB = traj[j], traj[min(j+1, len(traj)-1)]
            f = (t - aA[0]) / (bB[0] - aA[0]) if bB[0] > aA[0] else 0.0
            dy = bB[3] - aA[3]
            while dy > math.pi: dy -= 2*math.pi
            while dy < -math.pi: dy += 2*math.pi
            out.append((aA[1] + (bB[1]-aA[1])*f, aA[2] + (bB[2]-aA[2])*f, aA[3] + dy*f))
        return out

    cells = a.cells if a.cells else [os.path.join(REPO, "experiments", "data", c) for c in DEFAULT_CELLS]
    # warm up renderer
    set_rover(sx, sy, syaw); set_cam(sx, sy)
    for _ in range(20):
        app.update()

    made = []
    for cell_dir in cells:
        cell = os.path.basename(cell_dir.rstrip("/"))
        pfs = sorted(glob.glob(os.path.join(cell_dir, "paths", "*.csv")))[:a.trials]
        for pf in pfs:
            traj = [(float(r["t"]), float(r["x"]), float(r["y"]), float(r["yaw"]))
                    for r in csv.DictReader(open(pf))]
            if len(traj) < 2:
                continue
            samples = sample(traj, a.fps)
            if a.max_frames and len(samples) > a.max_frames:
                st = len(samples) / a.max_frames
                samples = [samples[min(len(samples)-1, int(i*st))] for i in range(a.max_frames)]
            tmp = tempfile.mkdtemp(prefix="rr_")
            writer.initialize(output_dir=tmp, rgb=True); writer.attach([rp])
            for (mx, my, myaw) in samples:
                wx, wy, wyaw = m2w(mx, my, myaw); set_rover(wx, wy, wyaw); set_cam(wx, wy)
                try:
                    rep.orchestrator.step(rt_subframes=a.settle)
                except TypeError:
                    rep.orchestrator.step()
            writer.detach()
            pngs = sorted(glob.glob(os.path.join(tmp, "rgb_*.png")))
            outmp4 = os.path.join(OUTDIR, os.path.splitext(os.path.basename(pf))[0] + ".mp4")
            wrote = 0
            if pngs:
                vw = cv2.VideoWriter(outmp4, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (WIDTH, HEIGHT))
                for p in pngs:
                    im = cv2.imread(p)
                    if im is None:
                        continue
                    if im.shape[1] != WIDTH or im.shape[0] != HEIGHT:
                        im = cv2.resize(im, (WIDTH, HEIGHT))
                    vw.write(im); wrote += 1
                vw.release()
            shutil.rmtree(tmp, ignore_errors=True)
            sz = os.path.getsize(outmp4) if os.path.exists(outmp4) else 0
            print(f"[render_replay] {os.path.basename(outmp4)}: {wrote} frames ({len(pngs)} pngs), {sz//1024} KB", flush=True)
            made.append(outmp4)
    app.close()
    print(f"[render_replay] done: {len(made)} videos in {OUTDIR}")


if __name__ == "__main__":
    main()
