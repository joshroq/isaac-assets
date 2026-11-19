#!/usr/bin/env python3
"""Overhead C920e exposure/gain on the Pi, tuned for sticker colour.

On auto exposure, harsh light washes the orange-red sticker out to near-white (it then
drops out mid-spin), and dim light pushes the gain to its maximum (grainy, lower frame
rate). A fixed exposure + gain chosen for the stickers fixes both, but it is tied to the
lighting: change the light -> re-run --tune, then calibrate_stickers.py.

    python3 calib/camera_exposure.py --show                 # current camera controls
    python3 calib/camera_exposure.py --tune                 # click the 3 stickers, sweep
                                                            #   exposure x gain, apply + save best
    python3 calib/camera_exposure.py --apply                # re-apply the saved setting
    python3 calib/camera_exposure.py --set 300 --gain 200   # manual
    python3 calib/camera_exposure.py --auto                 # back to the camera's auto exposure

The setting is saved to calib/camera_settings.json (with the sticker click points, which
calibrate_stickers.py --points-px can reuse). It lives in the camera, not on disk on the
Pi: a Pi reboot or camera replug resets it to auto, so preflight re-applies it.
Needs the rover parked with all three stickers visible, and nothing else reading UDP 5000.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SETTINGS = HERE / "camera_settings.json"
NAMES = ["green", "orange_red", "pink"]
# LAN mDNS first, then Tailscale (MagicDNS name, then its IP); PI_HOST overrides.
PI_HOSTS = [h for h in os.environ.get("PI_HOST", "").split(",") if h] or [
    "joshu@pi-cam.local", "joshu@pi-cam", "joshu@100.67.148.19"]
SSHOPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
           "-o", "StrictHostKeyChecking=accept-new"]
# same device choice as pi_sender/stream_camera.sh
DEV = ('D=""; for d in /dev/v4l/by-id/*C920e*index0 /dev/v4l/by-id/*C920*index0; do '
       '[ -e "$d" ] && D="$d" && break; done; D="${D:-/dev/video0}"; ')
EXPOSURES = [39, 78, 100, 156, 200, 250, 312]     # 100 us units; <= 333 keeps 30 fps
GAINS = [32, 96, 160, 224]
_host = None


# Run a shell command on the Pi over SSH, with $D already set to the C920e device
# node (DEV prefix). Tries each host in PI_HOSTS (LAN, then Tailscale) and remembers
# the first that answers in _host. ssh exit code 255 = could not connect -> next host.
def pi(cmd, timeout=15):
    """Run cmd on the Pi (first reachable host). -> stdout; raises RuntimeError."""
    global _host
    for host in ([_host] if _host else PI_HOSTS):
        try:
            ssh_result = subprocess.run(["ssh", *SSHOPTS, host, DEV + cmd], capture_output=True,
                               text=True, timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        if ssh_result.returncode == 255:            # ssh itself failed: try the next route
            continue
        _host = host
        if ssh_result.returncode != 0:
            raise RuntimeError(f"Pi command failed: {ssh_result.stderr.strip() or ssh_result.stdout.strip()}")
        return ssh_result.stdout
    raise RuntimeError(f"cannot reach the Pi over SSH ({', '.join(PI_HOSTS)})")


# Current camera controls as a dict, e.g. {'auto_exposure': '1', 'gain': '200', ...}.
def show():
    controls_text = pi('v4l2-ctl -d "$D" -C auto_exposure -C exposure_time_absolute -C gain')
    return {control_name.strip(): control_value.strip().split()[0] for control_name, control_value in
            (line.split(":", 1) for line in controls_text.splitlines() if ":" in line)}


# Manual exposure (units of 100 us) and gain (0-255). The camera keeps these until
# it is unplugged or the Pi reboots.
def set_manual(exposure, gain):
    pi(f'v4l2-ctl -d "$D" -c auto_exposure=1 -c exposure_time_absolute={int(exposure)} '
       f'-c gain={int(gain)}')


# Back to the camera's own auto exposure (v4l2 auto_exposure=3, aperture priority).
def set_auto():
    pi('v4l2-ctl -d "$D" -c auto_exposure=3')


# calib/camera_settings.json as a dict, or None if it does not exist yet.
def load_settings():
    return json.loads(SETTINGS.read_text()) if SETTINGS.exists() else None


# Push the saved setting to the camera (preflight does this every run, because a Pi
# reboot silently resets the camera to auto exposure).
def apply_saved(log=print):
    settings = load_settings()
    if not settings:
        log("no calib/camera_settings.json - camera left as it is")
        return None
    if settings.get("mode") == "auto":
        set_auto()
    else:
        set_manual(settings["exposure"], settings["gain"])
    log(f"camera: applied {describe(settings)}")
    return settings


# Human-readable summary of a settings dict.
def describe(s):
    return ("auto exposure" if s.get("mode") == "auto"
            else f"manual exposure {s['exposure']} gain {s['gain']}")


# Collect n NEW frames captured after a setting change. The frame counter from
# LatestFrame.read() tells new frames apart; `skip` frames (~1 s) are thrown away first
# because the stream arrives ~0.6 s late and those frames predate the change.
def fresh_frames(source, n=3, skip=30, timeout=8.0):
    """n frames captured after the change took effect: the stream arrives ~0.6 s late
    (measured camera latency), so skip ~1 s of frames that predate the new setting."""
    _, _, start_counter = source.read()
    frames, last_counter, deadline = [], start_counter, time.time() + timeout
    while len(frames) < n and time.time() < deadline:
        frame, _, frame_counter = source.read()
        if frame is not None and frame_counter != last_counter and frame_counter - start_counter > skip:
            frames.append(frame.copy())
            last_counter = frame_counter
        time.sleep(0.01)
    return frames


# For each clicked sticker centre: take a (2*radius+1)^2 pixel patch in every frame,
# keep the 12 pixels with the highest saturation*brightness (the sticker's core, not the
# chassis around it) and return (median saturation S, median brightness V), each 0-255.
def sticker_quality(frames, points, radius=4):
    """Per sticker (median S, median V) of its 12 most saturated*bright pixels."""
    import cv2
    sticker_rows = []
    for px_x, px_y in points:
        px_x, px_y = int(round(px_x)), int(round(px_y))
        brightest_pixels = []
        for frame in frames:
            patch_hsv = cv2.cvtColor(frame[px_y-radius:px_y+radius+1, px_x-radius:px_x+radius+1], cv2.COLOR_BGR2HSV)
            hsv_pixels = patch_hsv.reshape(-1, 3).astype(float)
            brightest_pixels.append(hsv_pixels[np.argsort(hsv_pixels[:, 1] * hsv_pixels[:, 2])[-12:]])
        hsv_pixels = np.concatenate(brightest_pixels)
        sticker_rows.append((float(np.median(hsv_pixels[:, 1])), float(np.median(hsv_pixels[:, 2]))))
    return sticker_rows


# One number per camera setting - higher is better. Per sticker:
#   S * min(1, V/150) * (0.75 if V is clipped at >= 252)
# i.e. strong colour, bright enough, not blown out. The WORST sticker decides (all three
# must be detectable), minus 0.05 per gain step to prefer less noisy settings on ties.
def score(rows, gain):
    """Worst sticker decides: saturated colour, not clipped, not dark; mild gain penalty."""
    sticker_scores = []
    for saturation, brightness in rows:
        brightness_factor = min(1.0, brightness / 150.0)
        clip_factor = 0.75 if brightness >= 252 else 1.0
        sticker_scores.append(saturation * brightness_factor * clip_factor)
    return min(sticker_scores) - 0.05 * gain


# Exposure tuning: 1) switch to auto exposure and let the user click the 3 stickers
# (green, orange-red, pink); 2) try every exposure x gain in the sweep, scoring each with
# score(); 3) apply the best, save it with the click points to calib/camera_settings.json.
def tune(points=None, exposures=EXPOSURES, gains=GAINS, log=print):
    sys.path.insert(0, str(HERE.parent / "capture"))
    from stream import LatestFrame
    set_auto()
    time.sleep(2.0)
    camera_stream = LatestFrame()
    try:
        first_frame = camera_stream.wait_first(20) if camera_stream.opened() else None
        if first_frame is None:
            raise RuntimeError("no overhead frames on UDP 5000 (stream off, or another "
                               "program is reading it)")
        if points is None:
            from calibrate_stickers import click_points
            time.sleep(1.0)
            frame, _, _ = camera_stream.read()
            points = click_points(frame)
            if points is None:
                raise RuntimeError("cancelled")
        points = [tuple(map(float, point)) for point in points]
        sweep_results = []
        auto_exposure_rows = sticker_quality(fresh_frames(camera_stream), points)
        log("  auto exposure      : " + "  ".join(
            f"{sticker_name} S{saturation:3.0f} V{brightness:3.0f}" for sticker_name, (saturation, brightness) in zip(NAMES, auto_exposure_rows)))
        for exposure in exposures:
            for gain in gains:
                set_manual(exposure, gain)
                settled_frames = fresh_frames(camera_stream)
                if not settled_frames:
                    log(f"  exposure {exposure} gain {gain}: no frames")
                    continue
                sticker_rows = sticker_quality(settled_frames, points)
                setting_score = score(sticker_rows, gain)
                sweep_results.append(dict(exposure=exposure, gain=gain, score=round(setting_score, 1),
                                    stickers={sticker_name: dict(S=round(saturation), V=round(brightness))
                                              for sticker_name, (saturation, brightness) in zip(NAMES, sticker_rows)}))
                log(f"  exposure {exposure:3d} gain {gain:3d}: " + "  ".join(
                    f"{sticker_name} S{saturation:3.0f} V{brightness:3.0f}" for sticker_name, (saturation, brightness) in zip(NAMES, sticker_rows))
                    + f"   score {setting_score:6.1f}")
    finally:
        camera_stream.release()
    if not sweep_results:
        raise RuntimeError("no exposure setting produced frames")
    best_setting = max(sweep_results, key=lambda r: r["score"])
    set_manual(best_setting["exposure"], best_setting["gain"])
    settings = dict(mode="manual", exposure=best_setting["exposure"], gain=best_setting["gain"],
             saved=time.strftime("%Y-%m-%d %H:%M:%S"), points_px=[list(point) for point in points],
             best=best_setting, auto_reference={sticker_name: dict(S=round(saturation), V=round(brightness))
                                        for sticker_name, (saturation, brightness) in zip(NAMES, auto_exposure_rows)},
             note="tuned by calib/camera_exposure.py --tune for the lighting at that time")
    SETTINGS.write_text(json.dumps(settings, indent=2) + "\n")
    log(f"best: exposure {best_setting['exposure']} gain {best_setting['gain']} (score {best_setting['score']}) "
        f"-> applied, saved {SETTINGS.name}")
    weakest_sticker = min(best_setting["stickers"].values(), key=lambda d: d["S"])
    if weakest_sticker["S"] < 60:
        log(f"WARN: weakest sticker saturation only {weakest_sticker['S']}: the light is too harsh or "
            "too dim for reliable colour; even it out if you can")
    return settings


# Command line: exactly one of --show / --tune / --apply / --set N / --auto.
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    action_group = parser.add_mutually_exclusive_group(required=True)
    action_group.add_argument("--show", action="store_true")
    action_group.add_argument("--tune", action="store_true")
    action_group.add_argument("--apply", action="store_true")
    action_group.add_argument("--set", type=int, metavar="EXPOSURE")
    action_group.add_argument("--auto", action="store_true")
    parser.add_argument("--gain", type=int, default=100)
    parser.add_argument("--points-px", help="'green_x,y orange_x,y pink_x,y' instead of clicking")
    args = parser.parse_args()
    try:
        if args.show:
            print(show())
            settings = load_settings()
            print("saved:", describe(settings) if settings else "none")
        elif args.tune:
            click_points_px = ([tuple(float(coordinate) for coordinate in pair_text.split(",")) for pair_text in args.points_px.split()]
                   if args.points_px else None)
            tune(click_points_px)
        elif args.apply:
            apply_saved()
        elif args.set is not None:
            set_manual(args.set, args.gain)
            SETTINGS.write_text(json.dumps(dict(
                mode="manual", exposure=args.set, gain=args.gain,
                saved=time.strftime("%Y-%m-%d %H:%M:%S"), note="set by hand"), indent=2) + "\n")
            print(f"applied + saved manual exposure {args.set} gain {args.gain}")
        elif args.auto:
            set_auto()
            SETTINGS.write_text(json.dumps(dict(
                mode="auto", saved=time.strftime("%Y-%m-%d %H:%M:%S")), indent=2) + "\n")
            print("applied + saved auto exposure")
    except RuntimeError as exc:
        print("ERROR:", exc, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
