#!/usr/bin/env python3
"""Save / restore / create the whole rig setup for one floor surface.

A surface's rig state is spread over several active files; a profile snapshots them all
under rig_profiles/<surface>/ so a surface can be put away and brought back later:

  loop/loop_config.yaml          workspace fence, start poses, homing gains, odom_scale,
                                 camera latency
  calib/grid_calib.json          pixel <-> floor metres
  calib/sticker_config.json      sticker colours (tied to the lighting + exposure)
  calib/camera_settings.json     overhead camera exposure/gain
  loop/stations/                 taught start stations + camera-guard reference image

    python3 rig_profile.py list
    python3 rig_profile.py save padded --note "19/72 trials, battery stop"
    python3 rig_profile.py new light_carpet --preset carpet   # saves the current rig first
    python3 rig_profile.py activate padded                    # bring a saved surface back
    python3 rig_profile.py show light_carpet

`new` starts from the CURRENT config with the preset's surface-dependent values (homing
turn gains, odometry scale, fence) and clears the taught stations; the grid, camera and
stickers must then be calibrated on the new floor (surface_demo.py does all of that).
Recorded sessions are never touched: they live in loop/runs/ with their own config copy.
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import sys
import time
from pathlib import Path

SC = Path(__file__).resolve().parent
PROFILES = SC / "rig_profiles"
STATIONS = SC / "loop" / "stations"
ACTIVE = {                                   # profile file name -> active location
    "loop_config.yaml": SC / "loop" / "loop_config.yaml",
    "grid_calib.json": SC / "calib" / "grid_calib.json",
    "sticker_config.json": SC / "calib" / "sticker_config.json",
    "camera_settings.json": SC / "calib" / "camera_settings.json",
}
# Surface-dependent loop_config values (measured: carpet 2026-10-20, concrete 2026-11-11,
# EVA foam 2026-11-18). odom_scale and cam latency are re-measured on the day anyway.
PRESETS = {
    "carpet":   dict(max_angular=0.9, min_angular=0.45, odom_scale=0.89),
    "concrete": dict(max_angular=1.6, min_angular=0.9, odom_scale=0.96),
    "foam":     dict(max_angular=1.6, min_angular=0.9, odom_scale=0.93),
}
FENCE = dict(x=[-0.15, 1.15], y=[0.00, 1.30])   # default fence for a fresh 1 m grid


# Timestamp used in backup / archive folder names, e.g. 20261118_202540.
def stamp():
    return time.strftime("%Y%m%d_%H%M%S")


# Plain files in loop/stations/ (taught stations + camera-guard reference); the
# archive_* sub-folders are directories and are left alone.
def station_files():
    return [path for path in STATIONS.iterdir()
            if path.is_file() and not path.name.startswith(".")] if STATIONS.exists() else []


# Copy every active rig file (ACTIVE) plus loop/stations/ files into `dest`.
def snapshot(dest: Path):
    dest.mkdir(parents=True, exist_ok=True)
    for name, active_path in ACTIVE.items():
        if active_path.exists():
            shutil.copy2(active_path, dest / name)
    stations_copy_dir = dest / "stations"
    stations_copy_dir.mkdir(exist_ok=True)
    for station_file in station_files():
        shutil.copy2(station_file, stations_copy_dir / station_file.name)


# rig_profiles/<surface>/profile.json as a dict (empty if the profile is new).
def read_meta(surface):
    meta_path = PROFILES / surface / "profile.json"
    return json.loads(meta_path.read_text()) if meta_path.exists() else {}


# Overwrite rig_profiles/<surface>/profile.json.
def write_meta(surface, meta):
    (PROFILES / surface / "profile.json").write_text(json.dumps(meta, indent=2) + "\n")


# Name stored in rig_profiles/ACTIVE (the surface whose files are live), or None.
def current_surface():
    active_marker = PROFILES / "ACTIVE"
    return active_marker.read_text().strip() if active_marker.exists() else None


# Record `surface` as the active one in rig_profiles/ACTIVE.
def set_current(surface):
    PROFILES.mkdir(exist_ok=True)
    (PROFILES / "ACTIVE").write_text(surface + "\n")


# Create and return `base`, or base_2, base_3 ... if that name is taken
# (two saves in the same second must not collide).
def unique_dir(base: Path) -> Path:
    candidate_dir, suffix_number = base, 2
    while candidate_dir.exists():
        candidate_dir, suffix_number = base.with_name(f"{base.name}_{suffix_number}"), suffix_number + 1
    candidate_dir.mkdir(parents=True)
    return candidate_dir


# Snapshot the active rig into rig_profiles/<surface>/. The previous snapshot's files
# move to history/<stamp>/ first; profile.json and demo_state.json stay in place.
def save(surface, note=None, log=print):
    profile_dir = PROFILES / surface
    previous_files = [profile_dir / file_name for file_name in (*ACTIVE, "stations") if (profile_dir / file_name).exists()]
    if previous_files:                                   # keep the previous snapshot, not demo state
        history_dir = unique_dir(profile_dir / "history" / stamp())
        for old_file in previous_files:
            shutil.move(str(old_file), history_dir / old_file.name)
    snapshot(profile_dir)
    meta = read_meta(surface)
    meta.update(surface=surface, saved=time.strftime("%Y-%m-%d %H:%M:%S"))
    if note:
        meta.setdefault("notes", []).append(f"{time.strftime('%Y-%m-%d %H:%M')} {note}")
    write_meta(surface, meta)
    set_current(surface)
    log(f"saved the active rig as profile '{surface}' -> {profile_dir.relative_to(SC)}")
    return profile_dir


# Make a saved profile live again: auto-save the current rig, copy the profile's files
# over the active ones, and swap loop/stations/ (old station files go to an archive folder).
def activate(surface, log=print):
    profile_dir = PROFILES / surface
    if not (profile_dir / "loop_config.yaml").exists():
        raise SystemExit(f"no saved profile '{surface}' ({profile_dir})")
    snapshot(unique_dir(PROFILES / "_autosave" / stamp()))   # never lose the active rig
    for name, active_path in ACTIVE.items():
        if (profile_dir / name).exists():
            shutil.copy2(profile_dir / name, active_path)
    old_station_files = station_files()
    if old_station_files:
        archive_dir = STATIONS / f"archive_{current_surface() or 'unknown'}_{stamp()}"
        archive_dir.mkdir()
        for station_file in old_station_files:
            shutil.move(str(station_file), archive_dir / station_file.name)
    for station_file in (profile_dir / "stations").glob("*"):
        if station_file.is_file():
            shutil.copy2(station_file, STATIONS / station_file.name)
    set_current(surface)
    log(f"activated profile '{surface}' (previous rig auto-saved in rig_profiles/_autosave/)")


# Edit loop_config.yaml TEXT (so its comments survive): set the preset's homing turn
# limits, odom_scale and the default workspace fence, and drop the continuation-comment
# lines that described the previous surface's values.
def apply_preset(text, preset):
    values = PRESETS[preset]
    substitutions = [
        (r"(\n  max_angular:\s*)[0-9.]+[^\n]*", rf"\g<1>{values['max_angular']}         # rad/s ({preset} preset)"),
        (r"(\n  min_angular:\s*)[0-9.]+[^\n]*", rf"\g<1>{values['min_angular']}        # {preset} preset; homing raises it if the rover stalls"),
        (r"(\n  odom_scale:\s*)[0-9.]+[^\n]*", rf"\g<1>{values['odom_scale']}         # {preset} preset - MEASURE: loop/measure_odom_scale.py --write"),
        (r"(\nworkspace:\n  x: )\[[^\]]*\]", rf"\g<1>[{FENCE['x'][0]:.2f}, {FENCE['x'][1]:.2f}]"),
        (r"(\nworkspace:\n  x: \[[^\]]*\]\n  y: )\[[^\]]*\]", rf"\g<1>[{FENCE['y'][0]:.2f}, {FENCE['y'][1]:.2f}]"),
    ]
    for pattern, replacement in substitutions:
        text, replacements = re.subn(pattern, replacement, text, count=1)
        if replacements != 1:
            raise SystemExit(f"loop_config.yaml: could not set {pattern!r}")
    # the following-line comments of the replaced values describe the OLD surface
    text = re.sub(r"(\n  min_angular:[^\n]*)(\n {27}#[^\n]*)+", r"\1", text)
    text = re.sub(r"(\n  odom_scale:[^\n]*)(\n {27}#[^\n]*)+", r"\1", text)
    return text


# Create a profile for a new floor: save the current one, apply the preset to
# loop_config.yaml, archive taught stations (they belong to the old floor), snapshot.
def new(surface, preset, note=None, log=print):
    if (PROFILES / surface / "loop_config.yaml").exists():
        raise SystemExit(f"profile '{surface}' already exists - use `activate {surface}`")
    previous_surface = current_surface()
    if previous_surface:
        save(previous_surface, note=f"auto-saved before creating '{surface}'", log=log)
    else:
        snapshot(unique_dir(PROFILES / "_autosave" / stamp()))
    config_path = ACTIVE["loop_config.yaml"]
    config_path.write_text(apply_preset(config_path.read_text(), preset))
    old_station_files = station_files()
    keep_names = {"camera_reference.json", "camera_reference.png"}   # rewritten by calibrate_grid
    files_to_archive = [station_file for station_file in old_station_files if station_file.name not in keep_names]
    if files_to_archive:
        archive_dir = STATIONS / f"archive_{previous_surface or 'unknown'}_{stamp()}"
        archive_dir.mkdir()
        for station_file in files_to_archive:
            shutil.move(str(station_file), archive_dir / station_file.name)
    profile_dir = PROFILES / surface
    snapshot(profile_dir)
    write_meta(surface, dict(surface=surface, preset=preset,
                             created=time.strftime("%Y-%m-%d %H:%M:%S"),
                             notes=[f"{time.strftime('%Y-%m-%d %H:%M')} created from "
                                    f"'{previous_surface}' with the {preset} preset"] + ([note] if note else []),
                             todo=["grid", "camera", "stickers", "rig", "workspace"]))
    set_current(surface)
    log(f"new profile '{surface}' ({preset} preset) is now active; taught stations cleared.\n"
        f"Next: calibrate the grid, camera exposure + stickers, then the rig "
        f"(surface_demo.py walks through it).")


# Print a profile's profile.json and the files it holds.
def show_profile(surface):
    profile_dir = PROFILES / surface
    if not profile_dir.exists():
        raise SystemExit(f"no profile '{surface}'")
    print(json.dumps(read_meta(surface), indent=2))
    for entry in sorted(profile_dir.iterdir()):
        print("  ", entry.name)


# Command line: list | save | activate | new | show (see the module docstring).
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    subparsers = parser.add_subparsers(dest="cmd", required=True)
    subparsers.add_parser("list")
    save_parser = subparsers.add_parser("save"); save_parser.add_argument("surface"); save_parser.add_argument("--note")
    activate_parser = subparsers.add_parser("activate"); activate_parser.add_argument("surface")
    new_parser = subparsers.add_parser("new"); new_parser.add_argument("surface")
    new_parser.add_argument("--preset", required=True, choices=sorted(PRESETS)); new_parser.add_argument("--note")
    show_parser = subparsers.add_parser("show"); show_parser.add_argument("surface")
    args = parser.parse_args()
    if args.cmd == "list":
        active_surface = current_surface()
        for profile_dir in sorted(PROFILES.glob("*")) if PROFILES.exists() else []:
            if profile_dir.is_dir() and (profile_dir / "loop_config.yaml").exists():
                meta = read_meta(profile_dir.name)
                print(f"{'*' if profile_dir.name == active_surface else ' '} {profile_dir.name:16s} saved {meta.get('saved', meta.get('created', '?'))}"
                      f"  {meta.get('notes', [''])[-1] if meta.get('notes') else ''}")
        print("(* = active)")
    elif args.cmd == "save":
        save(args.surface, args.note)
    elif args.cmd == "activate":
        activate(args.surface)
    elif args.cmd == "new":
        new(args.surface, args.preset, args.note)
    elif args.cmd == "show":
        show_profile(args.surface)
    return 0


if __name__ == "__main__":
    sys.exit(main())
