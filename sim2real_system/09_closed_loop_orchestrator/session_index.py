"""Make every demo / optimiser session self-describing and findable.

Each session folder gets:
  manifest.json   machine-readable: kind, date, command, git revision, drive mode, tests,
                  per-test metrics and every produced file (relative paths)
  index.md        the same for people: videos first, then per-test graphics (embedded) +
                  metrics + data files
  config/         copies of the calibration / model / config files the run actually used

and sim2real_calib/RESULTS.md is regenerated as one catalog of every session of both kinds
(newest first), so "where is the run I showed on Tuesday" is one click.

    from session_index import Session
    s = Session(dir, kind="demo", title="...", meta={...})
    s.snapshot_config({"grid_calib.json": path, ...})
    s.add_video("session window", "session_video.mp4")
    s.add_test("T2_spin", metrics={...}, files={"comparison": "T2_spin/compare.png", ...})
    s.write()            # rewrites manifest.json + index.md and refreshes RESULTS.md
"""
from __future__ import annotations

import datetime
import json
import shutil
import subprocess
import sys
from pathlib import Path

SC = Path(__file__).resolve().parents[1]
CATALOG = SC / "RESULTS.md"
ROOTS = (("demo", SC / "demo" / "runs"), ("optimizer", SC / "loop" / "runs"))
IMAGE_EXT = (".png", ".jpg")
VIDEO_EXT = (".mp4",)


# Short git commit of the code that produced a session (+ note if uncommitted).
def git_revision():
    try:
        commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=SC, text=True,
                             capture_output=True, timeout=5).stdout.strip()
        uncommitted = subprocess.run(["git", "status", "--porcelain", "--", "."], cwd=SC, text=True,
                               capture_output=True, timeout=10).stdout.strip()
        return commit + (" (+uncommitted changes)" if uncommitted else "") if commit else "unknown"
    except Exception:
        return "unknown"


# Bookkeeping for one demo / loop session: collects settings, videos, per-test
# metrics and files in self.data and writes them as manifest.json + index.md.
class Session:
    # Start a session record (kind, title, start time, command line, git revision).
    def __init__(self, path, kind, title, meta=None):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=True)
        self.data = {"kind": kind, "title": title,
                     "started": datetime.datetime.now().isoformat(timespec="seconds"),
                     "command": " ".join(sys.argv), "git": git_revision(),
                     "meta": dict(meta or {}), "videos": [], "tests": {}, "summary": [],
                     "config": {}}

    # Path relative to the session folder (absolute if outside it).
    def rel(self, p):
        p = Path(p)
        try:
            return str(p.resolve().relative_to(self.path.resolve()))
        except ValueError:
            return str(p)

    # Copy the config files actually used into <session>/config/.
    def snapshot_config(self, files):
        """{label: path} -> copies under config/ (skips missing files)."""
        config_dir = self.path / "config"
        config_dir.mkdir(exist_ok=True)
        for label, source_path in files.items():
            if source_path and Path(source_path).exists():
                copy_path = config_dir / Path(source_path).name
                if Path(source_path).resolve() != copy_path.resolve():
                    shutil.copy(source_path, copy_path)
                self.data["config"][label] = self.rel(copy_path)

    # List a video/image once under 'Videos'.
    def add_video(self, label, path):
        rel_path = self.rel(path)
        if all(video["file"] != rel_path for video in self.data["videos"]):
            self.data["videos"].append({"label": label, "file": rel_path})

    # Add metrics / existing files / a note to one test's section.
    def add_test(self, name, metrics=None, files=None, note=None):
        """files: {label: path}; only files that exist are listed."""
        entry = self.data["tests"].setdefault(name, {"metrics": {}, "files": {}})
        if metrics:
            entry["metrics"].update({metric: value for metric, value in metrics.items()
                                     if isinstance(value, (int, float, str)) or value is None})
        for label, file_path in (files or {}).items():
            if file_path and Path(file_path).exists():
                entry["files"][label] = self.rel(file_path)
        if note:
            entry["note"] = note

    # Summary lines shown at the top of index.md.
    def set_summary(self, lines):
        self.data["summary"] = list(lines)

    # Write manifest.json and index.md, then regenerate RESULTS.md.
    def write(self, finished=False):
        if finished:
            self.data["finished"] = datetime.datetime.now().isoformat(timespec="seconds")
        (self.path / "manifest.json").write_text(json.dumps(self.data, indent=2, default=str))
        (self.path / "index.md").write_text(render_index(self.data))
        try:
            write_catalog()
        except Exception as exc:                      # never let bookkeeping kill a run
            print(f"[index] RESULTS.md not updated: {exc}", file=sys.stderr)


# Table cell text: floats to 4 significant digits, None as empty.
def _fmt(v):
    if isinstance(v, float):
        return f"{v:.4g}"
    return "" if v is None else str(v)


# index.md for a session: header, settings table, summary, videos, then per test
# its metrics table, embedded images, video and data links, then the configs used.
def render_index(d):
    lines = [f"# {d['title']}", "",
         f"**{d['kind']}** · started {d['started']}"
         + (f" · finished {d['finished']}" if d.get("finished") else " · (in progress / interrupted)"),
         "", f"`{d['command']}`", "", f"git: {d['git']}", ""]
    if d["meta"]:
        lines += ["| setting | value |", "|---|---|"]
        lines += [f"| {key} | {_fmt(value)} |" for key, value in d["meta"].items()]
        lines.append("")
    if d["summary"]:
        lines += ["## Summary", ""] + [f"- {summary_line}" for summary_line in d["summary"]] + [""]
    if d["videos"]:
        lines += ["## Videos", ""] + [f"- [{value['label']}]({value['file']})" for value in d["videos"]] + [""]
    for name, test in d["tests"].items():
        lines += [f"## {name}", ""]
        if test.get("note"):
            lines += [test["note"], ""]
        if test["metrics"]:
            lines += ["| metric | value |", "|---|---|"]
            lines += [f"| {key} | {_fmt(value)} |" for key, value in test["metrics"].items()]
            lines.append("")
        images = {key: value for key, value in test["files"].items() if value.lower().endswith(IMAGE_EXT)}
        videos = {key: value for key, value in test["files"].items() if value.lower().endswith(VIDEO_EXT)}
        data_files = {key: value for key, value in test["files"].items() if key not in images and key not in videos}
        for key, value in images.items():
            lines += [f"**{key}**", "", f"![{key}]({value})", ""]
        if videos:
            lines += ["Videos: " + " · ".join(f"[{key}]({value})" for key, value in videos.items()), ""]
        if data_files:
            lines += ["Data: " + " · ".join(f"[{key}]({value})" for key, value in data_files.items()), ""]
    if d["config"]:
        lines += ["## Configuration used", ""] + [f"- {key}: [{value}]({value})" for key, value in d["config"].items()]
    return "\n".join(lines) + "\n"


# Catalog row for an old session folder without manifest.json.
def _legacy(kind, folder):
    """Sessions from before manifests: minimal row from what exists."""
    headline = ""
    if (folder / "summary.json").exists():
        try:
            summary_rows = json.loads((folder / "summary.json").read_text())
            headline = ", ".join(f"{record['test']} {record['endpoint_err_m'] * 100:.1f} cm" for record in summary_rows[:4])
        except Exception:
            pass
    tests = sorted(path.name for path in folder.iterdir() if path.is_dir() and not path.name.startswith(("iter_", "config")))
    if kind == "optimizer":
        tests = sorted({path.name for iter_dir in folder.glob("iter_*") for path in iter_dir.iterdir()
                        if path.is_dir() and path.name not in ("real", "opt")})
        iteration_lines = []
        for iteration_file in sorted(folder.glob("iter_*/iteration.json")):
            try:
                record = json.loads(iteration_file.read_text())
                iteration_lines.append(f"iter {record['iteration']}: rms z {record['rms_before']:.2f} -> {record['rms_after']:.2f}")
            except Exception:
                pass
        headline = "; ".join(iteration_lines)
    stamp = datetime.datetime.fromtimestamp(folder.stat().st_mtime).isoformat(timespec="minutes")
    return dict(kind=kind, started=stamp, title=folder.name, tests=tests, headline=headline,
                drive="", link=None)


# Regenerate RESULTS.md: one row per demo / optimizer session, newest first
# (fake-rover test sessions are left out).
def write_catalog():
    catalog_rows = []
    for kind, root in ROOTS:
        if not root.exists():
            continue
        for folder in sorted(path for path in root.iterdir() if path.is_dir() and not path.is_symlink()):
            manifest_path = folder / "manifest.json"
            if manifest_path.exists():
                manifest = json.loads(manifest_path.read_text())
                if manifest["kind"] == "test":            # fake-rover smoke tests: not results
                    continue
                test_list = manifest["meta"].get("tests")
                catalog_rows.append(dict(kind=manifest["kind"], started=manifest["started"], title=manifest["title"],
                                 tests=[test.strip() for test in test_list.split(",")] if test_list else list(manifest["tests"]),
                                 headline="; ".join(manifest.get("summary", [])[:1]),
                                 drive=manifest["meta"].get("drive", ""),
                                 link=str((folder / "index.md").relative_to(SC))))
            else:
                r = _legacy(kind, folder)
                r["link"] = str(folder.relative_to(SC))
                catalog_rows.append(r)
    catalog_rows.sort(key=lambda r: r["started"], reverse=True)
    lines = ["# sim2real results catalog", "",
         "Every demo (real + sim driven together, no optimiser) and optimiser (closed loop) "
         "session, newest first. Regenerated automatically at the end of each test / iteration "
         "(`viz/session_index.py`). Open a session's `index.md` for its videos, graphics and data.",
         "", "| started | kind | session | tests | drive | headline |", "|---|---|---|---|---|---|"]
    for r in catalog_rows:
        link_text = f"[{r['title']}]({r['link']})" if r["link"] else r["title"]
        test_list = ", ".join(r["tests"][:6]) + (" …" if len(r["tests"]) > 6 else "")
        lines.append(f"| {r['started'][:16].replace('T', ' ')} | {r['kind']} | {link_text} | {test_list} | "
                 f"{r['drive']} | {r['headline'][:90]} |")
    CATALOG.write_text("\n".join(lines) + "\n")
    return CATALOG


if __name__ == "__main__":
    print(f"wrote {write_catalog()}")
