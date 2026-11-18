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


def git_revision():
    try:
        rev = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=SC, text=True,
                             capture_output=True, timeout=5).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain", "--", "."], cwd=SC, text=True,
                               capture_output=True, timeout=10).stdout.strip()
        return rev + (" (+uncommitted changes)" if dirty else "") if rev else "unknown"
    except Exception:
        return "unknown"


class Session:
    def __init__(self, path, kind, title, meta=None):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=True)
        self.data = {"kind": kind, "title": title,
                     "started": datetime.datetime.now().isoformat(timespec="seconds"),
                     "command": " ".join(sys.argv), "git": git_revision(),
                     "meta": dict(meta or {}), "videos": [], "tests": {}, "summary": [],
                     "config": {}}

    def rel(self, p):
        p = Path(p)
        try:
            return str(p.resolve().relative_to(self.path.resolve()))
        except ValueError:
            return str(p)

    def snapshot_config(self, files):
        """{label: path} -> copies under config/ (skips missing files)."""
        cdir = self.path / "config"
        cdir.mkdir(exist_ok=True)
        for label, src in files.items():
            if src and Path(src).exists():
                dst = cdir / Path(src).name
                if Path(src).resolve() != dst.resolve():
                    shutil.copy(src, dst)
                self.data["config"][label] = self.rel(dst)

    def add_video(self, label, path):
        rel = self.rel(path)
        if all(v["file"] != rel for v in self.data["videos"]):
            self.data["videos"].append({"label": label, "file": rel})

    def add_test(self, name, metrics=None, files=None, note=None):
        """files: {label: path}; only files that exist are listed."""
        entry = self.data["tests"].setdefault(name, {"metrics": {}, "files": {}})
        if metrics:
            entry["metrics"].update({k: v for k, v in metrics.items()
                                     if isinstance(v, (int, float, str)) or v is None})
        for label, p in (files or {}).items():
            if p and Path(p).exists():
                entry["files"][label] = self.rel(p)
        if note:
            entry["note"] = note

    def set_summary(self, lines):
        self.data["summary"] = list(lines)

    def write(self, finished=False):
        if finished:
            self.data["finished"] = datetime.datetime.now().isoformat(timespec="seconds")
        (self.path / "manifest.json").write_text(json.dumps(self.data, indent=2, default=str))
        (self.path / "index.md").write_text(render_index(self.data))
        try:
            write_catalog()
        except Exception as exc:                      # never let bookkeeping kill a run
            print(f"[index] RESULTS.md not updated: {exc}", file=sys.stderr)


def _fmt(v):
    if isinstance(v, float):
        return f"{v:.4g}"
    return "" if v is None else str(v)


def render_index(d):
    L = [f"# {d['title']}", "",
         f"**{d['kind']}** · started {d['started']}"
         + (f" · finished {d['finished']}" if d.get("finished") else " · (in progress / interrupted)"),
         "", f"`{d['command']}`", "", f"git: {d['git']}", ""]
    if d["meta"]:
        L += ["| setting | value |", "|---|---|"]
        L += [f"| {k} | {_fmt(v)} |" for k, v in d["meta"].items()]
        L.append("")
    if d["summary"]:
        L += ["## Summary", ""] + [f"- {x}" for x in d["summary"]] + [""]
    if d["videos"]:
        L += ["## Videos", ""] + [f"- [{v['label']}]({v['file']})" for v in d["videos"]] + [""]
    for name, t in d["tests"].items():
        L += [f"## {name}", ""]
        if t.get("note"):
            L += [t["note"], ""]
        if t["metrics"]:
            L += ["| metric | value |", "|---|---|"]
            L += [f"| {k} | {_fmt(v)} |" for k, v in t["metrics"].items()]
            L.append("")
        imgs = {k: v for k, v in t["files"].items() if v.lower().endswith(IMAGE_EXT)}
        vids = {k: v for k, v in t["files"].items() if v.lower().endswith(VIDEO_EXT)}
        data = {k: v for k, v in t["files"].items() if k not in imgs and k not in vids}
        for k, v in imgs.items():
            L += [f"**{k}**", "", f"![{k}]({v})", ""]
        if vids:
            L += ["Videos: " + " · ".join(f"[{k}]({v})" for k, v in vids.items()), ""]
        if data:
            L += ["Data: " + " · ".join(f"[{k}]({v})" for k, v in data.items()), ""]
    if d["config"]:
        L += ["## Configuration used", ""] + [f"- {k}: [{v}]({v})" for k, v in d["config"].items()]
    return "\n".join(L) + "\n"


def _legacy(kind, folder):
    """Sessions from before manifests: minimal row from what exists."""
    head = ""
    if (folder / "summary.json").exists():
        try:
            rows = json.loads((folder / "summary.json").read_text())
            head = ", ".join(f"{r['test']} {r['endpoint_err_m'] * 100:.1f} cm" for r in rows[:4])
        except Exception:
            pass
    tests = sorted(p.name for p in folder.iterdir() if p.is_dir() and not p.name.startswith(("iter_", "config")))
    if kind == "optimizer":
        tests = sorted({p.name for it in folder.glob("iter_*") for p in it.iterdir()
                        if p.is_dir() and p.name not in ("real", "opt")})
        its = []
        for f in sorted(folder.glob("iter_*/iteration.json")):
            try:
                r = json.loads(f.read_text())
                its.append(f"iter {r['iteration']}: rms z {r['rms_before']:.2f} -> {r['rms_after']:.2f}")
            except Exception:
                pass
        head = "; ".join(its)
    stamp = datetime.datetime.fromtimestamp(folder.stat().st_mtime).isoformat(timespec="minutes")
    return dict(kind=kind, started=stamp, title=folder.name, tests=tests, headline=head,
                drive="", link=None)


def write_catalog():
    rows = []
    for kind, root in ROOTS:
        if not root.exists():
            continue
        for folder in sorted(p for p in root.iterdir() if p.is_dir() and not p.is_symlink()):
            m = folder / "manifest.json"
            if m.exists():
                d = json.loads(m.read_text())
                if d["kind"] == "test":            # fake-rover smoke tests: not results
                    continue
                tests = d["meta"].get("tests")
                rows.append(dict(kind=d["kind"], started=d["started"], title=d["title"],
                                 tests=[t.strip() for t in tests.split(",")] if tests else list(d["tests"]),
                                 headline="; ".join(d.get("summary", [])[:1]),
                                 drive=d["meta"].get("drive", ""),
                                 link=str((folder / "index.md").relative_to(SC))))
            else:
                r = _legacy(kind, folder)
                r["link"] = str(folder.relative_to(SC))
                rows.append(r)
    rows.sort(key=lambda r: r["started"], reverse=True)
    L = ["# sim2real results catalog", "",
         "Every demo (real + sim driven together, no optimiser) and optimiser (closed loop) "
         "session, newest first. Regenerated automatically at the end of each test / iteration "
         "(`viz/session_index.py`). Open a session's `index.md` for its videos, graphics and data.",
         "", "| started | kind | session | tests | drive | headline |", "|---|---|---|---|---|---|"]
    for r in rows:
        name = f"[{r['title']}]({r['link']})" if r["link"] else r["title"]
        tests = ", ".join(r["tests"][:6]) + (" …" if len(r["tests"]) > 6 else "")
        L.append(f"| {r['started'][:16].replace('T', ' ')} | {r['kind']} | {name} | {tests} | "
                 f"{r['drive']} | {r['headline'][:90]} |")
    CATALOG.write_text("\n".join(L) + "\n")
    return CATALOG


if __name__ == "__main__":
    print(f"wrote {write_catalog()}")
