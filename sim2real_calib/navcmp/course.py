#!/usr/bin/env python3
"""Navigation course: the one description both rovers drive.

A course YAML fixes everything that must be IDENTICAL between the real rover and the
Isaac rover: the arena (walls + box obstacles, which you build physically for the real
rover and which run_nav_course.py builds in Isaac), the start pose, the goal sequence and
the safety fence. Coordinates are in the COURSE frame: metres, x right / y up, the same as
the overhead grid frame (origin = grid bottom-left corner) so a course laid out on the
taped grid can be read straight off the tape.

Pure Python (yaml + math only): it is imported by the Isaac launcher (py3.12), by
mission_runner.py on the rover (py3.10, no numpy assumed) and by the desktop metrics.

    python3 navcmp/course.py navcmp/courses/grid2x2_box.yaml      # validate + summary
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

ROBOT_RADIUS = 0.15        # ugv_nav teb.yaml footprint radius (local/global costmap)
INFLATION = 0.20           # ugv_nav teb.yaml inflation_radius
LIDAR_PLANE_MIN = 0.25     # walls/boxes must reach above the LD19 scan plane (~0.2 m)


# ── SE(2) helpers ────────────────────────────────────────────────────────────
def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def compose(a, b):
    """a ∘ b for (x, y, yaw) poses: express b (given in a's frame) in a's parent."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by, ay + s * bx + c * by, wrap(ath + bth))


def inverse(a):
    x, y, th = a
    c, s = math.cos(th), math.sin(th)
    return (-c * x - s * y, s * x - c * y, wrap(-th))


def apply(T, x, y):
    """Transform a point by pose T."""
    c, s = math.cos(T[2]), math.sin(T[2])
    return (T[0] + c * x - s * y, T[1] + s * x + c * y)


# ── geometry ─────────────────────────────────────────────────────────────────
@dataclass
class Box:
    """Static obstacle: centre (cx, cy), footprint sx × sy, height sz, yaw (rad)."""
    name: str
    cx: float
    cy: float
    sx: float
    sy: float
    sz: float
    yaw: float = 0.0
    kind: str = "box"          # "box" | "wall" (only changes colour / docs)

    def corners(self):
        hx, hy = self.sx / 2.0, self.sy / 2.0
        T = (self.cx, self.cy, self.yaw)
        return [apply(T, px, py) for px, py in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]

    def distance(self, x: float, y: float) -> float:
        """Signed distance from a point to the box footprint (negative inside)."""
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        dx, dy = x - self.cx, y - self.cy
        lx, ly = c * dx - s * dy, s * dx + c * dy
        qx, qy = abs(lx) - self.sx / 2.0, abs(ly) - self.sy / 2.0
        outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
        inside = min(max(qx, qy), 0.0)
        return outside + inside


@dataclass
class Goal:
    name: str
    x: float
    y: float
    yaw: float                  # rad
    timeout_s: float


@dataclass
class Course:
    name: str
    description: str
    start: tuple                # (x, y, yaw) of base_footprint, course frame
    goals: list
    boxes: list
    fence: tuple                # (xmin, xmax, ymin, ymax)
    return_to_start: bool = True
    path: str = ""
    raw: dict = field(default_factory=dict)

    # legs one trial drives, in order (the closing return leg is scored separately)
    def legs(self):
        legs = list(self.goals)
        if self.return_to_start and not self._ends_at_start():
            sx, sy, sth = self.start
            legs.append(Goal("return", sx, sy, sth, self.goals[-1].timeout_s if self.goals else 120.0))
        return legs

    def _ends_at_start(self):
        if not self.goals:
            return True
        g = self.goals[-1]
        return math.hypot(g.x - self.start[0], g.y - self.start[1]) < 0.05

    def clearance(self, x: float, y: float) -> float:
        """Distance from (x, y) to the nearest obstacle footprint (inf if none)."""
        return min((b.distance(x, y) for b in self.boxes), default=math.inf)

    def inside_fence(self, x: float, y: float, margin: float = 0.0) -> bool:
        xmin, xmax, ymin, ymax = self.fence
        return (xmin - margin) <= x <= (xmax + margin) and (ymin - margin) <= y <= (ymax + margin)

    def bounds(self, pad: float = 0.1):
        xs = [self.fence[0], self.fence[1]]
        ys = [self.fence[2], self.fence[3]]
        for b in self.boxes:
            for px, py in b.corners():
                xs.append(px)
                ys.append(py)
        return min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad

    def validate(self) -> list:
        """Return a list of problems (empty = OK). Errors a course must not have."""
        problems = []
        if not self.goals:
            problems.append("course has no goals")
        need = ROBOT_RADIUS + 0.02
        pts = [("start", self.start[0], self.start[1])] + [(g.name, g.x, g.y) for g in self.goals]
        for name, x, y in pts:
            if not self.inside_fence(x, y):
                problems.append(f"{name} ({x:.2f},{y:.2f}) is outside the fence {self.fence}")
            c = self.clearance(x, y)
            if c < need:
                problems.append(f"{name} ({x:.2f},{y:.2f}) is {c:.2f} m from an obstacle "
                                f"(< robot radius {ROBOT_RADIUS} + 2 cm)")
        for b in self.boxes:
            if b.sz < LIDAR_PLANE_MIN:
                problems.append(f"{b.name} is {b.sz:.2f} m tall: the LD19 scan plane may pass "
                                f"over it (need >= {LIDAR_PLANE_MIN} m)")
            if min(b.sx, b.sy) <= 0 or b.sz <= 0:
                problems.append(f"{b.name} has a non-positive size")
        names = [g.name for g in self.goals]
        if len(set(names)) != len(names):
            problems.append("goal names must be unique")
        if "return" in names:
            problems.append("'return' is reserved for the automatic return leg")
        return problems

    def warnings(self) -> list:
        """Things that are legal but make the comparison weaker."""
        out = []
        comfy = ROBOT_RADIUS + INFLATION
        for g in self.goals:
            c = self.clearance(g.x, g.y)
            if c < comfy:
                out.append(f"{g.name} is inside the inflation zone ({c:.2f} m < {comfy:.2f} m): "
                           "Nav2 may refuse or creep onto it")
        return out

    def summary(self) -> str:
        lines = [f"course {self.name}: {len(self.goals)} goals, {len(self.boxes)} obstacles, "
                 f"start ({self.start[0]:.2f}, {self.start[1]:.2f}, {math.degrees(self.start[2]):.0f} deg)"]
        prev = self.start
        total = 0.0
        for leg in self.legs():
            d = math.hypot(leg.x - prev[0], leg.y - prev[1])
            total += d
            lines.append(f"  {leg.name:>8}: ({leg.x:.2f}, {leg.y:.2f}, {math.degrees(leg.yaw):.0f} deg)  "
                         f"straight {d:.2f} m  clearance {self.clearance(leg.x, leg.y):.2f} m  "
                         f"timeout {leg.timeout_s:.0f} s")
            prev = (leg.x, leg.y, leg.yaw)
        lines.append(f"  straight-line total {total:.2f} m per trial")
        return "\n".join(lines)


# ── loading ──────────────────────────────────────────────────────────────────
def _pose(d, what):
    try:
        return float(d["x"]), float(d["y"]), math.radians(float(d.get("yaw_deg", 0.0)))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{what}: need x, y (and optional yaw_deg): {d!r}") from exc


def _wall(i, w):
    (x0, y0), (x1, y1) = w["from"], w["to"]
    length = math.hypot(x1 - x0, y1 - y0)
    if length <= 0:
        raise ValueError(f"wall {i} has zero length")
    t = float(w.get("thickness", 0.02))
    return Box(name=w.get("name", f"wall{i}"), cx=(x0 + x1) / 2.0, cy=(y0 + y1) / 2.0,
               sx=length + t, sy=t, sz=float(w.get("height", 0.40)),
               yaw=math.atan2(y1 - y0, x1 - x0), kind="wall")


def _box(i, b):
    cx, cy = b["center"]
    sx, sy, sz = b["size"]
    return Box(name=b.get("name", f"box{i}"), cx=float(cx), cy=float(cy), sx=float(sx),
               sy=float(sy), sz=float(sz), yaw=math.radians(float(b.get("yaw_deg", 0.0))))


def from_dict(d: dict, path: str = "") -> Course:
    default_timeout = float(d.get("goal_timeout_s", 120.0))
    goals = []
    for i, g in enumerate(d.get("goals") or []):
        x, y, yaw = _pose(g, f"goal {i}")
        goals.append(Goal(name=str(g.get("name", f"G{i + 1}")), x=x, y=y, yaw=yaw,
                          timeout_s=float(g.get("timeout_s", default_timeout))))
    boxes = [_wall(i, w) for i, w in enumerate(d.get("walls") or [])]
    boxes += [_box(i, b) for i, b in enumerate(d.get("boxes") or [])]
    f = d.get("fence") or {}
    try:
        fence = (float(f["xmin"]), float(f["xmax"]), float(f["ymin"]), float(f["ymax"]))
    except KeyError as exc:
        raise ValueError("fence needs xmin, xmax, ymin, ymax") from exc
    return Course(name=str(d.get("name") or Path(path).stem or "course"),
                  description=str(d.get("description", "")).strip(),
                  start=_pose(d.get("start") or {}, "start"), goals=goals, boxes=boxes,
                  fence=fence, return_to_start=bool(d.get("return_to_start", True)),
                  path=str(path), raw=d)


def load(path) -> Course:
    import yaml
    d = yaml.safe_load(Path(path).read_text()) or {}
    course = from_dict(d, str(path))
    problems = course.validate()
    if problems:
        raise ValueError(f"{path}: " + "; ".join(problems))
    return course


# ── occupancy map -> obstacle boxes (sim twin of a real lidar map) ───────────
def boxes_from_occupancy(grid, width, height, resolution, origin, occupied=65,
                         height_m=0.40, min_cells=1, to_course=None):
    """Turn an occupancy grid into a short list of boxes (row runs merged vertically).

    grid is row-major int data (ROS OccupancyGrid order: row 0 = origin y). origin is the
    map pose (x, y, yaw) of cell (0, 0). to_course is an optional pose mapping map frame
    -> course frame. Used to give Isaac the SAME walls the real rover's lidar mapped.
    """
    runs = {}                                   # (x0, x1) -> list of rows
    for r in range(height):
        row = grid[r * width:(r + 1) * width]
        c = 0
        while c < width:
            if row[c] >= occupied:
                c0 = c
                while c < width and row[c] >= occupied:
                    c += 1
                runs.setdefault((c0, c), []).append(r)
            else:
                c += 1
    rects = []
    for (c0, c1), rows in runs.items():
        rows.sort()
        r0 = prev = rows[0]
        for r in rows[1:] + [None]:
            if r is not None and r == prev + 1:
                prev = r
                continue
            if (c1 - c0) * (prev - r0 + 1) >= min_cells:
                rects.append((c0, c1, r0, prev + 1))
            if r is not None:
                r0 = prev = r
    boxes = []
    ox, oy, oth = origin
    for i, (c0, c1, r0, r1) in enumerate(rects):
        lx, ly = (c0 + c1) / 2.0 * resolution, (r0 + r1) / 2.0 * resolution
        x, y = apply((ox, oy, oth), lx, ly)
        yaw = oth
        if to_course is not None:
            x, y = apply(to_course, x, y)
            yaw = wrap(yaw + to_course[2])
        boxes.append(Box(name=f"map{i}", cx=x, cy=y, sx=(c1 - c0) * resolution,
                         sy=(r1 - r0) * resolution, sz=height_m, yaw=yaw, kind="wall"))
    return boxes


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("course")
    a = ap.parse_args()
    c = load(a.course)
    print(c.summary())
    for w in c.warnings():
        print("  warning:", w)
    print("OK")


if __name__ == "__main__":
    main()
