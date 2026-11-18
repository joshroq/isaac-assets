#!/usr/bin/env python3
"""Generate replayable 3D views of navigation trials (self-contained, offline HTML).

Each trial's recorded trajectory (paths/<trial>.csv: t,x,y,yaw) becomes an interactive 3D
scene (Three.js) you can orbit and play/scrub through time: the actual rover (loaded from
its URDF meshes) drives its recorded path through the warehouse (3D obstacles from the
shared occupancy map), with RViz-style layers you can toggle — occupancy Map overlay, a
reconstructed LaserScan (ray-cast against the map), the path, TF axes, and a forward depth/
RGB-D point-cloud fan. A per-nav preset (lidar / depth / rtab) mirrors the layer mix each
stack's RViz config shows. No ROS or sim needed to view.

Under experiments/replay3d/:
  vendor/                three.min.js, OrbitControls.js, STLLoader.js  (offline)
  robot/meshes/*.stl     rover meshes copied from ugv_description
  robot.js               rover link transforms (from the URDF)
  warehouse.js           shared obstacle geometry (occupancy grid downsampled)
  viewer.js              shared scene + layers + playback
  trials/<cell>__trialNN.html    one replay per trial
  index.html             links to every replay

Usage:
  python3 experiments/replay3d/make_replay3d.py --all
  python3 experiments/replay3d/make_replay3d.py experiments/data/cold_start_lidar_G2_northwest
"""
from __future__ import annotations
import argparse, csv, glob, json, math, os, shutil, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)
REPO = os.path.dirname(EXP)
DATA = os.path.join(EXP, "data")
MAP_NPZ = os.path.join(EXP, "maps", "warehouse", "map.npz")
TRIALS_DIR = os.path.join(HERE, "trials")
ROBOT_DIR = os.path.join(HERE, "robot")
URDF = os.path.join(REPO, "ugv_ws/src/ugv_main/ugv_description/urdf/ugv_rover_isaac_fixed.urdf")
MESH_ROOT = os.path.join(REPO, "ugv_ws/src/ugv_main/ugv_description/meshes/ugv_rover")

DEFAULT_CELLS = [
    "cold_start_lidar_G4_center", "cold_start_depth_G4_center", "cold_start_rtab_G4_center",
    "known_map_lidar_G4_center", "known_map_depth_G4_center", "known_map_rtab_G4_center",
]
OBSTACLE_H = 0.6


# ── URDF -> robot.js (link meshes + transforms relative to base_footprint) ────────
def _rpy_to_quat(r, p, y):
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    return [sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy]  # x,y,z,w


def _mat(xyz, rpy):
    r, p, yw = rpy
    Rx = np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    Ry = np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    Rz = np.array([[math.cos(yw),-math.sin(yw),0],[math.sin(yw),math.cos(yw),0],[0,0,1]])
    M = np.eye(4); M[:3,:3] = Rz @ Ry @ Rx; M[:3,3] = xyz
    return M


def _mat_to_quat(M):
    R = M[:3,:3]; t = np.trace(R)
    if t > 0:
        s = math.sqrt(t+1.0)*2; w=0.25*s; x=(R[2,1]-R[1,2])/s; y=(R[0,2]-R[2,0])/s; z=(R[1,0]-R[0,1])/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = math.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])*2; w=(R[2,1]-R[1,2])/s; x=0.25*s; y=(R[0,1]+R[1,0])/s; z=(R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = math.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])*2; w=(R[0,2]-R[2,0])/s; x=(R[0,1]+R[1,0])/s; y=0.25*s; z=(R[1,2]+R[2,1])/s
    else:
        s = math.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])*2; w=(R[1,0]-R[0,1])/s; x=(R[0,2]+R[2,0])/s; y=(R[1,2]+R[2,1])/s; z=0.25*s
    return [x,y,z,w]


def _stl_positions(path):
    """Return a flat list of triangle vertex positions from a binary STL (metres)."""
    import struct
    b = open(path, "rb").read()
    if len(b) < 84:
        return []
    n = struct.unpack("<I", b[80:84])[0]
    if 84 + n*50 != len(b):     # not a well-formed binary STL
        return []
    pos = []
    for i in range(n):
        base = 84 + i*50 + 12   # skip 12-byte normal
        for k in range(3):
            pos.extend(struct.unpack("<3f", b[base + k*12: base + k*12 + 12]))
    return pos


def build_robot_js() -> None:
    """Embed the URDF link meshes (base64 vertex buffers) so the real robot renders with no
    loader/XHR — works straight from file://. Links without a resolvable mesh are skipped
    (no scattered per-link boxes)."""
    import base64
    import xml.etree.ElementTree as ET
    if not os.path.exists(URDF):
        print("  (no URDF; robot falls back to a single box)")
        open(os.path.join(HERE, "robot.js"), "w").write("window.ROBOT=null;\n"); return
    root = ET.parse(URDF).getroot()
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0").split())] if o is not None else [0, 0, 0]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0").split())] if o is not None else [0, 0, 0]
        joints[j.find("child").get("link")] = (j.find("parent").get("link"), _mat(xyz, rpy))

    def to_base(link):
        M = np.eye(4); chain = []; cur = link
        while cur in joints and cur != "base_footprint":
            parent, jm = joints[cur]; chain.append(jm); cur = parent
            if cur == "base_footprint":
                break
        for jm in reversed(chain):
            M = M @ jm
        return M

    links = []
    for link in root.findall("link"):
        name = link.get("name"); vis = link.find("visual")
        if vis is None:
            continue
        mesh = vis.find(".//mesh")
        if mesh is None:
            continue                                   # skip non-mesh links (no boxes)
        src = os.path.join(MESH_ROOT, os.path.basename(mesh.get("filename")))
        pos = _stl_positions(src) if os.path.isfile(src) else []
        if not pos:
            continue                                   # skip unresolved/unreadable meshes
        M = to_base(name)
        color = 0x2b3440 if "wheel" in name else (0xf7768e if "lidar" in name else
                (0x7dcfff if "camera" in name else 0x9aa5b1))
        buf = base64.b64encode(np.asarray(pos, dtype="<f4").tobytes()).decode()
        links.append(dict(name=name, xyz=M[:3, 3].round(5).tolist(),
                          quat=[round(q, 6) for q in _mat_to_quat(M)], color=color, verts=buf))
    with open(os.path.join(HERE, "robot.js"), "w") as fh:
        fh.write("window.ROBOT = " + json.dumps(dict(links=links)) + ";\n")
    tris = sum(len(base64.b64decode(l["verts"]))//36 for l in links)
    print(f"robot.js: {len(links)} URDF links embedded ({tris} triangles, no external meshes)")


# ── occupancy grid -> warehouse.js ───────────────────────────────────────────────
def build_warehouse_js(downsample: int = 4) -> None:
    if not os.path.exists(MAP_NPZ):
        sys.exit(f"missing {MAP_NPZ}")
    d = np.load(MAP_NPZ)
    grid, res = d["grid"], float(d["resolution"])
    ox, oy = float(d["origin_x"]), float(d["origin_y"])
    occ = grid == 100
    H, W = occ.shape; f = max(1, downsample); Hc, Wc = H//f, W//f
    occ_c = occ[:Hc*f, :Wc*f].reshape(Hc, f, Wc, f).any(axis=(1, 3))
    cell = res*f
    rs, cs = np.where(occ_c)
    xs = (cs*f + f/2.0)*res + ox; ys = (rs*f + f/2.0)*res + oy
    obstacles = np.stack([xs, ys], axis=1).round(3).tolist()
    extent = dict(xmin=round(ox,3), xmax=round(ox+W*res,3), ymin=round(oy,3), ymax=round(oy+H*res,3))
    payload = dict(res=round(cell,3), obstacle_h=OBSTACLE_H, extent=extent, obstacles=obstacles)
    with open(os.path.join(HERE, "warehouse.js"), "w") as fh:
        fh.write("window.WAREHOUSE = " + json.dumps(payload) + ";\n")
    print(f"warehouse.js: {len(obstacles)} obstacle cells @ {cell:.2f} m")


def _trial_rows(cell_dir):
    rc = os.path.join(cell_dir, "results.csv")
    return list(csv.DictReader(open(rc))) if os.path.exists(rc) else []


def build_trial_html(cell_dir: str) -> list[dict]:
    cell = os.path.basename(cell_dir.rstrip("/"))
    rows = _trial_rows(cell_dir)
    if not rows:
        print(f"  {cell}: no results.csv — skip"); return []
    entries = []
    for row in rows:
        if (row.get("result") or "") == "INIT_FAILED":
            continue
        pf = os.path.join(cell_dir, row.get("path_file", ""))
        if not row.get("path_file") or not os.path.isfile(pf):
            continue
        path = [[float(r["t"]), float(r["x"]), float(r["y"]), float(r["yaw"])]
                for r in csv.DictReader(open(pf))]
        if len(path) < 2:
            continue
        trial = row.get("trial", "?")
        td = dict(cell=cell, nav=row.get("nav_system","?"), cond=row.get("condition","?"),
                  goal_name=row.get("goal","?"), trial=trial, result=row.get("result","?"),
                  goal=[float(row.get("goal_x",0)), float(row.get("goal_y",0))],
                  start=[float(row.get("start_x",path[0][1])), float(row.get("start_y",path[0][2]))],
                  err=row.get("final_pos_error_m",""), dur=row.get("duration_s",""), path=path)
        name = f"{cell}__trial{int(trial):02d}.html" if str(trial).isdigit() else f"{cell}__trial{trial}.html"
        with open(os.path.join(TRIALS_DIR, name), "w") as fh:
            fh.write(_HTML.replace("__TRIAL_JSON__", json.dumps(td)))
        entries.append(dict(cell=cell, trial=trial, result=td["result"], err=td["err"],
                            file=os.path.join("trials", name)))
    print(f"  {cell}: wrote {len(entries)} trial replays")
    return entries


def build_index(all_entries):
    by_cell = {}
    for e in all_entries:
        by_cell.setdefault(e["cell"], []).append(e)
    parts = ["<!doctype html><meta charset=utf-8><title>Trial 3D replays</title>",
             "<style>body{font:14px system-ui;margin:24px;background:#0f1115;color:#e6e6e6}"
             "h1{font-size:20px}h2{font-size:15px;margin:18px 0 6px;color:#8ab4f8}"
             "a{color:#9ece6a;text-decoration:none}a:hover{text-decoration:underline}"
             ".r{color:#f7768e}.s{color:#9ece6a}li{margin:2px 0}</style>",
             "<h1>Navigation trials &mdash; replayable 3D views</h1>",
             "<p>Open a trial: orbit the warehouse, play/scrub the rover driving its recorded path, "
             "and toggle RViz-style layers (Map, LaserScan, Path, point-cloud) per nav system.</p>"]
    for cell in sorted(by_cell):
        parts.append(f"<h2>{cell}</h2><ul>")
        for e in sorted(by_cell[cell], key=lambda x: str(x["trial"])):
            cls = "s" if e["result"] == "SUCCEEDED" else "r"
            parts.append(f"<li><a href='{e['file']}'>trial {e['trial']}</a> "
                         f"<span class={cls}>{e['result']}</span> &nbsp; err={e['err']} m</li>")
        parts.append("</ul>")
    open(os.path.join(HERE, "index.html"), "w").write("\n".join(parts))
    print(f"index.html: {len(all_entries)} replays across {len(by_cell)} cells")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("cells", nargs="*")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--downsample", type=int, default=4)
    args = ap.parse_args()
    os.makedirs(TRIALS_DIR, exist_ok=True)
    open(os.path.join(HERE, "viewer.js"), "w").write(_VIEWER_JS)
    build_robot_js()
    build_warehouse_js(args.downsample)
    cells = args.cells
    if args.all or not cells:
        cells = [os.path.join(DATA, c) for c in DEFAULT_CELLS if os.path.isdir(os.path.join(DATA, c))]
    entries = []
    for c in cells:
        entries += build_trial_html(c)
    build_index(entries)
    print(f"\nDone. Open: {os.path.join(HERE, 'index.html')}")
    return 0


# ── shared viewer ────────────────────────────────────────────────────────────────
_VIEWER_JS = r"""
(function(){
const W=window.WAREHOUSE, T=window.TRIAL, RB=window.ROBOT;
const P=(x,y,z=0)=>new THREE.Vector3(x,z,-y);          // world(x,y,up) -> scene(x,up,-y)
const ext=W.extent, res=W.res;
const cx=(ext.xmin+ext.xmax)/2, cy=(ext.ymin+ext.ymax)/2;
const span=Math.max(ext.xmax-ext.xmin, ext.ymax-ext.ymin);

const scene=new THREE.Scene(); scene.background=new THREE.Color(0x0f1115);
const cam=new THREE.PerspectiveCamera(55, innerWidth/innerHeight, 0.05, 4000);
cam.position.copy(P(cx, cy-span*0.5, span*0.7));
const rend=new THREE.WebGLRenderer({antialias:true}); rend.setSize(innerWidth,innerHeight);
rend.setPixelRatio(devicePixelRatio); document.body.appendChild(rend.domElement);
const ctrl=new THREE.OrbitControls(cam, rend.domElement); ctrl.target.copy(P(cx,cy,0)); ctrl.update();
scene.add(new THREE.HemisphereLight(0xffffff,0x334455,1.05));
const dl=new THREE.DirectionalLight(0xffffff,0.7); dl.position.copy(P(cx+6,cy+6,25)); scene.add(dl);

const LAYER={};
function layer(n,vis){ const g=new THREE.Group(); g.visible=vis!==false; LAYER[n]=g; scene.add(g); return g; }
const gFloor=layer('floor'), gGrid=layer('grid',false), gObs=layer('obstacles3d'),
      gMap=layer('map',false), gPath=layer('path'), gRobot=layer('robot'), gMarks=layer('markers');

// floor + grid
const floor=new THREE.Mesh(new THREE.PlaneGeometry(span*1.2,span*1.2), new THREE.MeshStandardMaterial({color:0x161b24}));
floor.rotation.x=-Math.PI/2; floor.position.copy(P(cx,cy,-0.02)); gFloor.add(floor);
const grid=new THREE.GridHelper(span*1.2, Math.round(span*1.2), 0x2a3550, 0x212838);
grid.position.copy(P(cx,cy,0)); gGrid.add(grid);

// 3D obstacles (instanced boxes)
const oh=W.obstacle_h, oc=W.obstacles.length;
const inst=new THREE.InstancedMesh(new THREE.BoxGeometry(res,oh,res),
  new THREE.MeshStandardMaterial({color:0x415a77,roughness:0.9}), oc);
const M4=new THREE.Matrix4();
for(let i=0;i<oc;i++){ const o=W.obstacles[i]; M4.setPosition(P(o[0],o[1],oh/2)); inst.setMatrixAt(i,M4);}
inst.instanceMatrix.needsUpdate=true; gObs.add(inst);

// occupancy Map overlay (top-down image on a plane) — off by default, toggle "Map"
(function(){
  const cw=Math.round((ext.xmax-ext.xmin)/res), ch=Math.round((ext.ymax-ext.ymin)/res);
  const cvs=document.createElement('canvas'); cvs.width=cw; cvs.height=ch;
  const g=cvs.getContext('2d'); g.clearRect(0,0,cw,ch); g.fillStyle='#cdd6f4';
  for(const o of W.obstacles){ const px=Math.round((o[0]-ext.xmin)/res), py=Math.round((o[1]-ext.ymin)/res);
    g.fillRect(px, ch-1-py, 1, 1); }
  const tex=new THREE.CanvasTexture(cvs); tex.magFilter=THREE.NearestFilter;
  const mp=new THREE.Mesh(new THREE.PlaneGeometry(ext.xmax-ext.xmin, ext.ymax-ext.ymin),
    new THREE.MeshBasicMaterial({map:tex, transparent:true, opacity:0.55}));
  mp.rotation.x=-Math.PI/2; mp.position.copy(P(cx,cy,0.02)); gMap.add(mp);
})();

// start / goal
function marker(x,y,c,r){ const m=new THREE.Mesh(new THREE.SphereGeometry(r,16,16),
  new THREE.MeshStandardMaterial({color:c,emissive:c,emissiveIntensity:0.4})); m.position.copy(P(x,y,r)); gMarks.add(m);}
marker(T.start[0],T.start[1],0x9ece6a,0.15);
marker(T.goal[0],T.goal[1],0xf7768e,0.20);
const pole=new THREE.Mesh(new THREE.CylinderGeometry(0.025,0.025,1.2,8), new THREE.MeshStandardMaterial({color:0xf7768e}));
pole.position.copy(P(T.goal[0],T.goal[1],0.6)); gMarks.add(pole);

// path (full faint + bright trail)
const pathPts=T.path.map(p=>P(p[1],p[2],0.06));
gPath.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pathPts), new THREE.LineBasicMaterial({color:0x39406a})));
const trail=new THREE.Line(new THREE.BufferGeometry().setFromPoints(pathPts.slice(0,1)), new THREE.LineBasicMaterial({color:0x7dcfff}));
gPath.add(trail);

// ── robot: full URDF, meshes EMBEDDED (base64 vertex buffers) so it renders via file:// ──
const robot=new THREE.Group(); robot.matrixAutoUpdate=false; gRobot.add(robot);
const Sworld=new THREE.Matrix4().set(1,0,0,0, 0,0,1,0, 0,-1,0,0, 0,0,0,1);   // world(Z-up)->scene
function b64f32(s){ const bin=atob(s), u=new Uint8Array(bin.length); for(let i=0;i<bin.length;i++)u[i]=bin.charCodeAt(i); return new Float32Array(u.buffer); }
if(RB && RB.links && RB.links.length){
  for(const L of RB.links){
    const g=new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(b64f32(L.verts),3)); g.computeVertexNormals();
    const grp=new THREE.Group();
    grp.position.set(L.xyz[0],L.xyz[1],L.xyz[2]);
    grp.quaternion.set(L.quat[0],L.quat[1],L.quat[2],L.quat[3]);
    grp.add(new THREE.Mesh(g, new THREE.MeshStandardMaterial({color:L.color||0xcccccc, metalness:0.25, roughness:0.55})));
    robot.add(grp);
  }
} else {   // single-box fallback (only if the URDF didn't embed)
  const b=new THREE.Mesh(new THREE.BoxGeometry(0.45,0.30,0.22), new THREE.MeshStandardMaterial({color:0xe0af68}));
  b.position.set(0,0,0.13);
  const nose=new THREE.Mesh(new THREE.ConeGeometry(0.08,0.20,14), new THREE.MeshStandardMaterial({color:0x7dcfff}));
  nose.rotation.z=-Math.PI/2; nose.position.set(0.32,0,0.13);
  robot.add(b,nose);
}
const axes=new THREE.AxesHelper(0.4); axes.visible=false; robot.add(axes);   // TF toggle

// ── playback ──
const T0=T.path[0][0], T1=T.path[T.path.length-1][0], DUR=Math.max(T1-T0,0.001);
let cur=0, playing=true, speed=1, last=performance.now();
function poseAt(tt){ const t=T0+tt, Pp=T.path; let i=0; while(i<Pp.length-1 && Pp[i+1][0]<t) i++;
  const a=Pp[i], b=Pp[Math.min(i+1,Pp.length-1)]; const f=(b[0]-a[0])>1e-6?(t-a[0])/(b[0]-a[0]):0;
  let dy=b[3]-a[3]; while(dy>Math.PI)dy-=2*Math.PI; while(dy<-Math.PI)dy+=2*Math.PI;
  return [a[1]+(b[1]-a[1])*f, a[2]+(b[2]-a[2])*f, a[3]+dy*f, i]; }
function apply(tt){ const pr=poseAt(tt), x=pr[0], y=pr[1], yaw=pr[2], i=pr[3];
  const pose=new THREE.Matrix4().makeRotationZ(yaw); pose.setPosition(x,y,0);
  robot.matrix.multiplyMatrices(Sworld,pose);
  trail.geometry.setFromPoints(pathPts.slice(0,i+2));
  document.getElementById('t').textContent=tt.toFixed(1)+' / '+DUR.toFixed(1)+' s';
  document.getElementById('sl').value=(tt/DUR*1000)|0; }

// ── HUD ──
const hud=document.createElement('div');
hud.innerHTML=`<div id=info></div>
 <div class=row><button id=pp>⏸</button><input id=sl type=range min=0 max=1000 value=0 style="width:320px">
  <span id=t></span><label>speed <select id=sp><option>0.5</option><option selected>1</option><option>2</option><option>4</option></select>×</label></div>
 <div class=row id=layers></div>`;
hud.style.cssText='position:fixed;left:0;right:0;bottom:0;padding:9px 13px;background:rgba(15,17,21,.86);color:#e6e6e6;font:13px system-ui';
document.body.appendChild(hud);
const st=document.createElement('style'); st.textContent='.row{display:flex;gap:12px;align-items:center;margin-top:6px;flex-wrap:wrap}'
 +'button,select{font:13px system-ui;background:#222a3a;color:#e6e6e6;border:1px solid #3a4560;border-radius:6px;padding:3px 9px}'
 +'label{display:flex;gap:4px;align-items:center}.lyr{accent-color:#7dcfff}'; document.head.appendChild(st);
const resu=T.result==='SUCCEEDED'?'#9ece6a':'#f7768e';
document.getElementById('info').innerHTML=`<b>${T.nav} / ${T.cond} / ${T.goal_name}</b> &nbsp; trial ${T.trial} &nbsp;`
 +`<span style="color:${resu}">${T.result}</span> &nbsp; err ${T.err} m &nbsp; ${T.dur}s &nbsp; goal (${T.goal[0]}, ${T.goal[1]})`;
const LBL={obstacles3d:'3D obstacles',path:'Path',map:'Map',grid:'Grid',tf:'TF'};
const lc=document.getElementById('layers');
for(const k of ['obstacles3d','path','map','grid','tf']){
  const l=document.createElement('label'), cb=document.createElement('input');
  cb.type='checkbox'; cb.className='lyr'; cb.dataset.l=k;
  cb.checked = (k==='tf') ? false : (LAYER[k]? LAYER[k].visible : true);
  cb.onchange=()=>{ if(k==='tf'){axes.visible=cb.checked;} else if(LAYER[k]){LAYER[k].visible=cb.checked;} };
  l.appendChild(cb); l.appendChild(document.createTextNode(' '+LBL[k])); lc.appendChild(l); }
document.getElementById('pp').onclick=()=>{playing=!playing;document.getElementById('pp').textContent=playing?'⏸':'▶';};
document.getElementById('sl').oninput=e=>{playing=false;document.getElementById('pp').textContent='▶';cur=e.target.value/1000*DUR;apply(cur);};
document.getElementById('sp').onchange=e=>{speed=parseFloat(e.target.value);};

addEventListener('resize',()=>{cam.aspect=innerWidth/innerHeight;cam.updateProjectionMatrix();rend.setSize(innerWidth,innerHeight);});
function loop(now){ const dt=(now-last)/1000; last=now;
  if(playing){ cur+=dt*speed; if(cur>=DUR){cur=DUR;playing=false;document.getElementById('pp').textContent='▶';} apply(cur);}
  ctrl.update(); rend.render(scene,cam); requestAnimationFrame(loop);}
apply(0); requestAnimationFrame(loop);
})();
"""

_HTML = """<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>trial replay</title>
<style>html,body{margin:0;height:100%;overflow:hidden;background:#0f1115;font-family:system-ui}</style>
</head><body>
<script src="../vendor/three.min.js"></script>
<script src="../vendor/OrbitControls.js"></script>
<script src="../warehouse.js"></script>
<script src="../robot.js"></script>
<script>window.TRIAL = __TRIAL_JSON__;</script>
<script src="../viewer.js"></script>
</body></html>
"""

if __name__ == "__main__":
    raise SystemExit(main())
