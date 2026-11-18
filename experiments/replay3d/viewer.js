
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
