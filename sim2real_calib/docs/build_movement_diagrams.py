"""Build static explanatory figures; no Isaac launch or rover connection."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch, FancyBboxPatch

OUT = Path(__file__).with_name("movement_figures")
OUT.mkdir(exist_ok=True)
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11,
                     "axes.spines.top": False, "axes.spines.right": False})
BLUE, ORANGE, GREEN, RED = "#2366a8", "#d58022", "#348763", "#b73e43"

def save(fig, name):
    fig.savefig(OUT / f"{name}.png", dpi=170, bbox_inches="tight", facecolor="white")
    fig.savefig(OUT / f"{name}.svg", bbox_inches="tight", facecolor="white")
    fig.savefig(OUT / f"{name}.pdf", bbox_inches="tight", facecolor="white")
    plt.close(fig)

def arrow(ax, a, b, color=BLUE, **kwargs):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=15,
                               lw=1.8, color=color, **kwargs))

fig, ax = plt.subplots(figsize=(14, 5.2))
ax.set(xlim=(0, 14), ylim=(0, 5.2)); ax.axis("off")
boxes = [(0.1, "Command", "/cmd_vel\n$v_c,\Omega_c$"),
         (2.45, "Controller", "$R,D,v_{max},\Omega_{max}$\nwheel-speed targets"),
         (4.8, "Four drives", "$b,\\tau_{max}$\nwheel torques"),
         (7.15, "Six contacts", "$\mu_s,\mu_d$, combine\ntraction + scrub"),
         (9.5, "Rigid bodies", "$m,I,COM,d_l,d_a$\n$v_x,v_y,\Omega$"),
         (11.85, "Pose", "$x,y,\psi$\nphysics odometry")]
for x, title, txt in boxes:
    ax.add_patch(FancyBboxPatch((x, 3.0), 2.05, 1.55, boxstyle="round,pad=0.06",
                               facecolor="#edf3f9", edgecolor=BLUE))
    ax.text(x+1.025, 4.2, title, ha="center", weight="bold")
    ax.text(x+1.025, 3.55, txt, ha="center", va="center")
for x, _, _ in boxes[:-1]: arrow(ax, (x+2.08, 3.75), (x+2.29, 3.75))
ax.text(0.2, 2.2, "REAL: same sequence → Jetson driver → motor firmware → rover → overhead stickers",
        color=GREEN, weight="bold")
ax.text(0.2, 1.55, "COMPARE: camera pose and sim pose → motion features → weighted error → optimizer")
ax.text(0.2, .95, "APPLY: shared calib_params.py → in-memory trial override or saved USD calibration")
ax.text(0.2, .25, "The optimizer changes simulation parameters. Camera settings change the measurement, not the rover physics.",
        fontsize=10, color="#555555")
save(fig, "01_pipeline")

import json
AUDIT = json.loads(Path(__file__).with_name("movement_asset_audit.json").read_text())
CURRENT_MAT = {b["collider"].split("/")[3].replace("_wheel_link", ""): b["physics_material"].rsplit("/", 1)[-1]
               for b in AUDIT["wheel_material_bindings"]}
BEFORE_FIX_MAT = {w: "middle_wheel" for w in CURRENT_MAT}; BEFORE_FIX_MAT["right_up"] = "corner_wheel"
WHEEL_AT = {("left", .0855): "left_up", ("left", 0): "left_middle", ("left", -.0855): "left_down",
            ("right", .0855): "right_up", ("right", 0): "right_middle", ("right", -.0855): "right_down"}

fig, axes = plt.subplots(1, 3, figsize=(17, 6))
for ax in axes:
    ax.set_aspect("equal"); ax.set(xlim=(-.16, .16), ylim=(-.16, .16)); ax.axis("off")
    ax.add_patch(Rectangle((-.065, -.105), .13, .21, facecolor="#e8edf2", edgecolor="#667788"))
ax = axes[0]
for forward in [-.0855, 0, .0855]:
    for left in [-.08726, .08726]:
        driven = forward != 0
        ax.add_patch(Rectangle((-left-.014, forward-.028), .028, .056,
                              facecolor=BLUE if driven else "#a5abb3", edgecolor="white"))
ax.plot(0, 0, "ko", ms=4)
arrow(ax, (0, 0), (0, .067), GREEN)
arrow(ax, (0, 0), (-.055, 0), GREEN)
ax.text(.005, .056, "+x forward", fontsize=9)
ax.text(-.055, -.012, "+y left", fontsize=9, ha="center")
axes[0].set_title("Intended geometry and wheel roles", weight="bold")
axes[0].annotate("", (-.08726, -.127), (.08726, -.127), arrowprops={"arrowstyle":"<->"})
axes[0].text(0, -.145, "physical track B = 0.17452 m", ha="center", fontsize=10)
axes[0].annotate("", (-.135, -.0855), (-.135, .0855), arrowprops={"arrowstyle":"<->"})
axes[0].text(-.148, 0, "L = 0.171 m", rotation=90, va="center", fontsize=10)
axes[0].text(0, .14, "Blue = driven corner; gray = passive middle", ha="center", fontsize=10)
for ax, mats, title, note, col in (
        (axes[1], BEFORE_FIX_MAT, "USD bindings BEFORE the 2026-10-14 fix",
         "3 driven corners used middle_wheel", RED),
        (axes[2], CURRENT_MAT, "USD bindings NOW (fix_wheel_physics.py)",
         "matches the intended roles", GREEN)):
    ax.set_title(title, weight="bold")
    for forward in [-.0855, 0, .0855]:
        for side, left in (("left", .08726), ("right", -.08726)):
            mat = mats[WHEEL_AT[(side, forward)]]
            ax.add_patch(Rectangle((-left-.014, forward-.028), .028, .056,
                                  facecolor=BLUE if mat == "corner_wheel" else "#a5abb3", edgecolor="white"))
            ax.text(-left, forward, "C" if mat == "corner_wheel" else "M", ha="center", va="center",
                    color="white", weight="bold", fontsize=10)
    ax.text(0, .14, "C = corner_wheel material; M = middle_wheel", ha="center", fontsize=10)
    ax.text(0, -.145, note, ha="center", color=col, fontsize=10, weight="bold")
fig.text(.5, .01, "Top view: front is up. Right panel is read from movement_asset_audit.json.", ha="center")
fig.tight_layout(rect=(0, .04, 1, 1)); save(fig, "02_geometry_and_bindings")

fig, axes = plt.subplots(1, 2, figsize=(13, 5.3))
ax=axes[0]; ax.set_aspect("equal"); ax.axis("off"); ax.set(xlim=(-2, 2.8), ylim=(-1.8, 1.8))
ax.add_patch(Circle((0, 0), .9, facecolor="#e9eef3", edgecolor=BLUE, lw=2))
ax.plot([-1.8, 2.6], [-.9, -.9], color="#555555", lw=3)
arrow(ax, (0, -.9), (0, .2), GREEN); ax.text(.08, -.35, "$N_i$")
arrow(ax, (0, -.9), (1.3, -.9), BLUE); ax.text(.85, -.75, "$F_{x,i}$")
arrow(ax, (0, 0), (0, -.65), RED); ax.text(.1, -.6, "load")
arrow(ax, (-.7, 1.3), (.7, 1.3)); ax.text(.9, 1.25, "$v_i$")
ax.text(0, .25, "$r,\dot q_i,\\tau_i$", ha="center")
ax.text(-1.65, -1.55, "$I_w\ddot q_i=\\tau_i-rF_{x,i}$\nrolling: $v_i=r\dot q_i$; slipping: unequal")
ax.set_title("Wheel torque becomes ground force", weight="bold")
ax=axes[1]; mu=.75; a=np.linspace(0,2*np.pi,300)
ax.plot(mu*np.cos(a),mu*np.sin(a),color=BLUE,lw=2)
ax.fill(mu*np.cos(a),mu*np.sin(a),color="#edf3f9")
ax.arrow(0,0,.52,.34,width=.012,color=RED,length_includes_head=True)
ax.text(.25,.5,"combined demand",color=RED)
ax.set_aspect("equal"); ax.set(xlim=(-1,1),ylim=(-1,1),xlabel="$F_x/N$",ylabel="$F_y/N$")
ax.set_title("Traction and turning share a friction budget",weight="bold")
ax.text(0,-.92,"$F_x^2+F_y^2\leq (\mu N)^2$",ha="center")
ax.grid(alpha=.15); fig.tight_layout(); save(fig,"03_contact")

fig, axes=plt.subplots(1,2,figsize=(13,5.5))
ax=axes[0]; ax.set_aspect("equal"); ax.set(xlim=(-.16,.16),ylim=(-.14,.15)); ax.axis("off")
ax.add_patch(Rectangle((-.065,-.10),.13,.20,facecolor="#edf3f9",edgecolor=BLUE))
for x in [-.0855,0,.0855]:
    for y in [-.08726,.08726]:
        ax.plot(-y,x,'o',color=BLUE)
        lateral=.045*np.sign(x)
        if lateral: arrow(ax,(-y,x),(-y-lateral,x),RED)
ax.text(0,.135,"For a left spin: $v_{y,i}=\Omega x_i$",ha="center")
ax.text(0,-.135,"Front/rear scrub sideways; middle row has $x_i=0$",ha="center",fontsize=10)
ax.set_title("Why six parallel wheels must skid to turn",weight="bold")
ax=axes[1]; t=np.linspace(0,5,300); v=.12; omega=.4
ax.plot(v/omega*np.sin(omega*t),v/omega*(1-np.cos(omega*t)),lw=2,color=BLUE)
ax.plot(0,v/omega,"o",color=ORANGE); ax.plot(0,0,"ko")
ax.annotate("ICR",(0,v/omega),(-.10,.38),arrowprops={"arrowstyle":"->"})
ax.annotate("",(0,0),(0,v/omega),arrowprops={"arrowstyle":"<->","color":ORANGE})
ax.text(.01,.15,r"$\rho=v/\Omega$",color=ORANGE)
ax.set(xlabel="world x [m]",ylabel="world y [m]",title="Arc radius and instantaneous center of rotation")
ax.set_aspect("equal"); ax.grid(alpha=.2)
fig.tight_layout(); save(fig,"04_skid_and_arc")

fig,axes=plt.subplots(1,2,figsize=(13,5.2))
ax=axes[0]; ax.set(xlim=(-.1,1.4),ylim=(-.12,1.8)); ax.axis("off")
C=np.array([.15,1.52]); Q=np.array([1.1,0]); h=.15875
P=C+(1-h/C[1])*(Q-C)
ax.plot([-.1,1.35],[0,0],color="#555555",lw=2)
ax.plot([-.1,1.35],[h,h],"--",color=GREEN)
ax.plot([C[0],Q[0]],[C[1],Q[1]],color=BLUE)
ax.plot(*C,"o",color=BLUE);ax.text(C[0]+.04,C[1],"camera C")
ax.plot(*P,"o",color=GREEN);ax.text(P[0]-.27,P[1]+.1,"sticker P")
ax.plot(*Q,"o",color=RED);ax.text(Q[0]-.2,-.10,"floor projection Q")
ax.text(.02,.55,"$P_{xy}=C_{xy}+(1-h/C_z)(Q_{xy}-C_{xy})$")
ax.text(.02,.38,"h = 0.15875 m; C_z ≈ 1.51774 m",fontsize=10)
ax.set_title("Correct the sticker height before measuring motion",weight="bold")
ax=axes[1]; body=np.array([[.0650875,.0508],[.0650875,-.0508],[-.0650875,0]])
psi=.7; rot=np.array([[np.cos(psi),-np.sin(psi)],[np.sin(psi),np.cos(psi)]])
obs=body@rot.T+np.array([.22,.15])
for pts,col,label in [(body,BLUE,"known body offsets"),(obs,GREEN,"observed metric stickers")]:
    p=np.vstack([pts,pts[0]]);ax.plot(p[:,0],p[:,1],"o-",color=col,label=label)
ax.plot(0,0,"x",color=BLUE);ax.plot(.22,.15,"x",color=GREEN)
arrow(ax,(0,0),(.22,.15),ORANGE)
ax.text(.09,.03,"translation p",color=ORANGE)
ax.set_aspect("equal");ax.set(xlabel="x [m]",ylabel="y [m]",title="Rigid triangle fit gives center and yaw")
ax.legend(loc="upper left",fontsize=9);ax.grid(alpha=.2)
fig.tight_layout();save(fig,"05_camera_and_pose")

print(f"Wrote {len(list(OUT.glob('*.png')))} figures, each as PNG/SVG/PDF, to {OUT}")
