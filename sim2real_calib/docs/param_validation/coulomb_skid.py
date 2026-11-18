"""Planar rigid-body skid-steer with pure Coulomb wheel friction (no tyre slip stiffness)."""
import numpy as np, sys
m, Izz, g, mu, r, R = 2.19, 0.0122 + 6*0.08*0.0873**2, 9.81, 0.75, 0.04, 0.04
X = [0.0855, 0.0, -0.0855]; Y = 0.08726
wheels = [(x, s*Y, x != 0) for x in X for s in (1, -1)]       # (x, y, driven)
N = m*g/6
def run(v_c, w_c, D=0.5, T=6.0, dt=2e-4, eps=1e-3, stiff=None):
    vx = vy = w = 0.0
    out = []
    for k in range(int(T/dt)):
        Fx = Fy = M = 0.0
        for x, y, drv in wheels:
            u, lat = vx - w*y, vy + w*x
            if drv:
                side = 1 if y > 0 else -1                       # +1 left, -1 right
                rim = r*(v_c - side*D*w_c/2)/R
                sx = u - rim
            else:
                sx = 0.0                                     # free-rolling passive wheel
            sy = lat
            if stiff:                                       # linear tyre: F = -k*slip, capped at mu N
                fx, fy = -stiff*sx, -stiff*sy
                n = np.hypot(fx, fy); c = min(1, mu*N/max(n, 1e-12)); fx, fy = fx*c, fy*c
            else:
                n = np.sqrt(sx*sx + sy*sy + eps*eps); fx, fy = -mu*N*sx/n, -mu*N*sy/n
            Fx += fx; Fy += fy; M += x*fy - y*fx
        vx += (Fx/m + w*vy)*dt; vy += (Fy/m - w*vx)*dt; w += M/Izz*dt
        out.append((vx, vy, w))
    o = np.array(out[len(out)//2:]); return o[:, 0].mean(), o[:, 2].mean()
for name, kw in (("Coulomb (rate-independent)", {}), ("tyre slip stiffness k=30 N/(m/s)", {"stiff": 30.0})):
    print(name)
    for v, wc in ((0.0, 0.6), (0.12, 0.4), (0.15, 0.15)):
        vx, w = run(v, wc, **kw)
        print(f"   cmd v={v:.2f} w={wc:.2f} -> v={vx:+.3f} w={w:+.3f} ({100*w/wc:.0f}% of cmd yaw)")
