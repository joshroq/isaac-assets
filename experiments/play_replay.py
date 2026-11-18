#!/usr/bin/env python3
"""Media-remote replay viewer for a recorded trial USD in Isaac Sim.

Opens a Stage Recorder sampled replay USD and drives the timeline like a DVD/streaming
remote:

    Controls (Isaac window must have focus)
      SPACE / Enter ....... PLAY at 1x forward (always resets to normal speed)
      P / K ............... PAUSE (freeze in place)
      RIGHT / L / F ....... FAST-FORWARD: push the rate toward faster forward
      LEFT  / J / R ....... REWIND: push the rate toward faster reverse
      HOME / 0 ............ jump to start        END / 9 ... jump to end
      ESC / Q ............ quit

There is ONE signed playback rate. FAST-FORWARD nudges it toward faster forward, REWIND
toward faster reverse. So from forward 4x, REWIND steps the speed DOWN 4x->3x->2x->1x and
the next press flips to reverse 1x, then reverse ramps up 1x->2x->3x->4x; FAST-FORWARD is
the mirror. While already moving in a button's own direction it cycles 1x->2x->3x->4x->1x.
PAUSE freezes the current rate; PLAY always resumes forward at 1x. Auto-pauses at each end.

    isaacpy experiments/play_replay.py <replay.usda>
    isaacpy experiments/play_replay.py <replay.usda> --selftest   # headless logic check
"""
from __future__ import annotations

import argparse
import os
import sys
import time


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("usd", help="Stage Recorder sampled replay USD (*_stage_record_sampled.usda)")
    p.add_argument("--selftest", action="store_true",
                   help="headless: verify the transport state machine + that the USD "
                        "actually animates, then exit (no window, no keyboard)")
    return p.parse_args()


class Transport:
    """Remote-style transport driving ONE signed rate: direction {+1,-1} x speed {1..4},
    plus a separate `playing` flag (pause freezes the rate, it is not a third direction).

    FAST-FORWARD pushes the rate toward faster forward; REWIND pushes it toward faster
    reverse. So from forward Nx, REWIND steps the speed DOWN 4x->3x->2x->1x and the next
    press flips to reverse 1x, then reverse ramps 1x->2x->3x->4x (wrapping 4x->1x). FF is
    the mirror. Within a single direction the active button still cycles 1x->2x->3x->4x->1x.
    """

    LEVELS = 4

    def __init__(self, duration: float):
        self.duration = max(duration, 1e-6)
        self.t = 0.0
        self.direction = 1          # +1 forward, -1 reverse (sign of the rate)
        self.speed = 1              # level 1..4 == multiplier
        self.playing = False        # pause/stop when False (direction/speed retained)

    # transport buttons ------------------------------------------------------
    def play(self):
        self.direction, self.speed, self.playing = 1, 1, True   # always normal 1x forward

    def pause(self):
        self.playing = False                                    # freeze current rate

    def fast_forward(self):
        """Step the rate one notch toward faster-forward."""
        self.playing = True
        if self.direction == 1:
            self.speed = self.speed % self.LEVELS + 1           # 1->2->3->4->1
        elif self.speed > 1:
            self.speed -= 1                                     # reverse: slow down
        else:
            self.direction, self.speed = 1, 1                   # cross zero: -> forward 1x

    def rewind(self):
        """Step the rate one notch toward faster-reverse (decreases forward speed first)."""
        self.playing = True
        if self.direction == -1:
            self.speed = self.speed % self.LEVELS + 1           # rev 1->2->3->4->1
        elif self.speed > 1:
            self.speed -= 1                                     # forward: 4->3->2->1
        else:
            self.direction, self.speed = -1, 1                  # cross zero: -> reverse 1x

    def to_start(self):
        self.t, self.playing = 0.0, False

    def to_end(self):
        self.t, self.playing = self.duration, False

    # clock ------------------------------------------------------------------
    def advance(self, dt_wall: float):
        if not self.playing:
            return
        self.t += self.direction * self.speed * dt_wall
        if self.t <= 0.0:
            self.t, self.playing = 0.0, False                   # auto-pause at start
        elif self.t >= self.duration:
            self.t, self.playing = self.duration, False         # auto-pause at end

    def label(self) -> str:
        rate = f"{'>' if self.direction == 1 else '<'}{self.speed}x"
        if not self.playing:
            at_end = self.t <= 0.0 or self.t >= self.duration
            state = f"|| {'STOP' if at_end else 'PAUSED'} ({rate})"
        elif self.direction == 1:
            state = ">  PLAY 1x" if self.speed == 1 else f">> FF {self.speed}x"
        else:
            state = f"<< REW {self.speed}x"
        return f"{state}   t = {self.t:6.1f} / {self.duration:.1f} s"


def _selftest(usd: str) -> int:
    """Exercise the state machine and confirm the USD has real motion to show."""
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import omni.usd
    from pxr import Usd, UsdGeom
    ok = True

    tr = Transport(duration=10.0)
    # (dir, speed, playing) expected after each press
    seq = [("play", (1, 1, True)),           # normal forward
           ("fast_forward", (1, 2, True)),   # FF cycles up while forward
           ("fast_forward", (1, 3, True)),
           ("fast_forward", (1, 4, True)),
           ("fast_forward", (1, 1, True)),   # wrap 4->1
           ("fast_forward", (1, 2, True)),   # back up to 2x
           ("rewind", (1, 1, True)),         # REW steps DOWN 2x -> 1x (still forward)
           ("rewind", (-1, 1, True)),        # cross zero -> reverse 1x
           ("rewind", (-1, 2, True)),        # reverse ramps up
           ("fast_forward", (-1, 1, True)),  # FF slows reverse 2x -> 1x
           ("fast_forward", (1, 1, True)),   # cross zero -> forward 1x
           ("pause", (1, 1, False)),         # pause freezes (rate retained)
           ("play", (1, 1, True))]
    for fn, exp in seq:
        getattr(tr, fn)()
        got = (tr.direction, tr.speed, tr.playing)
        flag = "ok" if got == exp else "BAD"
        if got != exp:
            ok = False
        print(f"[selftest] {fn:<13} -> dir={tr.direction:+d} speed={tr.speed}x "
              f"playing={tr.playing}  [{flag}]")

    ctx = omni.usd.get_context()
    if not ctx.open_stage(usd):
        print(f"[selftest] FAILED to open {usd}")
        app.close()
        return 1
    for _ in range(10):
        app.update()
    stage = ctx.get_stage()
    fps = stage.GetTimeCodesPerSecond() or 30.0
    end = float(stage.GetEndTimeCode())
    rover = stage.GetPrimAtPath("/World/ugv_rover/base_footprint")
    xs = []
    for frac in (0.0, 0.5, 1.0):
        tc = end * frac
        m = UsdGeom.XformCache(Usd.TimeCode(tc)).GetLocalToWorldTransform(rover)
        p = m.ExtractTranslation()
        xs.append(float(p[0]))
        print(f"[selftest] t={tc/fps:5.1f}s  base_x={p[0]:6.2f} base_y={p[1]:6.2f}")
    moved = (max(xs) - min(xs)) > 0.5
    print(f"[selftest] duration={end/fps:.1f}s  animates={'YES' if moved else 'NO'}")
    app.close()
    return 0 if (ok and moved) else 1


def _run_viewer(usd: str) -> int:
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": False, "renderer": "RayTracedLighting"})

    import carb.input
    import omni.appwindow
    import omni.timeline
    import omni.usd
    import omni.ui as ui

    ctx = omni.usd.get_context()
    if not ctx.open_stage(usd):
        print(f"[play_replay] could not open {usd}")
        app.close()
        return 1
    for _ in range(20):
        app.update()
    stage = ctx.get_stage()
    fps = stage.GetTimeCodesPerSecond() or 30.0
    duration = float(stage.GetEndTimeCode()) / fps

    timeline = omni.timeline.get_timeline_interface()
    timeline.set_start_time(0.0)
    timeline.set_end_time(duration)
    timeline.set_looping(False)
    # We drive the clock ourselves via set_current_time each frame. Disable the timeline's
    # own auto-advance so our value is authoritative (verified this populates Fabric so the
    # RTX viewport re-samples the animation), then play() to put it in render-follow mode.
    try:
        timeline.set_auto_update(False)
    except Exception:
        pass
    timeline.play()

    tr = Transport(duration)

    K = carb.input.KeyboardInput
    # Built by name via getattr so one renamed enum across Isaac versions can't crash the
    # viewer — absent names are simply skipped.
    _BINDINGS = {
        "play": ("SPACE", "ENTER"),
        "pause": ("P", "K"),
        "ff": ("RIGHT", "L", "F"),
        "rew": ("LEFT", "J", "R"),
        "start": ("HOME", "NUMPAD_0", "KEY_0"),
        "end": ("END", "KEY_9", "NUMPAD_9"),
        "quit": ("ESCAPE", "Q"),
    }
    KEYMAP = {}
    for action, names in _BINDINGS.items():
        for name in names:
            key = getattr(K, name, None)
            if key is not None:
                KEYMAP[key] = action
    state = {"quit": False}

    def dispatch(action):
        if action == "play":
            tr.play()
        elif action == "pause":
            tr.pause()
        elif action == "ff":
            tr.fast_forward()
        elif action == "rew":
            tr.rewind()
        elif action == "start":
            tr.to_start()
        elif action == "end":
            tr.to_end()
        elif action == "quit":
            state["quit"] = True

    def on_keyboard(event, *_):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            action = KEYMAP.get(event.input)
            if action:
                dispatch(action)
        return True

    appwin = omni.appwindow.get_default_app_window()
    keyboard = appwin.get_keyboard()
    inp = carb.input.acquire_input_interface()
    sub = inp.subscribe_to_keyboard_events(keyboard, on_keyboard)

    hud = ui.Window("Replay Remote", width=360, height=150)
    with hud.frame:
        with ui.VStack(spacing=6):
            status = ui.Label("", alignment=ui.Alignment.CENTER,
                              style={"font_size": 24, "color": 0xFFF0F0F0})
            ui.Label("SPACE play  |  P pause  |  → FF(1-4x)  |  ← REW(1-4x)",
                     alignment=ui.Alignment.CENTER, style={"font_size": 13})
            ui.Label("HOME start  |  END end  |  ESC quit",
                     alignment=ui.Alignment.CENTER, style={"font_size": 13})

    banner = __doc__.split("\n\n")[1]
    print("[play_replay] " + usd)
    print(banner)
    print(f"[play_replay] duration = {duration:.1f}s @ {fps:g} fps. Click the Isaac window, "
          "then use the keys above.")

    tr.play()                            # start playing at 1x on open
    last = time.monotonic()
    while app.is_running() and not state["quit"]:
        now = time.monotonic()
        dt = now - last
        last = now
        tr.advance(dt)
        timeline.set_current_time(tr.t)  # scrub the animation to our virtual time
        try:
            timeline.commit()
        except Exception:
            pass
        status.text = tr.label()
        app.update()

    inp.unsubscribe_to_keyboard_events(keyboard, sub)
    app.close()
    return 0


def main() -> int:
    args = parse_args()
    usd = os.path.abspath(os.path.expanduser(args.usd))
    if not os.path.exists(usd):
        print(f"[play_replay] no such file: {usd}")
        return 2
    return _selftest(usd) if args.selftest else _run_viewer(usd)


if __name__ == "__main__":
    sys.exit(main())
