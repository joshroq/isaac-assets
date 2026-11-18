# replay_video — trials recorded INSIDE the Isaac sim

Third-person (chase-camera) MP4s of the real simulated rover driving each recorded trajectory
through the **actual full simulated warehouse** (not a reconstruction, not RViz, not an onboard
camera). This is the replay format to watch the robot navigate the real sim environment.

**View:** open `index.html` (plays all six G4 videos), or any `.mp4` directly.

**Regenerate / extend:**
```bash
isaacpy experiments/render_replay.py --all                    # G4 lidar/depth/rtab x cond, 1 trial each
isaacpy experiments/render_replay.py experiments/data/<cell> --trials 3 --fps 24 --max-frames 600
```
It boots one headless Isaac stage (rover + warehouse_basic_obstacles), kinematically drives the
rover model along `paths/<trial>.csv` (map-frame, transformed to world via the rover start pose),
and records a chase camera (BasicWriter frames -> cv2 MP4, no ffmpeg needed). `--max-frames`
caps/speeds long trials; `--settle` = RTX subframes per frame (quality vs speed).

Note: needs a clean Isaac boot (a fresh `/clock`); a stale/zombie sim from a prior crash breaks it.
