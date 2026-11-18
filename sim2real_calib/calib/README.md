# Grid calibration (any size)

`calibrate_grid.py` maps overhead pixels to floor metres for a tape grid of any
W x H size. Click four anchors; it then finds every tape crossing itself and fits
the homography to all of them, so the result does not depend on click precision.

```bash
# one consumer of UDP 5000 at a time: close live_tracking.py / receive.py first
python3 sim2real_calib/calib/calibrate_grid.py --source stream --size 1   --spacing 0.25
python3 sim2real_calib/calib/calibrate_grid.py --source stream --size 2x2 --spacing 0.5
python3 sim2real_calib/calib/calibrate_grid.py --source stream --size 1x4 --spacing 0.5
```

- **Frame:** origin at the outer corner that is bottom-left in the camera image,
  +X along the bottom edge, +Y up the left edge. `--size WxH` is the X extent times
  the Y extent, so a 1 x 4 m strip running up the image is `1x4`, and one running
  across the image is `4x1`.
- **Sizes are tape centre to tape centre.** Click the middle of each tape crossing.
  A magnifier follows the mouse, and a small map shows which corner comes next.
- **`--spacing`** is the distance between tape lines (`0.5`, or `0.5,0.25` for
  different X/Y spacing). It must divide the size. Without it, only the four outer
  corners are used.
- **Review window:** green dots are detected crossings, red X marks are crossings
  that were not found (rover in the way, out of view), and magenta arrows are
  residuals x10. Enter/S saves, U re-clicks, Q quits. When several arrows along one
  tape all point the same way, that strip is laid off-line. The live 1x1 m grid
  shows 6 mm rms of this, and it is the physical limit of the target.
- **Corners out of view:** click any 4+ known crossings instead, for example
  `--anchors "0,0.5 1,0.5 1,3.5 0,3.5"`. Crossings outside the image are skipped,
  and tape cut by the image border is ignored rather than mis-centred.
- **Coverage:** the C920e at its current ~1.5 m height sees about 2.1 x 1.2 m of
  floor. A 2x2 or 1x4 grid needs the camera higher, or rotated so the long side
  runs across the image (`4x1`).

When saving to the default `grid_calib.json`, the tool also:

1. backs up the old file (`grid_calib.before_regrid_<time>.json`);
2. re-expresses `sticker_config.json`'s `camera_position_m` (used for the 0.159 m
   sticker-height correction) in the new grid frame;
   - if the camera-guard reference shows the camera has not moved, the existing
     value is transformed as-is;
   - otherwise the homography estimate is used;
   - `--keep-sticker-camera` skips this step;
3. saves a new camera-guard reference. `loop/camera_guard.py` now refuses a
   reference that is older than the grid calibration, because it would quietly
   bring back the old one.

If the grid size changed, re-teach `loop/stations`.

Check a saved calibration against the live grid (no clicks):

```bash
python3 sim2real_calib/calib/calibrate_grid.py --verify --source stream [--save /tmp/v.png]
# files saved before this version store no spacing: add e.g. --spacing 0.25
```

The check reports the worst disagreement over the grid area between the saved
calibration and a fresh fit to the detected crossings. It exits 1 above
`--max-drift-mm` (default 10 mm).

Tests (synthetic 1x1, 2x2, 1x4 and 4x1 grids with a known camera; 10 px click
noise is corrected to <= 2.4 mm worst case):

```bash
python3 -m unittest discover -s sim2real_calib/calib -p test_grid_target.py -v
```

# Sticker verification

`verify_stickers.py` only tracks the stickers. It never drives the rover. Every
colour is detected on its own and projected to grid metres, and the tool shows
and prints each sticker's (x, y), detection rate and jitter. When all three
stickers are visible it also reports:

- chassis centre and yaw;
- each triangle edge compared with the measured geometry;
- the camera height implied by the triangle's apparent size.

A far-away blob of the same colour is never reported as the sticker. When a
sticker goes missing, the HSV values seen at its last position are recorded so
the colour ranges can be tuned.

```bash
python3 sim2real_calib/calib/verify_stickers.py                  # live window + 1 Hz printout
python3 sim2real_calib/calib/verify_stickers.py --duration 15 --csv /tmp/s.csv --report /tmp/s.json
python3 sim2real_calib/calib/verify_stickers.py --expect-center 0.5,0.5,90   # rover parked on a mark
python3 sim2real_calib/calib/verify_stickers.py --plane floor --expect green=0.5,0.5   # loose sticker on a crossing
```

Defaults:

- **Plane:** the rover's sticker plane (0.159 m up). Use `--plane floor` for loose
  stickers lying on the floor; on a known crossing they check the grid calibration
  directly.
- **Pass:** each sticker is seen at least 90% of the time with jitter of 3 mm or
  less, and every triangle edge is within 5 mm. Use `--moving` when the rover
  moves, which drops the jitter check.
- **`--expect`:** the expected position must be within 15 mm and the yaw within 3
  degrees.
- **Exit code:** 0 for PASS, 1 for FAIL.

On a stationary rover: 100 % detection, 0.2-0.5 mm jitter, all edges within 2.3 mm.

# Colored sticker motion tracking

The active tracker uses the installed triangle:

- Green: front-left.
- Orange-red: front-right.
- Pink: rear-center (was yellow until 2026-11-18; swapped because the harsh garage
  light made yellow hard to separate from the washed-out orange-red).
- Diameter: 3/4 inch (0.01905 m).
- Measured height: 6 1/4 inches (0.15875 m) above the floor.

`sticker_config.json` contains colors sampled from the actual overhead feed,
including red's hue wrap and how washed out orange-red looks under the garage lights. Candidate dots
are filtered by area/roundness, then all three must fit the configured triangle
with a proper rigid rotation and translation. Missing dots or invalid geometry
produce no pose; an old pose is never recorded as a fresh sample.

The moved camera uses the user's corrected `calib/grid_calib.json`. The current
height correction uses camera position estimated from that homography and the
nominal C920e field of view. This is explicitly marked as approximate.

## View tracking now

Run ONE camera consumer at a time:

```bash
# Simplest: live feed + grid + sticker tracking + position/heading panel
python3 sim2real_calib/live_tracking.py
python3 sim2real_calib/capture/receive.py --track
# Metric grid, tracking, hover measurements:
python3 sim2real_calib/viz/metric_overlay.py
# Text-only pose check:
python3 sim2real_calib/calib/calibrate_rover.py --mode check --duration 10
```

Green/orange-red identify the front edge; pink identifies the rear. The origin is now the **chassis center**, registered using the user's measured offsets:

| Sticker | Forward (m) | Left (m) |
|---|---:|---:|
| Green | +0.0650875 | +0.0508 |
| Orange-red | +0.0650875 | −0.0508 |
| Pink | −0.0650875 | 0 |

The rear sticker's "same distance" is interpreted as 2.5625 inches behind center.
The 3-inch spacing above the lower chassis is stored as a mounting measurement;
the camera projection continues to use 6.25 inches above the floor. "Chassis
center" refers to the horizontal reference between the middle-wheel hubs, not
the bottom surface of the rover.

The overlay labels this **chassis center (camera estimate)** because the camera
position still comes from nominal intrinsics. Measured body geometry is complete;
verified sticker-plane projection remains necessary for precision calibration trials.

An explicit diagnostic CSV is available before precision calibration:

```bash
python3 sim2real_calib/trials/record_traj.py --source camera \
  --duration 10 --allow-approximate --out /tmp/sticker_preview.csv
```

## Register the chassis center for sim-versus-real trials

The installed offsets have been entered. To change them later, measure each
sticker center from the midpoint between the middle-wheel hubs.
Coordinates are metres, +x forward, +y left. List them in green, orange-red,
pink order. Do not use the old rear-marker offset in `rover_calib.json`.

```bash
# Replace these illustrative offsets and camera coordinates with measurements:
python3 sim2real_calib/calib/calibrate_stickers.py \
  --positions-m '0.07,0.05 0.07,-0.05 -0.07,0' \
  --height-m 0.15875 --camera-position-m '0.50,-0.04,1.52'
```

The window captures the overhead feed. Click green, orange-red, pink centers;
U undoes a click, Enter saves, Q cancels. A saved overhead image can be used with
`--image`; `--points-px 'x,y x,y x,y'` supports a scripted setup. Setup validates
the detection before replacing its JSON configuration. Existing measured height
and body offsets are retained when their arguments are omitted.

Camera coordinates refer to the camera's optical center in the floor grid's XY
frame, with Z equal to measured height above the floor. To avoid relying on an
estimated camera position, another option is a sticker-plane calibration: put
four targets at 0.15875 m height directly above the four outer grid corners,
calibrate them into a separate JSON using `calib/calibrate_grid.py --no-refine --out`
(`--no-refine`: the targets are not tape, so don't snap to the floor grid), then
pass that JSON with `--plane-calib`. Keep the same corner order/grid origin.
Do not replace the floor calibration with the sticker-plane calibration.

Normal camera recording and real motion trials require measured body geometry
and verified sticker-plane projection. The recorder then reports the configured
chassis origin directly, with the same `t,x,y,yaw` CSV schema used by `sim_eval`.
It records only new camera frames and writes a `.tracking.json` sidecar containing
the reference point, configuration and grid calibration. Approximate diagnostics
require `--allow-approximate`; they are not silently used as chassis-center truth.

## Verification

```bash
python3 -m unittest discover -s sim2real_calib/calib -p test_sticker_tracker.py -v
```

Tests cover full heading rotation, the chassis-center offset, height correction,
red hue wrap, low-saturation colors, distractors, missing dots, reflections, invalid
triangles, changed resolution, measured setup and approximate-recording rejection.
The initial live recording tracked 240 frames without a missed triangle. After
switching to the user's corrected grid, the updated receiver tracked 181 frames
without a missed triangle. A separate six-second CSV check using the corrected
grid recorded 181 fresh samples, no missed triangles, and strictly increasing
timestamps. These were visibility/stability checks; no physical
rover command was sent.

The camera intrinsic-calibration tools can still use their ChArUco board. Rover
motion tracking no longer depends on an ArUco marker.

Implementation references: [OpenCV HSV color tracking](https://docs.opencv.org/4.13.0/df/d9d/tutorial_py_colorspaces.html) and [contour measurements](https://docs.opencv.org/4.13.0/dd/d49/tutorial_py_contour_features.html).
