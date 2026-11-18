# nav_experiment — randomized-warehouse navigation testing

> ## ⏸ PAUSED — not the current thesis focus (Part 1)
>
> **Randomized-object environment experiments are paused. They belong to the later
> policy-training/generalization phase, not the current SLAM/Nav2 fidelity baseline
> phase.**
>
> Current thesis **Part 1 is not randomized-object navigation.** Part 1 uses a
> carefully matched real/sim room to evaluate SLAM/Nav2 transfer (controlled,
> repeatable geometry, object placement, route, start pose and goal pose). The
> randomized-object code here is **kept and not deleted** — it is preserved for the
> later policy-training/generalization work. Do not create new randomized-object
> trials unless explicitly asked. See `ugv_controller_v2/README.md` → *Part 1
> SLAM/Nav2 fidelity baseline* for the active stack list.

Tools for driving the baked v2 rover to goals on a warehouse map with randomized
obstacles, and for benchmarking the different localizers behind the same Nav2 stack.
**(Paused for Part 1 — see the note above.)**

## Quick start (reliable stack: lidar / depth / rtab)

One command brings up Isaac (randomized obstacles) + the nav system + Nav2, in a
single tmux window, and **auto-drives the rover to goals on its own**. Use the
`navrandom` front door (the randomized-warehouse sibling of `navlaunch`), which
takes the same `-lidar` / `-depth` / `-rtab` tags:

```bash
navrandom -lidar        # auto-drive goals, lidar slam_toolbox
navrandom -depth        # lidar map + depth cloud
navrandom -rtab         # RTAB-Map RGB-D
navrandom -lidar --auto 5   # 5 goals (default count is 3)
```

`navrandom` is **autonomous by default**: it defaults to `--auto` (goals are picked
and sent by `nav_mission.py`, no RViz click needed) and to `--no-rviz` (RViz's GLX
context is the least stable process here and is not in the control loop — watch the
rover in the Isaac viewport). Opt out with `--no-auto` (bring Nav2 up ACTIVE and
wait) and `--rviz` (launch RViz too):

```bash
navrandom -lidar --no-auto --rviz    # Nav2 up + RViz; click 2D Goal Pose yourself
```

`navrandom` is a thin wrapper over `warehouse_lab.sh`, which you can also call
directly with the bare system word (warehouse_lab.sh keeps the RViz-click default —
it does **not** auto-drive unless you pass `--auto`):

```bash
ugv_controller_v2/nav_experiment/warehouse_lab.sh lidar          # Nav2 up, click 2D Goal in RViz
ugv_controller_v2/nav_experiment/warehouse_lab.sh rtab --auto 5  # auto-drive 5 goals
```

Obstacles are scattered **live with a fresh random layout every run** (no fixed
seed), so each launch puts the props in different places. Pass `--seed N` for a
reproducible layout, or `--baked` to load a pre-generated USD instead (Replicator
props do not always survive the USD export, which is why live is the default). The
lab also **tears down any leftover nav stack first** — a prior
navlaunch/Nav2/slam/DPVO/ORB-SLAM/RViz run otherwise keeps publishing `/map`, TF
and a ghost Nav2 into this one; use `--no-clean` to keep prior processes.

Useful options: `--seed N --count N --style props|primitives --baked --fresh
--no-clean --headless --no-rviz --no-attach`, and `-- <extra navlaunch args>` —
`navrandom` forwards them all to `warehouse_lab.sh`. See `navrandom --help` /
`warehouse_lab.sh --help`.

> `navrandom` fills the role you might expect from `navlaunch -v2 -random -lidar`
> (that exact spelling is even accepted — the `-v2`/`-random` tokens are ignored).
> `navlaunch` itself stays sim-agnostic: it launches only the ROS side and cannot
> start Isaac, so the whole-loop launcher lives here instead.

The three panes are: **[1]** Isaac + obstacles, **[2]** `navlaunch -v2 …`, **[3]**
`nav_mission.py`, which beats the Nav2 lifecycle-bringup race and leaves the stack
ACTIVE (`--loop 0`) or auto-drives (`--loop N`). Detach with `Ctrl-b d`; tear down
with `tmux kill-session -t warehouse_lab`.

## Clean exits

`navlaunch` now runs `ros2 launch` in its own process group and traps `Ctrl-C`,
`SIGTERM`, and terminal hangups, so stopping the helper should stop the Nav2
container, lifecycle managers, slam/localizer nodes, RViz, and visual-SLAM viewers
that it spawned.

For the whole randomized lab, use tmux as the owner of the run:

```bash
navrandom -lidar
tmux kill-session -t warehouse_lab
```

`warehouse_lab.sh` also starts each pane payload in its own process group, so
killing the tmux session sends a clean teardown through Isaac, `navlaunch`, and
`nav_mission.py`. Before starting a new run, `navrandom`/`warehouse_lab.sh` clean
stale project nav processes by default; pass `--no-clean` only when you
intentionally want another nav stack left alive on the same ROS graph.

If you ever suspect something survived, check with:

```bash
ps -eo pid,ppid,pgid,sid,stat,etime,cmd | rg -i \
  'navlaunch|navrandom|nav2|slam_toolbox|orbslam|ros2 launch .*nav_systems|/ugv_sim/scan'
```

## The two problems this fixes

### 1. Nav2 bringup aborts, so goals do nothing
Under Isaac + a GPU-heavy localizer the `lifecycle_manager` loses the configure
handshake and logs:

```
Failed to change state for node: controller_server
Failed to bring up all requested nodes. Aborting bringup.
```

The nodes actually finish configuring a moment later — it's a startup race, not a
real config error — but the stack is left INACTIVE, so `bt_navigator` ignores
goals. `nav_mission.py` re-issues lifecycle `STARTUP` (with retries) and waits
until `bt_navigator` is genuinely ACTIVE. To recover a stack by hand instead:

```bash
ros2 service call /lifecycle_manager_navigation/manage_nodes \
  nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"     # 0 = STARTUP
ros2 lifecycle get /bt_navigator                          # expect: active
```

### 2. DPVO / ORB-SLAM3 need to "drive around" before they localize
Both visual front-ends only broadcast `map->odom` **while tracking**, and they
only start tracking after they see motion through textured scenery (monocular
DPVO also needs translation for parallax + metric scale). `nav_mission.py`
automates this: it PAUSEs Nav2 (so nothing else touches `/cmd_vel`), drives a
safe reactive wander until `map->odom` is fresh and enough distance is covered,
then brings Nav2 up and sends the goal.

```bash
# Terminal A: Isaac
isaacpy ugv_controller_v2/nav_experiment/run_small_warehouse_random_nav.py
# Terminal B: the localizer under test
navlaunch -v2 -dpvo            # or: navlaunch -v2 -orbslam
# Terminal C: prime + goal(s)
python3 ugv_controller_v2/nav_experiment/nav_mission.py --loop 3
```

`nav_mission.py` options: `--goal X Y`, `--auto-map`, `--arena XMIN YMIN XMAX YMAX`,
`--loop N`, `--no-prime`, and priming/timeout tuning (`--prime-*`, `--slam-max-age`,
`--nav-timeout`, `--startup-retries`). `--help` lists them all. Run it from a plain
ROS 2 shell (system python), **not** the `dpvo` conda env.

## Obstacles use real warehouse assets

`random_small_warehouse_obstacles.py` now defaults to `--obstacle-style props`,
referencing real NVIDIA Simple_Warehouse / YCB assets (pallets, card boxes, KLT
bins, traffic cones) resolved against the Isaac asset root. If an asset can't be
resolved (offline Nucleus), that one obstacle falls back to a same-footprint
primitive, so a missing file never aborts the layout. Use `--obstacle-style
primitives` for a fully offline (no asset server) run.

Regenerate a baked layout explicitly with:

```bash
isaacpy ugv_controller_v2/nav_experiment/run_random_small_warehouse.py \
  --random-seed 1234 --random-obstacle-count 18 --obstacle-style props
```

## Known issue to watch: stamped /cmd_vel

`ugv_nav`'s params set `enable_stamped_cmd_vel: true`, but the baked rover's
`ROS2SubscribeTwist` graph (and the keyboard teleop) use a plain
`geometry_msgs/Twist` on `/cmd_vel`. If Nav2 accepts a goal and plans, but the
rover does **not** move, check the message type actually being published:

```bash
ros2 topic info /cmd_vel -v          # is Nav2 publishing Twist or TwistStamped?
ros2 topic echo /cmd_vel --once
```

If it's `TwistStamped` while the rover subscribes to `Twist`, either set
`enable_stamped_cmd_vel: false` in the relevant `ugv_nav/params/*.yaml`
(controller_server / velocity_smoother / collision_monitor) and rebuild/re-source
`ugv_ws`, or add a stamped→unstamped relay. Keyboard teleop working (plain Twist)
is the tell that the rover wants unstamped.
