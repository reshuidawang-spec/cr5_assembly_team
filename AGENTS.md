# AGENTS.md

## Project Context

This repository is the CR5 assembly team project at:

```bash
/home/vboxuser/桌面/cr5_assembly_team
```

The authoritative team repository is:

```text
https://github.com/reshuidawang-spec/cr5_assembly_team
```

The personal fork is `https://github.com/qixunqiwo/cr5_assembly_team`.

The user's responsibility is CR5A motion control, path planning, and tool actions. The deterministic RViz/MoveIt baseline, real `SimBridge`, formal R1-R5 visual `RobotExecutor(IRobotExecutor)` actions, and a fixed-order five-arm coordinator are implemented. R2 repeated acceptance remains paused until the scene team fixes the `PCB_Supply_Area`/`R2_Base` overlap; R3/R5 still need layout fixes for later physical payload validation, but no longer block the basic visual coordinated demo.

## Required Working Style

- Answer the user in Chinese unless they ask otherwise.
- Do not blindly trust previous AI summaries. Read the actual files, current git state, and relevant docs before deciding.
- Do not revert existing user or previous-agent changes unless the user explicitly asks.
- Prefer mature, working robotics tooling already in this repo over rewriting planners from scratch.
- RViz/MoveIt remains the behavioral reference, but the next milestone must run in the current CoppeliaSim scene. Do not treat old `/CompactCell` results as validation of `/FiveCR5A_Cell`.
- The user dislikes strange grasp poses, large detours from A to B, repeated pre-grasp motions, slow movement, and stuttery execution.
- The final five-arm demonstration must also look coordinated as one continuous
  cell process. Adjacent tasks must not have artificial gaps from stopping the
  simulation, reconnecting CLI processes, or planning on the critical handoff.
  In particular, after R1 clears/releases the assembly zone, R2 should begin
  useful motion within one scheduler cycle. Measure this in simulation time,
  not wall-clock time, because VM/API performance varies. Use an initial
  acceptance target of at most 0.5 s simulation-time handoff delay without
  weakening collision checks or precision-motion limits.
- The user explicitly deferred detailed wall-clock optimization until all five
  robot paths and the complete coordinated workflow are functionally correct.
  Continue R3-R5 first; do not destabilize validated R1/R2 paths with premature
  performance refactors. The final user-facing run command must nevertheless
  start useful R1 motion essentially immediately when CoppeliaSim and the
  long-lived coordinator are already READY. Measure command-to-first-motion
  wall-clock latency with an initial target of at most 1.0 s, ideally one
  control cycle. Cold CoppeliaSim/scene startup is a separate readiness phase.
- Treat Git-defined HOME/APP/TCP poses as protected baselines. If validation shows unreachable poses, collision, or object misalignment, report the evidence and proposed before/after values to the user and obtain explicit confirmation before changing any target position or orientation.
- The user's latest decision is to prioritize a clear visual motion demonstration.
  For this phase, a non-physical air grasp and visual payload attachment are
  acceptable. This does not authorize arm/gripper collisions with the cell,
  and it does not count as physical grasp validation.
- Do not keep blocking the visual R1/terminal workflow on box-wall grasp or
  suction calibration. Revisit physical grasping only when the user asks for it.
- If a suggested algorithm such as RRT* or Informed RRT* is not available or not suitable in the current MoveIt/OMPL setup, explain that directly and use a more reliable approach.

## Important Files

Read these first when continuing the path-planning work:

```bash
/home/vboxuser/桌面/cr5_assembly_team/docs/CR5_SESSION_MEMORY_2026-07-15.md
/home/vboxuser/桌面/cr5_assembly_team/PROJECT_CONTEXT.md
/home/vboxuser/桌面/cr5_assembly_team/robot_control/PICK_AND_PLACE_RVIZ.md
/home/vboxuser/桌面/cr5_assembly_team/robot_control/pick_and_place.py
/home/vboxuser/桌面/cr5_assembly_team/robot_control/R1_EXECUTOR.md
/home/vboxuser/桌面/cr5_assembly_team/robot_control/R2_EXECUTOR.md
/home/vboxuser/桌面/cr5_assembly_team/robot_control/robot_executor.py
/home/vboxuser/桌面/cr5_assembly_team/robot_control/r1_motion.py
/home/vboxuser/桌面/cr5_assembly_team/robot_control/r2_motion.py
/home/vboxuser/桌面/2026-07-13-工作总结.md
/home/vboxuser/桌面/cr5_assembly_team/docs/Five_CR5A_Cell_Control_Interface.md
/home/vboxuser/桌面/cr5_assembly_team/scenes/main_cell_generator.lua
/home/vboxuser/桌面/cr5_assembly_team/scenes/ros2_all_robot_bridge.lua
/home/vboxuser/桌面/workspace/TASK_PLAN.md
/home/vboxuser/桌面/workspace/robot_control/prepare_five_cr5a_scene.py
/home/vboxuser/桌面/workspace/robot_control/audit_five_cr5a_scene.py
/home/vboxuser/桌面/workspace/robot_control/demo_r1_box_motion.py
/home/vboxuser/桌面/workspace/robot_control/demo_r2_pcb_motion.py
```

Useful related workspace:

```bash
/home/vboxuser/桌面/workspace
```

If the local workspace is missing project files, compare with the upstream GitHub repository or the material folders available on the desktop before inventing replacements.

## Current Implementation Direction

The current pick-and-place implementation uses:

- MoveIt2 and RViz simulation.
- OMPL only for reaching a fixed HOME state when the current tool orientation is unsuitable.
- A deterministic fixed-orientation Cartesian chain for approach A, descent, lift plus A-to-B transfer, and B descent.
- MoveIt/FCL collision checking.
- A visible marker for the small block.
- Trajectory smoothing, resampling, and velocity filling to reduce visible stutter.

The preferred motion sequence is:

```text
initial_home (only when needed)
-> approach_A
-> hold above A
-> descend_A
-> close gripper
-> lift_A + transfer_A_to_B
-> hold above B
-> descend_B
-> open gripper
-> stop at B
```

Vertical retreat is optional through `--retreat-after-place`. The legacy random
IK approach is diagnostic only through `--sampled-approach`; do not use it by
default because equivalent IK branches produced visible wrist rotations.

## Current CoppeliaSim Direction

- Upstream baseline: team commit `74ff605`.
- New scene root: `/FiveCR5A_Cell`; robot roots: `/R1` through `/R5`.
- The new Lua files generate scene objects and forward string status commands, but do not implement joint trajectories, IK, collision avoidance, or physical grasping.
- Upstream did not include a runnable five-arm `.ttt`; a local validated scene now exists at `scenes/five_cr5a_cell.ttt`.
- The saved scene contains R1-R5, both script Dummies, 208 generated cell objects, and 42 target-tree Dummies.
- ZMQ port 23000 and all `/compact_cell/*` topics were verified; `RESET_CELL` returned `DONE:RESET_CELL`.
- Tip offsets and target orientations in the generator are placeholders and must be calibrated in the saved scene.
- The saved scene now includes CoppeliaSim's full `ROBOTIQ 85` model under
  `/R1/R1_ROBOTIQ85`; `R1_gripper_tip` is 146 mm ahead of Link6 at the
  finger center. Tool joints must never be mixed into CR5A `joint1..joint6` IK.
- Git box targets are still unmodified. With the real gripper, the Git zero
  orientation causes arm collisions. A temporary `(180, 0, -90)` degree
  orientation gives collision-free empty-tool Cartesian paths, but the center
  TCP remains 36.695 mm from the box and cannot physically grip it.
- With the complete box rigidly attached, lift and direct transfer are clear,
  but the current place descent collides with `Assembly_Fixture` at TCP
  `z ~= 0.35625 m` under strict physical validation.
- The user chose not to change the box targets now. The visual demo keeps all
  Git coordinates unchanged, overrides the tool orientation only at execution,
  attaches the box visually, and applies a 60 mm visual payload offset.
- `workspace/robot_control/demo_r1_box_motion.py` completed the full visible R1
  box sequence. Final TCP was `(-1.15030, 0.20041, 0.30009)`, final box position
  was `(-1.15030, 0.20028, 0.21597)`, and maximum final joint error was 0.0035 deg.
- The successful run used an older end-of-demo pause request. CoppeliaSim then
  became stuck in the transition to pause and later ZMQ calls timed out. The
  script now ends with `client.setStepping(False)`, but that patch has not yet
  been rerun. The stuck process was killed and the saved scene was reopened;
  the current scene is stopped, API-responsive, at zero arm joints and the
  original box supply pose.
- `workspace/robot_control/demo_r1_complete_cycle.py` now completes box pick/place,
  terminal pick/place, assembly-zone exit, and R1 return to zero. The user
  confirmed the complete visual cycle succeeded `10/10` on 2026-07-17.
- The complete cycle uses runtime-only vertical orientations `(180,0,-90)` for
  the box and `(180,0,-180)` for the terminal, 50 deg/s for open-space motion,
  and a 24 deg/s cap for precision descent. All Git targets remain unchanged.
- R1, R2, and R3 share assembly coordinates, so permanently disjoint complete
  workspaces are impossible. Keep private supply zones disjoint and protect the
  assembly zone with a time-exclusive scheduler lock.
- The current separate `run_r1_task.py` and `run_r2_task.py` commands validate
  paths but are not the final coordination architecture. The final workflow
  needs a long-lived scheduler/executor, one continuous simulation connection,
  cached/preloaded paths, per-robot state machines, and an event-driven shared-
  zone handoff. A next robot may pre-position or pick inside its disjoint
  private zone while the current robot owns assembly, but it must not enter the
  shared zone before the lock is released.
- `SimBridge(ISimBridge)` now implements the five-arm ZMQ scene contract and
  `RobotExecutor(IRobotExecutor)` accepts standard `Task` objects for
  `R1_BOX_PLACED`, `R1_TERMINAL_PLACED`, and `R1_COMPLETE_CYCLE`.
- The complete Task and the split box/terminal Tasks were actually executed.
  Successful tasks preserve the running scene and release stepping so another
  robot can continue; failures stop/reset. Do not stop the simulation between
  the split R1 tasks because CoppeliaSim resets the scene at simulation end.
- `cell_visual_owner=executor` prevents Generator product templates from
  duplicating real attached parts while preserving `/compact_cell/status`.
- R2 must run after actual `R1_BOX_PLACED`; it installs the real `PCB_Supply`
  into the real `Box_Blank`, while R1 remains at `R1_TERMINAL_PICK_APP`.
- The first R2 visual milestone uses a runtime-only 100 mm vacuum TCP,
  `(195,0,90)` degree orientation, and 52 mm visual PCB offset. The scene has
  no physical R2 gripper/vacuum tool, so this is not physical grasp validation.
- Two R2 visual runs succeeded. Final PCB was
  `(-1.150058,0.200161,0.281739)`, R2 returned to zero within 0.000766 deg,
  R1/R2/PCB minimum distance was 204.136 mm, and ROS2 returned
  `DONE:R2_PCB_PLACED` without showing a duplicate template.
- R2 is now self-contained in `robot_control/r2_motion.py` and mapped by the
  existing `RobotExecutor.execute_task(Task)` contract. The formal
  `R1_BOX_PLACED -> R2_PCB_PLACED` chain was actually executed successfully
  twice; the second run took 43.1/40.2 seconds and its independent final
  safety postflight passed. 28 automated tests pass. User-visible acceptance
  and 10/10 repetition are still pending, so do not claim final R2 acceptance
  or physical grasping.
- `SimBridge.set_stepping` must remain idempotent because CoppeliaSim counts
  repeated stepping-enable requests; an unmatched request freezes simulation
  time even while state reports running.
- On 2026-07-18 the user approved the formal R4 visual screw runtime: a
  runtime-only 100 mm screwdriver, vertical `(180,0,-135)` degree orientation,
  and unchanged Git APP/TCP/PRESS targets. The user confirmed the visual pose,
  speed, down-press, and two-turn rotation, and `R4_SCREW_DONE -> R4` completed
  formal visual acceptance `10/10`. The ten-run mean wall time was about
  22.011 seconds, maximum home error was 0.000632 deg, and every recorded
  postflight released stepping, removed all runtime tool/script objects, and
  passed environment/self-collision checks. Structured evidence is in
  `data/logs/r4_repeat_acceptance_2026-07-18.json`. This is visual rotation/
  down-press, not physical torque validation. The complete automated suite
  has 38 passing tests, including the new-scene audit contract.
- R5 defect sorting has a static 705-state candidate, but the good branch is
  blocked because the carried product hits R5 Link2 during lift. Do not map
  either R5 action to formal success until the good branch and the 26 mm
  conveyor-height mismatch are resolved.
- 2026-07-19 priority override: the team must demonstrate basic five-arm
  coordination in the current scene before the scene team can deliver layout
  fixes. R3 module/product transfer and both R5 branches are now implemented
  with runtime-only visual adaptations; all Git targets and the `.ttt` remain
  unchanged. `robot_control/run_five_arm_cycle.py` completed one clean-scene
  good cycle with all seven formal Tasks finished in one long-lived
  `SimBridge/RobotExecutor` connection. Evidence is in
  `data/logs/five_arm_good_cycle_2026-07-19.json`. A separate clean-scene
  defect cycle also finished 7/7 Tasks in about 296.252 s; evidence is in
  `data/logs/five_arm_defect_cycle_2026-07-19.json`.
- The complete automated suite now has 45 passing tests.
- The current-scene R5 good branch remains a visual handoff, not rigid payload
  acceptance: R5 carries the product to the good APP, clears the robot with an
  empty APP/TCP placement gesture, and only then lowers the visual product to
  the belt. R5 defect retains carried-product motion. Do not remove this
  distinction from docs or results.
- Long-running coordination must freeze the stock Robotiq script after each
  0.8 s open/close animation. Its original open mode applies unbounded positive
  velocity and otherwise lets `dummyMass8/10` drift about 0.55 m during later
  robot tasks. `SimBridge.freeze_gripper()` is the runtime fix; do not persist
  the transient tool state into the scene.
- Scene-team changes are consolidated in
  `docs/SCENE_CHANGE_REQUEST_2026-07-18.md`. Before recalibrating a replacement
  `.ttt`, open it stopped and run `sim_bridge/audit_five_cr5a_scene.py` against
  `configs/five_cr5a_scene_audit_baseline.json`. The tool is read-only and must
  report target/fingerprint changes rather than silently accepting old caches.
- Do not copy provisional Lua coordinates into official configs until R1 joint, TCP, target, and collision validation is complete.
- Follow `/home/vboxuser/桌面/workspace/TASK_PLAN.md` and update it after every CoppeliaSim work session.

## Common Commands

Start RViz/MoveIt in one terminal:

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ros2 launch cr5_moveit demo.launch.py
```

Run the pick-and-place demo in another terminal:

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
source /opt/ros/humble/setup.bash
source ./install/setup.bash

python3 ./robot_control/pick_and_place.py \
  --plan-time 3 \
  --attempts 2 \
  --hover-z 0.62
```

Run with custom A/B points:

```bash
python3 ./robot_control/pick_and_place.py \
  --plan-time 3 \
  --attempts 2 \
  --hover-z 0.62 \
  --pick 0.42,-0.18,0.50 \
  --place 0.30,0.25,0.50
```

Plan only:

```bash
python3 ./robot_control/pick_and_place.py \
  --plan-only \
  --plan-time 3 \
  --attempts 2 \
  --hover-z 0.62
```

Run the complete CoppeliaSim R1 visual cycle after opening the saved scene:

```bash
cd /home/vboxuser/桌面/workspace
python3 robot_control/demo_r1_complete_cycle.py
```

Run through the formal team interface instead:

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
python3 robot_control/run_r1_task.py R1_COMPLETE_CYCLE
```

Run the formal R1 box to R2 PCB handoff without stopping the simulation:

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
python3 robot_control/run_r1_task.py R1_BOX_PLACED
python3 robot_control/run_r2_task.py R2_PCB_PLACED
```

Run the formal R4 visual screw task from a clean stopped scene:

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
python3 robot_control/run_r4_task.py R4_SCREW_DONE
```

The default is deterministic full-path replay with a scene-fingerprint cache.
Use `--replan --plan-only` only when a fresh Cartesian plan is intentionally
required. Stop and reload the saved `.ttt` before each repeated demonstration.

## Verification

For script-only changes, at minimum run:

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
python3 -m py_compile robot_control/pick_and_place.py
```

For MoveIt/URDF/xacro configuration changes, also run when possible:

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
source /opt/ros/humble/setup.bash
source ./install/setup.bash
xacro src/DOBOT_6Axis_ROS2_V4/cr5_moveit/config/cr5_robot.urdf.xacro >/tmp/cr5_robot_check.urdf
check_urdf /tmp/cr5_robot_check.urdf
```

If behavior is changed, test in RViz/MoveIt rather than relying only on static checks.

For the current Coppelia workspace tools, also run:

```bash
cd /home/vboxuser/桌面/workspace
python3 -m py_compile robot_control/demo_r1_complete_cycle.py
```

Before resuming Coppelia work, verify port 23000 actually answers API calls. A
listening port alone is insufficient when CoppeliaSim is stuck in stepping or
pause transition. The current instance has already been cleanly reopened. If
the API times out again, restart `scenes/five_cr5a_cell.ttt`; do not save the
transient demo endpoint state.
