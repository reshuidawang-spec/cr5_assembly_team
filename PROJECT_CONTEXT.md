# CR5 Path Planning Project Context

Date: 2026-07-15

## Goal

This project is the CR5 assembly team project. The user's responsibility is CR5A motion control, path planning, and tool actions. The deterministic RViz demo is now a reference implementation; the active milestone is an R1 box pick-and-place cycle in the new five-arm CoppeliaSim scene.

The current task is:

```text
Move from an initial pose
-> go to point A above a small block
-> pause above A
-> descend to simulate grasping
-> lift the block
-> move to point B along a short collision-free path
-> pause above B
-> descend to simulate placing
-> release and stop at B (optional safe retreat)
```

The next version does not require a physical robot, but it must run in the current CoppeliaSim scene rather than only RViz.

The final goal is not merely five individually successful paths. The five CR5A
robots must execute as one coordinated, smooth cell sequence. There must be no
artificial pause between adjacent operations: after R1 clears the shared
assembly zone, R2 should start useful motion immediately, and the same rule
applies to later R2-to-R3, R3-to-R1/R4, and R4-to-R5 handoffs. Measure handoff
latency in CoppeliaSim simulation time; the initial acceptance target is at
most 0.5 seconds, ideally one scheduler/control cycle.

Achieving this requires a long-lived coordinator rather than separate CLI
processes: keep one simulation running, preload/cache validated trajectories,
use per-robot state machines, command all active robots from a unified stepping
loop, and gate only shared-zone entry with resource locks. Robots may safely
pre-position in disjoint private regions while another robot owns assembly.
Smoothness and short handoff delays must not be obtained by removing collision
checks, skipping workspace walls, or exceeding validated precision speeds.

The user decided to defer detailed wall-clock optimization until R1-R5 and the
complete coordinated workflow are functionally finished. The next functional
milestone therefore remains R3 rather than an R1/R2 performance rewrite. The
final operator experience must still be immediate: with CoppeliaSim, the scene,
and a long-lived coordinator already READY, the final run command should cause
the first useful R1 joint motion within 1.0 second wall-clock time, ideally one
control cycle. Cold simulator and scene loading are a separate readiness phase,
not something that can honestly have zero latency.

## R1 Complete-Cycle Milestone (2026-07-17)

The complete R1 visual cycle is now implemented at:

```text
/home/vboxuser/桌面/workspace/robot_control/demo_r1_complete_cycle.py
```

It performs box pick/place, terminal-block pick/place, exits the shared
assembly zone, and returns R1 to the validated zero state. The user confirmed
the same complete visual workflow succeeded `10/10`.

Key validated execution values:

```text
box runtime orientation       = (180, 0, -90) deg
terminal runtime orientation  = (180, 0, -180) deg
open-space peak speed         = 50 deg/s
precision descent cap         = 24 deg/s
R1 workspace                 = x[-2.25,-0.82], y[-0.12,1.18], z[0.04,1.55]
```

All eight Git box/terminal target positions and `{0,0,0}` orientations remain
unchanged. This is still a visual-attach milestone, not physical grasp
validation. R1/R2/R3 share the assembly coordinates, so the multi-arm design
must use disjoint private supply zones plus a time-exclusive assembly-zone lock.

The next implementation work is to migrate these validated primitives, region
ownership, and failure states into the team-facing `SimBridge` and
`IRobotExecutor`, then apply the same calibration process to R2-R5.

## Formal R1 Interface Milestone (2026-07-17)

The migration is complete and actually exercised in CoppeliaSim:

```text
sim_bridge/coppelia_client.py              -> SimBridge(ISimBridge)
robot_control/robot_executor.py            -> RobotExecutor(IRobotExecutor)
robot_control/r1_motion.py                 -> validated R1 runtime
robot_control/plans/r1_complete_cycle_plan.json
robot_control/run_r1_task.py               -> thin Task/TaskResult CLI
```

Both supported invocation patterns passed:

```text
R1_COMPLETE_CYCLE -> finished, R1 home, max error 0.001642 deg
R1_BOX_PLACED -> finished at R1_TERMINAL_PICK_APP
R1_TERMINAL_PLACED -> finished from the preserved box-task state, R1 home
```

The split execution preserves the running CoppeliaSim state between tasks and
releases deterministic stepping control. Stopping the simulation would reset
the scene and destroy the inter-task handoff. Failures still stop/reset and
return `TaskResult.status=failed`.

## R2 PCB First Visual Milestone (2026-07-17)

R2 now has a workspace executor at:

```text
/home/vboxuser/桌面/workspace/robot_control/demo_r2_pcb_motion.py
```

It was executed after the real formal `R1_BOX_PLACED` task, not against an
empty fixture or a Generator template. The actual `PCB_Supply` remained inside
the actual `Box_Blank`; R1 stayed at its validated terminal-supply APP pose.

The saved scene has no R2 gripper/vacuum geometry and its generated tip is at
the Link6 origin. The visual milestone therefore uses a runtime-only 100 mm
vacuum TCP, orientation `(195,0,90)` degrees, and a 52 mm PCB visual offset.
All Git R2 APP/TCP targets remain unchanged. Full-scene validation covered 618
states; a separate 605-state R1/R2/PCB check found 204.136 mm minimum distance.

Two R2 visual executions succeeded. The first real R1-to-R2 chain completed in
37.8 seconds wall time. Final R2 joint error was at most 0.000766 degrees, PCB
position was `(-1.150058,0.200161,0.281739)`, the box did not move, and neither
PCB-to-box nor R2-to-environment collision remained. ROS2 returned
`DONE:R2_PCB_PLACED` while executor visual ownership kept templates hidden.

## Formal R2 Interface Milestone (2026-07-17)

The validated R2 motion is now self-contained in the main repository:

```text
robot_control/r2_motion.py
robot_control/run_r2_task.py
robot_control/R2_EXECUTOR.md
```

`RobotExecutor.execute_task(Task)` maps `R2_PCB_PLACED` only to R2, selects an
independently injectable R2 controller, and passes R1 and R2 the same
per-executor assembly lock. Unknown R3-R5 tasks and wrong robot assignments
still return `failed`. All 28 automated tests pass.

A freshly reloaded scene completed the formal chain:

```text
formal R1_BOX_PLACED -> finished, 287.7 s wall time
formal R2_PCB_PLACED -> finished, 294.6 s wall time
R2 maximum home error = 0.000766 deg
PCB_Supply = (-1.150058,0.200161,0.281739)
Box_Blank  = (-1.150002,0.199900,0.215915)
```

A second freshly reloaded formal demonstration also completed successfully:
R1 took 43.1 seconds, R2 took 40.2 seconds, and the final positions and R2
home error matched the first formal chain. Its independent final collision,
self-collision, payload, workspace, parent, runtime-artifact, and stepping
postflight passed. Formal R1-to-R2 chains are now 2 successful runs, not 10/10.

The long wall times came from a slow virtual-machine Coppelia instance; the
validated 50/24 deg/s motion limits were unchanged. One earlier R1 attempt was
externally terminated at 180 s and was not counted. User-visible R2 acceptance
and repeated real R1-to-R2 runs remain pending, and this is still visual
suction rather than physical grasp validation.

## 2026-07-18 Scene Deferrals And Formal R4 Milestone

The user's latest decision overrides the older R2 repetition wording above:

- count the currently accepted formal `R1_BOX_PLACED -> R2_PCB_PLACED` evidence
  as one successful chain;
- pause R2 user-visible acceptance and 10/10 repetition until the scene team
  fixes the existing `PCB_Supply_Area`/`R2_Base` overlap;
- do not move either object or weaken R2 collision checks in this repository.

R3 module diagnostics found that the current template placement intersects the
real PCB board/main chip. A scene-team suggestion is to move the module center
from approximately `(-1.12,0.22,0.3015)` to approximately
`(-1.105,0.185,0.3035)` and update the template plus R3 APP/TCP XY together.
No protected R3 target has been modified here.

R4 visual screw control is now formally implemented:

```text
robot_control/r4_motion.py
robot_control/run_r4_task.py
robot_control/R4_EXECUTOR.md
R4_SCREW_DONE -> R4
```

The user approved a runtime-only 100 mm screwdriver and vertical
`(180,0,-135)` degree orientation. All Git R4 APP/TCP/PRESS values remain
unchanged. After two observed formal successes, the user confirmed the visual
pose, speed, down-press, and two-turn rotation. A reproducible runner completed
the remaining eight clean-scene runs, bringing formal visual acceptance to
`10/10`. The ten-run mean wall time was approximately `22.011 s`, maximum R4
home error was `0.000632 deg`, and every recorded postflight released stepping,
removed all runtime objects, preserved the inspection product, and passed
environment/self-collision checks. Structured evidence is in
`data/logs/r4_repeat_acceptance_2026-07-18.json`. This is not physical torque
validation. The complete automated suite has 38 passing tests, including the
read-only new-scene audit baseline and target/height comparison tests.

R5 defect sorting has a 705-state collision-free static candidate using a
runtime 100 mm TCP, `(195,-45,0)` degrees, and a private transfer waypoint.
At that diagnostic milestone, formal R5 remained unimplemented because both
good-sort pickup candidates made
the carried product hit R5 Link2, and both conveyor targets have a 26 mm
product-height mismatch. Do not claim R5 acceptance.

## 2026-07-19 Current-Scene Five-Arm Coordination Milestone

The team deadline changed the immediate priority: basic five-arm coordination
must work in the existing scene before layout fixes arrive. No scene object,
Git HOME/APP/TCP/PRESS target, or `.ttt` content was changed.

The formal executor now also supports:

```text
R3_MODULE_PLACED          -> R3
R3_PRODUCT_TO_INSPECTION -> R3
R5_SORT_GOOD_DONE        -> R5
R5_SORT_DEFECT_DONE      -> R5
```

`robot_control/run_five_arm_cycle.py` uses one long-lived
`SimBridge/RobotExecutor` and executes the seven-stage fixed process without
stopping, reloading, or reconnecting between tasks. A clean-scene good cycle
finished all seven Tasks in about `319.515 s` wall time. Normal robot-to-robot
handoffs recorded `0.0 s` simulation-time delay; the camera state transition
before R4 used one `0.05 s` simulation step. Structured evidence is in
`data/logs/five_arm_good_cycle_2026-07-19.json`.

A second clean-scene defect cycle also finished all seven Tasks in about
`296.252 s`. Its evidence is in
`data/logs/five_arm_defect_cycle_2026-07-19.json`; the defect branch retained
carried-product motion through release and left all robots home with no Runtime
objects.

Independent postflight found all five robots home, no Runtime objects, the
simulation still advancing, and no final robot environment/self collision.
Maximum home errors were approximately R1 `0.001642 deg`, R2 `0.000766 deg`,
R3 `0.001519 deg`, R4 `0.000632 deg`, and R5 `0.008758 deg`. The complete
automated suite now has 45 passing tests.

Current-scene visual adaptations remain explicit:

- the R1 Robotiq stock script is frozen after each short animation because its
  original open mode has no position limit and drifts during long coordination;
- coordinated terminal placement uses a `56 mm` visual Z offset to clear the
  installed PCB while standalone R1 retains the accepted `28 mm` offset;
- R3 module uses `46 mm` visual Z offset; full product transfer uses a front-side
  path and a temporary `100 mm` visual lift;
- both R5 branches correct the existing `26 mm` belt-height mismatch by moving
  the runtime TCP and attached product together, then releasing at the belt;
- the old R5 good `(150,45,45)` branch still collides with Link4 and is retired.
  The replacement reuses the defect pickup/lift, pre-turns joint1 by `-123 deg`,
  and follows a transformed high path to good APP/TCP with a fixed product-to-TCP
  transform through release.

These are visual coordination capabilities, not physical suction, physical-load,
or torque validation. The scene-team fixes remain desirable for the later
physical/integrated version, but they no longer block the basic five-arm demo.

The revised R5 good branch passed a standalone run in about `59.05 s`; its
maximum product-to-TCP transform error was `2.22e-16`. A clean full good cycle
at `60 deg/s` with `0.4 s` APP holds finished 7/7 Tasks in `237.246 s`; R5 used
`48.295 s` with transform error `1.67e-16`. Its structured evidence is
`data/logs/five_arm_good_rigid_60_2026-07-19.json`. Independent postflight found all
five robots home, no Runtime objects, simulation time advancing, and no final
robot environment/self collision. The final good product yaw is not yet
parallel to its conveyor and remains the first pending R5 task. The user
deferred the corresponding full defect `60 deg/s` regression.

## Repository And Workspaces

Main repository:

```bash
/home/vboxuser/桌面/cr5_assembly_team
```

Authoritative team repository:

```text
https://github.com/reshuidawang-spec/cr5_assembly_team
```

Personal fork:

```text
https://github.com/qixunqiwo/cr5_assembly_team
```

Related local experiment workspace currently visible:

```bash
/home/vboxuser/桌面/workspace
```

Authoritative local task plan for the CoppeliaSim migration:

```bash
/home/vboxuser/桌面/workspace/TASK_PLAN.md
```

Desktop historical work summary:

```bash
/home/vboxuser/桌面/2026-07-13-工作总结.md
```

Important project files:

```bash
/home/vboxuser/桌面/cr5_assembly_team/docs/CR5_SESSION_MEMORY_2026-07-15.md
/home/vboxuser/桌面/cr5_assembly_team/robot_control/pick_and_place.py
/home/vboxuser/桌面/cr5_assembly_team/robot_control/PICK_AND_PLACE_RVIZ.md
/home/vboxuser/桌面/cr5_assembly_team/src/DOBOT_6Axis_ROS2_V4/cr5_moveit/config/
```

## User Preferences And Constraints

- Use Chinese for normal communication.
- Do not blindly trust prior AI work or summaries.
- Inspect the current files and git state before making changes.
- Do not revert existing changes unless explicitly requested.
- Prefer reliable existing robotics tooling over writing new planners from scratch.
- The user cares about visible RViz behavior, not only theoretical planning success.
- Avoid strange grasping poses.
- Avoid large loops from A to B.
- Avoid repeated movement to A before grasping.
- Avoid slow or stuttery movement.
- The top/front end-effector part should pause above A and B for a short time.
- Multiple A/B tests are useful to judge whether the path behavior is robust.
- Git-defined HOME/APP/TCP positions and orientations must not be changed without explicit user confirmation. First report the failed IK/collision/alignment evidence, the likely cause, the proposed before/after values, and the expected impact.
- Latest user decision: the present milestone is visual motion representation,
  so an air grasp and visually flying payload are acceptable. Continue to
  enforce arm/gripper-to-cell collision checks, and keep physical grasp claims
  explicitly separate from the visual demo.

## Historical Attempts

The desktop work summary records several earlier attempts:

1. CoppeliaSim plus a self-written RRT* planner was attempted, but URDF FK did not match the scene well enough, so collision checks based on that FK were unreliable.
2. PyBullet plus self-written Informed RRT* was attempted, but collision checking, IK behavior, and execution smoothness became fragile.
3. ROS2 plus MoveIt2 plus OMPL was then used because MoveIt/FCL collision checking and OMPL planning are more mature for this repo.

Treat that file as historical context, not as ground truth. Some statements in it are older than the current `robot_control/pick_and_place.py` and `robot_control/PICK_AND_PLACE_RVIZ.md`.

## Five-CR5A Scene Update Audit

The local repository was fast-forwarded to team commit `74ff605` on 2026-07-15. The update adds:

```text
docs/Five_CR5A_Cell_Control_Interface.md
docs/SCENE_BUILDING_GUIDE.md
docs/SCENE_OBJECTS_REFERENCE.md
scenes/main_cell_generator.lua
scenes/ros2_all_robot_bridge.lua
```

Important conclusions:

- The new contract uses `/FiveCR5A_Cell` and `/R1` through `/R5`, not the legacy four-arm `/CompactCell` paths.
- Upstream still contains only the legacy `scenes/compact_cell.ttt`, but a local validated `scenes/five_cr5a_cell.ttt` has now been built from the embedded CR5 model.
- `ros2_all_robot_bridge.lua` forwards string process commands and status only. It does not command joints or execute paths.
- All generated tip offsets and target orientations are placeholders. The XY reach check is not IK or collision validation.
- Main-project configs, `scene_objects.py`, `coppelia_client.py`, and `robot_control` are still based on the old four-arm scene or Mock implementations.
- Workspace scene-control experiments are historical references and must be recalibrated before reuse.

Local five-arm scene verification completed on 2026-07-15:

- R1-R5 each contain six joints and exactly one correctly named generated tip;
- `/FiveCR5A_Cell` contains 208 objects and the Targets subtree contains 42 Dummies;
- `Main_Cell_Generator` and `ROS2_All_Robot_Bridge` are attached once each;
- the scene was saved, closed, reopened, and started again without duplicating objects;
- ZMQ port 23000 is available and all `/compact_cell/*` topics are visible;
- publishing `RESET_CELL` produced `DONE:RESET_CELL` on `/compact_cell/status`;
- the authoritative object snapshot is `workspace/five_cr5a_scene_snapshot.json`.

The personal fork tracking branch `origin/main` was refreshed after the merge and
also points to `74ff605`. The local uncommitted MoveIt/RViz work was not pushed.

The team repository has no standalone GitHub Issues assigning new work. Its only
API-visible issue entries are merged, unassigned PR #1 and PR #2. Responsibility
therefore follows README and the planning documents: member 3 owns
`robot_control/` and `IRobotExecutor`, and shares `sim_bridge/` integration with
member 2. Old documents still assign sorting to R4; the new five-arm scene moves
screw fastening to R4 and sorting to R5, so motion implementation follows the
new R1-R5 scene contract while the stale docs remain an explicit cleanup task.

The next work must follow `workspace/TASK_PLAN.md`: calibrate R1 joints, HOME and TCP, then validate the Git-defined target positions against real IK, orientation, object alignment, and collision geometry before implementing and repeat-testing the R1 box cycle.

## Current Implementation State

Current key script:

```bash
/home/vboxuser/桌面/cr5_assembly_team/robot_control/pick_and_place.py
```

Current run guide:

```bash
/home/vboxuser/桌面/cr5_assembly_team/robot_control/PICK_AND_PLACE_RVIZ.md
```

The current script implements an RViz/MoveIt2 pick-and-place demo using:

- MoveIt2 services:
  - `/plan_kinematic_path`
  - `/compute_cartesian_path`
  - `/compute_fk`
  - `/compute_ik` (legacy sampled-approach compatibility mode)
  - `/check_state_validity`
  - `/apply_planning_scene`
- FollowJointTrajectory action:
  - `/cr5_group_controller/follow_joint_trajectory`
- Planning frame:
  - `dummy_link`
- End-effector link:
  - `gripper_base`
- Move group:
  - `cr5_group`

Default target points are expressed as `gripper_base` positions in `dummy_link`:

```text
A = (0.40, -0.25, 0.50)
B = (0.35,  0.30, 0.50)
```

The current default planning strategy is:

- Keep one fixed downward end-effector orientation for the complete pick/place chain.
- If the current orientation is already suitable, start directly from the current state.
- On a cold start, use OMPL once to reach the fixed `initial_home` joint state.
- Use Cartesian paths for HOME/current-to-A, descent, lift plus A-to-B transfer, and B descent.
- Plan lift and A-to-B transfer together in one two-waypoint Cartesian request.
- Do not sample random A-pose IK branches in the default mode; the old behavior requires `--sampled-approach`.
- Stop at B after release by default; retreat requires `--retreat-after-place`.
- Smooth, resample, and fill velocities before execution to reduce RViz stutter.
- Recheck every prepared controller point for collision after smoothing.
- Include the carried block as an attached MoveIt collision object.
- Show the small block as a marker and simulate open/close gripper behavior.

Preferred sequence:

```text
initial_home
-> approach_A
-> hold above A
-> descend_A
-> close gripper at A
-> lift_A_and_transfer_A_to_B
-> hold above B
-> descend_B
-> open gripper at B
-> stop at B
```

## Current Run Commands

Start RViz/MoveIt:

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ros2 launch cr5_moveit demo.launch.py
```

Run the default demo:

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

Run planning only:

```bash
python3 ./robot_control/pick_and_place.py \
  --plan-only \
  --plan-time 3 \
  --attempts 2 \
  --hover-z 0.62
```

Faster demo:

```bash
python3 ./robot_control/pick_and_place.py \
  --plan-time 3 \
  --attempts 2 \
  --hover-z 0.62 \
  --joint-speed 0.80 \
  --min-point-time 0.04 \
  --max-joint-step 0.10
```

Legacy sampled-approach comparison only:

```bash
python3 ./robot_control/pick_and_place.py \
  --plan-time 3 \
  --attempts 2 \
  --hover-z 0.62 \
  --sampled-approach \
  --entry-planners RRTConnect,RRTstar \
  --entry-samples 3 \
  --optimize-approach
```

## Verification Already Known

The following checks were previously reported as successful:

```bash
python3 -m py_compile robot_control/pick_and_place.py
```

URDF check:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
xacro src/DOBOT_6Axis_ROS2_V4/cr5_moveit/config/cr5_robot.urdf.xacro >/tmp/cr5_robot_check.urdf
check_urdf /tmp/cr5_robot_check.urdf
```

Build:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cr5_moveit cra_description
```

There may be a colcon warning because `cra_description` also exists in an underlay workspace such as `/home/vboxuser/dobot_ws/install`. Source this repository overlay carefully.

## Current Git State To Be Aware Of

As of this context file, the repository has uncommitted changes. Key untracked deliverables include:

```bash
robot_control/pick_and_place.py
robot_control/PICK_AND_PLACE_RVIZ.md
configs/obstacles.yaml
scenes/five_cr5a_cell.ttt
src/DOBOT_6Axis_ROS2_V4/cra_description/urdf/gripper.xacro
```

Several MoveIt configuration files are modified under:

```bash
src/DOBOT_6Axis_ROS2_V4/cr5_moveit/config/
```

Do not reset or discard these changes without explicit user approval.

## Known Issues And Next Steps

### Current R1 CoppeliaSim calibration

The saved five-arm scene now contains CoppeliaSim's complete `ROBOTIQ 85`
model at `/R1/R1_ROBOTIQ85`. Its grasp-center Dummy is
`/R1/R1_ROBOTIQ85/R1_gripper_tip`, with Link6-relative pose:

```text
position = (0.0, -0.01468, 0.146) m
quaternion = (0, 0, 0, 1)
```

R1 arm discovery must select aliases `joint1..joint6`; the 24 Robotiq joints
are tool joints and must not enter the six-axis IK vector.

The protected Git box target coordinates and orientations have not been
changed. Validation with the real gripper found:

- Git `ori={0,0,0}` causes Link3/Link5 collisions at the APP/TCP poses;
- temporary fixed orientation `(roll,pitch,yaw)=(180,0,-90)` degrees gives
  collision-free endpoints and shortest straight Cartesian segments at the
  existing coordinates, with 0.143-0.631 mm endpoint error;
- the Robotiq fingers remain 36.695 mm away from `Box_Blank` at the center
  PICK_TCP, so attaching there would be a non-physical floating grasp;
- after rigidly attaching all 11 box shapes, vertical lift and direct APP to
  APP transfer are collision-free;
- place descent first collides at 77.5%, TCP `z ~= 0.35625 m`, between
  `Box_Blank_Bottom` and `Assembly_Fixture`;
- the supply shell bottom is at `z=0.156 m`, while the assembly fixture expects
  it at `z=0.216 m`. Equal pick/place TCP heights cannot preserve a rigid grasp.

Evidence is stored in:

```text
/home/vboxuser/桌面/workspace/r1_target_ik_evaluation_robotiq85.json
/home/vboxuser/桌面/workspace/r1_ik_collision_validation_robotiq85_active_stage.json
/home/vboxuser/桌面/workspace/r1_fixed_orientation_validation.json
```

The user subsequently decided not to adjust those targets during the visual
motion phase. The strict physical findings remain valid evidence, but they no
longer block visual R1 and terminal-block motion. The current demo is allowed
to use a floating attach and a 60 mm visual payload offset. It must not be
described as physical grasp or payload-collision acceptance.

### Current R1 visual CoppeliaSim demo

The current visual executor is:

```text
/home/vboxuser/桌面/workspace/robot_control/demo_r1_box_motion.py
```

Normal replay command:

```bash
cd /home/vboxuser/桌面/workspace
python3 robot_control/demo_r1_box_motion.py \
  --speed-deg-s 16 \
  --hold-seconds 2.0
```

It uses the original Git box coordinates, a runtime-only fixed orientation of
`(180,0,-90)` degrees, deterministic validated endpoint replay, Robotiq open/
close animation, visual attach/detach, and per-step arm/gripper environment
collision checks. Only gripper-to-attached-box contact is allowed.

A complete visible run succeeded with:

```text
R1 TCP       = (-1.15030, 0.20041, 0.30009)
Box_Blank    = (-1.15030, 0.20028, 0.21597)
max q error  = 0.0035 deg
```

Three earlier aborted runs identified and fixed these state-machine issues:

- attached gripper-to-box contact was incorrectly rejected;
- detach happened before the gripper was sufficiently open;
- a stale visible `assembly_shell` template duplicated the carried box.

The successful run requested pause at the end and left the simulator stuck in
the transition to pause; subsequent API calls timed out although port 23000
still listened. The script was then patched to end with
`client.setStepping(False)`, but this exact patch has not yet been rerun. The
stuck process was subsequently killed and the saved scene was reopened. The
current instance is stopped and API-responsive, with R1 at six-axis zero and
`Box_Blank` restored to `(-1.8,0.35,0.156)` under `/Parts`. The next session can
directly test the patched replay after rechecking those values.

The complete chronological handoff, including all user decisions and failure
evidence, is in `docs/CR5_SESSION_MEMORY_2026-07-15.md`.

If the user still reports stutter:

- First distinguish actual executed robot motion from RViz planned-path visualization.
- Try faster execution:

  ```bash
  --joint-speed 0.80 --min-point-time 0.04 --max-joint-step 0.10
  ```

- Try disabling smoothing only as a comparison:

  ```bash
  --no-smooth-execution
  ```

- Consider merging more execution segments if pauses happen between action goals.

If the user reports A-to-B detours or wrist rotation at A:

- Confirm the default mode is being used without `--sampled-approach` or `--rrt-full`.
- Use `--plan-only` and require every Cartesian segment to report `fraction=1.000`.
- The default mode must not log random `approach_A candidate` entries.

If a command fails with `No such file or directory`:

- Confirm the user is in:

  ```bash
  /home/vboxuser/桌面/cr5_assembly_team
  ```

- Use `桌面`, not `Desktop`.
- Run scripts with relative paths from the repository root:

  ```bash
  python3 ./robot_control/pick_and_place.py
  ```

## How To Resume This Project In A New Codex Session

At the start of a new session, the user should say:

```text
请先阅读 /home/vboxuser/桌面/cr5_assembly_team/AGENTS.md、
/home/vboxuser/桌面/cr5_assembly_team/PROJECT_CONTEXT.md、
/home/vboxuser/桌面/cr5_assembly_team/robot_control/PICK_AND_PLACE_RVIZ.md，
以及 /home/vboxuser/桌面/cr5_assembly_team/docs/CR5_SESSION_MEMORY_2026-07-15.md，
然后检查 git status 和 robot_control/pick_and_place.py，继续 CR5 路径规划工作。
```

After reading those files, continue from the current repository state rather than from memory alone.
