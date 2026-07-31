from __future__ import annotations

import unittest
from unittest import mock

from robot_control.coordinated_front_half import (
    CoordinatedFrontHalfRunner,
    DESCENT_SPEED_DEG_S,
    R1_BOX_RETREAT_SPEED_MULTIPLIER,
    R1_TERMINAL_RETURN_SPEED_MULTIPLIER,
    R3_PRODUCT_PICK_APP_SPEED_MULTIPLIER,
    R5_PREAPPROACH_START_DELAY_S,
    R5_WAIT_POINT,
)


class ReparentingBridge:
    def __init__(self):
        self.attached = False
        self.attach_calls: list[tuple[str, str]] = []

    def get_object_handle(self, object_name):
        if self.attached:
            raise RuntimeError(f"{object_name} original path no longer resolves")
        return 42

    def attach_object(self, object_name, robot_id):
        self.attach_calls.append((object_name, robot_id))
        self.attached = True


class CoordinatedFrontHalfRunnerTests(unittest.TestCase):
    def test_attach_keeps_handle_before_reparenting_part(self):
        runner = CoordinatedFrontHalfRunner.__new__(CoordinatedFrontHalfRunner)
        runner.bridge = ReparentingBridge()
        runner.payloads = {"R1": None}
        runner.step = lambda label, force_collision=False: None

        handle = runner._attach("BOX_BLANK", "R1")

        self.assertEqual(handle, 42)
        self.assertEqual(runner.payloads["R1"], 42)
        self.assertEqual(runner.bridge.attach_calls, [("BOX_BLANK", "R1")])

    def test_parallel_targets_are_sent_with_one_script_call(self):
        class FakeSim:
            def __init__(self):
                self.calls = []

            def callScriptFunction(self, *args):
                self.calls.append(args)

        runner = CoordinatedFrontHalfRunner.__new__(CoordinatedFrontHalfRunner)
        runner.joints = {
            "R1": [11, 12],
            "R2": [21, 22],
        }
        runner.command_script = 99
        runner.bridge = type("FakeBridge", (), {"sim": FakeSim()})()

        runner._set_parallel_targets(
            {
                "R1": [0.1, 0.2],
                "R2": [0.3, 0.4],
            }
        )

        self.assertEqual(
            runner.sim.calls,
            [
                (
                    "setJointTargets",
                    99,
                    [11, 12, 21, 22],
                    [0.1, 0.2, 0.3, 0.4],
                )
            ],
        )

    def test_single_robot_targets_use_the_command_script(self):
        class FakeSim:
            def __init__(self):
                self.script_calls = []
                self.direct_calls = []

            def callScriptFunction(self, *args):
                self.script_calls.append(args)

            def setJointTargetPosition(self, *args):
                self.direct_calls.append(args)

        runner = CoordinatedFrontHalfRunner.__new__(CoordinatedFrontHalfRunner)
        runner.joints = {"R1": [11, 12]}
        runner.command_script = 99
        runner.bridge = type("FakeBridge", (), {"sim": FakeSim()})()

        runner._set_targets("R1", [0.1, 0.2])

        self.assertEqual(
            runner.sim.script_calls,
            [("setJointTargets", 99, [11, 12], [0.1, 0.2])],
        )
        self.assertEqual(runner.sim.direct_calls, [])

    def test_collision_check_reuses_cached_collections(self):
        class FakeSim:
            def __init__(self):
                self.check_calls = []
                self.destroy_calls = []

            def checkCollision(self, first, second):
                self.check_calls.append((first, second))
                return 0, []

            def destroyCollection(self, collection):
                self.destroy_calls.append(collection)

        runner = CoordinatedFrontHalfRunner.__new__(CoordinatedFrontHalfRunner)
        runner.bridge = type("FakeBridge", (), {"sim": FakeSim()})()
        runner.collision_collections = {
            "R1": 101,
            "R2": 102,
            "R3": 103,
            "R4": 104,
            "R5": 105,
        }

        runner._check_inter_robot_collisions("probe")

        self.assertEqual(
            runner.sim.check_calls,
            [
                (101, 102),
                (101, 103),
                (101, 104),
                (101, 105),
                (102, 103),
                (102, 104),
                (102, 105),
                (103, 104),
                (103, 105),
                (104, 105),
            ],
        )
        self.assertEqual(runner.sim.destroy_calls, [])

    def test_startup_adopts_running_ready_simulation_without_resetting_joints(self):
        class RunningSim:
            simulation_stopped = 0
            jointfloatparam_maxvel = 301

            def __init__(self):
                self.joint_position_calls = []
                self.joint_target_calls = []
                self.float_param_calls = []

            def getSimulationState(self):
                return 1

            def getObjectFloatParam(self, joint, param):
                return 0.0

            def setObjectFloatParam(self, joint, param, value):
                self.float_param_calls.append((joint, param, value))

            def setJointPosition(self, joint, value):
                self.joint_position_calls.append((joint, value))

            def setJointTargetPosition(self, joint, value):
                self.joint_target_calls.append((joint, value))

            def getObject(self, name):
                return 900

        class RunningBridge:
            host = "127.0.0.1"
            port = 23000

            def __init__(self):
                self.sim = RunningSim()
                self.stop_calls = 0
                self.start_calls = 0

            def is_connected(self):
                return True

            def connect(self, host, port):
                return True

            def stop_simulation(self):
                self.stop_calls += 1
                return True

            def start_simulation(self):
                self.start_calls += 1
                return True

            def get_robot_joint_handles(self, robot_id):
                return [f"{robot_id}_J{index}" for index in range(6)]

        bridge = RunningBridge()
        runner = CoordinatedFrontHalfRunner.__new__(CoordinatedFrontHalfRunner)
        runner.bridge = bridge
        runner.speed = 0.8
        runner.r2_initial_speed = 1.92
        runner.r2_transfer_speed = 1.28
        runner.r1_box_retreat_speed = 1.28
        runner.joints = {}
        runner.original_max_velocities = {}
        runner.command_script = -1
        runner.collision_collections = {}
        runner.hold = lambda seconds, label: None
        runner._set_gripper = lambda robot_id, opened: None
        runner._create_inter_robot_collections = lambda: None

        with mock.patch(
            "robot_control.coordinated_front_half.create_command_script",
            return_value=901,
        ):
            runner._connect_and_start()

        self.assertEqual(bridge.stop_calls, 0)
        self.assertEqual(bridge.start_calls, 1)
        self.assertEqual(bridge.sim.joint_position_calls, [])
        self.assertEqual(bridge.sim.joint_target_calls, [])

    def test_r1_box_transfer_overlaps_r2_departure_to_safe_wait(self):
        class FakeSim:
            def getSimulationTime(self):
                return 12.5

        class FakeBridge:
            sim = FakeSim()

        class FakeR3ProductController:
            def __init__(self, *args, **kwargs):
                pass

            def set_continuous_stepping(self, enabled):
                pass

            def set_pre_positioned(self, action, config):
                pass

            def execute(self, action):
                return {"action": action}

        def config(value):
            return [float(value), 0.0, 0.0, 0.0, 0.0, 0.0]

        def path(first, second):
            return [config(first), config(second)]

        mid = config(10.5)
        paths = {
            "R1": {
                "initial_to_box_pick_app": path(0.0, 1.0),
                "box_descend": path(1.0, 2.0),
                "box_lift_and_transfer": path(2.0, 3.0),
                "box_place_descend": path(3.0, 4.0),
                "box_retreat_and_terminal_approach": [config(4.0), mid, config(5.0)],
                "terminal_descend": path(5.0, 6.0),
                "terminal_lift_and_transfer": [config(6.0), mid, config(7.0)],
                "terminal_place_descend": path(7.0, 8.0),
                "return_home": path(8.0, 0.0),
            },
            "R2": {
                "initial_to_pick_app": path(0.0, 1.0),
                "pick_descend": path(1.0, 2.0),
                "pick_tcp_to_safe_wait": path(2.0, 3.0),
                "safe_wait_to_place_app": path(3.0, 4.0),
                "place_descend": path(4.0, 5.0),
                "return_home": path(5.0, 1.0),
            },
            "R3_MODULE": {
                "initial_to_pick_app": path(0.0, 1.0),
                "pick_descend": path(1.0, 2.0),
                "lift_and_transfer": [config(2.0), config(1.0), config(3.0)],
                "place_descend": path(3.0, 4.0),
                "retreat_to_clear": path(4.0, 5.0),
            },
            "R3_PRODUCT": {"clear_to_pick_app": path(5.0, 9.0)},
            "R4": {"home_to_wait": path(0.0, 1.0)},
            "R5_GOOD": {"home_to_wait": path(0.0, 1.0)},
        }

        runner = CoordinatedFrontHalfRunner.__new__(CoordinatedFrontHalfRunner)
        runner.bridge = FakeBridge()
        runner.speed = 1.0
        runner.r2_initial_speed = 2.4
        runner.r2_transfer_speed = 1.6
        runner.r1_box_retreat_speed = 1.6
        runner.r1_terminal_return_speed = 2.2
        runner.r3_product_pick_app_speed = 0.45
        runner.descent_speed = 0.5
        runner.hold_seconds = 0.0
        runner.r1_plan = {
            "validation": {"r1_return_avoidance": {"mid1_rad": mid}}
        }
        runner._paths = lambda: paths
        runner._connect_and_start = lambda: None
        runner._restore = lambda: None
        runner.hold = lambda seconds, label: None
        runner._set_gripper = lambda robot_id, opened: None
        runner._attach = lambda object_name, robot_id: {
            "BOX_BLANK": 101,
            "PCB_SUPPLY": 202,
            "CONTROL_MODULE_SUPPLY": 303,
            "TERMINAL_BLOCK_SUPPLY": 404,
        }[object_name]

        detached: list[tuple[int, str]] = []
        runner._detach_to_parts = lambda handle, robot_id, position: detached.append(
            (handle, robot_id)
        )
        runner._align_payload_above_standard = (
            lambda robot_id, handle, position, label: list(position)
        )
        batches: list[list[tuple[str, float]]] = []
        runner.run_parallel = lambda motions: batches.append(
            [(motion.label, motion.peak_speed_rad_s) for motion in motions]
        )

        with mock.patch(
            "robot_control.coordinated_front_half.R3MotionController",
            FakeR3ProductController,
        ):
            result = runner.execute()

        self.assertEqual(result["status"], "finished")
        safe_wait_batches = [
            batch
            for batch in batches
            if "R2 pick_tcp_to_safe_wait" in [label for label, _ in batch]
        ]
        self.assertEqual(len(safe_wait_batches), 1)
        self.assertIn(
            "R1 box_lift_and_transfer",
            [label for label, _ in safe_wait_batches[0]],
        )
        retreat_batches = [
            batch
            for batch in batches
            if "R1 box_retreat_to_mid1" in [label for label, _ in batch]
        ]
        self.assertEqual(len(retreat_batches), 1)
        retreat_speeds = dict(retreat_batches[0])
        self.assertEqual(retreat_speeds["R1 box_retreat_to_mid1"], 1.6)
        product_pick_batches = [
            batch
            for batch in batches
            if "R3 clear_to_product_pick_app" in [label for label, _ in batch]
        ]
        self.assertEqual(len(product_pick_batches), 1)
        self.assertIn(
            "R1 return_home",
            [label for label, _ in product_pick_batches[0]],
        )
        product_pick_speeds = dict(product_pick_batches[0])
        self.assertEqual(product_pick_speeds["R1 return_home"], 2.2)
        self.assertEqual(product_pick_speeds["R3 clear_to_product_pick_app"], 0.45)
        self.assertEqual(R1_BOX_RETREAT_SPEED_MULTIPLIER, 1.6)
        self.assertEqual(R1_TERMINAL_RETURN_SPEED_MULTIPLIER, 2.2)
        self.assertEqual(R3_PRODUCT_PICK_APP_SPEED_MULTIPLIER, 0.45)
        self.assertEqual(DESCENT_SPEED_DEG_S, 36.0)
        self.assertIn(
            "R5 home_to_wait",
            [label for batch in batches for label, _ in batch],
        )
        self.assertIn("R5 home_to_wait", [label for label, _ in batches[0]])
        self.assertIn("R5", result["prepositioned_configs"])
        self.assertEqual(result["prepositioned_configs"]["R5"]["point"], R5_WAIT_POINT)
        self.assertEqual(
            result["prepositioned_configs"]["R5"]["config"],
            paths["R5_GOOD"]["home_to_wait"][-1],
        )
        self.assertEqual(
            result["r5_preapproach_start_delay_s"],
            R5_PREAPPROACH_START_DELAY_S,
        )
        self.assertFalse(result["r5_preapproach_skipped_for_r3_clearance"])

    def test_pcb_is_released_before_r1_terminal_grip_wait(self):
        class FakeSim:
            def getSimulationTime(self):
                return 12.5

        class FakeBridge:
            sim = FakeSim()

        class FakeR3ProductController:
            def __init__(self, *args, **kwargs):
                pass

            def set_continuous_stepping(self, enabled):
                pass

            def set_pre_positioned(self, action, config):
                pass

            def execute(self, action):
                return {"action": action}

        def config(value):
            return [float(value), 0.0, 0.0, 0.0, 0.0, 0.0]

        def path(first, second):
            return [config(first), config(second)]

        mid = config(10.5)
        paths = {
            "R1": {
                "initial_to_box_pick_app": path(0.0, 1.0),
                "box_descend": path(1.0, 2.0),
                "box_lift_and_transfer": path(2.0, 3.0),
                "box_place_descend": path(3.0, 4.0),
                "box_retreat_and_terminal_approach": [config(4.0), mid, config(5.0)],
                "terminal_descend": path(5.0, 6.0),
                "terminal_lift_and_transfer": [config(6.0), mid, config(7.0)],
                "terminal_place_descend": path(7.0, 8.0),
                "return_home": path(8.0, 0.0),
            },
            "R2": {
                "initial_to_pick_app": path(0.0, 1.0),
                "pick_descend": path(1.0, 2.0),
                "pick_tcp_to_safe_wait": path(2.0, 3.0),
                "safe_wait_to_place_app": path(3.0, 4.0),
                "place_descend": path(4.0, 5.0),
                "return_home": path(5.0, 1.0),
            },
            "R3_MODULE": {
                "initial_to_pick_app": path(0.0, 1.0),
                "pick_descend": path(1.0, 2.0),
                "lift_and_transfer": [config(2.0), config(1.0), config(3.0)],
                "place_descend": path(3.0, 4.0),
                "retreat_to_clear": path(4.0, 5.0),
            },
            "R3_PRODUCT": {"clear_to_pick_app": path(5.0, 9.0)},
            "R4": {"home_to_wait": path(0.0, 1.0)},
            "R5_GOOD": {"home_to_wait": path(0.0, 1.0)},
        }

        runner = CoordinatedFrontHalfRunner.__new__(CoordinatedFrontHalfRunner)
        runner.bridge = FakeBridge()
        runner.speed = 1.0
        runner.r2_initial_speed = 2.4
        runner.r2_transfer_speed = 1.6
        runner.r1_box_retreat_speed = 1.6
        runner.r1_terminal_return_speed = 2.2
        runner.r3_product_pick_app_speed = 0.45
        runner.descent_speed = 0.5
        runner.hold_seconds = 0.0
        runner.r1_plan = {
            "validation": {"r1_return_avoidance": {"mid1_rad": mid}}
        }
        runner._paths = lambda: paths
        runner._connect_and_start = lambda: None
        runner._restore = lambda: None
        runner._attach = lambda object_name, robot_id: {
            "BOX_BLANK": 101,
            "PCB_SUPPLY": 202,
            "CONTROL_MODULE_SUPPLY": 303,
            "TERMINAL_BLOCK_SUPPLY": 404,
        }[object_name]

        events: list[str] = []
        runner.hold = lambda seconds, label: events.append(f"hold:{label}")
        runner._set_gripper = lambda robot_id, opened: events.append(
            f"gripper:{robot_id}:{opened}"
        )
        runner._detach_to_parts = lambda handle, robot_id, position: events.append(
            f"detach:{handle}:{robot_id}"
        )
        runner._align_payload_above_standard = (
            lambda robot_id, handle, position, label: list(position)
        )
        runner.run_parallel = lambda motions: events.append(
            "batch:" + ",".join(motion.label for motion in motions)
        )

        with mock.patch(
            "robot_control.coordinated_front_half.R3MotionController",
            FakeR3ProductController,
        ):
            runner.execute()

        self.assertLess(
            events.index("detach:202:R2"),
            events.index("hold:R1 close terminal gripper"),
        )


if __name__ == "__main__":
    unittest.main()
