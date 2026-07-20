from __future__ import annotations

import unittest
from pathlib import Path

from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import (
    POINTS,
    PROCESS_COMMANDS,
    ROBOT_IDS,
    ROS_TOPICS,
    WORKSPACES,
    get_joint_alias,
    get_point_path,
    resolve_object_path,
)


class FakeSim:
    handle_all = -1
    object_joint_type = 1
    object_script_type = 2
    simulation_stopped = 0
    simulation_running = 17
    stringparam_scene_path_and_name = 100
    scriptintparam_enabled = 10104

    def __init__(self):
        self.state = self.simulation_stopped
        self.signals: dict[str, str] = {}
        self.joint_positions = {handle: 0.0 for handle in range(11, 17)}
        self.joint_targets: dict[int, float] = {}
        self.parent_updates: list[tuple[int, int, bool]] = []
        self.script_calls: list[tuple] = []
        self.int_params: dict[tuple[int, int], int] = {}
        self.joint_velocities: dict[int, float] = {}
        self.paths = {
            "/FiveCR5A_Cell": 1,
            "/FiveCR5A_Cell/Parts": 2,
            "/FiveCR5A_Cell/Parts/Box_Blank": 3,
            "/FiveCR5A_Cell/Parts/Assembly_ControlBox_Product": 4,
            "/R1": 10,
            "/R1/R1_ROBOTIQ85": 20,
            POINTS["R1_BOX_PICK_APP"]: 30,
        }
        self.aliases = {
            1: "FiveCR5A_Cell",
            2: "Parts",
            3: "Box_Blank",
            4: "Assembly_ControlBox_Product",
            10: "R1",
            **{handle: f"joint{handle - 10}" for handle in range(11, 17)},
            17: "R1_gripper_tip",
            18: "ROBOTIQ_85_active1",
            20: "R1_ROBOTIQ85",
            22: "Robotiq_Script",
            23: "active1",
            24: "active2",
            30: "R1_BOX_PICK_APP",
        }

    def getSimulationState(self):
        return self.state

    def getObject(self, path):
        if path not in self.paths:
            raise RuntimeError(f"missing object: {path}")
        return self.paths[path]

    def getObjectAlias(self, handle, options=0):
        if options == 1:
            return f"/{self.aliases[handle]}"
        return self.aliases[handle]

    def getObjectsInTree(self, root, object_type, options):
        if root == 10 and object_type == self.object_joint_type:
            return list(range(11, 17)) + [18]
        if root == 10 and object_type == self.handle_all:
            return list(range(11, 19)) + [20, 22]
        if root == 20 and object_type == self.object_script_type:
            return [22]
        if root == 20 and object_type == self.object_joint_type:
            return [23, 24]
        return []

    def setJointPosition(self, handle, value):
        self.joint_positions[handle] = value

    def setJointTargetPosition(self, handle, value):
        self.joint_targets[handle] = value

    def getJointPosition(self, handle):
        return self.joint_positions[handle]

    def getObjectPosition(self, handle, relative_to):
        return [-1.8, 0.35, 0.55] if handle == 30 else [1.0, 2.0, 3.0]

    def getObjectOrientation(self, handle, relative_to):
        return [0.1, 0.2, 0.3]

    def getObjectQuaternion(self, handle, relative_to):
        return [0.0, 0.0, 0.0, 1.0]

    def setStringSignal(self, name, value):
        self.signals[name] = value

    def getStringSignal(self, name):
        return self.signals.get(name)

    def clearStringSignal(self, name):
        self.signals.pop(name, None)

    def setObjectParent(self, child, parent, keep_in_place):
        self.parent_updates.append((child, parent, keep_in_place))

    def callScriptFunction(self, *args):
        self.script_calls.append(args)

    def getObjectInt32Param(self, handle, parameter):
        return self.int_params.get((handle, parameter), 1)

    def setObjectInt32Param(self, handle, parameter, value):
        self.int_params[(handle, parameter)] = value

    def setJointTargetVelocity(self, handle, value):
        self.joint_velocities[handle] = value

    def startSimulation(self):
        self.state = self.simulation_running

    def stopSimulation(self):
        self.state = self.simulation_stopped

    def getStringParam(self, parameter):
        return "/tmp/five_cr5a_cell.ttt"


class FakeClient:
    def __init__(self, sim):
        self.sim = sim
        self.stepping = False
        self.stepping_calls = []
        self.step_count = 0

    def require(self, name):
        if name != "sim":
            raise RuntimeError(name)
        return self.sim

    def setStepping(self, enabled):
        self.stepping_calls.append(bool(enabled))
        self.stepping = enabled

    def step(self):
        self.step_count += 1


class FakeClientFactory:
    def __init__(self):
        self.sim = FakeSim()
        self.client = FakeClient(self.sim)
        self.calls = []

    def __call__(self, **kwargs):
        self.calls.append(kwargs)
        return self.client


class SceneContractTests(unittest.TestCase):
    def test_five_arm_contract_names(self):
        self.assertEqual(ROBOT_IDS, ("R1", "R2", "R3", "R4", "R5"))
        self.assertEqual(
            get_point_path("R1_TERMINAL_PLACE_TCP"),
            "/FiveCR5A_Cell/Targets/R1_Targets/R1_TERMINAL_PLACE_TCP",
        )
        self.assertEqual(resolve_object_path("R5"), "/R5")
        self.assertEqual(ROS_TOPICS["R1_COMMAND"], "/compact_cell/r1_cmd")
        self.assertIn("R1_BOX_PLACED", PROCESS_COMMANDS)
        self.assertEqual(get_joint_alias("r1", 6), "joint6")
        with self.assertRaises(KeyError):
            get_point_path("P_FEED_01")

    def test_generator_has_executor_visual_owner_guard(self):
        generator = (
            Path(__file__).parents[1] / "scenes/main_cell_generator.lua"
        ).read_text(encoding="utf-8")
        self.assertIn("cell_visual_owner", generator)
        self.assertIn("executorOwnsProductVisuals", generator)
        self.assertIn("isTemplateProductState", generator)

    def test_r1_r2_private_supply_regions_do_not_overlap(self):
        r1 = WORKSPACES["R1_PRIVATE_SUPPLY"]
        r2 = WORKSPACES["R2_PRIVATE_SUPPLY"]
        self.assertLessEqual(r1["upper"][0], r2["lower"][0])
        self.assertFalse(r1["lock_required"])
        self.assertFalse(r2["lock_required"])
        self.assertTrue(WORKSPACES["ASSEMBLY_SHARED"]["lock_required"])
        self.assertEqual(
            WORKSPACES["R2"]["lower"], (-1.90, -0.55, 0.04)
        )
        self.assertEqual(
            WORKSPACES["R4"]["lower"], (-0.05, -0.20, 0.04)
        )
        self.assertEqual(
            WORKSPACES["R3"]["lower"], (-1.40, -0.38, 0.04)
        )
        self.assertEqual(
            WORKSPACES["R5"]["upper"], (0.98, 0.48, 1.55)
        )
        self.assertTrue(WORKSPACES["INSPECTION_SHARED"]["lock_required"])
        self.assertEqual(WORKSPACES["INSPECTION_SHARED"]["max_robots"], 1)


class SimBridgeTests(unittest.TestCase):
    def setUp(self):
        self.factory = FakeClientFactory()
        self.bridge = SimBridge(client_factory=self.factory)
        self.assertTrue(self.bridge.connect("localhost", 23000))

    def test_connect_sets_executor_visual_owner(self):
        self.assertTrue(self.bridge.is_connected())
        self.assertEqual(self.factory.calls, [{"host": "localhost", "port": 23000}])
        self.assertEqual(
            self.factory.sim.signals["cell_visual_owner"], "executor"
        )

    def test_resolves_contract_target_and_pose(self):
        self.assertEqual(self.bridge.get_object_handle("R1_BOX_PICK_APP"), 30)
        pose = self.bridge.get_target_pose("R1_BOX_PICK_APP")
        self.assertEqual(pose["position"], [-1.8, 0.35, 0.55])
        self.assertEqual(pose["quaternion"], [0.0, 0.0, 0.0, 1.0])

    def test_discovers_only_six_arm_joints(self):
        expected = [0.1 * index for index in range(1, 7)]
        self.assertEqual(
            self.bridge.get_robot_joint_handles("R1"), list(range(11, 17))
        )
        self.assertTrue(self.bridge.move_robot_joints("R1", expected))
        self.assertEqual(
            [self.factory.sim.joint_positions[handle] for handle in range(11, 17)],
            expected,
        )
        self.assertNotIn(18, self.factory.sim.joint_positions)

    def test_running_joint_command_uses_targets(self):
        self.factory.sim.state = self.factory.sim.simulation_running
        expected = [0.2] * 6
        self.assertTrue(self.bridge.move_robot_joints("R1", expected))
        self.assertEqual(
            [self.factory.sim.joint_targets[handle] for handle in range(11, 17)],
            expected,
        )

    def test_pose_move_fails_explicitly(self):
        self.assertFalse(self.bridge.move_robot_pose("R1", 1.0, 2.0, 3.0))
        self.assertIn("RobotExecutor", self.bridge.last_error)

    def test_reads_tip_pose(self):
        pose = self.bridge.get_robot_pose("R1")
        self.assertIsNotNone(pose)
        self.assertEqual(pose["tip"], "R1_gripper_tip")
        self.assertEqual((pose["x"], pose["y"], pose["z"]), (1.0, 2.0, 3.0))

    def test_gripper_requires_running_simulation(self):
        self.assertFalse(self.bridge.set_gripper("R1", True))
        self.factory.sim.state = self.factory.sim.simulation_running
        self.assertTrue(self.bridge.set_gripper("R1", False))
        self.assertEqual(
            self.factory.sim.script_calls[-1], ("closeClicked", 22, 0, 2)
        )
        self.assertEqual(
            self.factory.sim.int_params[(22, self.factory.sim.scriptintparam_enabled)],
            1,
        )
        self.assertTrue(self.bridge.freeze_gripper("R1"))
        self.assertEqual(
            self.factory.sim.int_params[(22, self.factory.sim.scriptintparam_enabled)],
            0,
        )
        self.assertEqual(self.factory.sim.joint_velocities, {23: 0.0, 24: 0.0})

    def test_start_step_stop(self):
        self.assertTrue(self.bridge.start_simulation())
        self.assertTrue(self.bridge.step())
        self.assertEqual(self.factory.client.step_count, 1)
        self.bridge.set_stepping(False)
        self.assertFalse(self.factory.client.stepping)

    def test_reenabled_gripper_script_is_initialized_before_command(self):
        self.factory.sim.state = self.factory.sim.simulation_running
        self.bridge.set_stepping(True)
        self.factory.sim.int_params[(22, self.factory.sim.scriptintparam_enabled)] = 0

        self.assertTrue(self.bridge.set_gripper("R1", True))

        self.assertEqual(self.factory.client.step_count, 1)
        self.assertEqual(
            self.factory.sim.script_calls[-1], ("openClicked", 22, 0, 1)
        )
        self.assertTrue(self.bridge.start_simulation())
        self.assertTrue(self.factory.client.stepping)
        self.assertTrue(self.bridge.stop_simulation())
        self.assertFalse(self.factory.client.stepping)

    def test_repeated_stepping_enable_is_idempotent(self):
        self.bridge.set_stepping(True)
        self.bridge.set_stepping(True)
        self.assertTrue(self.bridge.start_simulation())
        self.assertEqual(self.factory.client.stepping_calls, [True])
        self.bridge.disconnect()
        self.assertEqual(self.factory.client.stepping_calls, [True, False])

    def test_attach_and_detach_to_parts_root(self):
        self.bridge.attach_object("BOX_BLANK", "R1")
        self.bridge.detach_object(3)
        self.assertEqual(
            self.factory.sim.parent_updates,
            [(3, 17, True), (3, 2, True)],
        )


if __name__ == "__main__":
    unittest.main()
