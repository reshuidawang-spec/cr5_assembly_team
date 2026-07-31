from __future__ import annotations

import threading
import time
import unittest

from robot_control.motion_timing import PersistentJointMotionMonitorFactory


class FakeObservationSim:
    def __init__(self):
        self.simulation_time = 0.0

    def getSimulationTime(self):
        return self.simulation_time


class FakeObservationBridge:
    def __init__(self):
        self.sim = FakeObservationSim()
        self.connected = False
        self.connect_calls = 0
        self.disconnect_calls = 0
        self.last_error = ""
        self.positions = {"R1": [0.0] * 6, "R2": [0.0] * 6}

    def is_connected(self):
        return self.connected

    def connect(self):
        self.connect_calls += 1
        self.connected = True
        return True

    def disconnect(self):
        self.disconnect_calls += 1
        self.connected = False

    def get_robot_joint_positions(self, robot_id):
        return list(self.positions[robot_id])


class PersistentMotionMonitorTests(unittest.TestCase):
    @staticmethod
    def _wait_for_motion(lease):
        deadline = time.monotonic() + 1.0
        while (
            not lease.monitor._result["motion_detected"]
            and time.monotonic() < deadline
        ):
            time.sleep(0.005)

    def test_factory_reuses_one_connection_until_close(self):
        bridge = FakeObservationBridge()
        factory = PersistentJointMotionMonitorFactory(
            "127.0.0.1",
            23000,
            poll_interval_s=0.001,
            bridge_factory=lambda **kwargs: bridge,
        )

        factory.prepare()
        factory.prepare()
        first = factory("R1")
        first.start()
        bridge.positions["R1"][0] = 0.01
        self._wait_for_motion(first)
        first_result = first.stop()

        second = factory("R2")
        second.start()
        bridge.positions["R2"][1] = 0.01
        self._wait_for_motion(second)
        second_result = second.stop()

        self.assertTrue(first_result["motion_detected"])
        self.assertTrue(second_result["motion_detected"])
        self.assertEqual(bridge.connect_calls, 1)
        self.assertEqual(bridge.disconnect_calls, 0)
        factory.close()
        self.assertEqual(bridge.disconnect_calls, 1)

    def test_monitor_leases_serialize_access_to_observation_socket(self):
        bridge = FakeObservationBridge()
        factory = PersistentJointMotionMonitorFactory(
            "127.0.0.1",
            23000,
            bridge_factory=lambda **kwargs: bridge,
        )
        factory.prepare()
        first = factory("R1")
        second = factory("R2")
        first.start()
        second_started = threading.Event()

        def start_second():
            second.start()
            second_started.set()

        thread = threading.Thread(target=start_second)
        thread.start()
        self.assertFalse(second_started.wait(0.03))
        first.stop()
        self.assertTrue(second_started.wait(1.0))
        second.stop()
        thread.join(timeout=1.0)
        factory.close()


if __name__ == "__main__":
    unittest.main()
