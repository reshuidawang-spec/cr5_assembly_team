from __future__ import annotations

import json
import unittest
from pathlib import Path

from sim_bridge.audit_five_cr5a_scene import (
    DEFAULT_BASELINE,
    DEFAULT_SCENE,
    compare_target_snapshots,
    compute_r5_height_result,
    fingerprint_file,
    is_protected_target,
)
from sim_bridge.scene_objects import POINTS


class SceneAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.baseline = json.loads(DEFAULT_BASELINE.read_text(encoding="utf-8"))

    def test_baseline_covers_every_contract_target(self):
        self.assertEqual(set(self.baseline["targets"]), set(POINTS))

    def test_checked_in_scene_matches_baseline_fingerprint(self):
        self.assertEqual(
            fingerprint_file(DEFAULT_SCENE),
            {
                "size": self.baseline["scene"]["size"],
                "sha256": self.baseline["scene"]["sha256"],
            },
        )

    def test_target_comparison_distinguishes_protected_changes(self):
        actual = json.loads(json.dumps(self.baseline["targets"]))
        actual["R3_MODULE_PLACE_TCP"]["position"][0] += 0.015
        actual["R4_SCREW_TCP"]["orientation"][2] = 0.5
        changes = compare_target_snapshots(
            actual,
            self.baseline["targets"],
            self.baseline,
        )
        by_name = {change["name"]: change for change in changes}
        self.assertFalse(by_name["R3_MODULE_PLACE_TCP"]["protected"])
        self.assertTrue(by_name["R4_SCREW_TCP"]["protected"])
        self.assertTrue(is_protected_target("R2_PCB_PICK_TCP", self.baseline))
        self.assertFalse(is_protected_target("R5_GOOD_PLACE_TCP", self.baseline))

    def test_target_comparison_reports_missing_target(self):
        actual = json.loads(json.dumps(self.baseline["targets"]))
        actual.pop("R5_DEFECT_PLACE_TCP")
        changes = compare_target_snapshots(
            actual,
            self.baseline["targets"],
            self.baseline,
        )
        missing = [change for change in changes if change.get("missing")]
        self.assertEqual([change["name"] for change in missing], ["R5_DEFECT_PLACE_TCP"])

    def test_r5_height_audit_exposes_current_26_mm_error(self):
        current = compute_r5_height_result(0.216, 0.340, 0.420, 0.270)
        corrected = compute_r5_height_result(0.216, 0.340, 0.394, 0.270)
        self.assertAlmostEqual(current["height_error_m"], 0.026, places=9)
        self.assertAlmostEqual(corrected["height_error_m"], 0.0, places=9)


if __name__ == "__main__":
    unittest.main()
