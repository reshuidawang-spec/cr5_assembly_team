from __future__ import annotations

import unittest

from scripts.coordinated_pipeline import (
    B_CABINET_RED,
    B_MODULE_BODY,
    B_PCB_BOARD,
    B_TERMINAL_BODY,
    _b_assembled_shape_color,
    DEFECT_CONVEYOR_TRAVEL_M,
    R5_PIPELINE_STEP_DEG,
    back_sequences,
    defect_conveyor_target,
    front_sequences,
)
from scripts.coordinated_front import chaikin_smooth


class CoordinatedPipelineAppearanceTests(unittest.TestCase):
    def test_b_assembled_product_keeps_supply_part_colors(self):
        self.assertEqual(
            _b_assembled_shape_color(
                "Inspection_ControlBox_Product_Shell_Bottom"
            ),
            B_CABINET_RED,
        )
        self.assertEqual(
            _b_assembled_shape_color(
                "Inspection_ControlBox_Product_Control_Module_Body"
            ),
            B_MODULE_BODY,
        )
        self.assertEqual(
            _b_assembled_shape_color(
                "Inspection_ControlBox_Product_PCB_Board"
            ),
            B_PCB_BOARD,
        )
        self.assertEqual(
            _b_assembled_shape_color(
                "Inspection_ControlBox_Product_Terminal_Block_Body"
            ),
            B_TERMINAL_BODY,
        )

    def test_unrelated_shapes_are_not_recolored(self):
        self.assertIsNone(_b_assembled_shape_color("Inspection_Platform"))

    def test_b_integrated_route_keeps_r2_at_safe_wait(self):
        self.assertEqual(front_sequences(False, "B")["R2"], [])
        self.assertTrue(front_sequences(False, "A")["R2"])

    def test_defect_moves_from_entry_toward_conveyor_center(self):
        release = [-0.15, -1.12, 0.270]
        target = defect_conveyor_target(release)
        self.assertAlmostEqual(target[0], -0.60)
        self.assertAlmostEqual(release[0] - target[0], DEFECT_CONVEYOR_TRAVEL_M)
        self.assertEqual(target[1:], release[1:])

    def test_r5_pipeline_uses_dense_smooth_interpolation(self):
        self.assertLessEqual(R5_PIPELINE_STEP_DEG, 2.0)

    def test_r5_lifts_then_uses_symmetric_place_route(self):
        for defect, place_path in (
            (False, "good_place_to_wait_new"),
            (True, "defect_place_to_wait_new"),
        ):
            sequence = back_sequences(defect)["R5"]
            self.assertEqual(
                sequence,
                [
                    ("r5_wait_to_pick_app", 1),
                    ("r5_pick_descend", 1),
                    ("r5_pick_descend", -1),
                    ("r5_wait_to_pick_app", -1),
                    (place_path, -1),
                    (place_path, 1),
                ],
            )

    def test_r5_corner_rounding_preserves_safe_route_endpoints(self):
        points = [[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]]
        rounded = chaikin_smooth(points, passes=2)
        self.assertEqual(rounded[0], points[0])
        self.assertEqual(rounded[-1], points[-1])
        self.assertGreater(len(rounded), len(points))
        self.assertTrue(
            all(0.0 <= value <= 2.0 for point in rounded for value in point)
        )


if __name__ == "__main__":
    unittest.main()
