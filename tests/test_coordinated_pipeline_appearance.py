from __future__ import annotations

import unittest

from scripts.coordinated_pipeline import (
    B_CABINET_RED,
    B_MODULE_BODY,
    B_PCB_BOARD,
    B_TERMINAL_BODY,
    _b_assembled_shape_color,
)


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


if __name__ == "__main__":
    unittest.main()
