"""Generate the FiveCR5A_Cell Targets tree in the current CoppeliaSim scene.

Equivalent to Step03_Create_Process_Targets_60.lua but executed via ZMQ.
Run once per clean scene; safe to re-run (recreates all targets).
"""
from __future__ import annotations

import math
from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import SCENE_ROOT

# --- Height parameters from Step03_Create_Process_Targets_60.lua ---
TABLE_TOP_Z = 0.120
AREA_H = 0.035
FIXTURE_H = 0.060
FIXTURE_TOP_Z = TABLE_TOP_Z + AREA_H + FIXTURE_H  # 0.215
PRODUCT_BOTTOM_Z = FIXTURE_TOP_Z + 0.001  # 0.216
BOX_H = 0.072
BOX_CENTER_Z = PRODUCT_BOTTOM_Z + BOX_H / 2  # 0.252
BOX_TOP_Z = PRODUCT_BOTTOM_Z + BOX_H  # 0.288
PCB_H = 0.0048
MODULE_H = 0.021
TERMINAL_H = 0.021
PCB_PICK_Z = PRODUCT_BOTTOM_Z + 0.020
MODULE_PICK_Z = PRODUCT_BOTTOM_Z + 0.055
TERMINAL_PICK_Z = PRODUCT_BOTTOM_Z + 0.050
APP_LIFT = 0.180
SCREW_APP_LIFT = 0.160

# --- Planar coordinates from Step03 ---
P = {
    "boxSupply": (-1.86, 0.22),
    "terminalSupply": (-1.82, -0.02),
    "pcbSupply": (-1.22, -0.42),
    "moduleSupply": (-0.78, -0.20),
    "assembly": (-1.08, 0.12),
    "inspection": (0.15, 0.05),
    "goodPlace": (0.65, -1.10),
    "defectPlace": (-0.35, -1.12),
}

COLORS = {
    "R1": [1.00, 0.30, 0.25],
    "R2": [0.25, 0.60, 1.00],
    "R3": [0.25, 1.00, 0.35],
    "R4": [1.00, 0.75, 0.10],
    "R5": [0.95, 0.25, 1.00],
    "SENSOR": [0.50, 0.90, 1.00],
}


def _app_of(pos, lift=APP_LIFT):
    return pos[0], pos[1], pos[2] + lift


def _make_target(sim, group, name, pos, color):
    """Create a 35 mm target dummy."""
    d = sim.createDummy(0.035)
    sim.setObjectAlias(d, name)
    sim.setObjectParent(d, group, True)
    sim.setObjectPosition(d, -1, list(pos))
    sim.setObjectOrientation(d, -1, [0.0, 0.0, 0.0])
    sim.setObjectColor(d, 0, sim.colorcomponent_ambient_diffuse, color)
    return d


def _make_pair(sim, group, prefix, tcp_pos, color, lift=APP_LIFT):
    """Create APP + TCP dummy pair."""
    app = _app_of(tcp_pos, lift)
    _make_target(sim, group, f"{prefix}_APP", app, color)
    _make_target(sim, group, f"{prefix}_TCP", tcp_pos, color)
    print(f"  {prefix}_APP = {app}")
    print(f"  {prefix}_TCP = {tcp_pos}")


def main():
    bridge = SimBridge()
    bridge.connect(port=23000)
    sim = bridge.sim

    # Ensure /FiveCR5A_Cell/Targets exists
    cell = sim.getObject(SCENE_ROOT)
    try:
        old_targets = sim.getObject(f"{SCENE_ROOT}/Targets")
        print("Removing old Targets tree...")
        sim.removeObjects(sim.getObjectsInTree(old_targets, sim.handle_all, 0))
    except Exception:
        pass

    targets_root = sim.createDummy(0.030)
    sim.setObjectAlias(targets_root, "Targets")
    sim.setObjectParent(targets_root, cell, True)
    sim.setObjectPosition(targets_root, cell, [0, 0, 0])

    def _ensure_group(name):
        g = sim.createDummy(0.025)
        sim.setObjectAlias(g, name)
        sim.setObjectParent(g, targets_root, True)
        sim.setObjectPosition(g, targets_root, [0, 0, 0])
        sim.setObjectOrientation(g, targets_root, [0, 0, 0])
        return g

    print("=== Generating R1 Targets ===")
    gR1 = _ensure_group("R1_Targets")
    _make_target(sim, gR1, "R1_HOME_REF", (-1.55, 0.55, 0.70), COLORS["R1"])
    _make_pair(sim, gR1, "R1_BOX_PICK", (-1.86, 0.22, BOX_CENTER_Z), COLORS["R1"])
    _make_pair(sim, gR1, "R1_BOX_PLACE", (-1.08, 0.12, BOX_CENTER_Z), COLORS["R1"])
    _make_pair(sim, gR1, "R1_TERMINAL_PICK", (-1.82, -0.02, TERMINAL_PICK_Z), COLORS["R1"])
    _make_pair(
        sim, gR1, "R1_TERMINAL_PLACE",
        (P["assembly"][0] + 0.020, P["assembly"][1] - 0.035, BOX_TOP_Z + TERMINAL_H / 2),
        COLORS["R1"],
    )

    print("=== Generating R2 Targets ===")
    gR2 = _ensure_group("R2_Targets")
    _make_target(sim, gR2, "R2_HOME_REF", (-1.55, -0.20, 0.70), COLORS["R2"])
    _make_pair(sim, gR2, "R2_PCB_PICK", (-1.22, -0.42, PCB_PICK_Z), COLORS["R2"])
    _make_pair(
        sim, gR2, "R2_PCB_PLACE",
        (P["assembly"][0], P["assembly"][1], BOX_TOP_Z + PCB_H / 2),
        COLORS["R2"],
    )

    print("=== Generating R3 Targets ===")
    gR3 = _ensure_group("R3_Targets")
    _make_target(sim, gR3, "R3_HOME_REF", (-0.60, 0.35, 0.70), COLORS["R3"])
    _make_pair(sim, gR3, "R3_MODULE_PICK", (-0.78, -0.20, MODULE_PICK_Z), COLORS["R3"])
    _make_pair(
        sim, gR3, "R3_MODULE_PLACE",
        (P["assembly"][0] - 0.025, P["assembly"][1] + 0.025, BOX_TOP_Z + PCB_H + MODULE_H / 2),
        COLORS["R3"],
    )
    _make_pair(sim, gR3, "R3_PRODUCT_PICK", (P["assembly"][0], P["assembly"][1], BOX_CENTER_Z), COLORS["R3"])
    _make_pair(sim, gR3, "R3_PRODUCT_PLACE_INSPECTION", (P["inspection"][0], P["inspection"][1], BOX_CENTER_Z), COLORS["R3"])

    print("=== Generating R4 Targets ===")
    gR4 = _ensure_group("R4_Targets")
    _make_target(sim, gR4, "R4_HOME_REF", (0.55, 0.25, 0.70), COLORS["R4"])
    screw_tcp = (P["inspection"][0] + 0.020, P["inspection"][1] - 0.035, BOX_TOP_Z + TERMINAL_H + 0.030)
    screw_press = (screw_tcp[0], screw_tcp[1], screw_tcp[2] - 0.030)
    screw_app = _app_of(screw_tcp, SCREW_APP_LIFT)
    _make_target(sim, gR4, "R4_SCREW_APP", screw_app, COLORS["R4"])
    _make_target(sim, gR4, "R4_SCREW_TCP", screw_tcp, COLORS["R4"])
    _make_target(sim, gR4, "R4_SCREW_PRESS", screw_press, COLORS["R4"])
    print(f"  R4_SCREW_APP = {screw_app}")
    print(f"  R4_SCREW_TCP = {screw_tcp}")
    print(f"  R4_SCREW_PRESS = {screw_press}")

    print("=== Generating R5 Targets ===")
    gR5 = _ensure_group("R5_Targets")
    _make_target(sim, gR5, "R5_HOME_REF", (0.15, -0.45, 0.70), COLORS["R5"])
    _make_pair(sim, gR5, "R5_PRODUCT_PICK", (P["inspection"][0], P["inspection"][1], BOX_CENTER_Z), COLORS["R5"])
    _make_pair(sim, gR5, "R5_GOOD_PLACE", (P["goodPlace"][0], P["goodPlace"][1], BOX_CENTER_Z), COLORS["R5"])
    _make_pair(sim, gR5, "R5_DEFECT_PLACE", (P["defectPlace"][0], P["defectPlace"][1], BOX_CENTER_Z), COLORS["R5"])

    print("=== Generating Sensor Targets ===")
    gS = _ensure_group("Sensor_Targets")
    _make_target(sim, gS, "CAMERA_INSPECTION_CENTER", (P["inspection"][0], P["inspection"][1], BOX_TOP_Z + 0.120), COLORS["SENSOR"])

    bridge.disconnect()
    print("\nDone. Targets generated. Do NOT save the scene yet — verify first.")


if __name__ == "__main__":
    main()
