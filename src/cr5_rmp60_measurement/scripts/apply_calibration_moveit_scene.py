#!/usr/bin/env python3
"""Apply calibration-sphere workcell collision objects to MoveIt."""
import argparse
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_REFERENCE_FIT = PROJECT_DIR / "data/2026.6.16/yneg_near_fourth_refit_20260616.json"
OBJECT_IDS = (
    "calibration_table",
    "calibration_sphere",
    "calibration_sphere_stem",
    "calibration_magnetic_base",
)


def load_sphere_center(path):
    """Load sphere center."""
    data = json.loads(Path(path).read_text())
    center = data.get("sphere_center_mm")
    if not isinstance(center, list) or len(center) != 3:
        raise ValueError(f"{path}: missing 3-value sphere_center_mm")
    return [float(value) for value in center]


def pose_xyz_mm(x, y, z):
    """Pose xyz mm."""
    pose = Pose()
    pose.position.x = float(x) * 0.001
    pose.position.y = float(y) * 0.001
    pose.position.z = float(z) * 0.001
    pose.orientation.w = 1.0
    return pose


def primitive_box_mm(x, y, z):
    """Primitive box mm."""
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(x) * 0.001, float(y) * 0.001, float(z) * 0.001]
    return primitive


def primitive_sphere_mm(radius):
    """Primitive sphere mm."""
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [float(radius) * 0.001]
    return primitive


def primitive_cylinder_mm(height, radius):
    """Primitive cylinder mm."""
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.CYLINDER
    primitive.dimensions = [float(height) * 0.001, float(radius) * 0.001]
    return primitive


def collision_object(object_id, frame_id, primitive, pose, operation=CollisionObject.ADD):
    """Collision object."""
    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = object_id
    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)
    obj.operation = operation
    return obj


def remove_object(object_id, frame_id):
    """Remove object."""
    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = object_id
    obj.operation = CollisionObject.REMOVE
    return obj


def build_objects(args):
    """Build objects."""
    sphere_center = load_sphere_center(args.reference_fit_json)
    objects = []

    table_thickness = args.table_thickness_mm
    table_center_z = args.table_z_mm - table_thickness * 0.5
    objects.append(
        collision_object(
            "calibration_table",
            args.frame_id,
            primitive_box_mm(args.table_size_x_mm, args.table_size_y_mm, table_thickness),
            pose_xyz_mm(args.table_center_x_mm, args.table_center_y_mm, table_center_z),
        )
    )

    objects.append(
        collision_object(
            "calibration_sphere",
            args.frame_id,
            primitive_sphere_mm(args.sphere_radius_mm),
            pose_xyz_mm(*sphere_center),
        )
    )

    stem_top_z = sphere_center[2] - args.sphere_radius_mm
    stem_height = max(0.0, stem_top_z - args.table_z_mm)
    if stem_height > 1e-6:
        objects.append(
            collision_object(
                "calibration_sphere_stem",
                args.frame_id,
                primitive_cylinder_mm(stem_height, args.stem_radius_mm),
                pose_xyz_mm(sphere_center[0], sphere_center[1], args.table_z_mm + stem_height * 0.5),
            )
        )

    if args.magnetic_base_size_mm:
        if len(args.magnetic_base_size_mm) != 3 or len(args.magnetic_base_center_mm) != 3:
            raise ValueError("--magnetic-base-size-mm and --magnetic-base-center-mm must each contain 3 values")
        size = args.magnetic_base_size_mm
        center = args.magnetic_base_center_mm
        objects.append(
            collision_object(
                "calibration_magnetic_base",
                args.frame_id,
                primitive_box_mm(size[0], size[1], size[2]),
                pose_xyz_mm(center[0], center[1], center[2]),
            )
        )

    return objects


class SceneApplier(Node):
    def __init__(self):
        super().__init__("calibration_moveit_scene_applier")
        self.cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def apply(self, objects, timeout_sec):
        """Apply."""
        if not self.cli.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/apply_planning_scene service is not available; start MoveIt move_group first")
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = list(objects)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.cli.call_async(req)
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds / 1e9 > deadline:
                raise RuntimeError("timed out applying MoveIt planning scene")
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError("MoveIt rejected planning scene update")


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-fit-json", default=str(DEFAULT_REFERENCE_FIT))
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--sphere-radius-mm", type=float, default=10.0)
    parser.add_argument("--stem-radius-mm", type=float, default=4.0)
    parser.add_argument("--table-z-mm", type=float, default=0.0)
    parser.add_argument("--table-thickness-mm", type=float, default=20.0)
    parser.add_argument("--table-size-x-mm", type=float, default=1200.0)
    parser.add_argument("--table-size-y-mm", type=float, default=900.0)
    parser.add_argument("--table-center-x-mm", type=float, default=-400.0)
    parser.add_argument("--table-center-y-mm", type=float, default=120.0)
    parser.add_argument("--magnetic-base-size-mm", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--magnetic-base-center-mm", nargs=3, type=float, default=[-401.9, 126.5, 35.0])
    parser.add_argument("--remove", action="store_true", help="remove calibration objects instead of adding them")
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    return parser.parse_args()


def main():
    """Main."""
    args = parse_args()
    if args.timeout_sec <= 0:
        raise SystemExit("--timeout-sec must be positive")
    for label, value in (
        ("--sphere-radius-mm", args.sphere_radius_mm),
        ("--stem-radius-mm", args.stem_radius_mm),
        ("--table-thickness-mm", args.table_thickness_mm),
        ("--table-size-x-mm", args.table_size_x_mm),
        ("--table-size-y-mm", args.table_size_y_mm),
    ):
        if not math.isfinite(value) or value <= 0:
            raise SystemExit(f"{label} must be positive")

    try:
        objects = [remove_object(object_id, args.frame_id) for object_id in OBJECT_IDS] if args.remove else build_objects(args)
    except (OSError, ValueError, KeyError) as exc:
        raise SystemExit(str(exc)) from exc

    rclpy.init()
    node = SceneApplier()
    try:
        node.apply(objects, args.timeout_sec)
        action = "removed" if args.remove else "applied"
        print(f"{action} MoveIt calibration scene objects: {', '.join(obj.id for obj in objects)}")
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
