#!/usr/bin/env python3
"""Generate points along a line and measure them vertically."""
import argparse

import rclpy

from measure_points_csv import measure_points, validate_common_args
from probe_touch import ProbeTouch, add_vertical_probe_ack_argument, validate_probe_args


def generate_line_points(args):
    """Generate line points."""
    if args.points <= 0:
        raise SystemExit("--points must be positive")

    points = []
    if args.points == 1:
        fractions = [0.0]
    else:
        fractions = [i / (args.points - 1) for i in range(args.points)]
    for index, t in enumerate(fractions, start=1):
        x = args.x1 + (args.x2 - args.x1) * t
        y = args.y1 + (args.y2 - args.y1) * t
        points.append(
            {
                "name": f"{args.name_prefix}{index}",
                "x": x,
                "y": y,
                "safe_z": args.safe_z,
                "approach_mm": args.approach_mm,
                "retract_mm": args.retract_mm,
            }
        )
    return points


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="actually move the robot")
    parser.add_argument("--x1", type=float, required=True, help="line start X")
    parser.add_argument("--y1", type=float, required=True, help="line start Y")
    parser.add_argument("--x2", type=float, required=True, help="line end X")
    parser.add_argument("--y2", type=float, required=True, help="line end Y")
    parser.add_argument("--points", type=int, required=True, help="number of points on the line")
    parser.add_argument("--safe-z", type=float, required=True, help="safe approach Z")
    parser.add_argument("--approach-mm", type=float, default=20.0, help="vertical probing distance")
    parser.add_argument("--retract-mm", type=float, default=8.0, help="retract distance after trigger")
    parser.add_argument("--speed", type=int, default=1, help="SpeedFactor ratio during probing")
    parser.add_argument("--allow-long-approach", action="store_true", help="allow approach distance > 5 mm")
    parser.add_argument("--rx", type=float, help="explicit safe-pose RX orientation")
    parser.add_argument("--ry", type=float, help="explicit safe-pose RY orientation")
    parser.add_argument("--rz", type=float, help="explicit safe-pose RZ orientation")
    parser.add_argument(
        "--use-current-orientation",
        action="store_true",
        help="allow real execution with the robot's current RX/RY/RZ",
    )
    parser.add_argument("--continue-on-error", action="store_true", help="continue after a point fails")
    parser.add_argument("--settle-sec", type=float, default=1.0, help="delay between points")
    parser.add_argument("--flange-mm", type=float, default=125.4, help="robot flange to cross-stylus branch origin")
    parser.add_argument("--stylus-mm", type=float, default=75.0, help="vertical stylus ball-centre distance")
    parser.add_argument("--timeout", type=float, default=20.0, help="probing timeout seconds per point")
    parser.add_argument(
        "--probe-step-mm",
        type=float,
        default=0.5,
        help="segment probing MovL into this step size; 0 disables segmented probing",
    )
    parser.add_argument("--output", default="data/line_points.csv", help="CSV output path")
    parser.add_argument("--name-prefix", default="line_", help="prefix for generated point names")
    add_vertical_probe_ack_argument(parser)
    args = parser.parse_args()
    validate_common_args(args)
    try:
        validate_probe_args(args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    points = generate_line_points(args)

    if not args.execute and args.rx is not None:
        measure_points(None, points, args)
        return

    rclpy.init()
    node = ProbeTouch()
    try:
        node.wait_services(10.0)
        measure_points(node, points, args)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
