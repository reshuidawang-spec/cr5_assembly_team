#!/usr/bin/env python3
"""Generate a grid of XY points and measure them vertically."""
import argparse

import rclpy

from measure_points_csv import measure_points, validate_common_args
from probe_touch import ProbeTouch, add_vertical_probe_ack_argument, validate_probe_args


def generate_grid_points(args):
    """Generate grid points."""
    if args.rows <= 0:
        raise SystemExit("--rows must be positive")
    if args.cols <= 0:
        raise SystemExit("--cols must be positive")

    points = []
    xs = [args.x_min + (args.x_max - args.x_min) * c / max(args.cols - 1, 1) for c in range(args.cols)]
    ys = [args.y_min + (args.y_max - args.y_min) * r / max(args.rows - 1, 1) for r in range(args.rows)]

    for row_idx, y in enumerate(ys):
        for col_idx, x in enumerate(xs):
            points.append(
                {
                    "name": f"{args.name_prefix}r{row_idx + 1}c{col_idx + 1}",
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
    parser.add_argument("--x-min", type=float, required=True, help="grid minimum X")
    parser.add_argument("--x-max", type=float, required=True, help="grid maximum X")
    parser.add_argument("--y-min", type=float, required=True, help="grid minimum Y")
    parser.add_argument("--y-max", type=float, required=True, help="grid maximum Y")
    parser.add_argument("--rows", type=int, required=True, help="number of grid rows")
    parser.add_argument("--cols", type=int, required=True, help="number of grid columns")
    parser.add_argument("--safe-z", type=float, required=True, help="safe approach Z")
    parser.add_argument("--approach-mm", type=float, default=20.0, help="vertical probing distance")
    parser.add_argument("--retract-mm", type=float, default=8.0, help="retract distance after trigger")
    parser.add_argument("--speed", type=int, default=1, help="SpeedFactor ratio during probing")
    parser.add_argument("--allow-long-approach", action="store_true", help="allow approach distance > 5 mm")
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
    parser.add_argument("--output", default="data/grid_points.csv", help="CSV output path")
    parser.add_argument("--name-prefix", default="grid_", help="prefix for generated point names")
    parser.add_argument("--rx", type=float, default=None, help="explicit safe pose RX (deg)")
    parser.add_argument("--ry", type=float, default=None, help="explicit safe pose RY (deg)")
    parser.add_argument("--rz", type=float, default=None, help="explicit safe pose RZ (deg)")
    parser.add_argument("--use-current-orientation", action="store_true",
                        help="use current robot orientation; required when --rx/--ry/--rz not provided")
    add_vertical_probe_ack_argument(parser)
    args = parser.parse_args()
    validate_common_args(args)
    try:
        validate_probe_args(args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    if args.x_max <= args.x_min:
        raise SystemExit("--x-max must be greater than --x-min")
    if args.y_max <= args.y_min:
        raise SystemExit("--y-max must be greater than --y-min")
    if args.rx is None or args.ry is None or args.rz is None:
        if not args.use_current_orientation:
            raise SystemExit("provide --rx/--ry/--rz, or use --use-current-orientation")

    points = generate_grid_points(args)

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
