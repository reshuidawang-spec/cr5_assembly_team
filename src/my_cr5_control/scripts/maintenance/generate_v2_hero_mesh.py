#!/usr/bin/env python3
"""V2 hero mesh generator — creates a high-quality 3D mesh visualization of the V2 benchmark scene (box with holes) for paper figures and presentations."""

from pathlib import Path
import struct


ROOT = Path(__file__).resolve().parents[2]
MESH_DIR = ROOT / "src" / "meshes"


def tri(f, a, b, c):
    """Tri."""
    f.append((a, b, c))


def quad(f, a, b, c, d):
    """Quad."""
    tri(f, a, b, c)
    tri(f, a, c, d)


def box(f, x0, x1, y0, y1, z0, z1):
    """Box."""
    quad(f, (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0))
    quad(f, (x0, y0, z1), (x0, y1, z1), (x1, y1, z1), (x1, y0, z1))
    quad(f, (x0, y0, z0), (x0, y0, z1), (x1, y0, z1), (x1, y0, z0))
    quad(f, (x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1))
    quad(f, (x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1))
    quad(f, (x1, y0, z0), (x1, y0, z1), (x1, y1, z1), (x1, y1, z0))


def write_hero_mesh(path, cutaway=False):
    """Write hero mesh."""
    x0, x1 = 0.0, 880.0
    y0, y1 = 0.0, 880.0
    z0, z_top = 0.0, 760.0
    open_x0, open_x1 = 280.0, 600.0
    open_y0, open_y1 = 280.0, 600.0
    ledge_z = 570.0
    throat_x0, throat_x1 = 430.0, 570.0
    throat_y0, throat_y1 = 370.0, 510.0
    bottom_z = 80.0
    front_clip = 440.0 if cutaway else y0

    triangles = []

    # Outer box shell. The top face is split into a frame so the entrance is open.
    box(triangles, x0, x1, front_clip, y1, z0, 35.0)
    if not cutaway:
        quad(triangles, (x0, y0, z0), (x1, y0, z0), (x1, y0, z_top), (x0, y0, z_top))
    quad(triangles, (x0, y1, z0), (x0, y1, z_top), (x1, y1, z_top), (x1, y1, z0))
    quad(triangles, (x0, front_clip, z0), (x0, y1, z0), (x0, y1, z_top), (x0, front_clip, z_top))
    quad(triangles, (x1, front_clip, z0), (x1, front_clip, z_top), (x1, y1, z_top), (x1, y1, z0))

    top_bands = [
        (x0, open_x0, front_clip, y1),
        (open_x1, x1, front_clip, y1),
        (open_x0, open_x1, front_clip, open_y0),
        (open_x0, open_x1, open_y1, y1),
    ]
    for xa, xb, ya, yb in top_bands:
        if xb > xa and yb > ya:
            quad(triangles, (xa, ya, z_top), (xb, ya, z_top), (xb, yb, z_top), (xa, yb, z_top))

    # Entrance walls down to the transition ledge.
    if not cutaway:
        quad(triangles, (open_x0, open_y0, ledge_z), (open_x1, open_y0, ledge_z),
             (open_x1, open_y0, z_top), (open_x0, open_y0, z_top))
    quad(triangles, (open_x0, open_y1, ledge_z), (open_x0, open_y1, z_top),
         (open_x1, open_y1, z_top), (open_x1, open_y1, ledge_z))
    quad(triangles, (open_x0, open_y0, ledge_z), (open_x0, open_y0, z_top),
         (open_x0, open_y1, z_top), (open_x0, open_y1, ledge_z))
    quad(triangles, (open_x1, open_y0, ledge_z), (open_x1, open_y1, ledge_z),
         (open_x1, open_y1, z_top), (open_x1, open_y0, z_top))

    # Transition ledge with an offset secondary throat.
    ledge_bands = [
        (open_x0, throat_x0, open_y0, open_y1),
        (throat_x1, open_x1, open_y0, open_y1),
        (throat_x0, throat_x1, open_y0, throat_y0),
        (throat_x0, throat_x1, throat_y1, open_y1),
    ]
    for xa, xb, ya, yb in ledge_bands:
        if xb > xa and yb > ya and yb > front_clip:
            quad(triangles, (xa, max(ya, front_clip), ledge_z), (xb, max(ya, front_clip), ledge_z),
                 (xb, yb, ledge_z), (xa, yb, ledge_z))

    # Offset throat walls and deep cavity floor.
    if not cutaway:
        quad(triangles, (throat_x0, throat_y0, bottom_z), (throat_x1, throat_y0, bottom_z),
             (throat_x1, throat_y0, ledge_z), (throat_x0, throat_y0, ledge_z))
    quad(triangles, (throat_x0, throat_y1, bottom_z), (throat_x0, throat_y1, ledge_z),
         (throat_x1, throat_y1, ledge_z), (throat_x1, throat_y1, bottom_z))
    quad(triangles, (throat_x0, throat_y0, bottom_z), (throat_x0, throat_y0, ledge_z),
         (throat_x0, throat_y1, ledge_z), (throat_x0, throat_y1, bottom_z))
    quad(triangles, (throat_x1, throat_y0, bottom_z), (throat_x1, throat_y1, bottom_z),
         (throat_x1, throat_y1, ledge_z), (throat_x1, throat_y0, ledge_z))
    quad(triangles, (throat_x0, max(throat_y0, front_clip), bottom_z),
         (throat_x1, max(throat_y0, front_clip), bottom_z),
         (throat_x1, throat_y1, bottom_z),
         (throat_x0, throat_y1, bottom_z))

    header = f"{path.stem}".encode("ascii")[:80].ljust(80, b"\0")
    with path.open("wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(triangles)))
        for a, b, c in triangles:
            f.write(struct.pack("<3f", 0.0, 0.0, 0.0))
            for p in (a, b, c):
                f.write(struct.pack("<3f", *p))
            f.write(struct.pack("<H", 0))


def main():
    """Main."""
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    write_hero_mesh(MESH_DIR / "ws119_v2_hero_offset_throat.stl", cutaway=False)
    write_hero_mesh(MESH_DIR / "ws119_v2_hero_offset_throat_cutaway.stl", cutaway=True)


if __name__ == "__main__":
    main()
