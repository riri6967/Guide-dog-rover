#!/usr/bin/env python3

from pathlib import Path
import sys

import numpy as np

scripts_dir = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(scripts_dir))

from v9_global_planner import StructuralOccupancyMap


WORLD = (
    Path(__file__).resolve().parents[2]
    / "worlds"
    / "assistive_world.sdf"
)


def main():

    out_dir = Path(__file__).resolve().parent

    planner = StructuralOccupancyMap(
        world_path=WORLD,
        resolution=0.05,
        inflation_radius=0.0,
        bounds=(-13.0, 13.0, -8.0, 8.0),
    )

    # Nav2 map:
    # 255 = free
    # 0   = occupied
    image = np.where(
        planner.grid > 0,
        0,
        255,
    ).astype(np.uint8)

    # World Y increases upward; PGM rows increase downward.
    image = np.flipud(image)

    pgm = out_dir / "assistive_world.pgm"
    yaml = out_dir / "assistive_world.yaml"

    with open(pgm, "wb") as f:
        f.write(
            f"P5\n{planner.nx} {planner.ny}\n255\n".encode()
        )
        f.write(image.tobytes())

    yaml.write_text(
        f"""image: {pgm.name}
mode: trinary
resolution: {planner.resolution:.4f}
origin: [{planner.xmin:.4f}, {planner.ymin:.4f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    )

    print("============================================")
    print("NAV2 STATIC MAP GENERATED")
    print(f"PGM        : {pgm}")
    print(f"YAML       : {yaml}")
    print(f"Grid       : {planner.nx} x {planner.ny}")
    print(f"Resolution : {planner.resolution:.3f} m")
    print(
        f"Bounds     : "
        f"x=[{planner.xmin},{planner.xmax}] "
        f"y=[{planner.ymin},{planner.ymax}]"
    )
    print(f"Occupied   : {int(planner.grid.sum())}")
    print("============================================")


if __name__ == "__main__":
    main()
