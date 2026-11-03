#!/usr/bin/env python3
"""Bake the rover_description meshes into p0_meshes.js for the browser simulator.

Every part is placed exactly where rover.urdf.xacro / handle.urdf.xacro put it (handle at its
design pose, joint = 0), merged by URDF material, decimated by vertex clustering so the page
stays small, and stored as quantised int16 positions (0.1 mm) in three.js axes:
three (x, y, z) = ROS (x, z, -y). Static parts are in base_link; wheel parts in the wheel frame.

Run from anywhere:  python export_urdf_meshes.py
If the URDF numbers change, update the transforms below to match.
"""
import base64, json, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
MESH = HERE.parents[2] / "raspberry_pi" / "ros" / "rover_description" / "meshes"
OUT = HERE.parent / "p0_meshes.js"

WHEEL_R, CAD_WHEEL_R = 0.055, 0.0598
HALF_TRACK, HALF_BASE, BODY_WALL_Y = 0.145, 0.1473 / 2, 0.0813
EXT_OFFSET = HALF_TRACK - BODY_WALL_Y
MM = 0.001

COLORS = {  # URDF materials
    "grey": "#BFBFBF", "black": "#1A1A1A", "dark": "#333333", "green": "#1A7333", "blue": "#334DCC",
    "orange": "#E3873B", "sensor": "#2B8A52", "aluminium": "#BFC2C7", "handle_grey": "#8C9199",
    "handle_grip": "#D9CFBA", "vib_board": "#141417",
}


def load(name):
    d = (MESH / name).read_bytes()
    n = int.from_bytes(d[80:84], "little")
    a = np.frombuffer(d[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n)
    return a["v"].reshape(-1, 3).astype(np.float64)        # 3 vertices per triangle


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def place(v, xyz=(0, 0, 0), angles=(0, 0, 0), scale=(1, 1, 1)):
    return (v * np.array(scale)) @ rpy(*angles).T + np.array(xyz)


def cluster(v, cell):
    """Vertex-clustering decimation: snap to a grid, merge, drop collapsed triangles."""
    key = np.round(v / cell).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    sums = np.zeros((len(uniq), 3)); np.add.at(sums, inv, v)
    cnt = np.bincount(inv, minlength=len(uniq))[:, None]
    pos = sums / cnt
    tri = inv.reshape(-1, 3)
    ok = (tri[:, 0] != tri[:, 1]) & (tri[:, 1] != tri[:, 2]) & (tri[:, 0] != tri[:, 2])
    tri = tri[ok]
    tri = np.unique(np.sort(tri, axis=1), axis=0, return_index=True)[1]
    tri = inv.reshape(-1, 3)[ok][np.sort(tri)]
    used = np.unique(tri)
    remap = -np.ones(len(pos), dtype=np.int64); remap[used] = np.arange(len(used))
    return pos[used], remap[tri]


def to_three(v):
    return np.stack([v[:, 0], v[:, 2], -v[:, 1]], axis=1)


def pack(parts, cell):
    v = np.concatenate(parts)
    pos, tri = cluster(v, cell)
    pos = to_three(pos)
    q = np.round(pos / 0.0001).astype(np.int16)
    idx = tri.astype(np.uint16 if len(pos) < 65536 else np.uint32)
    # triangles flipped by the y -> -z mirror keep their winding because to_three is a proper rotation
    return {"p": base64.b64encode(q.tobytes()).decode(), "i": base64.b64encode(idx.tobytes()).decode(),
            "i32": idx.dtype == np.uint32, "n": int(len(tri))}


groups = {}  # name -> (color, [vertex arrays], cell)


def add(group, color, verts, cell=0.0012):
    groups.setdefault(group, (color, [], cell))[1].append(verts)


# chassis tub
add("chassis", "grey", load("chassis.stl"), 0.001)
# front encoders: holder + slot sensor on each side wall
for side, hand in ((1, "left"), (-1, "right")):
    o = (HALF_BASE, side * BODY_WALL_Y, 0); a = (0, 0, side * math.pi / 2)
    add("encoder_holders", "blue", place(load(f"encoder_holder_{hand}.stl"), o, a, (MM,) * 3))
    add("encoder_sensors", "sensor", place(load(f"encoder_sensor_{hand}.stl"), o, a, (MM,) * 3), 0.0015)
# RPLIDAR A1, IMU, camera
add("lidar", "dark", place(load("lidar.stl"), (0.08805, -0.0011, 0.1730), (0, 0, math.pi)), 0.0025)
add("imu", "blue", place(load("imu.stl"), (-0.00455, -0.0080, -0.0097), (0, 0, -math.pi / 2)), 0.0008)
add("camera", "green", place(load("camera.stl"), (0.13975, 0, 0.0469)), 0.0008)
# handle: mount, rod, frame, grip at the design pose; four vibration modules
MOUNT = np.array([-0.1172, 0, 0.1393])
add("handle_mount", "dark", place(load("handle_mount.stl"), MOUNT, scale=(MM,) * 3))
add("handle_rod", "aluminium", place(load("handle_rod.stl"), MOUNT, scale=(MM,) * 3), 0.0015)
add("handle_frame", "handle_grey", place(load("handle_frame.stl"), MOUNT, scale=(MM,) * 3), 0.0015)
add("handle_grip", "handle_grip", place(load("handle_grip.stl"), MOUNT, scale=(MM,) * 3))
HAPTIC = {"haptic_front": ((-0.38731, 0, 0.57634), 0), "haptic_back": ((-0.42121, 0, 0.57584), math.pi),
          "haptic_left": ((-0.40431, 0.059, 0.57734), math.pi / 2), "haptic_right": ((-0.40431, -0.059, 0.57734), -math.pi / 2)}
for name, (o, yaw) in HAPTIC.items():
    add(name, "vib_board", place(place(load("vibration_module.stl"), o, (0, 0, yaw), (MM,) * 3), MOUNT), 0.0006)
# wheel parts, in the wheel frame (axle along ROS +Y)
s = WHEEL_R / CAD_WHEEL_R
add("tire", "black", place(load("front_left_wheel.stl"), scale=(s, 1, s)), 0.003)
for side, tag in ((1, "L"), (-1, "R")):
    o = (0, -side * EXT_OFFSET, 0); a = (0, 0, side * math.pi / 2)
    add("extender_" + tag, "orange", place(load("shaft_extender.stl"), o, a, (MM,) * 3), 0.0012)
    add("disc_" + tag, "black", place(load("encoder_disc.stl"), o, a, (MM,) * 3), 0.0012)

out, total = {}, 0
for name, (color, parts, cell) in groups.items():
    p = pack(parts, cell); p["c"] = COLORS[color]; out[name] = p; total += p["n"]
frames = {"wheel_radius": WHEEL_R, "half_track": HALF_TRACK, "half_base": HALF_BASE,
          "grip": [float(x) for x in (MOUNT + np.array([-0.40431, 0, 0.5770]))]}
js = "/* GENERATED by tools/export_urdf_meshes.py from raspberry_pi/ros/rover_description - do not edit. */\n"
js += "const URDF_MESHES = " + json.dumps(out, separators=(",", ":")) + ";\n"
js += "const URDF_FRAMES = " + json.dumps(frames) + ";\n"
OUT.write_text(js, encoding="utf-8")
print(f"{len(out)} meshes, {total} triangles, {OUT.stat().st_size/1024:.0f} KB -> {OUT}")
