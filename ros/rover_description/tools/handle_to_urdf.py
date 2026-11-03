"""Haptic handle -> URDF: meshes plus urdf/handle.urdf.xacro (generated, do not edit by hand).

    python handle_to_urdf.py <folder with build_handle.py> <folder with Grip Top Part A/B.STEP>

build_handle.py is on the cane-design branch (_cane_later/cad/). Needs CadQuery.

Links written:
    handle_mount            the hinge bracket, fixed on the rover's rear top
    handle                  rod + grip + frame, swinging on handle_joint (the M6 pivot, about +Y);
                            joint 0 = the design pose (rod 50 deg above horizontal), the stop bolts
                            allow 35-75 deg
    haptic_{front,back,left,right}
                            one per vibration module, origin on the face of the coin motor where it
                            presses on the 1 mm skin of the grip, +X pointing out through the skin
                            (the direction the user feels it). left/right are the ROBOT's (+Y = left),
                            which is also the user's left when walking behind the rover.

Frames: handle_mount and handle (at joint 0) have base_link's orientation, origin on the pivot axis.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import cadquery as cq
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf

HERE = Path(__file__).resolve().parent
MESHES, URDF = HERE.parent / 'meshes', HERE.parent / 'urdf'

# rover attachment, base_link (mm): bracket 20 mm in from the rear edge, pivot 25 mm above the top
CHASSIS_TOP, CHASSIS_REAR = 114.3, -137.2
PIVOT_BASE = np.array([CHASSIS_REAR + 20, 0.0, CHASSIS_TOP + 25])
# grip frame (X along the bar, Y up, Z toward the rover) -> base_link orientation
H = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)

# Vibration module: coin motor (10 x 2.7) on a small board, as in the photo (KitsGuru / Robu
# 611201 type). The board size is not measured: modelled to fit the grip's pads (20 x 22).
MOTOR_D, MOTOR_T = 10.0, 2.7
BOARD = (19.5, 21.5, 1.6)          # along the wall, up, thickness
# The four thinned pads in the user's grip (measured from Grip Top Part A/B.STEP by ray casting):
# centre of the pad's inner face, and the outward normal, in grip coordinates.
PADS = {
    'front': ((0.0, -1.0, 17.0), (0, 0, 1)),     # Part A front wall, X -10..10, Y -13..11
    'back': ((0.0, -1.5, -16.9), (0, 0, -1)),    # Part B, X -10..10, Y -14..11
    'left': ((59.0, 0.0, 0.0), (1, 0, 0)),       # end wall at grip +X = robot +Y, 22 x 23
    'right': ((-59.0, 0.0, 0.0), (-1, 0, 0)),    # end wall at grip -X = robot -Y
}

# masses for the simulator (g): printed parts from the slicer (supports removed), rod 16 x 1 aluminium
MASS_G = {'grip': 46, 'frame': 100, 'rod': 85, 'modules': 4 * 5}


def export(shape, name, tol=0.2):
    cq.exporters.export(cq.Workplane(obj=shape), str(MESHES / f'{name}.stl'), tolerance=tol, angularTolerance=0.3)


def to_handle(wp, t):
    """Grip-frame shape -> handle frame (base_link orientation, origin on the pivot), mm."""
    s = wp.val() if isinstance(wp, cq.Workplane) else wp
    if isinstance(wp, cq.Workplane) and len(wp.vals()) > 1:
        s = cq.Compound.makeCompound(wp.vals())
    T = gp_Trsf()
    T.SetValues(*H[0], t[0], *H[1], t[1], *H[2], t[2])
    return cq.Shape.cast(BRepBuilderAPI_Transform(s.wrapped, T, True).Shape())


def module_mesh():
    """Module in its own frame: motor face at X = 0, motor and board behind it (X < 0)."""
    m = cq.Solid.makeCylinder(MOTOR_D / 2, MOTOR_T, cq.Vector(-MOTOR_T, 0, 0), cq.Vector(1, 0, 0))
    bw, bh, bt = BOARD
    board = cq.Solid.makeBox(bt, bw, bh, cq.Vector(-MOTOR_T - bt, -bw / 2, -bh / 2))
    header = cq.Solid.makeBox(2.5, 7.6, 2.5, cq.Vector(-MOTOR_T - bt - 2.5, -3.8, -bh / 2 + 1))
    return cq.Compound.makeCompound([m, board, header])


def rpy_from_x(n):
    """Yaw/pitch that turn +X onto n (no roll)."""
    x, y, z = n
    return 0.0, -math.asin(max(-1, min(1, z))), math.atan2(y, x)


def f(v):
    return ' '.join(f'{a:.5g}' for a in v)


def main(hdir, grip_dir):
    sys.path.insert(0, hdir)
    import build_handle as bh
    gA = cq.importers.importStep(str(Path(grip_dir) / 'Grip Top Part A.STEP'))
    gB = cq.importers.importStep(str(Path(grip_dir) / 'Grip Top Part B.STEP')).rotate((0, 0, 0), (0, 1, 0), -90).translate((0, -0.44, -0.07))
    tube, plug, bracket, pivot, _ = bh.rod_parts()
    pv = np.array([pivot.x, pivot.y, pivot.z])
    t = -H @ pv                                      # pivot -> origin
    cap_r, cap_l = bh.caps()
    frame_parts = [bh.frame(), cap_r, cap_l, bh.bail(), bh.pod_lid()]

    grip = cq.Compound.makeCompound([to_handle(gA, t), to_handle(gB, t)])
    frame = cq.Compound.makeCompound([to_handle(p, t) for p in frame_parts])
    rod = cq.Compound.makeCompound([to_handle(tube, t), to_handle(plug, t)])
    export(grip, 'handle_grip', 0.3); export(frame, 'handle_frame', 0.3); export(rod, 'handle_rod', 0.3)
    export(to_handle(bracket, t), 'handle_mount', 0.2)
    export(module_mesh(), 'vibration_module', 0.1)

    # rod collision cylinder: tube from the socket to the plug
    rod_dir = H @ np.array([bh.ROD_DIR.x, bh.ROD_DIR.y, bh.ROD_DIR.z])        # grip -> rover
    t_start = H @ (np.array([*(bh.SOCKET_BASE + bh.ROD_DIR * 6).toTuple()])) + t
    rod_len = bh.TUBE_LEN + 25
    rod_mid = t_start + rod_dir * rod_len / 2
    pitch = math.atan2(rod_dir[0], rod_dir[2])        # cylinder Z axis onto the rod
    gbb = grip.BoundingBox()
    fbb = frame.BoundingBox()

    # modules: pose in the handle frame
    mods = {}
    for k, (p, n) in PADS.items():
        mods[k] = (H @ np.array(p) + t, rpy_from_x(H @ np.array(n)))

    # mass / centre of mass / inertia (point masses + slender rod) for simulators
    cg = lambda b: np.array([(b.xmin + b.xmax) / 2, (b.ymin + b.ymax) / 2, (b.zmin + b.zmax) / 2])
    pts = [(MASS_G['grip'], cg(gbb)), (MASS_G['frame'], cg(fbb)), (MASS_G['rod'], rod_mid),
           (MASS_G['modules'], cg(gbb))]
    m = sum(a for a, _ in pts) / 1000
    com = sum(a * p for a, p in pts) / sum(a for a, _ in pts) / 1000
    I = np.zeros((3, 3))
    for a, p in pts:
        r = p / 1000 - com
        I += a / 1000 * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    L = rod_len / 1000; d = rod_dir
    I += MASS_G['rod'] / 1000 * L ** 2 / 12 * (np.eye(3) - np.outer(d, d))
    I += np.eye(3) * 1e-5

    mm = lambda v: np.asarray(v) / 1000
    lo, hi = math.radians(35 - bh.ROD_ANGLE), math.radians(75 - bh.ROD_ANGLE)
    xml = [f'''<?xml version="1.0"?>
<!-- GENERATED by tools/handle_to_urdf.py from the handle CAD - do not edit by hand.
     Haptic handle: hinge bracket on the rover, the rod and grip on a pivot, and the four
     vibration modules. Included from rover.urdf.xacro. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="haptic_handle" params="joint_type:=revolute">
    <material name="handle_grey"><color rgba="0.55 0.57 0.61 1"/></material>
    <material name="handle_grip"><color rgba="0.85 0.81 0.73 1"/></material>
    <material name="aluminium"> <color rgba="0.75 0.76 0.78 1"/></material>
    <material name="vib_board"> <color rgba="0.08 0.08 0.09 1"/></material>

    <!-- Hinge bracket, bolted to the rover's rear top. Frame on the pivot axis. -->
    <joint name="handle_mount_joint" type="fixed">
      <parent link="base_link"/>
      <child  link="handle_mount"/>
      <origin xyz="{f(mm(PIVOT_BASE))}" rpy="0 0 0"/>
    </joint>
    <link name="handle_mount">
      <visual>
        <geometry><mesh filename="${{mesh_dir}}/handle_mount.stl" scale="${{mm}}"/></geometry>
        <material name="dark"/>
      </visual>
    </link>

    <!-- The M6 pivot. 0 = design pose (rod {bh.ROD_ANGLE:g} deg above horizontal); positive = rod steeper.
         The stop bolts limit it to 35-75 deg. Pass joint_type:=fixed for a simulator that
         should not let the handle fall over. -->
    <joint name="handle_joint" type="${{joint_type}}">
      <parent link="handle_mount"/>
      <child  link="handle"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="{lo:.4f}" upper="{hi:.4f}" effort="20" velocity="2"/>
      <dynamics damping="0.5" friction="0.2"/>
    </joint>

    <link name="handle">
      <visual>
        <geometry><mesh filename="${{mesh_dir}}/handle_rod.stl" scale="${{mm}}"/></geometry>
        <material name="aluminium"/>
      </visual>
      <visual>
        <geometry><mesh filename="${{mesh_dir}}/handle_frame.stl" scale="${{mm}}"/></geometry>
        <material name="handle_grey"/>
      </visual>
      <visual>
        <geometry><mesh filename="${{mesh_dir}}/handle_grip.stl" scale="${{mm}}"/></geometry>
        <material name="handle_grip"/>
      </visual>
      <!-- The rod is what the LiDAR sees behind the robot: keep this collision for self-filtering. -->
      <collision>
        <origin xyz="{f(mm(rod_mid))}" rpy="0 {pitch:.5f} 0"/>
        <geometry><cylinder radius="0.008" length="{rod_len / 1000:.4f}"/></geometry>
      </collision>
      <collision>
        <origin xyz="{f(mm(cg(fbb)))}" rpy="0 0 0"/>
        <geometry><box size="{f(mm([fbb.xlen, fbb.ylen, fbb.zlen]))}"/></geometry>
      </collision>
      <inertial>
        <origin xyz="{f(com)}" rpy="0 0 0"/>
        <mass value="{m:.3f}"/>
        <inertia ixx="{I[0,0]:.3e}" ixy="{I[0,1]:.3e}" ixz="{I[0,2]:.3e}" iyy="{I[1,1]:.3e}" iyz="{I[1,2]:.3e}" izz="{I[2,2]:.3e}"/>
      </inertial>
    </link>
''']
    for k, (p, rpy) in mods.items():
        xml.append(f'''
    <!-- {k} vibration module (driven by PIN_HAP_{k.upper()} in rover_microros/pins.h). Origin: motor face on the grip's 1 mm skin; +X = out through the skin. -->
    <joint name="haptic_{k}_joint" type="fixed">
      <parent link="handle"/>
      <child  link="haptic_{k}"/>
      <origin xyz="{f(mm(p))}" rpy="{f(rpy)}"/>
    </joint>
    <link name="haptic_{k}">
      <visual>
        <geometry><mesh filename="${{mesh_dir}}/vibration_module.stl" scale="${{mm}}"/></geometry>
        <material name="vib_board"/>
      </visual>
    </link>
''')
    xml.append('  </xacro:macro>\n</robot>\n')
    (URDF / 'handle.urdf.xacro').write_text(''.join(xml), encoding='utf-8', newline='\n')

    print('pivot (base_link, m):', mm(PIVOT_BASE))
    print(f'joint limits {lo:.3f} .. {hi:.3f} rad;  mass {m:.3f} kg;  com {com}')
    for k, (p, rpy) in mods.items():
        print(f'haptic_{k:5s} in handle: {f(mm(p))}  rpy {f(rpy)}   -> base_link {f(mm(p + PIVOT_BASE))}')
    print('grip box (handle frame, mm):', [round(v) for v in (gbb.xmin, gbb.xmax, gbb.ymin, gbb.ymax, gbb.zmin, gbb.zmax)])


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
