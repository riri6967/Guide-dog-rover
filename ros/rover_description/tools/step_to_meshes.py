"""Export the rover's CAD assembly as one STL mesh per URDF link.

    python step_to_meshes.py <Assem1.STEP> <out_dir>

Needs OpenCascade's Python bindings (`pip install cadquery-ocp numpy`). This
runs on the Windows machine with the CAD, not on the Pi. The Pi only needs the
STLs it produces, which are committed under meshes/.

Each mesh is written in its own link's frame, in metres, so the URDF can use
it with a zero origin. If the CAD changes, re-run this AND update the poses
below and in urdf/rover.urdf.xacro - the two must agree.

How the CAD maps onto ROS (SolidWorks export, millimetres):

    CAD +Y  = up         -> ROS +Z
    CAD -Z  = forward    -> ROS +X   (the camera end)
    CAD -X  = left       -> ROS +Y

base_link is the centre of the four wheel axles, at axle height.
"""
import math
import struct
import sys
from pathlib import Path

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.collections import Sequence_TDF_Label

# base_link origin in CAD coordinates (mm): midpoint of the four wheel centres.
# Wheel centres are at x 434.6 / 649.2, y 917.8, z 1268.6 / 1415.9.
ORIGIN_MM = np.array([541.9, 917.8, 1342.25])

# CAD -> base_link rotation (rows are ROS x, y, z expressed in CAD axes).
CAD_TO_ROS = np.array([[0, 0, -1],
                       [-1, 0, 0],
                       [0, 1, 0]], dtype=float)

# Link poses relative to base_link: (xyz in metres, rpy in radians).
# Keep these identical to the joint origins in urdf/rover.urdf.xacro.
LINKS = {
    # mesh name    CAD component name prefix   pose
    "chassis":           ("Body",             ((0, 0, 0), (0, 0, 0))),
    "front_left_wheel":  ("Wheel#FL",         ((0.07365, 0.1073, 0), (0, 0, 0))),
    "front_right_wheel": ("Wheel#FR",         ((0.07365, -0.1073, 0), (0, 0, 0))),
    "rear_left_wheel":   ("Wheel#RL",         ((-0.07365, 0.1073, 0), (0, 0, 0))),
    "rear_right_wheel":  ("Wheel#RR",         ((-0.07365, -0.1073, 0), (0, 0, 0))),
    "lidar":             ("12.STEP",          ((0.08805, -0.0011, 0.1730), (0, 0, math.pi))),
    "camera":            ("Cam",              ((0.13975, 0.0, 0.0469), (0, 0, 0))),
    "imu":               ("MPU6050",          ((-0.00455, -0.0080, -0.0097), (0, 0, -math.pi / 2))),
}


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def label_name(lab):
    a = TDataStd_Name()
    return a.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), a) else ""


def load_components(path):
    """Top-level components as [(name, shape in world coordinates)]."""
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != 1:
        sys.exit(f"cannot read {path}")
    reader.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    free = Sequence_TDF_Label()
    st.GetFreeShapes(free)
    kids = Sequence_TDF_Label()
    st.GetComponents_s(free.Value(1), kids)

    out = []
    for i in range(1, kids.Length() + 1):
        k = kids.Value(i)
        ref = TDF_Label()
        st.GetReferredShape_s(k, ref)
        out.append((label_name(ref), st.GetShape_s(ref).Moved(st.GetLocation_s(k))))
    return out


def triangles_mm(shape, deflection, angular=0.5):
    """All triangles of a shape, world coordinates, as an (N, 3, 3) array."""
    BRepMesh_IncrementalMesh(shape, deflection, False, angular, True)
    tris = []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            pts = np.array([[p.X(), p.Y(), p.Z()] for p in
                            (tri.Node(i).Transformed(trsf) for i in range(1, tri.NbNodes() + 1))])
            flip = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                tris.append(pts[[a - 1, c - 1, b - 1] if flip else [a - 1, b - 1, c - 1]])
        ex.Next()
    return np.array(tris)


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"rover_description".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0])
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack("<12fH", *n, *t[0], *t[1], *t[2], 0))


def main(step_path, out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    comps = load_components(step_path)

    # The four wheels are all called "Wheel"; tell them apart by position.
    named = []
    for name, shape in comps:
        if name == "Wheel":
            tris = triangles_mm(shape, 0.5)
            c = CAD_TO_ROS @ (tris.reshape(-1, 3).mean(axis=0) - ORIGIN_MM)
            name = "Wheel#" + ("F" if c[0] > 0 else "R") + ("L" if c[1] > 0 else "R")
        named.append((name, shape))

    for mesh, (prefix, (xyz, rpy)) in LINKS.items():
        shape = next((s for n, s in named if n.startswith(prefix)), None)
        if shape is None:
            sys.exit(f"no CAD component starting with {prefix!r}")
        # Coarse is plenty for RViz and keeps the files small for the Pi.
        tris = triangles_mm(shape, 0.3, 0.8) if mesh in ("camera", "imu") else triangles_mm(shape, 1.0, 1.0)
        base = (tris.reshape(-1, 3) - ORIGIN_MM) @ CAD_TO_ROS.T / 1000.0
        link = (base - np.array(xyz)) @ rpy_matrix(*rpy)
        write_stl(out_dir / f"{mesh}.stl", link.reshape(-1, 3, 3))
        lo, hi = link.min(axis=0), link.max(axis=0)
        print(f"{mesh:18} {len(tris):6d} tris   link-frame bbox "
              f"x[{lo[0]:+.4f},{hi[0]:+.4f}] y[{lo[1]:+.4f},{hi[1]:+.4f}] z[{lo[2]:+.4f},{hi[2]:+.4f}]")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
