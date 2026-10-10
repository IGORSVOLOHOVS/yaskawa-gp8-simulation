#!/usr/bin/env python3
"""Derive the Yaskawa Motoman GP8 link mass properties from the real link geometry.

The seven visual meshes in src/yaskawa_workcell_description/meshes/visual/ are the
upstream ROS-Industrial STL files derived from Yaskawa CAD.  They are closed
triangle meshes, so volume, centroid and the full inertia tensor can be
integrated exactly by tetrahedron decomposition - no box or rod approximation is
needed anywhere in this file.

What this tool does, in order:

  1. Parse each STL (binary and ASCII both handled).
  2. Check the mesh is watertight: the sum of the outward area vectors must
     vanish, and every directed edge must be matched by its reverse.  A mesh
     that fails is reported and NOT integrated.
  3. Integrate volume, centroid and the inertia tensor about the centroid.
  4. Solve for the single effective density that makes the seven link masses sum
     to the published GP8 robot mass of 32 kg (DS-699-H page 2, GP8 column), and
     report the implied fill fraction against aluminium and steel. The published
     mass is disputed - see PUBLISHED_ROBOT_MASS_KG below - and everything scales
     linearly with it, so the tool also reports the single factor that carries
     every mass and inertia to the manual's 35 kg figure.  ASSUMPTION, stated plainly: each link is
     modelled as a solid of uniform effective density, identical for all links.
     The real links are hollow castings with motors, gears and cabling inside,
     so this density is an area-weighted stand-in, not a material property.
  5. Rotate and translate every centroid and inertia tensor out of the mesh
     frame (= the URDF link frame) into the link's standard-DH frame, which is
     what study/dynamics.cpp consumes.  The transform is validated against the
     engine's own forward kinematics at random joint values.
  6. Emit cpp_solver/include/study/gp8_mass_properties.hpp (committed, so the
     build needs neither Python nor the meshes) and
     assets/gp8_mass_properties.json.

Self-verification, all of it mandatory and all of it printed:

  * total mass equals the published 32 kg to 1e-9
  * every inertia tensor is symmetric, positive definite, and satisfies the
     triangle inequality on its principal moments (a tensor that does not is not
     a physical rigid body, and would mean the integration or the frame
     transform is wrong)
  * every centroid lies inside its mesh bounding box
  * an analytic cross-check: a cube and a sphere STL are generated in a
     temporary directory and pushed through the very same code path, then
     compared against the closed-form inertia
  * a datasheet sanity check: the wrist links' inertia about their own axes
     against the published allowable wrist inertias of 0.5 / 0.5 / 0.2 kg m^2

Standard library only: no numpy, no trimesh, no network.  Output is
deterministic - two runs produce byte-identical files.

Usage:
    python tools/compute_link_mass_properties.py            # write the outputs
    python tools/compute_link_mass_properties.py --check     # verify only
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import struct
import sys
import tempfile
from typing import Dict, List, Sequence, Tuple

# ---------------------------------------------------------------------------
# Published figures (Yaskawa Motoman GP8 datasheet / instruction manual)
# ---------------------------------------------------------------------------

# The robot mass is disputed between Yaskawa's own two documents, and the figure
# this project used before was neither of them:
#
#   32 kg  DS-699-H page 2, SPECIFICATIONS row Weight, GP8 column   <- PRIMARY
#   35 kg  HW1484385 page 4, Table 5-1 Approx. Mass (type YR-1-06VX8-F00)
#   34 kg  DS-699-H page 2, same row, GP7 column - the adjacent robot. This is
#          what cpp_solver/include/study/gp8_model.hpp carried as the GP8 mass
#          by mistake, and what an earlier run of this tool fitted to.
#
# The committed numbers are fitted to the 32 kg datasheet figure. Nothing is
# averaged and nothing is reconciled by adjusting a number: the disagreement is
# carried into the generated header, the JSON and docs/GP8_MASS_PROPERTIES.md.
# Mass and inertia are both linear in the density, so refitting to 35 kg is one
# multiplication by MANUAL_SCALE_FACTOR with the centres of mass unchanged.
# Page citations live in assets/gp8_published_specification.json.
PUBLISHED_ROBOT_MASS_KG = 32.0
PUBLISHED_MASS_SOURCE = "DS-699-H page 2, SPECIFICATIONS row Weight, GP8 column"
MANUAL_ROBOT_MASS_KG = 35.0
MANUAL_MASS_SOURCE = "HW1484385 page 4, Table 5-1 Approx. Mass"
GP7_MASS_KG = 34.0
MANUAL_SCALE_FACTOR = MANUAL_ROBOT_MASS_KG / PUBLISHED_ROBOT_MASS_KG
PUBLISHED_PAYLOAD_KG = 8.0
PUBLISHED_REACH_M = 0.727
PUBLISHED_WRIST_ALLOWABLE_INERTIA = {"R": 0.5, "B": 0.5, "T": 0.2}

DENSITY_ALUMINIUM = 2700.0
DENSITY_STEEL = 7850.0

TOOL_NAME = "tools/compute_link_mass_properties.py"

# Mesh order is fixed: base first, then the six moving links.  The axis letter is
# None for the base, which does not move and carries no DH frame.
MESHES: Sequence[Tuple[str, str]] = (
    ("gp8_base_link.stl", ""),
    ("gp8_link_1_s.stl", "S"),
    ("gp8_link_2_l.stl", "L"),
    ("gp8_link_3_u.stl", "U"),
    ("gp8_link_4_r.stl", "R"),
    ("gp8_link_5_b.stl", "B"),
    ("gp8_link_6_t.stl", "T"),
)

# ---------------------------------------------------------------------------
# Kinematic model, duplicated from the two places that already agree on it.
# ---------------------------------------------------------------------------

# cpp_solver/include/study/gp8_model.hpp, GP8_DH: a, alpha, d, theta_offset.
GP8_DH = (
    (0.040, -math.pi / 2.0, 0.330, 0.0),
    (0.345, 0.0, 0.000, -math.pi / 2.0),
    (0.040, -math.pi / 2.0, 0.000, 0.0),
    (0.000, math.pi / 2.0, 0.340, 0.0),
    (0.000, -math.pi / 2.0, 0.000, 0.0),
    (0.000, 0.0, 0.000, 0.0),
)

# src/yaskawa_workcell_description/urdf/gp8_macro.xacro joint origins and axes,
# identical to YaskawaKinematics::forwardKinematicsCached in the engine.
URDF_CHAIN = (
    ((0.00, 0.0, 0.330), "z"),
    ((0.04, 0.0, 0.000), "y"),
    ((0.00, 0.0, 0.345), "y"),
    ((0.34, 0.0, 0.040), "x"),
    ((0.00, 0.0, 0.000), "y"),
    ((0.00, 0.0, 0.000), "x"),
)

# gp8_model.hpp GP8_FLANGE_CORRECTION, needed only to prove the two chains agree.
FLANGE_CORRECTION = ((0.0, 0.0, 1.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0))

Mat3 = Tuple[Tuple[float, float, float], ...]
Vec3 = Tuple[float, float, float]

# ---------------------------------------------------------------------------
# Tiny linear algebra, 3x3 and 4x4, pure Python
# ---------------------------------------------------------------------------

IDENTITY3: Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def mat3_mul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3)
    )


def mat3_transpose(a: Mat3) -> Mat3:
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def mat3_vec(a: Mat3, v: Vec3) -> Vec3:
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def mat3_scale(a: Mat3, s: float) -> Mat3:
    return tuple(tuple(a[i][j] * s for j in range(3)) for i in range(3))


def mat3_add(a: Mat3, b: Mat3) -> Mat3:
    return tuple(tuple(a[i][j] + b[i][j] for j in range(3)) for i in range(3))


def mat3_sub(a: Mat3, b: Mat3) -> Mat3:
    return tuple(tuple(a[i][j] - b[i][j] for j in range(3)) for i in range(3))


def mat3_det(a: Mat3) -> float:
    return (
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    )


def rot_x(t: float) -> Mat3:
    c, s = math.cos(t), math.sin(t)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def rot_y(t: float) -> Mat3:
    c, s = math.cos(t), math.sin(t)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def rot_z(t: float) -> Mat3:
    c, s = math.cos(t), math.sin(t)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


class Transform:
    """A rigid transform, rotation R and translation t, acting as p -> R p + t."""

    __slots__ = ("R", "t")

    def __init__(self, R: Mat3 = IDENTITY3, t: Vec3 = (0.0, 0.0, 0.0)) -> None:
        self.R = R
        self.t = t

    def __mul__(self, other: "Transform") -> "Transform":
        return Transform(
            mat3_mul(self.R, other.R),
            tuple(a + b for a, b in zip(mat3_vec(self.R, other.t), self.t)),
        )

    def apply(self, p: Vec3) -> Vec3:
        return tuple(a + b for a, b in zip(mat3_vec(self.R, p), self.t))

    def inverse(self) -> "Transform":
        Rt = mat3_transpose(self.R)
        return Transform(Rt, tuple(-x for x in mat3_vec(Rt, self.t)))


def dh_transform(params: Tuple[float, float, float, float], q: float) -> Transform:
    a, alpha, d, offset = params
    theta = q + offset
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    R = ((ct, -st * ca, st * sa), (st, ct * ca, -ct * sa), (0.0, sa, ca))
    return Transform(R, (a * ct, a * st, d))


def dh_link_frames(q: Sequence[float]) -> List[Transform]:
    """T_0_i for i = 1..6, the standard-DH chain of gp8_model.hpp."""
    frames: List[Transform] = []
    T = Transform()
    for i in range(6):
        T = T * dh_transform(GP8_DH[i], q[i])
        frames.append(T)
    return frames


def joint_axis_in_dh_frame(index: int) -> Vec3:
    """The axis joint i+1 turns about, written in link i's own DH frame.

    Standard (distal) DH puts the axis of joint i along z_{i-1}, not z_i, so the
    link's own rotation axis seen from its own frame is the third row of that
    frame's rotation, (0, sin alpha_i, cos alpha_i). Getting this backwards is
    the classic way to misreport a wrist inertia by a factor of several.
    """
    alpha = GP8_DH[index][1]
    return (0.0, math.sin(alpha), math.cos(alpha))


def inertia_about_axis(inertia: Mat3, axis: Vec3) -> float:
    """a^T I a, the scalar moment of inertia about the unit direction a."""
    return sum(axis[i] * inertia[i][j] * axis[j] for i in range(3) for j in range(3))


def urdf_link_frames(q: Sequence[float]) -> List[Transform]:
    """T_0_i for i = 1..6 of the URDF / engine chain. These are the mesh frames."""
    axis_rot = {"x": rot_x, "y": rot_y, "z": rot_z}
    frames: List[Transform] = []
    T = Transform()
    for i in range(6):
        offset, axis = URDF_CHAIN[i]
        T = T * Transform(IDENTITY3, offset) * Transform(axis_rot[axis](q[i]))
        frames.append(T)
    return frames


def jacobi_eigenvalues(a: Mat3) -> List[float]:
    """Eigenvalues of a symmetric 3x3 matrix, ascending, by cyclic Jacobi."""
    m = [list(row) for row in a]
    for _ in range(100):
        off = abs(m[0][1]) + abs(m[0][2]) + abs(m[1][2])
        if off < 1e-18 * (1.0 + abs(m[0][0]) + abs(m[1][1]) + abs(m[2][2])):
            break
        for p, q in ((0, 1), (0, 2), (1, 2)):
            if abs(m[p][q]) < 1e-300:
                continue
            theta = (m[q][q] - m[p][p]) / (2.0 * m[p][q])
            t = math.copysign(1.0, theta) / (abs(theta) + math.sqrt(theta * theta + 1.0))
            c = 1.0 / math.sqrt(t * t + 1.0)
            s = t * c
            for k in range(3):
                akp, akq = m[k][p], m[k][q]
                m[k][p] = c * akp - s * akq
                m[k][q] = s * akp + c * akq
            for k in range(3):
                apk, aqk = m[p][k], m[q][k]
                m[p][k] = c * apk - s * aqk
                m[q][k] = s * apk + c * aqk
    return sorted(m[i][i] for i in range(3))


# ---------------------------------------------------------------------------
# STL parsing
# ---------------------------------------------------------------------------


def parse_stl(path: str) -> List[Tuple[Vec3, Vec3, Vec3]]:
    """Read a binary or ASCII STL into a list of (a, b, c) vertex triples."""
    with open(path, "rb") as handle:
        data = handle.read()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + 50 * count:
            return _parse_binary_stl(data, count)
    if data[:5].lower().lstrip() == b"solid" or b"facet normal" in data[:4096]:
        return _parse_ascii_stl(data)
    raise ValueError(f"{path}: neither a binary nor an ASCII STL")


def _parse_binary_stl(data: bytes, count: int) -> List[Tuple[Vec3, Vec3, Vec3]]:
    triangles: List[Tuple[Vec3, Vec3, Vec3]] = []
    for i in range(count):
        v = struct.unpack_from("<12f", data, 84 + 50 * i)
        triangles.append(
            (
                (float(v[3]), float(v[4]), float(v[5])),
                (float(v[6]), float(v[7]), float(v[8])),
                (float(v[9]), float(v[10]), float(v[11])),
            )
        )
    return triangles


def _parse_ascii_stl(data: bytes) -> List[Tuple[Vec3, Vec3, Vec3]]:
    triangles: List[Tuple[Vec3, Vec3, Vec3]] = []
    current: List[Vec3] = []
    for raw in data.decode("ascii", errors="replace").splitlines():
        token = raw.split()
        if not token:
            continue
        if token[0] == "vertex" and len(token) >= 4:
            current.append((float(token[1]), float(token[2]), float(token[3])))
        elif token[0] == "endloop":
            if len(current) != 3:
                raise ValueError(f"ASCII STL facet with {len(current)} vertices")
            triangles.append((current[0], current[1], current[2]))
            current = []
    return triangles


# ---------------------------------------------------------------------------
# Exact integration over a closed triangle mesh
# ---------------------------------------------------------------------------

# Second-moment integral over the canonical tetrahedron {x,y,z >= 0, x+y+z <= 1}:
#   int x_i x_j dV  =  1/60 on the diagonal, 1/120 off it.
CANONICAL_COVARIANCE: Mat3 = (
    (2.0 / 120.0, 1.0 / 120.0, 1.0 / 120.0),
    (1.0 / 120.0, 2.0 / 120.0, 1.0 / 120.0),
    (1.0 / 120.0, 1.0 / 120.0, 2.0 / 120.0),
)


class MeshProperties:
    """Volume, centroid and unit-density inertia of one closed mesh."""

    __slots__ = (
        "name",
        "triangles",
        "volume",
        "centroid",
        "unit_inertia",
        "area",
        "closure_error",
        "closure_relative",
        "unmatched_edges",
        "bbox_min",
        "bbox_max",
    )

    def __init__(self, name: str, triangles: List[Tuple[Vec3, Vec3, Vec3]]) -> None:
        self.name = name
        self.triangles = len(triangles)

        volume = 0.0
        first = [0.0, 0.0, 0.0]
        covariance = [[0.0] * 3 for _ in range(3)]
        area_vector = [0.0, 0.0, 0.0]
        area = 0.0
        lo = [float("inf")] * 3
        hi = [float("-inf")] * 3

        for a, b, c in triangles:
            for p in (a, b, c):
                for k in range(3):
                    if p[k] < lo[k]:
                        lo[k] = p[k]
                    if p[k] > hi[k]:
                        hi[k] = p[k]

            # Outward area vector of the facet, 0.5 * (b-a) x (c-a).
            u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            w = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
            cross = (
                u[1] * w[2] - u[2] * w[1],
                u[2] * w[0] - u[0] * w[2],
                u[0] * w[1] - u[1] * w[0],
            )
            for k in range(3):
                area_vector[k] += 0.5 * cross[k]
            area += 0.5 * math.sqrt(cross[0] ** 2 + cross[1] ** 2 + cross[2] ** 2)

            # The tetrahedron (origin, a, b, c): signed volume det(A)/6 with the
            # rows of A being a, b and c.
            A: Mat3 = (a, b, c)
            det = mat3_det(A)
            volume += det / 6.0

            # First moment: det/6 * (a+b+c)/4, i.e. the tetra's own centroid.
            weight = det / 24.0
            for k in range(3):
                first[k] += weight * (a[k] + b[k] + c[k])

            # Second moments: int x x^T dV = det(A) * A^T C0 A.
            block = mat3_scale(mat3_mul(mat3_transpose(A), mat3_mul(CANONICAL_COVARIANCE, A)), det)
            for i in range(3):
                for j in range(3):
                    covariance[i][j] += block[i][j]

        self.volume = volume
        self.area = area
        self.closure_error = math.sqrt(sum(x * x for x in area_vector))
        self.closure_relative = self.closure_error / area if area > 0.0 else float("inf")
        self.unmatched_edges = _count_unmatched_edges(triangles)
        self.bbox_min = tuple(lo)
        self.bbox_max = tuple(hi)

        if volume <= 0.0:
            raise ValueError(f"{name}: non-positive signed volume {volume!r}; winding is inverted")
        self.centroid = tuple(first[k] / volume for k in range(3))

        # Inertia about the origin from the covariance, then the parallel-axis
        # shift onto the centroid.  Unit density, so mass == volume.
        trace = covariance[0][0] + covariance[1][1] + covariance[2][2]
        origin_inertia = tuple(
            tuple((trace if i == j else 0.0) - covariance[i][j] for j in range(3)) for i in range(3)
        )
        self.unit_inertia = _shift_to_centroid(origin_inertia, volume, self.centroid)


def _count_unmatched_edges(triangles: List[Tuple[Vec3, Vec3, Vec3]]) -> int:
    """Directed edges with no reverse partner. Zero for a watertight manifold."""
    seen: Dict[Tuple[Tuple[int, int, int], Tuple[int, int, int]], int] = {}
    scale = 1e7  # 0.1 micrometre, well below any real mesh feature

    def key(p: Vec3) -> Tuple[int, int, int]:
        return (round(p[0] * scale), round(p[1] * scale), round(p[2] * scale))

    for a, b, c in triangles:
        ka, kb, kc = key(a), key(b), key(c)
        for u, v in ((ka, kb), (kb, kc), (kc, ka)):
            seen[(u, v)] = seen.get((u, v), 0) + 1
    unmatched = 0
    for (u, v), n in seen.items():
        if seen.get((v, u), 0) != n:
            unmatched += 1
    return unmatched


def _shift_to_centroid(inertia_origin: Mat3, mass: float, r: Vec3) -> Mat3:
    """I_com = I_origin - m (|r|^2 I - r r^T)."""
    rr = sum(x * x for x in r)
    shift = tuple(
        tuple(mass * ((rr if i == j else 0.0) - r[i] * r[j]) for j in range(3)) for i in range(3)
    )
    return mat3_sub(inertia_origin, shift)


def rotate_inertia(inertia: Mat3, R: Mat3) -> Mat3:
    """I' = R I R^T, the tensor seen from axes rotated by R."""
    return mat3_mul(R, mat3_mul(inertia, mat3_transpose(R)))


def symmetrise(a: Mat3) -> Mat3:
    return tuple(tuple(0.5 * (a[i][j] + a[j][i]) for j in range(3)) for i in range(3))


# ---------------------------------------------------------------------------
# Analytic cross-check meshes
# ---------------------------------------------------------------------------


def cube_triangles(side: float) -> List[Tuple[Vec3, Vec3, Vec3]]:
    """An axis-aligned cube centred on the origin, outward winding."""
    h = side / 2.0
    corner = [
        (-h, -h, -h),
        (h, -h, -h),
        (h, h, -h),
        (-h, h, -h),
        (-h, -h, h),
        (h, -h, h),
        (h, h, h),
        (-h, h, h),
    ]
    quads = (
        (0, 3, 2, 1),  # z = -h, normal -z
        (4, 5, 6, 7),  # z = +h
        (0, 1, 5, 4),  # y = -h
        (2, 3, 7, 6),  # y = +h
        (0, 4, 7, 3),  # x = -h
        (1, 2, 6, 5),  # x = +h
    )
    tris: List[Tuple[Vec3, Vec3, Vec3]] = []
    for a, b, c, d in quads:
        tris.append((corner[a], corner[b], corner[c]))
        tris.append((corner[a], corner[c], corner[d]))
    return tris


def sphere_triangles(radius: float, subdivisions: int) -> List[Tuple[Vec3, Vec3, Vec3]]:
    """A subdivided icosahedron projected onto the sphere, outward winding."""
    phi = (1.0 + math.sqrt(5.0)) / 2.0
    verts = [
        (-1.0, phi, 0.0),
        (1.0, phi, 0.0),
        (-1.0, -phi, 0.0),
        (1.0, -phi, 0.0),
        (0.0, -1.0, phi),
        (0.0, 1.0, phi),
        (0.0, -1.0, -phi),
        (0.0, 1.0, -phi),
        (phi, 0.0, -1.0),
        (phi, 0.0, 1.0),
        (-phi, 0.0, -1.0),
        (-phi, 0.0, 1.0),
    ]
    faces = [
        (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
        (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
        (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
        (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1),
    ]

    def project(p: Vec3) -> Vec3:
        n = math.sqrt(sum(x * x for x in p))
        return tuple(radius * x / n for x in p)

    tris = [tuple(project(verts[i]) for i in face) for face in faces]
    for _ in range(subdivisions):
        nxt: List[Tuple[Vec3, Vec3, Vec3]] = []
        for a, b, c in tris:
            ab = project(tuple(0.5 * (a[k] + b[k]) for k in range(3)))
            bc = project(tuple(0.5 * (b[k] + c[k]) for k in range(3)))
            ca = project(tuple(0.5 * (c[k] + a[k]) for k in range(3)))
            nxt.extend([(a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)])
        tris = nxt
    return tris


def write_binary_stl(path: str, triangles: List[Tuple[Vec3, Vec3, Vec3]]) -> None:
    with open(path, "wb") as handle:
        handle.write(b"generated by " + TOOL_NAME.encode("ascii"))
        handle.write(b"\0" * (80 - 13 - len(TOOL_NAME)))
        handle.write(struct.pack("<I", len(triangles)))
        for a, b, c in triangles:
            u = tuple(b[k] - a[k] for k in range(3))
            w = tuple(c[k] - a[k] for k in range(3))
            n = (
                u[1] * w[2] - u[2] * w[1],
                u[2] * w[0] - u[0] * w[2],
                u[0] * w[1] - u[1] * w[0],
            )
            length = math.sqrt(sum(x * x for x in n)) or 1.0
            handle.write(struct.pack("<3f", *(x / length for x in n)))
            for p in (a, b, c):
                handle.write(struct.pack("<3f", *p))
            handle.write(struct.pack("<H", 0))


def write_ascii_stl(path: str, triangles: List[Tuple[Vec3, Vec3, Vec3]]) -> None:
    with open(path, "w", encoding="ascii", newline="\n") as handle:
        handle.write("solid crosscheck\n")
        for a, b, c in triangles:
            handle.write("  facet normal 0 0 0\n    outer loop\n")
            for p in (a, b, c):
                handle.write("      vertex %.9e %.9e %.9e\n" % p)
            handle.write("    endloop\n  endfacet\n")
        handle.write("endsolid crosscheck\n")


# ---------------------------------------------------------------------------
# Formatting helpers - deterministic, locale independent
# ---------------------------------------------------------------------------


def clean(x: float) -> float:
    """Collapse -0.0 and integration dust to exactly 0.0 so output is stable."""
    return 0.0 if abs(x) < 1e-15 else x


def fmt(x: float) -> str:
    return "%.12e" % clean(x)


def fmt_short(x: float) -> str:
    return "%.6f" % clean(x)


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------


class LinkResult:
    __slots__ = (
        "mesh",
        "axis",
        "index",
        "props",
        "mass",
        "com_mesh",
        "com_dh",
        "inertia_mesh",
        "inertia_dh",
        "sha256",
    )


def run_crosscheck() -> List[str]:
    """Push a cube and a sphere of known inertia through the same code path."""
    lines: List[str] = []
    side = 0.2
    radius = 0.1
    subdivisions = 4
    with tempfile.TemporaryDirectory(prefix="gp8_massprops_") as tmp:
        cube_ascii = os.path.join(tmp, "cube_ascii.stl")
        cube_binary = os.path.join(tmp, "cube_binary.stl")
        sphere_binary = os.path.join(tmp, "sphere_binary.stl")
        write_ascii_stl(cube_ascii, cube_triangles(side))
        write_binary_stl(cube_binary, cube_triangles(side))
        write_binary_stl(sphere_binary, sphere_triangles(radius, subdivisions))

        # A cube of side L, unit density: V = L^3, I_xx = I_yy = I_zz = m L^2 / 6,
        # all products of inertia zero, centroid at the origin.
        cube_volume = side ** 3
        cube_inertia = cube_volume * side * side / 6.0
        for path, tol in ((cube_ascii, 1.0e-13), (cube_binary, 1.0e-6)):
            cube = MeshProperties(os.path.basename(path), parse_stl(path))
            vol_err = abs(cube.volume - cube_volume) / cube_volume
            diag_err = max(
                abs(cube.unit_inertia[k][k] - cube_inertia) / cube_inertia for k in range(3)
            )
            off_err = max(abs(cube.unit_inertia[i][j]) for i in range(3) for j in range(3) if i != j)
            com_err = max(abs(c) for c in cube.centroid)
            lines.append(
                "  cube %.2f m side, %-13s %5d facets: volume rel err %.3e, "
                "diagonal inertia rel err %.3e, worst product of inertia %.3e, "
                "centroid err %.3e m, closure %.3e"
                % (
                    side,
                    os.path.basename(path) + ",",
                    cube.triangles,
                    vol_err,
                    diag_err,
                    off_err,
                    com_err,
                    cube.closure_error,
                )
            )
            assert vol_err < tol, (path, vol_err)
            assert diag_err < tol, (path, diag_err)
            assert off_err < tol * cube_inertia + 1e-18, (path, off_err)
            assert com_err < 1e-9, (path, com_err)

        # A solid sphere of radius R, unit density: V = 4/3 pi R^3,
        # I = 2/5 m R^2 about every axis through the centre. The icosphere is
        # inscribed, so it is smaller than the sphere by O(edge^2); running three
        # subdivision levels shows that error falling as 4^-level, which is the
        # proof that the residual is the polyhedron and not the integrator.
        sphere_volume = 4.0 / 3.0 * math.pi * radius ** 3
        sphere_inertia = 0.4 * sphere_volume * radius * radius
        previous_vol_err = None
        for level in (2, 3, subdivisions):
            path = os.path.join(tmp, "sphere_%d.stl" % level)
            write_binary_stl(path, sphere_triangles(radius, level))
            sphere = MeshProperties(os.path.basename(path), parse_stl(path))
            vol_err = abs(sphere.volume - sphere_volume) / sphere_volume
            diag_err = max(
                abs(sphere.unit_inertia[k][k] - sphere_inertia) / sphere_inertia
                for k in range(3)
            )
            com_err = max(abs(c) for c in sphere.centroid)
            ratio = "" if previous_vol_err is None else " (%.2fx finer than the level before)" % (
                previous_vol_err / vol_err
            )
            lines.append(
                "  sphere %.2f m radius, icosphere level %d, %6d facets: volume rel err "
                "%.3e, diagonal inertia rel err %.3e, centroid err %.3e m, closure %.3e%s"
                % (
                    radius,
                    level,
                    sphere.triangles,
                    vol_err,
                    diag_err,
                    com_err,
                    sphere.closure_error,
                    ratio,
                )
            )
            if previous_vol_err is not None:
                assert 3.5 < previous_vol_err / vol_err < 4.5, previous_vol_err / vol_err
            previous_vol_err = vol_err
            assert com_err < 1e-9, com_err
        assert previous_vol_err < 3.0e-3, previous_vol_err

        lines.append(
            "  Reading these honestly: the ASCII cube agrees to 1e-13, the integrator's"
        )
        lines.append(
            "  own accuracy - tetrahedron decomposition is exact for a polyhedron. The"
        )
        lines.append(
            "  binary cube is held to ~1e-7 by the float32 vertices that the binary STL"
        )
        lines.append(
            "  format stores, which is exactly the precision the GP8 meshes carry. The"
        )
        lines.append(
            "  sphere error is the inscribed icosphere being smaller than the sphere it"
        )
        lines.append(
            "  approximates: it divides by four for every subdivision, as the printed"
        )
        lines.append(
            "  ratios show, so it is geometry and not integration error."
        )
    return lines


def verify_frame_transform(dh_to_mesh: List[Transform]) -> List[str]:
    """The mesh frame sits rigidly in the DH frame: prove the offset is constant."""
    lines: List[str] = []
    worst = 0.0
    # A spread of joint values, hard coded so the check is reproducible.
    samples = (
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        (0.7, -0.4, 1.1, -2.0, 0.9, 3.0),
        (-2.9, 2.5, -1.2, 3.3, -2.3, -6.2),
        (1.234, 0.567, -0.891, 2.345, -1.678, 4.901),
    )
    for q in samples:
        dh = dh_link_frames(q)
        urdf = urdf_link_frames(q)
        for i in range(6):
            offset = dh[i].inverse() * urdf[i]
            for r in range(3):
                worst = max(worst, abs(offset.t[r] - dh_to_mesh[i].t[r]))
                for c in range(3):
                    worst = max(worst, abs(offset.R[r][c] - dh_to_mesh[i].R[r][c]))
    lines.append(
        "  mesh-frame-in-DH-frame offset is constant over 4 joint configurations: "
        "worst deviation %.3e" % worst
    )
    assert worst < 1e-12, worst

    # And the two chains describe the same robot, flange correction included.
    worst_fk = 0.0
    for q in samples:
        dh_flange = dh_link_frames(q)[5] * Transform(FLANGE_CORRECTION)
        urdf_flange = urdf_link_frames(q)[5]
        for r in range(3):
            worst_fk = max(worst_fk, abs(dh_flange.t[r] - urdf_flange.t[r]))
            for c in range(3):
                worst_fk = max(worst_fk, abs(dh_flange.R[r][c] - urdf_flange.R[r][c]))
    lines.append(
        "  DH chain x flange correction == URDF/engine forward kinematics: "
        "worst deviation %.3e" % worst_fk
    )
    assert worst_fk < 1e-12, worst_fk
    return lines


def main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--repo",
        default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        help="repository root (default: the parent of tools/)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="run every check and compare the outputs, but write nothing",
    )
    args = parser.parse_args(list(argv))

    repo = args.repo
    mesh_dir = os.path.join(repo, "src", "yaskawa_workcell_description", "meshes", "visual")
    header_path = os.path.join(repo, "cpp_solver", "include", "study", "gp8_mass_properties.hpp")
    json_path = os.path.join(repo, "assets", "gp8_mass_properties.json")

    print("Yaskawa Motoman GP8 link mass properties, integrated from the CAD meshes")
    print("=" * 78)

    # ---------------------------------------------------------------- integrate
    results: List[LinkResult] = []
    print("\n[1] mesh integration (unit density), mesh frame = URDF link frame")
    print(
        "    %-22s %7s %12s %11s %10s"
        % ("mesh", "facets", "volume [m^3]", "closure", "open edges")
    )
    for index, (mesh_name, axis) in enumerate(MESHES):
        path = os.path.join(mesh_dir, mesh_name)
        if not os.path.isfile(path):
            print("FATAL: missing mesh %s" % path)
            return 2
        with open(path, "rb") as handle:
            digest = hashlib.sha256(handle.read()).hexdigest()
        props = MeshProperties(mesh_name, parse_stl(path))

        watertight = props.unmatched_edges == 0 and props.closure_relative < 1e-9
        print(
            "    %-22s %7d %12.9f %11.3e %10d%s"
            % (
                mesh_name,
                props.triangles,
                props.volume,
                props.closure_error,
                props.unmatched_edges,
                "" if watertight else "   <-- NOT WATERTIGHT",
            )
        )
        if not watertight:
            print(
                "FATAL: %s is not a closed surface (%d unmatched directed edges, "
                "relative closure error %.3e). Refusing to integrate it: the volume "
                "and inertia of an open surface are not defined."
                % (mesh_name, props.unmatched_edges, props.closure_relative)
            )
            return 2

        result = LinkResult()
        result.mesh = mesh_name
        result.axis = axis
        result.index = index - 1  # -1 for the base, 0..5 for the moving links
        result.props = props
        result.sha256 = digest
        results.append(result)

    total_volume = sum(r.props.volume for r in results)

    # ------------------------------------------------------------- solve density
    density = PUBLISHED_ROBOT_MASS_KG / total_volume
    fill_aluminium = density / DENSITY_ALUMINIUM
    fill_steel = density / DENSITY_STEEL

    print("\n[2] effective density, solved so the seven link masses sum to the datasheet mass")
    print("    total enclosed mesh volume            %.9f m^3" % total_volume)
    print("    published GP8 robot mass              %.3f kg" % PUBLISHED_ROBOT_MASS_KG)
    print("      source                              %s" % PUBLISHED_MASS_SOURCE)
    print("    solved effective density              %.3f kg/m^3" % density)
    print("    implied fill fraction vs aluminium    %.4f  (2700 kg/m^3)" % fill_aluminium)
    print("    implied fill fraction vs steel        %.4f  (7850 kg/m^3)" % fill_steel)
    print("    DISPUTED PUBLISHED MASS. Yaskawa\'s own two documents disagree:")
    print("      %5.1f kg  %s   <- fitted to this"
          % (PUBLISHED_ROBOT_MASS_KG, PUBLISHED_MASS_SOURCE))
    print("      %5.1f kg  %s" % (MANUAL_ROBOT_MASS_KG, MANUAL_MASS_SOURCE))
    print("      %5.1f kg  DS-699-H page 2, same row, GP7 column - the adjacent robot,"
          % GP7_MASS_KG)
    print("              which this project previously used as the GP8 mass by mistake.")
    print("    Mass and inertia are both linear in the density, so refitting to the")
    print("    manual %.1f kg is one multiplication of every mass and every inertia"
          % MANUAL_ROBOT_MASS_KG)
    print("    component by %.6f, centres of mass unchanged. That would give a"
          % MANUAL_SCALE_FACTOR)
    print("    density of %.3f kg/m^3 and an aluminium fill fraction of %.4f."
          % (MANUAL_ROBOT_MASS_KG / total_volume,
             MANUAL_ROBOT_MASS_KG / total_volume / DENSITY_ALUMINIUM))
    print("    ASSUMPTION: each link is a solid of uniform effective density, the same")
    print("    density for every link. The real links are hollow castings holding")
    print("    motors, gearboxes and cabling, so this is a stand-in for the true")
    print("    mass distribution, not a material property.")
    absurd = fill_aluminium > 1.0 or fill_aluminium < 0.1
    if absurd:
        print("    !!! WARNING: the implied aluminium fill fraction %.4f is outside" % fill_aluminium)
        print("    !!! the physically sensible band [0.1, 1.0]. Either the meshes do not")
        print("    !!! enclose the volume they appear to, or the published mass does not")
        print("    !!! belong to this geometry. Treat every number below as suspect.")
    else:
        print(
            "    The aluminium fill fraction %.4f is inside the sensible band [0.1, 1.0]:"
            % fill_aluminium
        )
        print("    consistent with cast-aluminium links that are mostly hollow.")

    # --------------------------------------------------- mesh frame -> DH frame
    dh_zero = dh_link_frames([0.0] * 6)
    urdf_zero = urdf_link_frames([0.0] * 6)
    dh_to_mesh = [dh_zero[i].inverse() * urdf_zero[i] for i in range(6)]

    print("\n[3] frame transform verification")
    for line in verify_frame_transform(dh_to_mesh):
        print(line)

    for r in results:
        r.mass = density * r.props.volume
        r.com_mesh = tuple(clean(x) for x in r.props.centroid)
        r.inertia_mesh = symmetrise(mat3_scale(r.props.unit_inertia, density))
        if r.index < 0:
            # The base does not move: its DH frame is the base frame itself.
            r.com_dh = r.com_mesh
            r.inertia_dh = r.inertia_mesh
        else:
            X = dh_to_mesh[r.index]
            r.com_dh = tuple(clean(x) for x in X.apply(r.props.centroid))
            r.inertia_dh = symmetrise(rotate_inertia(r.inertia_mesh, X.R))

    # ------------------------------------------------------------- self-checks
    print("\n[4] self-checks")
    mass_sum = sum(r.mass for r in results)
    mass_error = abs(mass_sum - PUBLISHED_ROBOT_MASS_KG)
    print("    sum of the seven masses               %.15f kg (error %.3e)" % (mass_sum, mass_error))
    assert mass_error < 1e-9, mass_error

    worst_asym = 0.0
    worst_eig = float("inf")
    worst_triangle = float("inf")
    for r in results:
        for frame_name, I in (("mesh", r.inertia_mesh), ("DH", r.inertia_dh)):
            for i in range(3):
                for j in range(3):
                    worst_asym = max(worst_asym, abs(I[i][j] - I[j][i]))
            eig = jacobi_eigenvalues(I)
            worst_eig = min(worst_eig, eig[0])
            margins = (
                eig[0] + eig[1] - eig[2],
                eig[0] + eig[2] - eig[1],
                eig[1] + eig[2] - eig[0],
            )
            worst_triangle = min(worst_triangle, min(margins) / eig[2])
            assert eig[0] > 0.0, (r.mesh, frame_name, eig)
            assert min(margins) > -1e-12 * eig[2], (r.mesh, frame_name, eig)
    print("    worst inertia asymmetry |I_ij - I_ji|  %.3e kg m^2" % worst_asym)
    print("    smallest principal moment              %.6e kg m^2 (> 0: positive definite)"
          % worst_eig)
    print("    tightest triangle-inequality margin    %.4f of the largest moment"
          % worst_triangle)

    worst_bbox = 0.0
    for r in results:
        for k in range(3):
            lo, hi = r.props.bbox_min[k], r.props.bbox_max[k]
            c = r.props.centroid[k]
            worst_bbox = max(worst_bbox, lo - c, c - hi)
        assert all(
            r.props.bbox_min[k] <= r.props.centroid[k] <= r.props.bbox_max[k] for k in range(3)
        ), r.mesh
    print("    centroid-outside-bbox worst excursion  %.3e m (<= 0: all inside)" % worst_bbox)

    print("\n[5] analytic cross-check of the integrator")
    for line in run_crosscheck():
        print(line)

    # ------------------------------------------------- datasheet sanity checks
    print("\n[6] datasheet sanity check: wrist link inertia about its own joint axis")
    print("    Standard (distal) DH puts the axis of joint i along z_{i-1}, so a link's")
    print("    own rotation axis seen from its own DH frame is (0, sin alpha_i, cos")
    print("    alpha_i) - y_4 for R, -y_5 for B, z_6 for T. The moment reported here is")
    print("    a^T I a about exactly that direction, through the link's centre of mass.")
    print("    Yaskawa publishes the allowable *payload* moment of inertia at the wrist,")
    print("    which bounds what the drive is rated to swing; the bare casting's own")
    print("    inertia is a different quantity and should sit below it.")
    print("    %-5s %20s %20s %10s"
          % ("axis", "link I_axis [kg m^2]", "allowable [kg m^2]", "ratio"))
    for r in results:
        if r.axis not in PUBLISHED_WRIST_ALLOWABLE_INERTIA:
            continue
        allowable = PUBLISHED_WRIST_ALLOWABLE_INERTIA[r.axis]
        own = inertia_about_axis(r.inertia_dh, joint_axis_in_dh_frame(r.index))
        print("    %-5s %20.6f %20.3f %10.4f" % (r.axis, own, allowable, own / allowable))
    print("    Honest reading: R comes out at a few percent of its rating and B and T at")
    print("    well under one percent. That is the expected shape - the rating has to")
    print("    cover an 8 kg tool swung at arm's length, which dwarfs a 0.68 kg B casting")
    print("    turning about its own centre - but it is also an unflattering check: it")
    print("    only shows the numbers are not absurd, it cannot confirm them. A ratio")
    print("    above 1 would have meant the link cannot move its own mass inside its")
    print("    own rating, i.e. that the integration or the frame transform was wrong.")

    # ---------------------------------------------------------------- emit files
    header = render_header(results, density, total_volume, fill_aluminium, fill_steel)
    payload = render_json(results, density, total_volume, fill_aluminium, fill_steel)

    print("\n[7] outputs")
    for path, text in ((header_path, header), (json_path, payload)):
        existing = None
        if os.path.isfile(path):
            with open(path, "r", encoding="utf-8", newline="") as handle:
                existing = handle.read()
        if args.check:
            state = "up to date" if existing == text else "DIFFERS from the generated content"
            print("    %-58s %s" % (os.path.relpath(path, repo).replace("\\", "/"), state))
            if existing != text:
                return 1
            continue
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="\n") as handle:
            handle.write(text)
        print(
            "    %-58s %s (%d bytes)"
            % (
                os.path.relpath(path, repo).replace("\\", "/"),
                "unchanged" if existing == text else "written",
                len(text),
            )
        )

    print("\n[8] per-link summary (DH frame, about the link centre of mass)")
    print(
        "    %-6s %-22s %8s %28s %10s"
        % ("axis", "mesh", "mass[kg]", "com in DH frame [m]", "I_axis")
    )
    for r in results:
        print(
            "    %-6s %-22s %8.4f  (%8.5f %8.5f %8.5f) %10.6f"
            % (
                r.axis or "base",
                r.mesh,
                r.mass,
                r.com_dh[0],
                r.com_dh[1],
                r.com_dh[2],
                inertia_about_axis(r.inertia_dh, joint_axis_in_dh_frame(r.index))
                if r.index >= 0
                else r.inertia_dh[2][2],
            )
        )
    moving = sum(r.mass for r in results if r.index >= 0)
    print(
        "    six moving links %.4f kg, non-moving base %.4f kg, total %.4f kg"
        % (moving, results[0].mass, mass_sum)
    )
    print("\nAll checks passed.")
    return 0


def render_header(
    results: List[LinkResult],
    density: float,
    total_volume: float,
    fill_aluminium: float,
    fill_steel: float,
) -> str:
    out: List[str] = []
    w = out.append
    w("#ifndef YASKAWA_STUDY_GP8_MASS_PROPERTIES_HPP")
    w("#define YASKAWA_STUDY_GP8_MASS_PROPERTIES_HPP")
    w("")
    w("// GENERATED FILE - do not edit by hand.")
    w("//")
    w("// Generator : %s" % TOOL_NAME)
    w("// Regenerate: python %s" % TOOL_NAME)
    w("// Verify    : python %s --check" % TOOL_NAME)
    w("//")
    w("// Every number below is integrated from the Yaskawa CAD geometry shipped as the")
    w("// upstream ROS-Industrial STL meshes, by exact tetrahedron decomposition of the")
    w("// closed triangle surface: signed volume, first moments, second moments, then a")
    w("// parallel-axis shift onto the centroid. No box, rod or disc approximation is")
    w("// involved. The tensors are then rotated out of the mesh frame (which is the")
    w("// URDF link frame) into each link's standard-DH frame, the frame the dynamics")
    w("// module works in.")
    w("//")
    w("// FRAME CONVENTION. Standard (distal) DH: frame i sits at the origin of joint")
    w("// i+1 and the axis of joint i is z_{i-1}, NOT z_i. So the com and tensor below")
    w("// are in frame i, and the moment of link i about its own rotation axis is")
    w("// a^T I a with a = (0, sin alpha_i, cos alpha_i) - it is not simply izz. The")
    w("// control module's joint_side_inertia() already takes the axis from")
    w("// frames[i-1].col(2), which is the same thing done correctly.")
    w("//")
    w("// DENSITY. The links are hollow castings, so raw mesh volume times a solid")
    w("// density would overestimate the mass badly. One effective density is solved")
    w("// instead, such that the seven link masses sum to the published robot mass:")
    w("//")
    w("//     total enclosed mesh volume    %.9f m^3" % total_volume)
    w("//     published GP8 robot mass      %.3f kg   (DATASHEET)" % PUBLISHED_ROBOT_MASS_KG)
    w("//     solved effective density      %.4f kg/m^3" % density)
    w("//     fill fraction vs aluminium    %.4f   (2700 kg/m^3)" % fill_aluminium)
    w("//     fill fraction vs steel        %.4f   (7850 kg/m^3)" % fill_steel)
    w("//")
    w("// DISPUTED PUBLISHED MASS. Yaskawa\'s own two documents do not agree on what a")
    w("// GP8 weighs, and the figure this project carried before was from neither of")
    w("// them. All three, with their page citations:")
    w("//")
    w("//     %5.1f kg  %s" % (PUBLISHED_ROBOT_MASS_KG, PUBLISHED_MASS_SOURCE))
    w("//              -> the numbers in this file are fitted to THIS one.")
    w("//     %5.1f kg  %s" % (MANUAL_ROBOT_MASS_KG, MANUAL_MASS_SOURCE))
    w("//     %5.1f kg  DS-699-H page 2, same row, GP7 column - the adjacent robot."
      % GP7_MASS_KG)
    w("//              gp8_model.hpp carried this as the GP8 mass by mistake, and an")
    w("//              earlier generation of this file was fitted to it.")
    w("//")
    w("// Nothing is averaged and nothing is reconciled by nudging a number. Mass and")
    w("// inertia are both linear in the density, so this whole table refitted to the")
    w("// manual %.1f kg is this file multiplied by MANUAL_MASS_SCALE_FACTOR ="
      % MANUAL_ROBOT_MASS_KG)
    w("// %.9f - every mass and every inertia component, centres of mass unchanged."
      % MANUAL_SCALE_FACTOR)
    w("// That is why no second table is emitted. Page citations for all three figures")
    w("// are in assets/gp8_published_specification.json.")
    w("//")
    w("// ASSUMPTION, stated plainly: each link is modelled as a solid of uniform")
    w("// effective density, the same density for every link. That is a model of the")
    w("// mass distribution, not a measurement of it: the true distribution is")
    w("// dominated by the motor and gearbox at each joint, which the outer casting")
    w("// geometry cannot know about. What IS exact here is the geometry - the volume,")
    w("// centroid and shape of the inertia tensor of the real castings - and the")
    w("// total, which is pinned to the datasheet.")
    w("//")
    w("// Source meshes, sha256 of the file as committed:")
    for r in results:
        w("//     %-22s %s" % (r.mesh, r.sha256))
    w("//")
    w("// Self-checks the generator runs and refuses to emit without: total mass equal")
    w("// to the published mass to 1e-9; every tensor symmetric, positive definite and")
    w("// obeying the triangle inequality on its principal moments; every centroid")
    w("// inside its mesh bounding box; the mesh-to-DH transform constant over joint")
    w("// space; and an analytic cross-check of the integrator against a cube and a")
    w("// sphere of known closed-form inertia.")
    w("")
    w("#include <array>")
    w("#include <cstddef>")
    w("")
    w("namespace yaskawa::study::massprops {")
    w("")
    w("// The published mass the density was fitted to [kg]: the GP8 datasheet figure.")
    w("inline constexpr double FITTED_TOTAL_MASS_KG = %s;" % fmt(PUBLISHED_ROBOT_MASS_KG))
    w("")
    w("// The competing figure from Yaskawa's instruction manual [kg], and the single")
    w("// factor that carries every mass and every inertia component in this file onto")
    w("// it. Centres of mass do not scale. Provided so a module can report the spread")
    w("// without a second table existing to drift out of step with this one.")
    w("inline constexpr double MANUAL_TOTAL_MASS_KG = %s;" % fmt(MANUAL_ROBOT_MASS_KG))
    w("inline constexpr double MANUAL_MASS_SCALE_FACTOR = %s;" % fmt(MANUAL_SCALE_FACTOR))
    w("")
    w("// The GP7 column figure [kg] this project used as the GP8 mass by mistake. Named")
    w("// so a reader who meets it in an older commit knows what it was.")
    w("inline constexpr double GP7_COLUMN_MASS_KG = %s;" % fmt(GP7_MASS_KG))
    w("")
    w("// Sum of the enclosed volumes of the seven meshes [m^3].")
    w("inline constexpr double TOTAL_MESH_VOLUME_M3 = %s;" % fmt(total_volume))
    w("")
    w("// The one effective density that reproduces FITTED_TOTAL_MASS_KG [kg/m^3].")
    w("inline constexpr double EFFECTIVE_DENSITY_KG_M3 = %s;" % fmt(density))
    w("")
    w("// Implied fill fractions of that density against solid metal.")
    w("inline constexpr double FILL_FRACTION_ALUMINIUM = %s;" % fmt(fill_aluminium))
    w("inline constexpr double FILL_FRACTION_STEEL = %s;" % fmt(fill_steel))
    w("")
    w("// Mass properties of one link, integrated from one mesh.")
    w("struct LinkMassProperties {")
    w("    const char* mesh;              // the STL the numbers were integrated from")
    w("    const char* axis;              // Yaskawa axis letter, \"base\" for the pedestal")
    w("    double volume;                 // [m^3] enclosed mesh volume")
    w("    double mass;                   // [kg]  volume * EFFECTIVE_DENSITY_KG_M3")
    w("    std::array<double, 3> com;     // [m]   centre of mass in the link's DH frame")
    w("    double ixx;                    // [kg m^2] about the com, DH frame axes")
    w("    double iyy;")
    w("    double izz;")
    w("    double ixy;")
    w("    double ixz;")
    w("    double iyz;")
    w("};")
    w("")
    w("// The six moving links, index 0..5 = axes S, L, U, R, B, T, each expressed in")
    w("// that link's own standard-DH frame - see the FRAME CONVENTION note above for")
    w("// which direction that link actually rotates about.")
    w("inline constexpr std::array<LinkMassProperties, 6> GP8_MOVING_LINKS = {{")
    for r in results:
        if r.index < 0:
            continue
        w("    {")
        w('        "%s", "%s",' % (r.mesh, r.axis))
        w("        %s,  // volume [m^3], integrated from %s" % (fmt(r.props.volume), r.mesh))
        w("        %s,  // mass [kg] = volume * effective density" % fmt(r.mass))
        w("        {%s, %s, %s},  // com [m], DH frame %d, from %s"
          % (fmt(r.com_dh[0]), fmt(r.com_dh[1]), fmt(r.com_dh[2]), r.index + 1, r.mesh))
        w("        %s,  // ixx [kg m^2] about com, DH frame %d, from %s"
          % (fmt(r.inertia_dh[0][0]), r.index + 1, r.mesh))
        w("        %s,  // iyy, from %s" % (fmt(r.inertia_dh[1][1]), r.mesh))
        w("        %s,  // izz, from %s" % (fmt(r.inertia_dh[2][2]), r.mesh))
        w("        %s,  // ixy, from %s" % (fmt(r.inertia_dh[0][1]), r.mesh))
        w("        %s,  // ixz, from %s" % (fmt(r.inertia_dh[0][2]), r.mesh))
        w("        %s,  // iyz, from %s" % (fmt(r.inertia_dh[1][2]), r.mesh))
        w("    },")
    w("}};")
    w("")
    base = results[0]
    w("// The pedestal. It does not move, so it carries no DH frame: its centre of mass")
    w("// and inertia are given in the base frame, which is also its mesh frame. It is")
    w("// part of the %.0f kg the density was fitted to, and no part of the moving"
      % PUBLISHED_ROBOT_MASS_KG)
    w("// chain, which is why the six masses above sum to less than the robot mass.")
    w("inline constexpr LinkMassProperties GP8_BASE_LINK = {")
    w('    "%s", "base",' % base.mesh)
    w("    %s,  // volume [m^3]" % fmt(base.props.volume))
    w("    %s,  // mass [kg]" % fmt(base.mass))
    w("    {%s, %s, %s},  // com [m], base frame"
      % (fmt(base.com_dh[0]), fmt(base.com_dh[1]), fmt(base.com_dh[2])))
    w("    %s,  // ixx [kg m^2] about com, base frame axes" % fmt(base.inertia_dh[0][0]))
    w("    %s,  // iyy" % fmt(base.inertia_dh[1][1]))
    w("    %s,  // izz" % fmt(base.inertia_dh[2][2]))
    w("    %s,  // ixy" % fmt(base.inertia_dh[0][1]))
    w("    %s,  // ixz" % fmt(base.inertia_dh[0][2]))
    w("    %s,  // iyz" % fmt(base.inertia_dh[1][2]))
    w("};")
    w("")
    w("// The six independent components expanded into a row-major 3x3 tensor.")
    w("[[nodiscard]] constexpr std::array<double, 9> inertia_matrix(")
    w("    const LinkMassProperties& p) noexcept {")
    w("    return {p.ixx, p.ixy, p.ixz,")
    w("            p.ixy, p.iyy, p.iyz,")
    w("            p.ixz, p.iyz, p.izz};")
    w("}")
    w("")
    w("[[nodiscard]] constexpr double moving_mass_sum() noexcept {")
    w("    double sum = 0.0;")
    w("    for (std::size_t i = 0; i < GP8_MOVING_LINKS.size(); ++i) {")
    w("        sum += GP8_MOVING_LINKS[i].mass;")
    w("    }")
    w("    return sum;")
    w("}")
    w("")
    w("[[nodiscard]] constexpr double fitted_mass_sum() noexcept {")
    w("    return moving_mass_sum() + GP8_BASE_LINK.mass;")
    w("}")
    w("")
    w("// The fit the generator solved for, re-checked by the compiler.")
    w("static_assert(fitted_mass_sum() > FITTED_TOTAL_MASS_KG - 1.0e-9 &&")
    w("                  fitted_mass_sum() < FITTED_TOTAL_MASS_KG + 1.0e-9,")
    w("              \"the seven integrated link masses must sum to the published mass\");")
    w("static_assert(moving_mass_sum() < FITTED_TOTAL_MASS_KG,")
    w("              \"the moving chain cannot outweigh the whole robot\");")
    w("")
    w("}  // namespace yaskawa::study::massprops")
    w("")
    w("#endif  // YASKAWA_STUDY_GP8_MASS_PROPERTIES_HPP")
    return "\n".join(out) + "\n"


def render_json(
    results: List[LinkResult],
    density: float,
    total_volume: float,
    fill_aluminium: float,
    fill_steel: float,
) -> str:
    def link_entry(r: LinkResult) -> dict:
        return {
            "mesh": r.mesh,
            "axis": r.axis or "base",
            "dh_index": r.index,
            "sha256": r.sha256,
            "triangles": r.props.triangles,
            "volume_m3": clean(r.props.volume),
            "mass_kg": clean(r.mass),
            "surface_area_m2": clean(r.props.area),
            "closure_error": clean(r.props.closure_error),
            "unmatched_directed_edges": r.props.unmatched_edges,
            "bbox_min_m": [clean(x) for x in r.props.bbox_min],
            "bbox_max_m": [clean(x) for x in r.props.bbox_max],
            "com_mesh_frame_m": [clean(x) for x in r.com_mesh],
            "com_dh_frame_m": [clean(x) for x in r.com_dh],
            "inertia_dh_frame_about_com_kgm2": {
                "ixx": clean(r.inertia_dh[0][0]),
                "iyy": clean(r.inertia_dh[1][1]),
                "izz": clean(r.inertia_dh[2][2]),
                "ixy": clean(r.inertia_dh[0][1]),
                "ixz": clean(r.inertia_dh[0][2]),
                "iyz": clean(r.inertia_dh[1][2]),
            },
            "principal_moments_kgm2": [clean(x) for x in jacobi_eigenvalues(r.inertia_dh)],
            "own_joint_axis_in_dh_frame": (
                [clean(x) for x in joint_axis_in_dh_frame(r.index)] if r.index >= 0 else None
            ),
            "inertia_about_own_joint_axis_kgm2": (
                clean(inertia_about_axis(r.inertia_dh, joint_axis_in_dh_frame(r.index)))
                if r.index >= 0
                else None
            ),
        }

    document = {
        "generated_by": TOOL_NAME,
        "method": (
            "exact tetrahedron decomposition of the closed STL surface; volume, first "
            "and second moments integrated analytically, shifted to the centroid by the "
            "parallel-axis theorem, then rotated from the mesh frame (URDF link frame) "
            "into the link's standard-DH frame"
        ),
        "assumption": (
            "each link is a solid of uniform effective density, the same density for "
            "every link; the real links are hollow castings, so the density is a "
            "stand-in fitted to the published total mass, not a material property"
        ),
        "published_mass_dispute": {
            "fitted_to_kg": PUBLISHED_ROBOT_MASS_KG,
            "fitted_to_source": PUBLISHED_MASS_SOURCE,
            "manual_kg": MANUAL_ROBOT_MASS_KG,
            "manual_source": MANUAL_MASS_SOURCE,
            "gp7_column_kg": GP7_MASS_KG,
            "gp7_column_source": "DS-699-H page 2, SPECIFICATIONS row Weight, GP7 column",
            "manual_mass_scale_factor": clean(MANUAL_SCALE_FACTOR),
            "note": (
                "Yaskawa's own two documents disagree on the GP8 mass: DS-699-H page 2 "
                "prints 32 kg in the GP8 column and HW1484385 page 4 Table 5-1 prints "
                "35 kg. The 34 kg this project previously used is the GP7 column of the "
                "same datasheet row, i.e. the adjacent robot. Every number in this file "
                "is fitted to the 32 kg GP8 datasheet figure. Mass and inertia are both "
                "linear in the density, so the 35 kg fit is this file scaled by "
                "manual_mass_scale_factor with the centres of mass unchanged; no second "
                "table is emitted."
            ),
        },
        "datasheet": {
            "robot_mass_kg": PUBLISHED_ROBOT_MASS_KG,
            "robot_mass_source": PUBLISHED_MASS_SOURCE,
            "payload_kg": PUBLISHED_PAYLOAD_KG,
            "horizontal_reach_m": PUBLISHED_REACH_M,
            "allowable_wrist_inertia_kgm2": PUBLISHED_WRIST_ALLOWABLE_INERTIA,
        },
        "total_mesh_volume_m3": clean(total_volume),
        "effective_density_kg_m3": clean(density),
        "fill_fraction_aluminium": clean(fill_aluminium),
        "fill_fraction_steel": clean(fill_steel),
        "moving_mass_kg": clean(sum(r.mass for r in results if r.index >= 0)),
        "base_mass_kg": clean(results[0].mass),
        "total_mass_kg": clean(sum(r.mass for r in results)),
        "base_link": link_entry(results[0]),
        "moving_links": [link_entry(r) for r in results if r.index >= 0],
    }
    return json.dumps(document, indent=2, sort_keys=True) + "\n"


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
