# GP8 link mass properties

The link masses, centres of mass and inertia tensors in
`cpp_solver/include/study/gp8_model.hpp` used to be box and rod approximations
marked `ESTIMATED`. They are now integrated from the robot's real geometry.

## Where the numbers come from

`tools/compute_link_mass_properties.py` (standard library only, no network)
reads the seven visual meshes in
`src/yaskawa_workcell_description/meshes/visual/` - the upstream
ROS-Industrial STLs derived from Yaskawa CAD - and for each one:

1. parses the STL (binary and ASCII both handled);
2. checks the surface is closed: the sum of the outward area vectors must
   vanish and every directed edge must have its reverse. All seven pass with a
   closure error of at most 2.0e-17 and zero unmatched edges. An open mesh is
   reported and refused, never integrated;
3. integrates the signed volume, the centroid and the full 3x3 inertia tensor by
   tetrahedron decomposition - every triangle plus the origin is one tetrahedron
   - then shifts the tensor onto the centroid with the parallel-axis theorem.
   This is exact for a polyhedron; there is no approximation of the shape;
4. rotates and translates the result from the mesh frame (which is the URDF link
   frame) into that link's standard-DH frame, the frame `study/dynamics.cpp`
   works in.

It writes `cpp_solver/include/study/gp8_mass_properties.hpp` (committed, so the
build needs neither Python nor the meshes) and `assets/gp8_mass_properties.json`.
Output is deterministic: two runs are byte-identical, and
`python tools/compute_link_mass_properties.py --check` verifies the committed
files against a fresh run without writing anything.

## The published mass is disputed

Yaskawa's own two documents do not agree on what a GP8 weighs, and the figure
this project carried before was from neither of them.

| figure | source | status |
| --- | --- | --- |
| **32 kg** | DS-699-H page 2, SPECIFICATIONS row Weight, **GP8 column** | **the committed numbers are fitted to this** |
| 35 kg | HW1484385 page 4, Table 5-1 Approx. Mass (type YR-1-06VX8-F00) | Yaskawa's own manual, disagrees with its own datasheet |
| 34 kg | DS-699-H page 2, same row, **GP7 column** | the adjacent robot; `gp8_model.hpp` carried it as the GP8 mass by mistake, and the first generation of this table was fitted to it |

Page citations for all three are in `assets/gp8_published_specification.json`.
Nothing is averaged and nothing is reconciled by nudging a number.

Mass and inertia are both linear in the density, so the whole table refitted to
the manual's 35 kg is this table multiplied by a **single scale factor of
1.09375** - every mass and every inertia component, centres of mass unchanged.
That is why no second table exists here to drift out of step with the first; the
factor is exported as `massprops::MANUAL_MASS_SCALE_FACTOR`, alongside
`MANUAL_TOTAL_MASS_KG` and `GP7_COLUMN_MASS_KG`.

## The one fitted parameter

The links are hollow castings, so raw volume times a solid density would
overestimate the mass badly. The tool solves for a single effective density such
that the seven masses sum to the published GP8 mass:

| quantity | value |
| --- | --- |
| total enclosed mesh volume | 0.020219401 m^3 |
| published GP8 robot mass (DS-699-H p2) | 32.0 kg |
| solved effective density | 1582.64 kg/m^3 |
| implied fill fraction vs aluminium (2700) | 0.5862 |
| implied fill fraction vs steel (7850) | 0.2016 |
| (at the manual's 35 kg) | 1731.01 kg/m^3, fill 0.6411 |

**Assumption, stated plainly: each link is a solid of uniform effective density,
the same density for every link.** The real mass distribution is dominated by
the motor and gearbox at each joint, which the outer casting geometry cannot
know about. What is exact here is the geometry - each link's volume, centroid and
the shape of its inertia tensor - and the total, which is pinned to the
datasheet. How the 32 kg is split between the links follows from the volumes,
and that split is the modelling assumption.

A fill fraction of 0.59 against cast aluminium is physically sensible for
castings that are mostly hollow. The tool says so loudly if the fraction ever
leaves the band [0.1, 1.0].

## Results

| axis | mesh | mass [kg] | com in DH frame [m] | I about its own joint axis [kg m^2] |
| --- | --- | --- | --- | --- |
| base | `gp8_base_link.stl` | 9.078 | (-0.0058, -0.0000, 0.0938) | 0.045250 (about base z) |
| S | `gp8_link_1_s.stl` | 5.403 | (-0.0249, 0.0438, -0.0000) | 0.018003 |
| L | `gp8_link_2_l.stl` | 8.776 | (-0.1849, -0.0282, 0.0003) | 0.107848 |
| U | `gp8_link_3_u.stl` | 4.602 | (-0.0206, 0.0007, -0.0176) | 0.016504 |
| R | `gp8_link_4_r.stl` | 3.462 | (-0.0000, -0.1489, -0.0001) | 0.005744 |
| B | `gp8_link_5_b.stl` | 0.639 | (0.0002, -0.0000, 0.0176) | 0.000776 |
| T | `gp8_link_6_t.stl` | 0.040 | (0.0001, 0.0000, 0.0741) | 0.000014 |

Six moving links 22.923 kg, non-moving pedestal 9.078 kg, total 32.000 kg.
Multiply every mass and inertia by 1.09375 for the manual's 35 kg figure.

### Frame convention, the easy thing to get wrong

Standard (distal) DH puts frame *i* at the origin of joint *i+1*, and the axis of
joint *i* is **z of frame i-1, not z of frame i**. So the moment of a link about
the axis it actually rotates about is `a' I a` with
`a = (0, sin alpha_i, cos alpha_i)` in its own frame - y for R, -y for B, z for
T. It is not simply `izz`. `control_system.cpp`'s `joint_side_inertia()` already
takes the axis from `frames[i-1].col(2)`, which is the same thing done right.

## What is still estimated

Gear ratios, rotor inertias, viscous and Coulomb friction. Yaskawa publishes
none of them - the datasheet and the instruction manual stop at the axis
ratings - and the visual meshes are outer castings, so the drive train is not
recoverable from geometry either. These four fields per link stay labelled
`ESTIMATED` in `gp8_model.hpp` and remain what a dynamics or control module is
expected to identify from data. The DH table itself is unchanged: it already
reproduces the engine's forward kinematics to 3e-16.

## Verification

Run by the tool on every invocation, and it refuses to emit without them:

- total mass equals 32 kg to 3.6e-15 kg;
- every tensor symmetric to 0, positive definite (smallest principal moment
  7.66e-06 kg m^2), and obeying the triangle inequality with at least 7.8 % of
  the largest moment to spare;
- every centroid inside its mesh bounding box;
- the mesh-to-DH offset constant over joint space to 2.9e-16, and the DH chain
  times the flange correction equal to the engine's forward kinematics to
  2.4e-16;
- an analytic cross-check through the same code path: a cube agrees with
  `m L^2 / 6` to 1.3e-16 as ASCII STL and to 7.5e-08 as binary STL (the limit
  is the float32 vertices the binary format stores, the same precision the GP8
  meshes carry), and an icosphere agrees with `2/5 m R^2` to 3.6e-03 at
  subdivision 4, an error that divides by four with each subdivision - it is the
  inscribed polyhedron, not the integrator;
- a datasheet sanity check: the wrist links' inertia about their own axes
  against the published allowable wrist inertias of 0.5 / 0.5 / 0.2 kg m^2 -
  R 0.0057 (1.1 %), B 0.00078 (0.16 %), T 0.000014 (0.01 %). The rating covers
  an 8 kg tool at arm's length, so a bare casting sitting far below it is the
  expected shape; a ratio above 1 would have meant the integration or the frame
  transform was wrong. It is a weak check and is reported as one.

All CMake suites (`test_cpp`, `test_study_cpp`, `test_study_kinematics`,
`test_study_dynamics`, `test_study_control`, `test_study_dsp`,
`test_study_intelligence`, `test_study_specification`) pass on the new numbers.
Two test expectations moved with them, both because the physics moved and not
because a check was relaxed; each is explained in a comment at its site in
`src/test_study_dynamics.cpp` (energy-drift bound 5e-4 J -> 6e-5 J, measured
drift 2.6e-4 -> 2.66e-5 J) and `src/test_study_dsp.cpp` (bearing-fault
modulation 23 Hz -> 7 Hz, because the joint-2 structural carrier fell from
25.26 Hz to 20.38 Hz and 23 Hz no longer sits below it).
