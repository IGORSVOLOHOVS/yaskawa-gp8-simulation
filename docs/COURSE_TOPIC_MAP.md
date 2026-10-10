# Course Topic Map

Every topic of the TSI MSc semester (Master of Engineering, Smart Electronic
Systems & Robotics) mapped to the place in this Yaskawa GP8 simulator where it
becomes visible, adjustable and studyable.

## 1. What the study layer is

The six courses of the semester are taught as mathematics on paper and as
Python/MATLAB scripts that print a number. This repository already holds a real
6-axis GP8: `YaskawaKinematics` (FK, numeric Jacobian, damped-least-squares IK,
the six `GP8_JOINT_LIMITS`), `QuinticTrajectoryPlanner`, and a Three.js scene in
`cpp_solver/web/static/index.html` that drives six STL link groups `j1Group` …
`j6Group` from six sliders. The study layer turns each syllabus topic into a
module that computes on **that** robot, so the formula and the moving arm are the
same object.

**The one rule:** the user interface is generated from the code's own
self-description. A panel's sliders, units, ranges and formula come from the
module's `describe()` response and from nowhere else, so a panel cannot drift
away from the formula it claims to show. The wire protocol, the
`ModuleDescription` shape and the type vocabulary are in
[STUDY_MODULE_CONTRACT.md](STUDY_MODULE_CONTRACT.md); this document does not
repeat them.

The modules:

- **spatial_math** — position vectors, rotation matrices, Euler/RPY, SE(3) composition.
- **dh_kinematics** — the DH table, FK, IK branches, workspace, singular configurations.
- **jacobian_statics** — geometric/analytical Jacobian, velocity propagation, rank, force duality.
- **dynamics** — energy, Lagrangian, `M(q)q̈+C(q,q̇)q̇+g(q)=τ`, Newton-Euler recursion.
- **trajectory_profiles** — cubic/quintic joint paths, via points, LSPB, constraint feasibility.
- **control_system** — the joint plant, block algebra, stability, quality, accuracy, PID.
- **digital_control** — sampling, discretisation, the z-domain, sample-rate and quantisation effects.
- **dsp_system** — identification, FIR/IIR synthesis, phase correction, DFT, fixed-point realisation.
- **sensing_models** — encoders, resolvers, tacho, force-torque, IMU, noise and conditioning.
- **vision_camera** — pinhole projection, hand-eye chain, measurement mapped into the base frame.
- **learning_models** — data preparation, regression, trees/ensembles, perceptron/MLP, SOM, GA/GP.
- **probability_lab** — combinatorics, conditional probability, Bayes, discrete and normal distributions.
- **linear_algebra_lab** — matrix algebra, determinants, SLAE, Gauss, rank, basis, decomposition. *(added)*
- **calculus_optimization** — derivatives, integrals, several variables, partial derivatives, stationary points. *(added)*
- **agent_architecture** — PEAS, agent types, hierarchical vs reactive paradigms, intelligent control. *(added)*
- **research_corpus** — the 74 shared papers, each attached to the module whose topic it extends.

## 2. Module roster

| Module | Courses served | Source path | Topic rows carried |
| :--- | :--- | :--- | ---: |
| `spatial_math` | 3883 (Block 1), 3286 (rotation matrices) | `cpp_solver/include/study/spatial_math.hpp` | 7 |
| `dh_kinematics` | 3883 (Block 2, Block 6) | `cpp_solver/include/study/dh_kinematics.hpp` | 9 |
| `jacobian_statics` | 3883 (Block 3) | `cpp_solver/include/study/jacobian_statics.hpp` | 5 |
| `dynamics` | 3883 (Block 4), 3884 (plant) | `cpp_solver/include/study/dynamics.hpp` | 6 |
| `trajectory_profiles` | 3883 (Block 5) | `cpp_solver/include/study/trajectory_profiles.hpp` | 7 |
| `control_system` | 3884 (Sessions 1-24) | `cpp_solver/include/study/control_system.hpp` | 24 |
| `digital_control` | 3884 (Sessions 25-26) | `cpp_solver/include/study/digital_control.hpp` | 2 |
| `dsp_system` | 3882 (all 16 sessions), 3884 (Session 28) | `cpp_solver/include/study/dsp_system.hpp` | 17 |
| `sensing_models` | 3884 (Sessions 27-28) | `cpp_solver/include/study/sensing_models.hpp` | 2 |
| `vision_camera` | 3884 (Session 27), 3883 (exam Q1a) | `cpp_solver/include/study/vision_camera.hpp` | 1 |
| `learning_models` | 2953 (ML, ANN, GA/GP), 3884 (Session 29) | `cpp_solver/include/study/learning_models.hpp` | 9 |
| `probability_lab` | 3286 (probability, random variables, normal) | `cpp_solver/include/study/probability_lab.hpp` | 9 |
| `linear_algebra_lab` | 3286 (linear algebra half) | `cpp_solver/include/study/linear_algebra_lab.hpp` | 15 |
| `calculus_optimization` | 3286 (calculus and optimization), 2953 (gradient training) | `cpp_solver/include/study/calculus_optimization.hpp` | 4 |
| `agent_architecture` | 3884 (Session 29), 2953 (Artificial Agents) | `cpp_solver/include/study/agent_architecture.hpp` | 2 |
| `research_corpus` | 3900 (Blocks 1-3), 2953 (group project), 3883 (Block 6) | `cpp_solver/include/study/research_corpus.hpp` | 8 |

Counts are rows in the map of §3, measured from this document; a row whose
Module.op cell names two modules counts in both, which is why the column does
not sum to the 129 rows of the map.

Three modules were added to the roster:

- **`linear_algebra_lab`** — 3286's exam list devotes questions 1-12 to matrix
  algebra, determinants, Laplace expansion, Cramer, the inverse-matrix method,
  Gaussian elimination, rank, Rouché-Capelli, basis decomposition, adjacency and
  incidence matrices and flow networks. None of that is spatial; `spatial_math`
  is about SO(3)/SE(3) and would be lying if it claimed Laplace expansion. The
  GP8 still supplies the data: the URDF link-joint tree is the graph whose
  adjacency and incidence matrices the course asks for, and the IK solver's
  damped normal equations `(JJᵀ + λI)x = e` are the SLAE Gauss solves.
- **`calculus_optimization`** — 3286 exam questions 28-34 are derivatives,
  stationary points, indefinite and definite integrals, area between curves,
  functions of several variables, partial derivatives and the min/max/saddle
  classification of a two-variable stationary point. `probability_lab` is the
  wrong home and `learning_models` only consumes the gradient it needs.
- **`agent_architecture`** — 3884 Session 29 (hierarchical and reactive
  paradigms, intelligent control) and 2953's *Introduction to Artificial Agent*
  are about the *structure* of the decision layer above the servo loop, not about
  loop dynamics (`control_system`) and not about fitting a model
  (`learning_models`). The 3900 corpus pushes the same way (`AbouAli2026`
  agentic AI, `Lee2026a` autonomy for empty-headed robots, `Zhao2026b` edge
  general intelligence).

No roster module came out empty, but **`vision_camera` is thin and honestly so**:
no session in any of the six courses is *about* computer vision. It exists
because 3883's exam checklist requires "mapping a camera/LiDAR measurement into
the base frame" and mock-paper Q1(a) is exactly a hand-eye chain, and because
3884 Session 27 lists vision among the information devices. One row in the map and
one exam question; it is a service module, not a course module.

Two facts the implementing agents must know before they start:

- `cpp_solver/src/yaskawa_kinematics.cpp` builds FK as a hand-written chain of
  `T.translate(...)` / rotate steps (0.33 m to joint 1, 0.345 m to joint 3,
  `(0.34, 0, 0.04)` to joint 4), **not** from a DH table. `dh_kinematics` must
  introduce the real standard-DH table for the GP8 and prove it reproduces the
  existing chain pose-for-pose; that cross-check is itself a 3883 Block 6
  deliverable.
- `src/yaskawa_workcell_description/urdf/gp8_macro.xacro` has joint axes, origins
  and `<limit>` values but **zero `<inertial>` blocks** — no link masses, no
  inertia tensors. `dynamics` cannot be derived from the URDF as it stands; the
  masses, inertias and gear ratios go into `study/gp8_model.hpp` as the contract
  requires, documented as estimates with their source.

## 3. The map

### 3.1 Course 3883 — Robotics Modelling (M-407-01)

Moodle publishes only **Session 1** and **Session 2** of 32 sessions
(`course_3883_report.json` states "Session 1 of 32"); every later session is not
yet published. The authoritative topic list below is therefore the six-block
week-by-week plan in `texts/00-course-guide-m-407-01.txt`, with the two published
session titles marked. 33 rows.

| Topic (as the course names it) | Module.op | What you see on the robot | What you can adjust | What proves you understood it |
| :--- | :--- | :--- | :--- | :--- |
| Block 1 · Position vectors *(published: Session 1 · Position, Orientation and Rotation Matrices)* | `spatial_math.transform_point` | The flange origin drawn as an arrow from the base frame, its x/y/z components printed and redrawn as joint 1 sweeps its ±2.967 rad range | Six joint angles; which frame the point is expressed in (base, `link_3_u`, flange) | Express the same tool tip in base and in `link_3_u`: the three numbers change, the drawn arrow tip does not move |
| Block 1 · Rotation matrices *(published: Session 1)* | `spatial_math.rotation_from_rpy`, `.orthonormality_check` | The live 3×3 R of `link_6_t`, its three columns drawn as the tool triad on the flange | Roll, pitch, yaw; a perturbation ε injected into one element of R | With ε=0, det R = 1 and ‖RᵀR−I‖ ≈ 1e−16; with ε=0.05 the verdict flips and the drawn triad is visibly no longer square |
| Block 1 · Euler angles *(published: Session 2 · Euler Angles and Roll-Pitch-Yaw Representations)* | `spatial_math.rotation_from_euler` | The wrist (joints 4-5-6) driven from a ZYZ triple, the flange triad following, and the triple recovered back out of R printed beside the input | The three ZYZ angles; the recovery branch sign | At β=0 two different (α, γ) pairs give the identical wrist pose on screen: the representation is singular, the robot is not |
| Block 1 · Roll-pitch-yaw representations *(published: Session 2)* | `spatial_math.rpy_from_rotation`, `.gimbal_lock_sweep` | Pitch swept 80°→90°: recovered roll and yaw diverge on the readout while the arm barely moves, and the sensitivity curve spikes | Sweep range and step of the middle angle; fixed-axis vs moving-axis convention | Name the pitch at which the RPY parameterisation loses rank and show the arm pose is still well defined there |
| Block 1 · Homogeneous transformation matrices | `spatial_math.compose` | The 4×4 `⁰T₆` printed with its R block and p column colour-separated, and the flange frame it describes drawn in the scene | `rpy` and `p` of each factor; number of factors | Swap two factors and show the printed p moves while det R stays 1 — rotation composes, translation does not commute with it |
| Block 1 · Composition of transformations | `spatial_math.frame_of_joint` | All seven GP8 frames (`base_link`, `link_1_s` … `link_6_t`, `flange`) as nested triads, with the partial product `⁰Tᵏ` printed for the frame you select | Which link k to stop the product at; a tool offset appended after the flange | Multiply the six link transforms by hand for one pose and match `⁰T₆` to 1e−9 against the panel |
| Block 2 · DH convention (standard vs modified) | `dh_kinematics.dh_table` | The GP8 standard-DH table as a 6×4 table of (θ, d, a, α) beside the existing hand-written translate/rotate chain, each row's frame drawn on the arm | Standard vs modified convention; which row to highlight; the 0.33 / 0.345 / 0.34 / 0.04 m offsets | The DH product and `YaskawaKinematics::forwardKinematics` agree to 1e−9 on 10 000 random poses; a disagreement is your frame assignment, not the solver |
| Block 2 · Kinematic chain construction | `dh_kinematics.fk_chain` | Each `Aᵢ` applied one at a time, the arm assembling link by link with the intermediate frame left visible | How many `Aᵢ` to apply (1-6); the joint vector | Predict where the wrist lands after only `A₁A₂A₃` and show the panel puts it there |
| Block 2 · Forward kinematics for a 3-DOF manipulator | `dh_kinematics.fk_3dof` | Joints 4-6 frozen at zero, the GP8 reduced to its S-L-U shoulder-elbow, the closed-form (x, y, z) printed next to the full-chain answer | θ₁, θ₂, θ₃; the frozen wrist values | Derive the three-term closed form on paper and match the panel; then unfreeze joint 4 and explain why (x, y, z) does not move |
| Block 2 · Forward kinematics for a 6-DOF manipulator | `dh_kinematics.fk` | Position in mm and orientation in degrees for the full six-axis pose, with the flange triad and the tool-point trail in the 3D view | All six joints, bounded by `GP8_JOINT_LIMITS` | Reach one pose from two different joint vectors and show FK maps both to the identical `⁰T₆` |
| Block 2 · Inverse kinematics (geometric approach) | `dh_kinematics.ik_geometric` | A target frame dragged in the 3D scene, the arm snapping to the closed-form solution, every elbow-up/elbow-down/wrist-flip branch drawn as a ghost arm | Target pose; branch selection (shoulder, elbow, wrist flip) | Count the branches returned for one reachable pose and show each ghost arm's FK lands on the same target |
| Block 2 · Inverse kinematics (algebraic approach) | `dh_kinematics.ik_numeric` | The damped least-squares iteration animated — the arm creeping to the target — with per-iteration position error and λ plotted | Damping λ, position and rotation tolerance, `max_iters`, the seed `q_init` | Seed one target from two `q_init` values and show the numeric solver lands on two different branches while the geometric op enumerates both at once |
| Block 2 · Workspace analysis | `dh_kinematics.workspace_sample` | A point cloud of reachable flange positions in the scene, the reachable envelope and the dexterous (full-orientation) subset shaded differently | Sample count; which joints to sweep; the orientation constraint defining "dexterous" | Read the GP8 maximum reach off the cloud, then confirm it by summing link lengths by hand |
| Block 2 · Singularities | `dh_kinematics.singularity_scan` | The arm driven into the wrist singularity (axes 4 and 6 aligned) and into the shoulder singularity, with det J collapsing to zero on a live trace | Which singularity to approach; the approach distance | At the wrist singularity, a Cartesian velocity that was fine 1° earlier now demands joint rates above `GP8_JOINT_LIMITS[3].max_vel` = 9.60 rad/s |
| Block 3 · Jacobian matrix derivation | `jacobian_statics.jacobian_geometric`, `.jacobian_analytical` | The 6×6 J printed with linear and angular blocks separated, each column drawn as the tool-tip velocity that one joint alone would cause | Joint vector; geometric vs analytical form; numeric-difference reference | Geometric J, analytical J and `YaskawaKinematics::computeJacobian` agree to 1e−7 at a non-singular pose |
| Block 3 · Linear and angular velocity propagation | `jacobian_statics.velocity_propagation` | The recursion run link by link: ωᵏ and vᵏ drawn as arrows at each of the six frames while joint 2 turns at 1 rad/s | Per-joint rates q̇; which link to inspect | The propagated v₆ equals J·q̇ to 1e−9 — recursion and matrix are the same statement |
| Block 3 · Singularity analysis | `jacobian_statics.det_rank`, `.manipulability` | det J, the rank, the six singular values, and the manipulability ellipsoid drawn at the tool tip flattening into a disc as the arm straightens | Joint vector; a path dragged through the singular set | Drive a straight Cartesian line through a singularity and show the condition number blowing up before det J reaches zero |
| Block 3 · Force-torque duality via Jacobian transpose | `jacobian_statics.force_duality`, `.static_torque` | A force arrow applied at the flange and the six joint torques drawn as bars against each joint's `<limit effort>` (100/100/100/50/50/30 N·m) | Force and moment vector at the tool; the pose | Apply 10 N along +x, check τ = JᵀF bar by bar against hand arithmetic, then find the pose where the same 10 N overruns joint 2's 100 N·m budget |
| Block 4 · Energy methods review | `dynamics.energy` | Kinetic and potential energy of the six links plotted during a motion, with the whole-arm centre of mass drawn and its height tracked | Link masses and inertias from `study/gp8_model.hpp`; the motion | In a zero-torque gravity fall, T+V stays constant to within the integrator tolerance |
| Block 4 · Lagrangian formulation | `dynamics.lagrangian` | ℒ = T − V and each Euler-Lagrange term evaluated at the pose the arm is holding on screen | Pose and velocity; which joint's equation to expand | Reproduce `mL²θ̈ + mgL cos θ = τ` by zeroing five links in the panel |
| Block 4 · Derivation of the equations of motion | `dynamics.eom_standard_form`, `.mass_matrix`, `.coriolis`, `.gravity` | `M(q)q̈`, `C(q,q̇)q̇` and `g(q)` as three stacked torque bars per joint during a fast two-joint move: the gravity bar holds steady while the Coriolis bar grows with speed | q, q̇, move speed, payload mass at the flange | Double q̇ and show the Coriolis/centrifugal share quadruples while the gravity term is unchanged |
| Block 4 · Newton-Euler recursive formulation | `dynamics.newton_euler` | The outward velocity/acceleration pass and the inward force/moment pass animated along the six links, with each joint's reaction force drawn | Base acceleration; tool wrench; which pass to step through | The recursive τ matches the closed-form `Mq̈ + Cq̇ + g` to 1e−9 at the same state |
| Block 4 · Critical comparison of approaches | `dynamics.method_compare` | The two formulations' torque outputs side by side with the measured `us` of each op and the operation count against DOF | DOF of the reduced model (2 to 6); repetitions | Show Newton-Euler time growing about linearly with DOF while closed-form evaluation does not — the O(n) argument, measured on this robot |
| Block 5 · Joint-space trajectories — cubic polynomials | `trajectory_profiles.cubic` | Joint 2 moving 0°→90° in 2 s with position, velocity and acceleration curves under the 3D view and the acceleration jump at t=0 and t=tf marked | Δθ, duration, boundary velocities, which joint | Peak velocity reads 3Δθ/(2tf) = 67.5 °/s for the 90°/2 s move, matching the hand result, and the acceleration trace steps rather than ramps |
| Block 5 · Joint-space trajectories — quintic polynomials | `trajectory_profiles.quintic` | The same 90° swing re-planned with quintic boundary conditions: acceleration now starting and ending at zero, jerk finite instead of impulsive | Boundary accelerations; duration; which of the six joints | Overlay cubic and quintic jerk for the same move and state what the quintic buys the payload and the gearbox |
| Block 5 · Via-point sequences | `trajectory_profiles.via_points` | A three-point pick-move-place path, the tool-tip trail in 3D, and the velocity continuity (or the corner) at each via point | Via-point joint vectors; dwell time; blend continuity (C¹ vs C²) | Drop the velocity-continuity condition at the middle point and show the velocity trace stepping while the arm visibly jolts |
| Block 5 · Cartesian-space trajectories (LSPB) | `trajectory_profiles.lspb`, `.cartesian_lspb` | A straight-line Cartesian move with the trapezoidal velocity profile drawn, blend times marked, and the tool tip tracing an actually straight line | Cruise velocity, acceleration, blend fraction, start and goal pose | Lower the cruise velocity until the profile degenerates to the triangular case, with cycle time tracking the analytic formula |
| Block 5 · Operational space planning | `trajectory_profiles.operational_space` | The Cartesian path executed through IK at every sample, with the resulting joint curves — and joint rates spiking where the path passes near a singularity | The path's clearance from the singular set; sample rate | Move the straight path 50 mm clear of the wrist singularity and show peak joint-4 rate dropping below its 9.60 rad/s limit |
| Block 5 · Evaluation against physical constraints (actuator limits, jerk, cycle time) | `trajectory_profiles.limit_check` | A per-joint pass/fail table against `GP8_JOINT_LIMITS` velocities and the `<limit effort>` torques, the violating segment highlighted in red on the curve and on the 3D path | Duration, payload, profile type | Shorten the duration until joint 2 is declared infeasible, compute the minimum feasible time by hand, and confirm it is the boundary the panel found |
| Block 6 · Integration of kinematic-dynamic models in MATLAB/Python/ROS-Gazebo | `dh_kinematics.fk` + `dynamics.eom_standard_form` cross-check op | One panel running the study-layer model and the repository's existing `YaskawaKinematics` / `QuinticTrajectoryPlanner` on the same input, with the per-sample difference plotted | Which pair to cross-check; sample count; tolerance | A difference plot sitting on zero to 1e−9 across a full trajectory — the independent source of truth the project brief says the report is won or lost on |
| Block 6 · Model validation against engineering specifications | `trajectory_profiles.limit_check` + `jacobian_statics.static_torque` | The GP8's published envelope (reach, payload, repeatability) drawn as the specification with the model's achieved values overlaid and the shortfall labelled | The specification values; payload; speed | Name one specification the idealised model meets only because it has no friction, no backlash and rigid links |
| Block 6 · Project work sessions | `research_corpus.by_module` (3883 filter) | The project's chapter list with, per chapter, the panel that produced its figure and the `us` that figure cost | Which chapter; which figure | Every analytical claim in the report has a panel that regenerates its figure from committed source |
| Block 6 · Student presentations and critical course review | *no panel — see §4* | — | — | — |

### 3.2 Course 3884 — Robot Control and Feedback Systems (M-408-01)

All 29 session titles are published in `course_3884_structure.json`. The six
guided labs (`texts/labT1-lab.txt` … `labT5-lab.txt`) sit on top of sessions 4,
8, 13, 16-17, 23-24 and 28 and use a DC-motor robot joint with a gearbox as the
plant — the same plant `control_system.plant_joint` must expose. 29 rows.

| Topic (as the course names it) | Module.op | What you see on the robot | What you can adjust | What proves you understood it |
| :--- | :--- | :--- | :--- | :--- |
| Session 1 · Basic definitions of automatic control theory; classification of robot control systems | `control_system.loop_anatomy` | The GP8 joint-2 loop drawn over the live 3D arm: setpoint, error, controller, amplifier, motor, gearbox, encoder, disturbance torque — each block lighting up with its current numeric value as the joint moves | Open-loop vs closed-loop; which signal to probe; disturbance on/off | Break the feedback path and show joint 2 sagging under gravity with no correction, then close it and show the error driven back toward zero |
| Session 2 · Modelling robot subsystems: differential equations and the Laplace transform | `control_system.plant_joint`, `.laplace_tf` | The joint's second-order ODE and its transfer function `W(s) = K/(s(Ts+1))` printed from the motor constants, with the pole map beside the arm | `R`, `L`, `K_t`, `K_e`, `J`, `b`, gear ratio; which joint (1-6) | Change `J` to the value a 3 kg payload adds and show the time constant T and the pole positions move exactly as the algebra says |
| Session 3 · Block diagrams of control systems and their algebra | `control_system.block_reduce` | The joint-2 loop reduced step by step on screen — series, parallel, feedback — each step collapsing blocks while the closed-loop pole readout stays unchanged | Which reduction rule to apply next; the three gains | Reduce the loop by hand with the three rules and match the final closed-loop transfer function coefficient by coefficient |
| Session 4 · Practical Work 1 — Modelling and transfer functions of a robot joint | `control_system.step_response` | Joint 2 commanded 0°→90° with the commanded and measured angle on one axis, overshoot and settling time marked, and the arm visibly ringing in the 3D view at low damping | Proportional gain `Kp`; `J`; the step size | The closed-loop damping ratio read off the overshoot matches the ζ computed from the characteristic polynomial, and a time-domain simulation and the pole prediction agree (lab T1's two independent routes) |
| Session 5 · Time-domain characteristics: transient and steady-state response of robot loops | `control_system.step_response`, `.quality_indices` | The same 90° swing annotated with rise time, peak time, overshoot %, settling time and the steady-state residual offset, with the residual visible as the arm stopping short of the target line | `Kp`; damping; the final-value tolerance band | Raise `Kp` until overshoot doubles and settling time *worsens*, then state why faster is not better |
| Session 6 · Types of standard dynamic elements (links) of control systems | `control_system.standard_links` | Each of the seven elementary links driven on its own — gain, first-order motor lag, integrator, second-order compliant joint, differentiator, pure transport delay — with the GP8 joint actually moving under each one | Which link; its one or two parameters; step vs ramp input | Identify which link the GP8 joint-2 drive behaves as below 5 Hz and which extra link appears above it |
| Session 7 · Connections of dynamic elements and the frequency response of robot plants | `control_system.freq_response`, `.interconnect` | Bode magnitude and phase of the joint plant computed at `s = jω` with the arm shaken at the cursor frequency so you see the amplitude actually falling as ω rises | Frequency range; series/parallel/feedback connection; each block's parameters | Build the loop's Bode plot by adding the individual links' decibels and degrees by hand and land on the panel's curve |
| Session 8 · Practical Work 2 — Standard links and frequency characteristics | `control_system.standard_links` + `.freq_response` | Each link's step response and Bode pair shown together, with the GP8 joint driven by that link so the time curve and the shaking arm are the same event | Which of the seven links; its parameters; ω grid | Compute one magnitude and one phase point by hand from `s = jω` and hit the panel's value — lab T2's requirement to use no control toolbox |
| Session 9 · Characteristics of interconnected systems and model build-up for a full axis | `control_system.axis_model` | The full joint-2 axis assembled from amplifier, motor, gearbox, arm inertia and encoder, with the resulting single transfer function and the arm responding to it | Which subsystem to include or bypass; gear ratio; encoder resolution | Bypass the gearbox and show the reflected inertia change moving the dominant pole by the square of the gear ratio |
| Session 10 · The stability problem and algebraic stability criteria for robot loops | `control_system.routh` | The Routh array printed for the joint loop's characteristic polynomial, the stable gain range stated, and the arm shown oscillating with growing amplitude just above that range | Loop gain `K`; the polynomial coefficients; added inertia | Compute the Routh range by hand, set `K` 5 % above the limit, and watch the 3D arm diverge exactly as the sign change predicted (lab T3 stage 2) |
| Session 11 · Frequency-domain stability: Nyquist and Bode (margins) criteria | `control_system.nyquist`, `.bode_margins` | The Nyquist contour with its encirclements of −1 and the Bode plot with gain margin and phase margin drawn as labelled vertical gaps, next to the arm settling or ringing at that gain | Loop gain; added sensor latency in milliseconds; frequency range | Add 20 ms of encoder latency and show the phase margin eroding toward zero while the Nyquist contour walks into −1 — lab T3's "delay erosion" step |
| Session 12 · Root-locus view of stability and gain selection | `control_system.root_locus` | The closed-loop poles sliding along the locus as gain increases, with the crossing of the imaginary axis marked and the arm's step response redrawn at each selected gain | Loop gain sweep range; the chosen operating gain; an added pole or zero | Pick the gain where the locus crosses ζ = 0.707 and show the measured overshoot lands near 4.3 % |
| Session 13 · Practical Work 3 — Stability analysis of a robot control loop | `control_system.routh` + `.bode_margins` + delay term | The same loop judged three ways on one panel — Routh range, gain/phase margins, latency tolerance — with a single verdict line and the arm demonstrating each verdict | Gain; latency; inertia | All three methods agree on the stability boundary; where they disagree, you find the arithmetic error rather than trusting the verdict (lab T3's stated habit) |
| Session 14 · Control-quality indicators: transient performance specifications | `control_system.quality_indices` | Overshoot, settling time, rise time, IAE and ITAE computed for a gain sweep and plotted against `Kp`, with the arm replaying the best and the worst gain back to back | `Kp` sweep range; which index to minimise; step size | Name the gain that minimises ITAE, show it is not the gain that minimises overshoot, and explain the trade |
| Session 15 · Steady-state accuracy: error constants and system type | `control_system.error_constants` | The system type (0/1/2), `Kp`, `Kv`, `Ka`, and the residual error for step, ramp and parabolic commands — with the gravity droop drawn as the gap between the commanded and the held angle of joint 2 | Which input; integrator present or absent; payload mass | Add the integrator and show the droop go to exactly zero for the step while the ramp error becomes finite and non-zero |
| Session 16 · Practical Work 4 — Quality and accuracy assessment (part 1) | `control_system.quality_indices` (gain sweep) | A sweep table of overshoot/settling/IAE/ITAE per gain with the arm replaying the selected row, reproducible against `scipy.signal` | Gain grid; measurement window; tolerance band | Every number in the panel's table is reproduced by an independent routine to three significant figures (lab T4's verification rule) |
| Session 17 · Practical Work 4 — Quality and accuracy assessment (part 2) | `control_system.error_constants` (accuracy half) | System type and the three error constants beside the measured step, ramp and gravity-droop errors for the chosen controller, with joint 2 holding against gravity on screen | Controller structure (P, PI); payload; ramp slope | Predict the ramp error from `Kv` alone, then measure it and match — the accuracy half of lab T4 |
| Session 18 · Robustness and parametric/nonparametric uncertainty of robot systems | `control_system.uncertainty` | A family of step responses for the payload range 0-8 kg and ±20 % inertia drawn as a shaded envelope, with the worst-case arm motion animated | Payload range; inertia and friction tolerance; unmodelled delay bound | Find the payload at which the nominal-tuned loop first exceeds its overshoot specification, and say whether that is parametric or non-parametric uncertainty |
| Session 19 · Designing for robustness: margins, sensitivity and the payload range | `control_system.robust_design` | Gain and phase margin plotted across the whole payload range with the specification floor drawn, and the sensitivity function `S(jω)` beside it | Target margins; payload range to certify; controller gains | Retune so the 6 dB / 45° margins hold across 0-8 kg and show what bandwidth you paid for it |
| Session 20 · PID control: structure and the role of P, I and D terms in robot loops | `control_system.pid` | The three terms' contributions drawn as separate stacked traces during one 90° joint-2 swing, so you see D killing the ring and I eating the droop while the arm settles | `Kp`, `Ki`, `Kd` independently; derivative filter; step size | Switch off I and show the droop reappear; switch off D and show the overshoot return — each term's job demonstrated on the arm, not asserted |
| Session 21 · PID tuning methods for robot joints | `control_system.pid_tune` | Ziegler-Nichols, pole-placement and hand-tuned gain sets compared on the same 90° swing, with all three responses overlaid and the arm replaying each | Tuning method; desired ζ and ωn; the ultimate gain and period | Place the closed-loop poles where you chose and show the measured overshoot and settling time match the placement (lab T5's design brief) |
| Session 22 · Advanced controller structures: cascade, feedforward and disturbance compensation | `control_system.cascade_feedforward` | An inner velocity loop inside the outer position loop, plus a gravity feedforward term `g(q)` taken straight from `dynamics.gravity`, with the droop disappearing before the integrator has to act | Inner/outer bandwidth ratio; feedforward on/off; disturbance step size | Apply a 20 N·m load step and show the cascade plus feedforward rejects it in a fraction of the time single-loop PID needs |
| Session 23 · Practical Work 5 — PID controller design and tuning (part 1) | `control_system.pid_tune` (design half) | The designed PID's pole placement, its step response and the arm executing the designed 90° move, with the specification band drawn around the curve | Desired overshoot and settling time; the three gains | The response fits inside the specification band you set before tuning, and an independent route (pole polynomial) confirms the same gains |
| Session 24 · Practical Work 5 — PID controller design and tuning (part 2) | `control_system.pid` + `.error_constants` (load-rejection half) | A constant load torque applied mid-move: the P-only loop settles below target with the arm visibly sagging, the PID loop returns to the commanded angle exactly | Load torque magnitude and timing; `Ki`; anti-windup limit | Show the steady-state error is exactly zero with I and exactly `τ_load / (Kp·K)` without it — lab T5's closing claim |
| Session 25 · Digital control systems: sampling, discretisation and the z-domain | `digital_control.sample_hold`, `.discretise`, `.zdomain_map` | The continuous command and its zero-order-hold staircase on one axis, the s-plane poles and their z-plane images on the unit circle, and the arm stepping in visible increments | Sample period `T`; discretisation method (ZOH, Tustin, matched); the continuous gains | Map a stable s-plane pole into z, show it is inside the unit circle, then raise `T` until it leaves and the arm's motion diverges |
| Session 26 · Digital controller implementation and sample-rate effects | `digital_control.sample_rate_sweep`, `.quantise_controller` | Phase margin and overshoot plotted against sample rate from 1 kHz down to 50 Hz, with the arm replaying the 90° swing at the selected rate and visibly chattering at the low end | Sample rate; computational delay of one sample; coefficient word length | Quantify the extra phase lag as `ωT/2` at your sample rate, show it matches the margin loss on the plot, and state the minimum rate the joint tolerates |
| Session 27 · Information (sensing) devices of robotic systems | `sensing_models.encoder`, `.resolver`, `.tacho`, `.force_torque`, `.imu`, `vision_camera.measure_pose` | Each GP8 feedback device simulated on the real motion: encoder counts stepping at its resolution, tacho output, a wrist force-torque wrench, an IMU on link 3, a camera pose measurement — all with their quantisation and noise visible on the trace | Which device; resolution/bits; noise σ; bandwidth; latency | Show the encoder resolution, not the controller, sets the smallest commandable joint increment, and compute that increment from the bit count |
| Session 28 · Practical Work 6 — Sensor feedback, signal conditioning and filtering | `sensing_models.filter_chain` → `dsp_system.fir_design` | The noisy encoder-derived velocity signal, the filtered signal and the resulting arm motion, with the filter's added group delay drawn as the horizontal shift between the two traces | Filter type and order; cutoff; sample rate | Show a sharp low-pass that cleans the signal beautifully and destabilises the loop by its group delay — the trap the 3882 project brief names explicitly |
| Session 29 · Robot control paradigms (hierarchical, reactive) and intelligent control methods | `agent_architecture.hierarchical`, `.reactive`, `.intelligent_control` | The same pick-and-place task run three ways: a hierarchical plan-then-execute stack, a reactive sense-act layer, and a learned-gain controller, with the arm's path and cycle time differing visibly in the 3D view | Which paradigm; the obstacle appearing mid-task; replanning latency | Introduce an obstacle mid-motion and show the reactive layer avoids it while the hierarchical plan drives into it until the next replan — the architectural difference made physical |

### 3.3 Course 3882 — Digital Signal Processing Algorithms and Systems (M-403-02)

All 16 session titles are published in `course_3882_structure.json`, grouped into
five topics (Discrete-System Characterisation and Phase Structure; Inverse
Modelling, Equalisation and Identification Lab; Arbitrary-Magnitude
Approximation; Phase Correctors and All-Pass Networks; Full-System Synthesis and
Fixed-Point Realisation). The signal under study is always a real GP8 channel —
the encoder-derived joint velocity, the wrist force-torque wrench or an IMU on
link 3 — supplied by `sensing_models`. 16 rows.

| Topic (as the course names it) | Module.op | What you see on the robot | What you can adjust | What proves you understood it |
| :--- | :--- | :--- | :--- | :--- |
| Session 1 · Linear Discrete Systems: Minimum, Maximum and Linear Phase | `dsp_system.phase_class` | The pole-zero plot of the joint-velocity filter on the unit circle, its group delay, and two overlaid versions of the same GP8 velocity burst — one through a linear-phase filter (waveform shape preserved) and one through a maximum-phase filter (the burst visibly skewed) | Zero radii inside/outside the unit circle; filter order; which channel | Reflect one zero from 0.5 to 2.0, show the magnitude response is unchanged and the group delay is not, and name the resulting phase class |
| Session 2 · Transfer-Function Determination and Test-Signal Selection | `dsp_system.tf_identify`, `.test_signal` | The GP8 joint-2 drive excited by an impulse, a step, white noise and a swept sine in turn, with the identified `H(z)` coefficients and the fit residual printed for each test signal | Test-signal type, amplitude, length, SNR | Show the swept sine and the noise excitation recover the same `H(z)` while a single step does not excite the high band enough to identify it |
| Session 3 · The Generalised Correlation Method and Delay Elimination (Part 1) | `dsp_system.correlation_id` | Cross-correlation of the commanded and the measured joint-2 velocity, the peak offset reading out the loop's transport delay in samples, and the delay removed so the two traces superimpose | True delay injected; correlation window length; noise level | Inject 7 samples of delay, recover 7 from the correlation peak, and show the identified `H(z)` is minimum-phase only after the delay is stripped |
| Session 4 · Generalised Correlation: Smoothing of Frequency Responses (Part 2) | `dsp_system.smooth_response` | The raw, ragged identified magnitude response of the joint channel and the smoothed estimate drawn over it, with the variance reduction and the resolution loss both quantified | Smoothing window type and width; number of averaged records | Over-smooth until a real 40 Hz structural resonance of the arm disappears from the estimate, and state the resolution you traded for the variance |
| Session 5 · Inverse Transfer Function and Amplitude-Response Equalisation | `dsp_system.inverse_tf`, `.equalise` | The identified channel's magnitude, its inverse, and the equalised cascade sitting flat — plus the noise floor rising at the spectral null where the inverse has a pole | Regularisation ε; the band to equalise; target flatness | Equalise with ε=0 and show the noise explosion at the null, then find the smallest ε that keeps the output SNR above the input's |
| Session 6 · Lab: Transfer-Function Determination of a Linear Discrete System (Part 1) | `dsp_system.tf_identify` (lab preset) | The guided identification run end to end on the joint-2 channel: excitation, measurement, estimate, residual — with the arm actually executing the excitation motion | Excitation preset; record length; sample rate | The identified poles and zeros match the plant `control_system.plant_joint` discretised at the same rate, to within the stated residual |
| Session 7 · Lab: Transfer-Function Determination of a Linear Discrete System (Part 2) | `dsp_system.tf_identify` (validation half) | The identified model's predicted output overlaid on a held-out GP8 motion it was never fitted to, with the prediction error trace below | Which held-out motion; model order | The held-out error stays at the fitted error's level when the order is right and grows when the order is too low — the honest validation the project brief demands |
| Session 8 · From Identified Model to Design Specification | `dsp_system.spec_from_model` | The identified channel response with a pass/stop-band mask drawn on top of it, derived from what the control loop actually needs (bandwidth above the loop crossover, stop-band below the resonance) | Pass-band edge, ripple, stop-band edge, attenuation | Justify every number in the mask from a measured property of the GP8 loop rather than from a textbook default |
| Session 9 · Arbitrary-Magnitude Approximation with FIR Systems (Part 1) | `dsp_system.fir_design` | The windowed-FIR magnitude response drawn against the mask from Session 8, the tap stem plot, and the MAC-per-sample count for the GP8's 1 kHz control rate | Window (rectangular, Hann, Hamming, Blackman, Kaiser β), order N, cutoff | Show Hann (~44 dB) and Hamming (~53 dB) cannot reach a 60 dB stop-band while Blackman and Kaiser β≈5.65 can, by reading the achieved attenuation off the panel |
| Session 10 · Arbitrary-Magnitude Approximation with FIR Systems (Part 2) | `dsp_system.fir_arbitrary`, `.fir_types` | An arbitrary multi-band magnitude target (notch the 40 Hz arm resonance, pass the servo band) with the least-squares FIR fit and its error, plus the four linear-phase types' symmetry and forced zeros | Target band edges and weights; filter type I-IV; order | Attempt a high-pass as a Type II FIR and show the forced zero at z = −1 making it impossible — the exam's non-negotiable insight |
| Session 11 · Arbitrary-Magnitude Approximation with IIR Systems: Prony's Method (Part 1) | `dsp_system.iir_prony` | Prony's fit of the measured joint-channel impulse response, with the fitted poles and zeros on the unit circle and the order needed printed next to the FIR order for the same mask | Numerator and denominator order; impulse-response length used | Meet the same mask at order 6 that the FIR needed ~37 taps for, and state the latency and stability price you paid |
| Session 12 · IIR Approximation: Parametric Equalisers and Realisation (Part 2) | `dsp_system.param_eq`, `.biquad_cascade` | A parametric peaking/notching section tuned onto the GP8's measured arm resonance, the cascade of biquads drawn as a signal-flow graph, and the vibration in the measured trace dropping when it is enabled | Centre frequency, Q, gain; direct form vs cascaded second-order sections | Notch the resonance and show the loop's step response stops ringing without any change to the controller gains |
| Session 13 · Phase Correctors Based on FIR; the Hilbert Transform (Part 1) | `dsp_system.hilbert`, `.fir_phase_corrector` | The analytic signal of the wrist vibration channel, its envelope drawn over the raw trace, and an FIR corrector flattening the group delay of the identified channel | Hilbert FIR order; the band to correct; which channel | Compute the envelope spectrum of a bearing-fault signature and show the fault frequency appearing in the envelope where it was invisible in the raw spectrum |
| Session 14 · FIR Phase Correctors and Quadrature Processing (Part 2) | `dsp_system.quadrature` | The in-phase and quadrature components of the joint-velocity channel, the 90° relationship verified across the band, and the instantaneous frequency of a swept GP8 motion tracked | Quadrature filter order; sweep rate; band limits | Show the quadrature pair stays 90° apart only inside the designed band, and name what happens to the instantaneous-frequency estimate outside it |
| Session 15 · Phase Correctors Based on IIR (All-Pass Networks) | `dsp_system.allpass` | An all-pass section's flat magnitude and its shaped group delay, applied to align the force-torque channel with the encoder channel — the two traces snapping into time alignment on screen | All-pass pole radius and angle; number of sections; target delay | Align two GP8 channels to within one sample without changing either magnitude response by more than 0.01 dB |
| Session 16 · Lab: Synthesis of a Linear Discrete System with Specified Amplitude Response | `dsp_system.fixed_point`, `.biquad_cascade`, `.synthesis_report` | The finished filter chain running on the live GP8 channel in floating point and in Q1.15, with the quantised magnitude response overlaid on the mask and the quantised pole displacement drawn on the unit circle | Q-format (integer and fractional bits); direct form vs SOS; dither on/off | Quantise a biquad with poles at radius 0.825, show how far they move and whether they stay inside the circle, and reproduce the limit cycle that appears with zero input before suppressing it |

### 3.4 Course 3286 — Mathematics for Data Analytics

Topic names are taken from the section and activity names in
`course_3286_structure.json`; the granularity below matches the 34-question exam
list in `texts/list_of_questions_exam_premasters_2026.txt`. The GP8 supplies the
matrices: its URDF link-joint tree is a graph, its IK normal equations are an
SLAE, its rotation matrices are the course's transformation matrices, and its
repeatability and cycle-time statistics are the random variables. 28 rows.

| Topic (as the course names it) | Module.op | What you see on the robot | What you can adjust | What proves you understood it |
| :--- | :--- | :--- | :--- | :--- |
| Matrices: transposition and inversion, transformation matrices | `linear_algebra_lab.matrix_ops` | The GP8's 4×4 `⁰T₆`, its transpose, and its inverse shown as the transform that maps the flange frame back to the base — with both frames drawn | Which matrix (R, `⁰T₆`, J); the operation (transpose, inverse, scalar multiple) | Show that for a rotation block the transpose *is* the inverse, and that for the full `⁰T₆` it is not — the inverse needs `−Rᵀp` |
| Homework-1 (matrices) | `linear_algebra_lab.matrix_ops` (multiply) | Two link transforms multiplied element by element with the running sums shown, and the composed frame appearing in the 3D scene | Matrix dimensions; which two link transforms | Multiply a 4×4 by a 4×1 point by hand and match the panel; then show A·B ≠ B·A on two GP8 link transforms |
| Homework-2 (determinants) | `linear_algebra_lab.determinant`, `.laplace_expansion` | The Laplace expansion of the 6×6 Jacobian along a chosen row, every cofactor printed, and det J plotted as the arm sweeps toward a singular pose | Which row or column to expand along; the pose | Expand along two different rows and get the same determinant, then find the pose where it is zero and see the arm lose a direction of motion |
| Application of matrices-1 (networks) / Seven bridges problem | `linear_algebra_lab.adjacency_incidence` | The GP8 kinematic tree as a graph: its 7×7 adjacency matrix and 7×6 link-joint incidence matrix printed, with the corresponding links highlighted in the 3D view | Whether to include the fixed `joint_flange`; tool or fixture links added | Read the chain's degree sequence off the adjacency matrix and show the robot is a path graph, not a tree with branches — and say what would change for a dual-arm cell |
| Application of matrices-2 (rotation matrices) | `spatial_math.rotation_from_rpy` (2D and 3D presets) | The 2D rotation matrix applied to a point in the robot's horizontal plane and the 3D version applied to the whole tool triad, both drawn before and after | Angle; axis (x, y, z); 2D vs 3D | Rotate by θ then by −θ and show the composition is the identity to 1e−16, and that det = 1 for both |
| practice_lin+algebra_linear_transform (linear transformations) | `linear_algebra_lab.linear_transform` | A cube of workspace points pushed through a chosen linear map and redrawn in the scene — a rotation keeps it a cube, a singular map flattens it into a plane | The 3×3 map's entries; a preset (rotation, scaling, shear, projection) | Show that the map's determinant is exactly the volume ratio of the before and after point clouds, and zero when the cube collapses |
| Systems of linear equations and intro to matrices | `linear_algebra_lab.cramer`, `.inverse_method` | The IK step solved as a 6×6 SLAE `J·dq = e`, once by Cramer's rule and once by the inverse-matrix method, with the resulting joint increment applied to the arm | Which method; the target error vector e; the pose | Both methods return the same `dq` to 1e−12 at a well-conditioned pose, and both degrade at the same pose Cramer's denominator approaches zero |
| Gauss elimination for 1) SLAE 2) determinant 3) inverse 4) ranks | `linear_algebra_lab.gauss` | The same 6×6 IK system reduced to row echelon form one pivot at a time, each elementary operation listed, with the determinant and rank falling out of the final form | Pivoting strategy (none, partial); which of the four uses; the matrix | Use one elimination to produce the solution, the determinant and the rank, and show the determinant equals the product of pivots times the row-swap sign |
| Homework-3 (SLAE for flow networks) / application of SLAE | `linear_algebra_lab.flow_network` | The workcell's material flow (part in, pick, place, part out) as a flow network with balance equations at each node and the solved flows drawn on the 3D cell | Node demands; a blocked edge; capacity | Add a node without adding an equation and show the system becoming underdetermined — one degree of freedom you can see in the solution family |
| Practice-4 (ranks) | `linear_algebra_lab.rank` | The rank of the 6×6 Jacobian printed as the arm moves, dropping from 6 to 5 at the wrist singularity, with the lost direction of end-effector motion drawn as a greyed-out arrow | Pose; rank tolerance; which submatrix | Show rank J = 6 almost everywhere and exactly 5 on the singular set, and name the physical motion the robot cannot produce there |
| Range, rank, its properties | `linear_algebra_lab.rank` (range half) | The column space of J drawn as the set of achievable tool velocities (an ellipsoid when rank is 6, a flattened disc when it is 5) and the null space drawn as the self-motion direction | Pose; which space to draw (range, null space) | Verify rank + nullity = 6 at every pose, including the singular one, and point at the null-space joint motion that moves the arm without moving the tool |
| Rouche-Capelli theorem | `linear_algebra_lab.rouche_capelli` | A commanded tool velocity tested for achievability: the theorem's rank comparison printed, and the arm either executing it or reporting the unreachable component | The commanded twist; the pose | Command a velocity in the lost direction at a singular pose and show rank(A) ≠ rank(A\|b) — the inconsistency theorem, demonstrated as a motion the robot cannot do |
| Decomposition of the vector through the basis | `linear_algebra_lab.basis_decompose` | A commanded tool velocity decomposed onto the six Jacobian columns, each coefficient drawn as the joint rate it implies, and the reconstructed velocity overlaid on the commanded one | The commanded velocity; the basis (Jacobian columns, singular vectors) | Reconstruct the commanded vector from its coefficients to 1e−12 and show the coefficients are unique only while the basis is independent |
| Linear spaces: linear (in)dependence, basis | `linear_algebra_lab.independence` | The six Jacobian columns drawn as vectors at the tool tip, with the pair that becomes collinear at the wrist singularity flagged and drawn overlapping | Pose; which columns to test; tolerance | Find the pose where two columns become dependent and show it is exactly the pose where the rank drops and det J = 0 |
| Matrix decomposition | `linear_algebra_lab.decomposition` | The Jacobian's LU and SVD shown side by side, the six singular values as a bar chart, and the manipulability ellipsoid they generate drawn at the tool tip | Which decomposition (LU, QR, SVD); the pose | Reconstruct J from its factors to 1e−12, then show the smallest singular value — not the determinant — is the honest distance to singularity |
| Midterm Test 1 (Linear algebra) | `linear_algebra_lab.drill` | A generated problem set whose matrices are real GP8 matrices, each with a worked solution and the robot pose it came from | Difficulty; which topics to include; number of problems | Answer each generated problem on paper first, then let the panel mark it — a score, not a feeling, before the real test |
| Introduction to Probability | `probability_lab.event_algebra` | A tray of parts with per-part pick outcomes, the sample space enumerated, and sums, products, complements and mutually exclusive events computed over real pick attempts in the cell | Number of parts; per-part success probability; which events to combine | Verify the inclusion-exclusion principle on two overlapping pick-failure events by counting the simulated trials, not by trusting the formula |
| Combinatorics | `probability_lab.combinatorics` | The number of distinct pick orders for n parts in the tray (permutations), orders of k of them (partial permutations) and unordered selections (combinations), with one sampled order animated on the arm | n, k; with or without repetition; ordered or not | Compute the cycle-time cost of the best and the worst of the 8! = 40 320 pick orders for 8 parts and state which combinatorial object you just counted |
| Probability of sum, conditional probability, Bayess | `probability_lab.conditional` | Pick success conditioned on part orientation, shown as a contingency table built from simulated picks with the conditional probabilities and their sums printed | Prior orientation mix; per-orientation success rates; sample size | Show P(A∪B) = P(A)+P(B)−P(A∩B) holds on the counted picks, and that conditioning on orientation changes the success estimate materially |
| List_of_problems_prob_4 (total probability, Bayes) | `probability_lab.total_prob`, `.bayes` | A failed pick traced backward: the posterior probability that the part was misoriented, given the failure, computed from the prior mix and the per-orientation failure rates | Priors; likelihoods; the observed outcome | Compute the posterior by hand for one failure and match the panel, then show how the posterior moves when the prior orientation mix changes |
| 4. Random discrete variables | `probability_lab.discrete_rv` | The number of successful picks per 10-part tray as a discrete distribution built from repeated simulated trays, with mean, variance and standard deviation printed on the bar chart | Trials per tray; success probability; number of trays | Compute E[X] and Var[X] from the definition by hand, match the simulated moments, and state how fast the agreement improves with more trays |
| List_of_problems_6 (binom, Poisson) | `probability_lab.binomial`, `.poisson` | The binomial distribution of successful picks in n attempts and the Poisson distribution of rare collision-stop events per shift, drawn together so the approximation's quality is visible | n, p for the binomial; λ for the Poisson; the approximation toggle | Show the Poisson approximation tracking the binomial while np stays small and visibly failing once p grows — the condition, demonstrated rather than quoted |
| Normal (Gauss) distribution | `probability_lab.normal`, `.three_sigma`, `.demoivre` | The GP8's repeatability error at a taught point sampled many times, drawn as a histogram with the fitted normal, the ±3σ band shaded, and the out-of-band outliers marked in the 3D view | σ of the positioning noise; sample count; the tolerance band | State the fraction of picks outside ±3σ, match it to 0.27 %, and show the De Moivre-Laplace normal approximation of the binomial pick count converging as n grows |
| Midterm Test 2 (probability) | `probability_lab.drill` | A generated problem set drawn from the cell's own statistics, each problem with its worked solution and the simulation it came from | Difficulty; topics; number of problems | Answer on paper, then mark against the panel — before the real test |
| Introduction to calculus and optimization · Derivatives (overview) | `calculus_optimization.derivative`, `.stationary_points` | The joint-2 position curve of a trajectory with its first and second derivatives drawn underneath, the stationary points marked, and the arm paused at the instant velocity reaches zero | Which trajectory; which joint; the polynomial order | Differentiate the cubic by the power rule on paper, locate the velocity peak at t = tf/2, and show the panel's marker sits there |
| Integration | `calculus_optimization.integral`, `.area_between` | The area under the joint-velocity curve shaded, with the accumulated angle printed and the arm shown at exactly that angle — and the area *between* the commanded and the measured velocity curves shaded as the tracking error integral | Integration limits; which two curves; number of samples | Show the definite integral of velocity equals the joint's actual angular displacement to 1e−9, and that the area between the two curves is the IAE `control_system.quality_indices` reports |
| Function_several_var / homework-12 (partial derivatives) | `calculus_optimization.partial_derivatives`, `.hessian_classify` | The tool height z as a function of (θ₂, θ₃) drawn as a surface over the two-joint plane, its two partial derivatives as tangent slopes, and the stationary points classified as minimum, maximum or saddle with the arm driven to each | Which two joints; the function (height, reach, manipulability); the grid | Find the saddle point of the two-joint reach surface, classify it from the Hessian determinant by hand, and show the arm's behaviour there — one joint increases reach while the other decreases it |
| Probability, Optimization | `calculus_optimization.gradient_descent` + `probability_lab.normal` | Gradient descent minimising the expected cycle time of a pick path under normally distributed positioning noise, with the iterate path drawn on the cost surface and the arm replaying the best path found | Learning rate; noise σ; starting point; iteration cap | Show the optimum found under noise differs from the deterministic optimum, and that the gradient the panel uses matches a finite-difference check to 1e−6 |

### 3.5 Course 2953 — Intelligent Systems 1

Topic names are the section and resource names in
`course_2953_structure.json`; the 58-item `texts/List of Exam Questions.txt`
fixes the granularity. The dataset is always generated by this robot: pick
attempts labelled success/failure, joint-torque traces labelled
collision/no-collision, poses labelled reachable/unreachable. 15 rows.

| Topic (as the course names it) | Module.op | What you see on the robot | What you can adjust | What proves you understood it |
| :--- | :--- | :--- | :--- | :--- |
| Presentation N1. Introduction (definitions, key features, 7 aspects of AI) | *no panel — see §4* | — | — | — |
| History of AI | *no panel — see §4* | — | — | — |
| A. M. Turing (1950) Computing Machinery and Intelligence | *no panel — see §4* | — | — | — |
| Presentation N 2 (AI impact on information technology, society, economics) | `research_corpus.by_module` (AI-impact filter) | The corpus papers that make the same claims with evidence — `Rikalovic2022` on Industry 4.0 implementation, `Tallat2024` and `Slamani2026` on the human-centric transition — each tagged to the robot module it touches | Which claim; which domain | Replace one lecture assertion about AI's economic impact with a citation from the corpus that actually measures it |
| Introduction to ML. Types of ML Approaches. Data Preparation for ML tasks. | `learning_models.dataset`, `.prepare`, `.normalise`, `.bucket` | The robot's own pick log as a feature table (pose, payload, speed, torque peak, outcome), with missing rows, un-normalised ranges and raw categorical orientation labels shown before and after preparation | Which features; normalisation (min-max, z-score); bucket count; categorical encoding | Feed the un-normalised table and the normalised one to the same model and show the accuracy gap — the reason the preparation step exists |
| ML Models. Models Performance Evaluation. Types of ML models | `learning_models.linear_reg`, `.logistic`, `.decision_tree`, `.random_forest`, `.gboost`, `.metrics` | Each model predicting whether the next pick succeeds, with the confusion matrix, accuracy, precision, recall, specificity, F1 and the ROC/AUC curve, and the mispredicted picks replayed on the arm | Model type and hyperparameters; decision threshold; which features | Move the threshold and show precision and recall trading against each other on the same model — then pick the threshold a real cell would ship and justify it from the cost of a missed pick |
| Task for Practical Work 1 (classification problem; data vectorization) | `learning_models.prepare` + `.split_kfold` | The pick log vectorised, rows with empties dropped, the array dimensions printed, and the hold-out and k-fold splits drawn as coloured bands over the chronological log | Split ratio; k; stratification on/off; random seed | Show a chronological split and a random split giving different accuracies on the same data, and say which one honestly estimates tomorrow's performance |
| Examples of Logistic Regression, Decision Tree, Random Forest, AdaBoost, XGBoost implementation | `learning_models.decision_tree`, `.random_forest`, `.gboost` | The decision tree drawn as a tree whose leaves name a joint-torque threshold, the forest's per-tree votes, and the boosting rounds' residuals shrinking round by round | Tree depth; number of trees; learning rate; bagging vs boosting | Show bagging reducing variance (forest beats a deep single tree) and boosting reducing bias (accuracy climbing with rounds until it overfits), on the same robot dataset |
| Introduction to Neural Networks | `learning_models.perceptron`, `.activation` | A single perceptron deciding reachable vs unreachable from (x, z), its decision line drawn across the robot's vertical workspace slice, with the weights and bias as sliders | The two weights and bias; activation (step, sigmoid, ReLU, tanh); learning rate | Place the line by hand to separate the reachable region, then let training find it — and show no single line can separate the real annular workspace, which is why you need a second layer |
| ANN. "Artificial Intelligence - A Modern Approach" Russel, Norvig | `learning_models.mlp_train`, `.overfit` | A two-layer network learning the GP8's reachable workspace boundary, the decision surface redrawn every epoch over the 3D point cloud, with training and validation loss plotted | Layer widths; epochs; learning rate; momentum; regularisation | Train until validation loss turns upward while training loss keeps falling, point at the epoch, and show the over-trained surface hugging individual sample points |
| Demo of Simple Perception Training in xls | `learning_models.perceptron` (step-by-step mode) | One weight update at a time: the misclassified pose highlighted in the 3D view, the weight change printed, and the decision line visibly rotating toward it | Which sample to present next; learning rate; initial weights | Do three updates by hand on paper and match the panel's weights exactly — the arithmetic of learning, not its metaphor |
| Evolutionary algorithms · Introduction to Genetic Algorithms and Genetic Programming | `learning_models.ga`, `.gp` | A GA evolving the joint-space via points of a pick path for minimum cycle time: the population's paths drawn as faint trails, the best-so-far path animated on the arm, and the fitness curve climbing per generation | Population size; mutation and crossover rates; selection (tournament, roulette); fitness weights (time vs jerk vs torque) | Set mutation to zero and show the search stalling at a local optimum, then restore it and show the escape — selection, crossover and mutation each doing the job the lecture assigns them |
| Artificial Agents · Introduction to Artificial Agent | `agent_architecture.peas`, `.reflex_agent`, `.model_based`, `.goal_based`, `.utility_based`, `.learning_agent` | The same tray-clearing task run by each agent type, with the agent's percepts and chosen action printed each tick and the arm's resulting behaviour differing visibly — the reflex agent re-trying a failed pick forever, the utility agent skipping it | Agent type; the PEAS description; percept noise; utility weights | Make the environment partially observable (hide part orientation) and show the simple reflex agent failing where the model-based agent, keeping internal state, succeeds |
| Closure Lecture: Best Practices and RAI | `learning_models.model_card` | A generated model card for whichever learned model is loaded: training data provenance (which robot runs), measured accuracy per orientation class, failure modes, and the safety consequence of a false "pick will succeed" | Which model; which fairness/robustness slice to report | Find a slice of the data (one part orientation, one payload band) where the model is materially worse than its headline accuracy, and state what the cell should do about it |
| Group Project | `research_corpus.gap_map` (2953 filter) | The project's chosen business domain with the corpus papers that cover it and the robot panel that would supply its evidence | Domain; which papers; which panels | A project claim that is backed by a panel output rather than by a slide |

### 3.6 Course 3900 — Trends and Challenges in Smart Systems and Robotics (M-412-01)

Moodle publishes only **Session 1** and **Session 2**; the rest of the course is
not yet published, so the authoritative topic list is the four-block plan in
`texts/00-course-guide-m-412-01.txt`. This is a workshop course assessed by a
report (60%) and a presentation (40%) — there is no exam and most of it is
literature work, not robot work. The honest mapping is `research_corpus`, which
attaches each of the 74 papers to the module whose topic it extends; see §4.
8 rows.

| Topic (as the course names it) | Module.op | What you see on the robot | What you can adjust | What proves you understood it |
| :--- | :--- | :--- | :--- | :--- |
| Session 1 · The Technological Landscape (IoT and smart embedded systems, collaborative robotics, applied AI and autonomous systems) | `research_corpus.by_module` | The 74 papers bucketed onto the modules they touch, each bucket shown with the panel it extends — e.g. `Lesi2023` (IoT-enabled motion control) and `Baumann2021` (wireless control for smart manufacturing) landing on `control_system`, `Gao2026a` (multi-motor coordinated control) on `dynamics`, `Mirmohammadsadeghi2026` (visual SLAM deployment) and `Guesmi2023` (physical adversarial attacks on camera-based systems) on `vision_camera`, `Hu2023` (ML for tactile perception) and `Huang2026a` (flexible capacitive pressure sensors) on `sensing_models` | The domain filter (IoT, cobots, applied AI, autonomous systems); the module filter | Every one of the 74 keys is assigned to at least one module, and the leftovers — the healthcare, agriculture and space papers — are listed as leftovers rather than forced onto a panel |
| Session 2 · Reading Critically & Reading at the Right Depth | `research_corpus.depth_read` | For a selected paper, the three reading depths recorded against it (skim / structured / deep) and the specific panel claim the deep read was performed to check | Which paper; which depth; the claim being checked | For one paper you read deeply, name the number in it you could not reproduce from the paper alone and say what the authors did not report |
| Block 2: State-of-the-Art Investigation (independent literature survey, structured peer discussion, critical comparison of approaches) | `research_corpus.compare` | A comparison table across the papers in one chosen domain — method, platform, reported metric, what was not evaluated — with a column naming the GP8 panel that could reproduce their experiment | Domain; the comparison axes; which papers | Two papers claiming the same result by different methods, with the methodological difference stated and, where a panel exists, the difference reproduced on the GP8 |
| Block 3: Gap Identification and Research Question Formulation | `research_corpus.gap_map` | Each identified gap recorded against the module whose panel it would be investigated in, with the panel's current capability and the capability the gap needs | The gap statement; the module; the significance argument | A gap whose investigation names a specific module op and a specific adjustable parameter — that is the test of whether it is researchable or merely interesting |
| Block 4: Presentations and Critical Discussion | *no panel — see §4* | — | — | — |
| Shared corpus · Reading list of the 74 papers | `research_corpus.list`, `.paper_to_op` | The full alphabetical list by citation key (`Abdulkareem2019` … `Zhao2026b`) with, per paper, the module, the op and the course topic it extends | Sort (key, year, module, venue); the module filter | Open any panel and see the papers attached to it; open any paper and see the op it extends — a bidirectional link with no unlinked papers |
| Reports · 60% | *no panel — see §4* | — | — | — |
| Presentations · 40% | *no panel — see §4* | — | — | — |

## 4. The topics that do not fit a robot panel

Seven rows above say *no panel*. Forcing a visualisation onto them would make
the study layer dishonest, which is the one thing the self-description rule
exists to prevent. What to do with each instead:

| Topic | Course | Why no panel | What to do instead |
| :--- | :--- | :--- | :--- |
| Presentation N1. Introduction — definitions of Intelligent System, AI, BI; Copeland's features; weak/strong/general/narrow AI; 7 aspects of AI | 2953 | These are definitional. A slider cannot make a definition true or false | `research_corpus` holds a flashcard set keyed to exam questions 1-5 and 13, each card citing the lecture page it came from. Revision, not simulation |
| History of AI; Role of Turing; Test of Turing | 2953 | A historical timeline is not a property of a 6-axis arm | The same flashcard set, plus `texts/turing_paper.txt` linked in full — exam questions 6, 7 and 43 are answered by reading, not by driving a panel |
| Block 6 · Student presentations and critical course review | 3883 | An oral defence is the assessment, not the model | The panels *are* the slides: every figure in the presentation is regenerated live from the op that produced it, with its `source` path and `us` on screen |
| Block 4: Presentations and Critical Discussion | 3900 | Same | Same — plus `research_corpus.gap_map` as the presentation's backing table |
| Reports · 60% and Presentations · 40% | 3900 | A 2 000-2 500 word critical review is writing work | `research_corpus` is the writing instrument: it stores the per-paper depth, the comparison axes and the gap statement, and exports the citation keys in the Zotero form the course fixed (`texts/00-shared-corpus-zotero-library-of-the-74-papers-fixed-citation-keys.txt`) |
| Plagiarism Control Regulations of TSI | 3882, 3883, 3884, 3286, 3900 (General section of all five) | A regulation is a rule about authorship, not a computation | Keep it out of the study layer entirely. Its practical consequence lives in the repository: every panel shows its `source` path, so a figure in a report can always be traced to the commit that generated it, and the AI co-development logs the labs require are commit history rather than prose |
| Examination logistics — 3882's two-day "program the chip" hardware task; 3883's closed-book 2 h paper; 3884's 2 h written exam; 3286's two midterms plus exam; the AI-free rule on all of them | all | A closed, AI-free assessment cannot be sat inside a panel, and the hardware half of 3882's Part A needs an actual DSP/MCU/FPGA this repository does not have | §6 names the panel that rehearses each examinable question. For 3882 Part A, `dsp_system.fixed_point` reproduces everything *except* the device: the Q-format quantisation, the pole displacement, the limit cycle and the mask check. The programming and the bench sweep stay on hardware, and the document says so rather than pretending |
| Course feedback questionnaires; BigBlueButton rooms; announcement forums | all | Administrative | Not in scope. They are listed here only so the map can claim to have read every section of every course |

The `research_corpus` module is the load-bearing answer for the literature work.
It is not a viewer: for each of the 74 keys it stores the module, the op and the
course topic the paper extends, so the corpus is navigable from the panel and the
panel is reachable from the corpus. Worked examples of the attachment, from the
titles in `texts/00-shared-corpus-reading-list-of-the-74-papers.txt`:

| Paper | Attaches to | Because |
| :--- | :--- | :--- |
| `Lesi2023` — IoT-Enabled Motion Control: Architectural Design Challenges and Solutions | `control_system.axis_model`, `digital_control.sample_rate_sweep` | It is about what a network does to a motion loop's sample rate and latency, which is exactly the parameter those two ops expose |
| `Gao2026a` — A review of multi-motor coordinated control technologies | `dynamics.method_compare`, `control_system.cascade_feedforward` | Coordinated multi-axis control is the six-joint coupling `M(q)` describes |
| `Mirmohammadsadeghi2026` — On the Real-World Deployment of Visual SLAM | `vision_camera.hand_eye` | Its cost analysis is about the calibration chain the hand-eye op computes |
| `Guesmi2023` — Physical Adversarial Attacks for Camera-Based Smart Systems | `vision_camera.measure_pose` | It is an attack on exactly the measurement that op maps into the base frame |
| `Hu2023` — Machine Learning for Tactile Perception; `Huang2026a` — flexible capacitive pressure sensors | `sensing_models.force_torque` | Both are about what a contact sensor can and cannot report |
| `Deshmukh2025`, `Ullah2024` — indoor positioning and mobile robot localisation | `probability_lab.bayes` | Both are Bayesian state estimation under sensor noise |
| `Fitas2025` — Neuro-Symbolic AI for Advanced Signal and Image Processing | `dsp_system`, `learning_models` | It sits on the seam between the two modules, which is the reason it is interesting |
| `AbouAli2026`, `Lee2026a`, `Zhao2026b` — agentic AI, autonomy for empty-headed robots, edge general intelligence | `agent_architecture` | They are about the decision layer above the servo loop, the module's whole subject |
| `Birchler2025` — A Roadmap for Simulation-Based Testing of Autonomous Cyber-Physical Systems | the study layer itself | It is the methodological critique of what this repository does, and belongs in the project report's limitations section |
| `Qin2026`, `Sharma2022c` — robot digital twins, digital twins state of the art | `dh_kinematics` + `dynamics` cross-check op | "Digital twin" is the claim this simulator makes; these two papers are the standard it should be judged against |

The corpus's healthcare, agriculture, construction, food-processing and space
papers (`Moglia2022`, `Jararweh2023`, `Turner2021`, `Hussein2026`, `Oche2024`,
`Arzo2023` and about twenty more) attach to no module. They are recorded as
unattached, with their domain, because 3900 is a survey course and the breadth
is the point — not because a GP8 panel can be invented for orchard management.

## 5. Dependency order

```
tier 0   linear_algebra_lab   calculus_optimization   probability_lab
             |                      |                      |
tier 1   spatial_math  <------------+                      |
             |                                             |
tier 2   dh_kinematics                                     |
             |         \                                   |
tier 3   jacobian_statics   vision_camera                  |
             |                                             |
tier 4   dynamics                                          |
             |      \                                      |
tier 5   trajectory_profiles                               |
             |                                             |
tier 6   control_system                                    |
             |       \                                     |
tier 7   digital_control   sensing_models                  |
                                |                          |
tier 8                      dsp_system                     |
                                |                          |
tier 9   learning_models <------+--------------------------+   agent_architecture

any tier   research_corpus (no code dependency; describe() references module names)
```

| Module | Must exist first | Why |
| :--- | :--- | :--- |
| `linear_algebra_lab`, `calculus_optimization`, `probability_lab` | — | Pure mathematics on supplied matrices and samples. They can be built and tested before the robot model is touched, and they are the cheapest way to prove the wire protocol and the `describe()`-driven UI work |
| `spatial_math` | `linear_algebra_lab` (conceptually: determinant, transpose, inverse) | SO(3)/SE(3) is the first thing that needs the robot's frames, and everything geometric downstream composes its transforms |
| `dh_kinematics` | `spatial_math` | FK is a product of the `Aᵢ` that `spatial_math.compose` builds. The DH table must also be reconciled with the existing hand-written chain in `yaskawa_kinematics.cpp` before anything trusts a pose |
| `jacobian_statics` | `dh_kinematics` | The Jacobian columns are derived from the link frames; rank and manipulability are meaningless without a validated FK |
| `vision_camera` | `spatial_math` plus the existing Three.js scene | The hand-eye chain is an SE(3) product; the camera needs the 3D scene to have something to look at, which `index.html` already provides via the six STL link groups |
| `dynamics` | `dh_kinematics`, `jacobian_statics`, and new mass/inertia data in `study/gp8_model.hpp` | `M(q)` is built from link frames and link Jacobians. The URDF has no `<inertial>` block, so this module is blocked on data, not only on code |
| `trajectory_profiles` | `dh_kinematics` (Cartesian paths need IK), `dynamics` (torque feasibility in `limit_check`) | Joint-space cubic and quintic alone need neither, so the module can ship in two stages: profiles first, feasibility once `dynamics` exists |
| `control_system` | `dynamics` | The plant is the joint's equation of motion. Without it the loop is a textbook second-order system that happens to be drawn next to a robot — which is exactly the dishonesty this layer is built to avoid. Sessions 1-9 of 3884 need only the single-joint plant; Sessions 18-19 (payload-range robustness) need the full `M(q)` |
| `digital_control` | `control_system` | Sampling, discretisation and sample-rate effects are transformations *of* a loop that already exists |
| `sensing_models` | `control_system` | A sensor model is only interesting when there is a loop whose behaviour changes when the measurement degrades |
| `dsp_system` | `sensing_models` (for a real noisy channel), `control_system` (for the spec the mask must meet) | Identification needs a system to excite and a measurement to record; the 3882 mask in Session 8 is derived from the loop's crossover and resonance. Building `dsp_system` first would leave 16 sessions filtering synthetic noise, which is the one thing the course project forbids |
| `learning_models` | `trajectory_profiles`, `jacobian_statics`, `dsp_system`, `calculus_optimization` | Its datasets are generated by driving the robot: labelled pick outcomes, torque traces, reachability samples. Gradient training reuses `calculus_optimization.gradient_descent` |
| `agent_architecture` | `control_system`, `trajectory_profiles`, `sensing_models` | An agent is a policy over percepts and actions; it needs something to perceive and a loop to command |
| `research_corpus` | nothing in code | But build it last among the data-carrying modules: its `paper_to_op` entries name ops, so the op names must have stopped moving |

## 6. Exam leverage

Which published exam question each panel answers directly.

### 3883 — Robotics Modelling · mock paper in `texts/03-exam-preparation-mock-paper.txt` (closed-book, 2 h, 100 marks)

| Question | Panel to drive | What the panel gives you |
| :--- | :--- | :--- |
| Q1(a) [5] — a camera measures `ᶜTᵖ`, hand-eye `ᵉTᶜ` and FK `⁰Tᵉ` are known; write the part's pose in the base frame and explain why multiplication order matters | `vision_camera.hand_eye` with `spatial_math.compose` | Build `⁰Tᵖ = ⁰Tᵉ · ᵉTᶜ · ᶜTᵖ`, then reorder two factors and watch the predicted part position move in the 3D scene — non-commutativity stops being a sentence you memorise |
| Q1(b) [9] — compare rotation matrix, ZYZ-Euler and fixed-axis RPY; which for a perception-to-control pipeline; explain gimbal lock and where it bites | `spatial_math.rotation_from_euler`, `.rpy_from_rot`, `.gimbal_lock_sweep` | Three representations of one wrist pose side by side, and the pitch sweep where the RPY recovery blows up while the arm does not move |
| Q2 [12] — assign frames and write the standard-DH table for a 3-DOF anthropomorphic arm; state your convention | `dh_kinematics.dh_table` (3-DOF preset) | The GP8's S-L-U *is* that arm: the α₁ = π/2 twist between the vertical base axis and the parallel shoulder-elbow pair, with frames drawn so a wrong assignment is visible |
| Q3 [12] — write `⁰T₂(θ₁, θ₂)` for a planar 2R arm and evaluate at 30°, 60° | `dh_kinematics.fk_3dof` (planar 2R preset, L₁ = 0.5 m, L₂ = 0.4 m) | Check your hand answer — (0.433, 0.650) m, φ = 90° — before the exam rather than after |
| Q4 [14] — derive 2R inverse kinematics, both branches, the reachability condition, solve (0.6, 0.3) | `dh_kinematics.ik_geometric` (planar 2R preset) | Both branches as ghost arms (θ₂ ≈ ±84.3°), the annulus 0.1 ≤ r ≤ 0.9 m shaded, and the two solutions visibly merging as you drag the target onto the boundary |
| Q5(a)(b) [12] — derive the 2×2 positional Jacobian; find the singular configurations and interpret them | `jacobian_statics.jacobian_analytical`, `.det_rank` | det J = L₁L₂ sin θ₂ plotted against θ₂, zero at 0 and π, with joint rates diverging as you approach it |
| Q5(c) [4] — joint torques resisting a 10 N force along +x at 30°, 60° | `jacobian_statics.force_duality` | τ = JᵀF drawn as bars; confirm τ₁ = −6.5 N·m and τ₂ = −4.0 N·m |
| Q6(a)(b) [12] — single-link Lagrangian equation of motion; identify the inertial, Coriolis and gravitational terms of the 2-link arm | `dynamics.lagrangian`, `.eom_standard_form` | `mL²θ̈ + mgL cos θ = τ` from the one-link preset, then the three terms as separate stacked bars so "what each does physically" is something you watched |
| Q6(c) [4] — Lagrangian or Newton-Euler for a real-time computed-torque controller; justify | `dynamics.method_compare` | Measured `us` per call at DOF 2 to 6 — the O(n) argument with this repository's own numbers behind it |
| Q7(a)(b) [13] — cubic coefficients for 0°→90° in 2 s from rest; peak velocity; feasibility against a 50 °/s limit; minimum time | `trajectory_profiles.cubic` + `.limit_check` | a₂ = 67.5, a₃ = −22.5, peak 67.5 °/s at t = 1 s flagged infeasible, and the 2.7 s minimum found by dragging the duration until the flag clears |
| Q7(c) [3] — when to choose a quintic or an LSPB instead | `trajectory_profiles.quintic`, `.lspb` | Cubic, quintic and LSPB jerk and peak-velocity traces overlaid for the identical move |

### 3882 — DSP · mock Part B in `texts/exam_guide.txt` (~90 min, 100 marks) plus Part A (2-day device task)

| Question | Panel to drive | What the panel gives you |
| :--- | :--- | :--- |
| Q1 [22] — from `y[n] = x[n] − 0.5x[n−1] + 0.7y[n−1] − 0.10y[n−2]`: write H(z), poles and zeros, the ROC, stability, minimum phase, and whether waveform timing is preserved | `dsp_system.phase_class` | Enter the coefficients and see the zero at 0.5 and the poles at 0.5 and 0.2 inside the unit circle, the group delay varying with frequency, and the all-pass equaliser fixing the timing without touching the magnitude |
| Q2 [26] — windowed-FIR anti-alias design, fs = 8 kHz, pass 0-1.2 kHz at ≤0.5 dB, stop ≥2.0 kHz at ≥60 dB; window choice, order estimate, FIR vs IIR, the four linear-phase types | `dsp_system.fir_design`, `.fir_types`, `.iir_prony` | Hann topping out near 44 dB and Hamming near 53 dB against the 60 dB line; Kaiser β ≈ 5.65 reaching it at about 37 taps; the order-6 elliptic IIR meeting the same mask; Type II refusing to be a high-pass |
| Q3 [24] — the 4-point DFT of [1, 1, 0, 0] by hand; a 700 Hz tone sampled at 1 kHz; spectral leakage and its remedy | `dsp_system.dft`, `.window` | X = [2, 1−j, 0, 1+j] to check your arithmetic; the 700 Hz tone appearing at 300 Hz with the fold drawn; sidelobes collapsing as you change the window and the main lobe widening as you pay for it |
| Q4 [28] — the biquad `H(z) = (1 + z⁻¹)/(1 − 1.6z⁻¹ + 0.68z⁻²)`: poles and radius, Q1.7 quantisation, cascaded SOS, limit cycles | `dsp_system.fixed_point`, `.biquad_cascade` | Poles 0.8 ± j0.2 at radius 0.825; the −1.6 coefficient overflowing Q1.7's [−1, +0.9922] range on screen — the headline insight of the question; the pole displacement once you move to Q2.6; a limit cycle appearing at zero input and dying when dither is enabled |
| Part A — design, quantise and deploy a 200-800 Hz band-pass at fs = 4 kHz in Q1.15 and prove the device meets the mask | `dsp_system.fir_design` → `.fixed_point` → `.synthesis_report` | Everything but the chip: the floating-point design on the mask, the Q1.15 coefficients, the pole displacement, the quantised response still inside the mask. Programming the device and the swept-sine bench test stay hardware work |

### 3884 — Robot Control and Feedback Systems

No List of Exam Questions is published for 3884. The exam is a 2 h written paper
over LO1-LO5 (`texts/guide.txt`), and the five guided labs are the published
statement of what it will ask. Leverage by learning outcome:

| Outcome / lab | Panel to drive | What the panel gives you |
| :--- | :--- | :--- |
| LO1 — structural diagrams of ACS, their transfer functions, link types, connection types and characteristics | `control_system.loop_anatomy`, `.block_reduce`, `.standard_links` | The loop reduced rule by rule with the closed-loop poles unchanged at each step, and each of the seven elementary links driving the arm on its own |
| LO2 — obtain transfer functions and determine quality indicators (labs T1, T2, T4) | `control_system.plant_joint`, `.freq_response`, `.quality_indices`, `.error_constants` | W(s) from the motor constants; Bode magnitude and phase computed at s = jω with no control toolbox, as lab T2 requires; overshoot, settling, IAE and ITAE across a gain sweep; system type with Kp, Kv, Ka and the gravity droop |
| LO2 — stability (lab T3: Routh range, gain and phase margins, delay erosion) | `control_system.routh`, `.nyquist`, `.bode_margins` | Three independent verdicts on one screen, the arm diverging just above the Routh gain limit, and the phase margin eroding as you add encoder latency |
| LO3 — develop PID controllers and determine their parameters (lab T5) | `control_system.pid`, `.pid_tune`, `.cascade_feedforward` | Each term's contribution as its own trace; pole placement checked against the measured overshoot; zero steady-state error under a load torque P-only control cannot hold |
| LO4 — features of the information devices of robotic systems (Sessions 27-28) | `sensing_models.*` with `dsp_system.fir_design` | Encoder, resolver, tacho, force-torque, IMU and camera on the same motion with their resolution, noise and latency visible, and the cleaning filter shown destabilising the loop through its group delay |
| LO5 — principles of construction and operation of robot control systems (Session 29) | `agent_architecture.hierarchical`, `.reactive` | The same task under both paradigms with an obstacle introduced mid-motion |

### 3286 — Mathematics for Data Analytics · `texts/list_of_questions_exam_premasters_2026.txt` (34 questions)

| Questions | Panel to drive |
| :--- | :--- |
| 1-4 — matrix definitions and classification; unit, zero and equal matrices; transposition; addition and scalar multiplication; matrix multiplication | `linear_algebra_lab.matrix_ops` on the GP8 link transforms |
| 5 — adjacency and incidence matrices of a simple graph | `linear_algebra_lab.adjacency_incidence` on the link-joint tree |
| 6-7 — 2×2 and 3×3 determinants, Laplace expansion for order n, properties of determinants | `linear_algebra_lab.determinant`, `.laplace_expansion` on the 6×6 Jacobian |
| 8 — the inverse of a 2×2 matrix | `linear_algebra_lab.matrix_ops` (inverse) |
| 9 — SLAE by Cramer's rule | `linear_algebra_lab.cramer` on the IK step |
| 10 — SLAE by the inverse-matrix method | `linear_algebra_lab.inverse_method` |
| 11 — SLAE by Gaussian elimination | `linear_algebra_lab.gauss` |
| 12 — flow networks, balance conditions, SLAE applied to them | `linear_algebra_lab.flow_network` on the workcell's part flow |
| 13-16 — random events and sample space; sums, products and complements of events; mutually exclusive events; inclusion-exclusion; the classical definition of probability | `probability_lab.event_algebra` on simulated pick outcomes |
| 17-19 — permutations, partial permutations, combinations | `probability_lab.combinatorics` on pick-order counting |
| 20-21 — probability of the sum of events; conditional probability and the law of total probability | `probability_lab.conditional`, `.total_prob` |
| 22 — Bayes' theorem | `probability_lab.bayes` on a failed pick |
| 23 — discrete random variables: expectation, variance, standard deviation | `probability_lab.discrete_rv` |
| 24-25 — Bernoulli trials and the binomial distribution; the Poisson approximation | `probability_lab.binomial`, `.poisson` |
| 26-27 — the normal distribution, the standard normal, the three-sigma rule; De Moivre-Laplace | `probability_lab.normal`, `.three_sigma`, `.demoivre` on the GP8's repeatability error |
| 28-29 — derivatives and the power rule; stationary points and extrema of a polynomial | `calculus_optimization.derivative`, `.stationary_points` on a joint trajectory |
| 30-32 — indefinite integrals; definite integrals and their geometric interpretation; area between curves | `calculus_optimization.integral`, `.area_between` on the velocity curve and the tracking error |
| 33-34 — functions of several variables and partial derivatives; stationary points of a two-variable function and the minimum/maximum/saddle classification | `calculus_optimization.partial_derivatives`, `.hessian_classify` on the tool-height surface over (θ₂, θ₃) |

### 2953 — Intelligent Systems 1 · `texts/List of Exam Questions.txt` (58 questions)

| Questions | Panel to drive |
| :--- | :--- |
| 1-11, 13 — definitions of intelligent/artificial/business intelligent systems; Copeland's features; weak, strong, general and narrow AI; Turing's role and test; AI business domains; AI's impact on IT, society and economics; knowledge fields of AI | *no panel* — flashcards from the lecture texts (§4) |
| 12, 14-15 — the definition of ML and its relation to AI; kinds of ML; supervised vs unsupervised vs semi-supervised vs reinforcement learning | `learning_models.dataset`, the same robot log labelled three ways |
| 16-23 — the ML pipeline and each phase; the data-preparation procedure; kinds of normalisation; bucketing; categorical transformation and feature engineering; k-fold cross-validation; hold-out; data splitting | `learning_models.prepare`, `.normalise`, `.bucket`, `.split_kfold` |
| 24 — linear regression: problem statement and performance metrics | `learning_models.linear_reg` on cycle time against payload |
| 25-32 — the classification problem and its algorithms; confusion matrix, accuracy, precision, recall, specificity, F1, threshold, ROC, AUC | `learning_models.metrics` with the threshold slider |
| 33 — logistic regression: the sigmoid and the threshold problem | `learning_models.logistic` |
| 34-35 — decision-tree building blocks; random forest creation and parametrisation | `learning_models.decision_tree`, `.random_forest` |
| 36-41 — gradient boosting: additive modelling, bagging vs boosting, AdaBoost M1, GBM, GBM for regression trees, regularisation | `learning_models.gboost` with the round-by-round residual trace |
| 42, 44-51 — ANN definitions and applications; the perceptron's graphical and mathematical model and why it is linear; activation functions and their choice; ANN classification; single-layer and multilayer models in scalar and matrix form; the two reasons for multiple layers; regression in ANN; input and output encoding | `learning_models.perceptron`, `.activation`, `.mlp_train` on the reachable-workspace boundary |
| 52-56 — ANN training as an optimisation problem; gradient-based training for single- and multi-layer networks; momentum; overfitting; kinds of training algorithm | `learning_models.mlp_train`, `.overfit`, with `calculus_optimization.gradient_descent` |
| 57-58 — unsupervised ANN: WTA architecture and training; Kohonen self-organising maps | `learning_models.som_wta` clustering GP8 poses into workspace regions |
| 43 — the history of ANN development | *no panel* — flashcards (§4) |

### 3900 — Trends and Challenges

No examination exists: the assessment is a report (60%) and a presentation (40%).
The leverage is `research_corpus.gap_map`, which forces each proposed research gap
to name the module and the adjustable parameter its investigation would use —
the practical test of learning outcome 2, that the question is researchable
rather than merely interesting.
