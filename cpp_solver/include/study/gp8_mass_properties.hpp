#ifndef YASKAWA_STUDY_GP8_MASS_PROPERTIES_HPP
#define YASKAWA_STUDY_GP8_MASS_PROPERTIES_HPP

// GENERATED FILE - do not edit by hand.
//
// Generator : tools/compute_link_mass_properties.py
// Regenerate: python tools/compute_link_mass_properties.py
// Verify    : python tools/compute_link_mass_properties.py --check
//
// Every number below is integrated from the Yaskawa CAD geometry shipped as the
// upstream ROS-Industrial STL meshes, by exact tetrahedron decomposition of the
// closed triangle surface: signed volume, first moments, second moments, then a
// parallel-axis shift onto the centroid. No box, rod or disc approximation is
// involved. The tensors are then rotated out of the mesh frame (which is the
// URDF link frame) into each link's standard-DH frame, the frame the dynamics
// module works in.
//
// FRAME CONVENTION. Standard (distal) DH: frame i sits at the origin of joint
// i+1 and the axis of joint i is z_{i-1}, NOT z_i. So the com and tensor below
// are in frame i, and the moment of link i about its own rotation axis is
// a^T I a with a = (0, sin alpha_i, cos alpha_i) - it is not simply izz. The
// control module's joint_side_inertia() already takes the axis from
// frames[i-1].col(2), which is the same thing done correctly.
//
// DENSITY. The links are hollow castings, so raw mesh volume times a solid
// density would overestimate the mass badly. One effective density is solved
// instead, such that the seven link masses sum to the published robot mass:
//
//     total enclosed mesh volume    0.020219401 m^3
//     published GP8 robot mass      32.000 kg   (DATASHEET)
//     solved effective density      1582.6384 kg/m^3
//     fill fraction vs aluminium    0.5862   (2700 kg/m^3)
//     fill fraction vs steel        0.2016   (7850 kg/m^3)
//
// DISPUTED PUBLISHED MASS. Yaskawa's own two documents do not agree on what a
// GP8 weighs, and the figure this project carried before was from neither of
// them. All three, with their page citations:
//
//      32.0 kg  DS-699-H page 2, SPECIFICATIONS row Weight, GP8 column
//              -> the numbers in this file are fitted to THIS one.
//      35.0 kg  HW1484385 page 4, Table 5-1 Approx. Mass
//      34.0 kg  DS-699-H page 2, same row, GP7 column - the adjacent robot.
//              gp8_model.hpp carried this as the GP8 mass by mistake, and an
//              earlier generation of this file was fitted to it.
//
// Nothing is averaged and nothing is reconciled by nudging a number. Mass and
// inertia are both linear in the density, so this whole table refitted to the
// manual 35.0 kg is this file multiplied by MANUAL_MASS_SCALE_FACTOR =
// 1.093750000 - every mass and every inertia component, centres of mass unchanged.
// That is why no second table is emitted. Page citations for all three figures
// are in assets/gp8_published_specification.json.
//
// ASSUMPTION, stated plainly: each link is modelled as a solid of uniform
// effective density, the same density for every link. That is a model of the
// mass distribution, not a measurement of it: the true distribution is
// dominated by the motor and gearbox at each joint, which the outer casting
// geometry cannot know about. What IS exact here is the geometry - the volume,
// centroid and shape of the inertia tensor of the real castings - and the
// total, which is pinned to the datasheet.
//
// Source meshes, sha256 of the file as committed:
//     gp8_base_link.stl      704b3b09b4d948f68ade306357e29933453e760716b949fcefacdd1da816e6c4
//     gp8_link_1_s.stl       3bfb02014a83690a7724e6ea378162431d56fca4ff536f9331eea7c7ef340036
//     gp8_link_2_l.stl       521f448f29717011e6a0f89d9f9429b4f95e1439fb6938ee40ed4c4669156272
//     gp8_link_3_u.stl       13d212cc99fc47adf4e09054dc089be3b53f88c3da4293b5a64769de587ab986
//     gp8_link_4_r.stl       7b16986bf8a07f8d4b4dc6fb61debe40f614aeaf74c71bd20830b2f27163b128
//     gp8_link_5_b.stl       e6bfa28b82735be7b0f8ed7c39d22b6ad4da184fd6f76c899b525b0ea8c457fd
//     gp8_link_6_t.stl       96b447d0b63871ef6a3a77521e49a54e577b59c7ec59cadf6966e58260fbd0dc
//
// Self-checks the generator runs and refuses to emit without: total mass equal
// to the published mass to 1e-9; every tensor symmetric, positive definite and
// obeying the triangle inequality on its principal moments; every centroid
// inside its mesh bounding box; the mesh-to-DH transform constant over joint
// space; and an analytic cross-check of the integrator against a cube and a
// sphere of known closed-form inertia.

#include <array>
#include <cstddef>

namespace yaskawa::study::massprops {

// The published mass the density was fitted to [kg]: the GP8 datasheet figure.
inline constexpr double FITTED_TOTAL_MASS_KG = 3.200000000000e+01;

// The competing figure from Yaskawa's instruction manual [kg], and the single
// factor that carries every mass and every inertia component in this file onto
// it. Centres of mass do not scale. Provided so a module can report the spread
// without a second table existing to drift out of step with this one.
inline constexpr double MANUAL_TOTAL_MASS_KG = 3.500000000000e+01;
inline constexpr double MANUAL_MASS_SCALE_FACTOR = 1.093750000000e+00;

// The GP7 column figure [kg] this project used as the GP8 mass by mistake. Named
// so a reader who meets it in an older commit knows what it was.
inline constexpr double GP7_COLUMN_MASS_KG = 3.400000000000e+01;

// Sum of the enclosed volumes of the seven meshes [m^3].
inline constexpr double TOTAL_MESH_VOLUME_M3 = 2.021940095167e-02;

// The one effective density that reproduces FITTED_TOTAL_MASS_KG [kg/m^3].
inline constexpr double EFFECTIVE_DENSITY_KG_M3 = 1.582638381646e+03;

// Implied fill fractions of that density against solid metal.
inline constexpr double FILL_FRACTION_ALUMINIUM = 5.861623635726e-01;
inline constexpr double FILL_FRACTION_STEEL = 2.016099849231e-01;

// Mass properties of one link, integrated from one mesh.
struct LinkMassProperties {
    const char* mesh;              // the STL the numbers were integrated from
    const char* axis;              // Yaskawa axis letter, "base" for the pedestal
    double volume;                 // [m^3] enclosed mesh volume
    double mass;                   // [kg]  volume * EFFECTIVE_DENSITY_KG_M3
    std::array<double, 3> com;     // [m]   centre of mass in the link's DH frame
    double ixx;                    // [kg m^2] about the com, DH frame axes
    double iyy;
    double izz;
    double ixy;
    double ixz;
    double iyz;
};

// The six moving links, index 0..5 = axes S, L, U, R, B, T, each expressed in
// that link's own standard-DH frame - see the FRAME CONVENTION note above for
// which direction that link actually rotates about.
inline constexpr std::array<LinkMassProperties, 6> GP8_MOVING_LINKS = {{
    {
        "gp8_link_1_s.stl", "S",
        3.413871534915e-03,  // volume [m^3], integrated from gp8_link_1_s.stl
        5.402924121165e+00,  // mass [kg] = volume * effective density
        {-2.493216046587e-02, 4.383369634975e-02, -2.631679487988e-05},  // com [m], DH frame 1, from gp8_link_1_s.stl
        2.276791241309e-02,  // ixx [kg m^2] about com, DH frame 1, from gp8_link_1_s.stl
        1.800279622478e-02,  // iyy, from gp8_link_1_s.stl
        2.519636528752e-02,  // izz, from gp8_link_1_s.stl
        3.893082400549e-03,  // ixy, from gp8_link_1_s.stl
        -6.906499423633e-06,  // ixz, from gp8_link_1_s.stl
        1.604986447213e-05,  // iyz, from gp8_link_1_s.stl
    },
    {
        "gp8_link_2_l.stl", "L",
        5.545374918672e-03,  // volume [m^3], integrated from gp8_link_2_l.stl
        8.776323186908e+00,  // mass [kg] = volume * effective density
        {-1.848985732958e-01, -2.824494892296e-02, 3.106761840758e-04},  // com [m], DH frame 2, from gp8_link_2_l.stl
        6.470676983693e-02,  // ixx [kg m^2] about com, DH frame 2, from gp8_link_2_l.stl
        1.531078819961e-01,  // iyy, from gp8_link_2_l.stl
        1.078477564311e-01,  // izz, from gp8_link_2_l.stl
        1.994800975106e-03,  // ixy, from gp8_link_2_l.stl
        -5.428984654016e-04,  // ixz, from gp8_link_2_l.stl
        -6.690179284935e-05,  // iyz, from gp8_link_2_l.stl
    },
    {
        "gp8_link_3_u.stl", "U",
        2.907807253539e-03,  // volume [m^3], integrated from gp8_link_3_u.stl
        4.602007365880e+00,  // mass [kg] = volume * effective density
        {-2.059778188851e-02, 7.347972769282e-04, -1.762046953474e-02},  // com [m], DH frame 3, from gp8_link_3_u.stl
        1.534269557992e-02,  // ixx [kg m^2] about com, DH frame 3, from gp8_link_3_u.stl
        1.650414050688e-02,  // iyy, from gp8_link_3_u.stl
        1.371123205094e-02,  // izz, from gp8_link_3_u.stl
        6.059860480926e-05,  // ixy, from gp8_link_3_u.stl
        -1.055860439340e-03,  // ixz, from gp8_link_3_u.stl
        -1.140697164534e-04,  // iyz, from gp8_link_3_u.stl
    },
    {
        "gp8_link_4_r.stl", "R",
        2.187557559086e-03,  // volume [m^3], integrated from gp8_link_4_r.stl
        3.462112555070e+00,  // mass [kg] = volume * effective density
        {-4.317905501007e-05, -1.489468982103e-01, -9.337172981827e-05},  // com [m], DH frame 4, from gp8_link_4_r.stl
        2.118134423164e-02,  // ixx [kg m^2] about com, DH frame 4, from gp8_link_4_r.stl
        5.743550779949e-03,  // iyy, from gp8_link_4_r.stl
        2.024393096075e-02,  // izz, from gp8_link_4_r.stl
        -5.747691452043e-08,  // ixy, from gp8_link_4_r.stl
        -9.286767437668e-07,  // ixz, from gp8_link_4_r.stl
        8.021398134230e-06,  // iyz, from gp8_link_4_r.stl
    },
    {
        "gp8_link_5_b.stl", "B",
        4.040564625744e-04,  // volume [m^3], integrated from gp8_link_5_b.stl
        6.394752660223e-01,  // mass [kg] = volume * effective density
        {1.677250726636e-04, -1.664101943913e-05, 1.756923033865e-02},  // com [m], DH frame 5, from gp8_link_5_b.stl
        7.499690583425e-04,  // ixx [kg m^2] about com, DH frame 5, from gp8_link_5_b.stl
        7.759749504082e-04,  // iyy, from gp8_link_5_b.stl
        4.158368826999e-04,  // izz, from gp8_link_5_b.stl
        -6.497004612947e-07,  // ixy, from gp8_link_5_b.stl
        -2.098688033326e-06,  // ixz, from gp8_link_5_b.stl
        -3.074778661803e-07,  // iyz, from gp8_link_5_b.stl
    },
    {
        "gp8_link_6_t.stl", "T",
        2.507733058272e-05,  // volume [m^3], integrated from gp8_link_6_t.stl
        3.968834588944e-02,  // mass [kg] = volume * effective density
        {5.562482253414e-05, 1.091542286446e-07, 7.405778739624e-02},  // com [m], DH frame 6, from gp8_link_6_t.stl
        7.710494345983e-06,  // ixx [kg m^2] about com, DH frame 6, from gp8_link_6_t.stl
        7.663540354182e-06,  // iyy, from gp8_link_6_t.stl
        1.425915821178e-05,  // izz, from gp8_link_6_t.stl
        -5.739420504985e-09,  // ixy, from gp8_link_6_t.stl
        -7.325558455415e-09,  // ixz, from gp8_link_6_t.stl
        -1.197916791489e-09,  // iyz, from gp8_link_6_t.stl
    },
}};

// The pedestal. It does not move, so it carries no DH frame: its centre of mass
// and inertia are given in the base frame, which is also its mesh frame. It is
// part of the 32 kg the density was fitted to, and no part of the moving
// chain, which is why the six masses above sum to less than the robot mass.
inline constexpr LinkMassProperties GP8_BASE_LINK = {
    "gp8_base_link.stl", "base",
    5.735655892299e-03,  // volume [m^3]
    9.077469159066e+00,  // mass [kg]
    {-5.797216918896e-03, -3.576146248709e-05, 9.380115837546e-02},  // com [m], base frame
    4.723219311837e-02,  // ixx [kg m^2] about com, base frame axes
    5.210778689126e-02,  // iyy
    4.524977790199e-02,  // izz
    -1.069346090645e-05,  // ixy
    -4.166083940719e-04,  // ixz
    -2.005463833941e-05,  // iyz
};

// The six independent components expanded into a row-major 3x3 tensor.
[[nodiscard]] constexpr std::array<double, 9> inertia_matrix(
    const LinkMassProperties& p) noexcept {
    return {p.ixx, p.ixy, p.ixz,
            p.ixy, p.iyy, p.iyz,
            p.ixz, p.iyz, p.izz};
}

[[nodiscard]] constexpr double moving_mass_sum() noexcept {
    double sum = 0.0;
    for (std::size_t i = 0; i < GP8_MOVING_LINKS.size(); ++i) {
        sum += GP8_MOVING_LINKS[i].mass;
    }
    return sum;
}

[[nodiscard]] constexpr double fitted_mass_sum() noexcept {
    return moving_mass_sum() + GP8_BASE_LINK.mass;
}

// The fit the generator solved for, re-checked by the compiler.
static_assert(fitted_mass_sum() > FITTED_TOTAL_MASS_KG - 1.0e-9 &&
                  fitted_mass_sum() < FITTED_TOTAL_MASS_KG + 1.0e-9,
              "the seven integrated link masses must sum to the published mass");
static_assert(moving_mass_sum() < FITTED_TOTAL_MASS_KG,
              "the moving chain cannot outweigh the whole robot");

}  // namespace yaskawa::study::massprops

#endif  // YASKAWA_STUDY_GP8_MASS_PROPERTIES_HPP
