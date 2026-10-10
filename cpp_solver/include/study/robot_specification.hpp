#ifndef YASKAWA_STUDY_ROBOT_SPECIFICATION_HPP
#define YASKAWA_STUDY_ROBOT_SPECIFICATION_HPP

// Course 3883 "Robotics Modelling" (M-407-01), Block 6 "Model validation
// against engineering specifications", and course 3884 "Robot Control and
// Feedback Systems" (M-408-01) for the actuator and wrist load limits.
//
// What this module is for: making the *published* GP8 specification visible
// next to what the engine actually does with it, so a published number and an
// engine constant are never confused for one another.
//
//   PUBLISHED  Every value in `published_fields()` was read off a Yaskawa
//              document and carries that document's id and the page it was
//              read on. The same values, with the same citations, are
//              committed as assets/gp8_published_specification.json;
//              test_study_specification.cpp asserts the two agree field by
//              field, so the asset cannot drift away from the code.
//
// The two documents, both fetched and read in full:
//   DS-699-H    "GP7 and GP8 Robots", Yaskawa America datasheet, 2019-05, 2 pp.
//               Two columns, GP7 and GP8. Only the GP8 column is used here.
//   HW1484385   "MOTOMAN-GP8, -GP7 Supplemental Instructions", Yaskawa
//               Electric, 21 pp. Table 5-1 (GP8) on page 4, Table 3-3 Ambient
//               Conditions on page 3.
// Where the two disagree - robot mass, repeatability, the L- and U-axis ranges
// - both values are kept with their own citation and `axis_limits` prints the
// disagreement rather than choosing a winner.

#include "study/json.hpp"
#include "study/study_module.hpp"

#include <Eigen/Dense>

#include <array>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace yaskawa::study {

namespace specification {

// Repository-relative path of the committed specification asset. The module
// itself does not need the file at run time - every published value is compiled
// in - but the loader below exists so the test can prove the two agree.
inline constexpr const char* kAssetRelativePath = "assets/gp8_published_specification.json";

// ---------------------------------------------------------------------------
// The published specification, one record per cited field
// ---------------------------------------------------------------------------

struct DocumentRef {
    const char* id;             // the id used in every citation below
    const char* title;
    const char* manual_number;  // Yaskawa manual number, "" when it has none
    const char* document_number;
    const char* publisher;
    const char* url;
    int page_count;
};

struct SpecField {
    const char* path;        // JSON path into the committed asset
    const char* group;       // datasheet filter: general, electrical, ...
    const char* parameter;   // human label for a table row
    bool numeric;            // false when the document prints text, not a number
    double value;            // valid when numeric
    const char* text;        // what the document prints when not numeric
    const char* unit;        // always present, "" for a dimensionless count
    const char* document;    // DocumentRef::id
    int page;                // the page the value was read on, 1-based
    const char* tolerance;   // "" when the document states none
    const char* note;        // "" when there is nothing to warn about
};

[[nodiscard]] std::span<const DocumentRef> documents() noexcept;
[[nodiscard]] std::span<const SpecField> published_fields() noexcept;

// The `group` values in declaration order, for the datasheet op's enum.
[[nodiscard]] const std::vector<std::string>& field_groups();

// Throws StudyError when the path is not a published field.
[[nodiscard]] const SpecField& published_field(std::string_view path);
[[nodiscard]] double published_value(std::string_view path);

// ---------------------------------------------------------------------------
// Per-axis published ratings, assembled from published_fields()
// ---------------------------------------------------------------------------

struct AxisPublished {
    const char* axis;              // S, L, U, R, B, T
    const char* name;              // as the manual names the joint
    double datasheet_min_deg;      // DS-699-H page 2
    double datasheet_max_deg;      // DS-699-H page 2
    double manual_min_deg;         // HW1484385 page 4
    double manual_max_deg;         // HW1484385 page 4
    double datasheet_speed_deg_s;  // DS-699-H page 2
    double manual_speed_rad_s;     // HW1484385 page 4
    bool has_wrist_rating;         // only R, B and T are rated
    double allowable_moment_nm;
    double allowable_inertia_kgm2;
};

[[nodiscard]] const std::array<AxisPublished, 6>& published_axes();

// ---------------------------------------------------------------------------
// Wrist capacity: a payload checked against the R/B/T ratings
// ---------------------------------------------------------------------------

struct WristAxisLoad {
    const char* axis;
    double lever_arm_m;            // payload offset perpendicular to this axis
    double moment_nm;              // worst-case gravity moment about the axis
    double allowable_moment_nm;    // published rating
    double moment_utilisation;     // moment / rating
    double moment_margin_nm;       // rating - moment, negative when over
    double inertia_kgm2;           // payload inertia about the axis
    double allowable_inertia_kgm2; // published rating
    double inertia_utilisation;
    double inertia_margin_kgm2;
    bool moment_ok;
    bool inertia_ok;
    bool ok;
};

struct WristAssessment {
    std::array<WristAxisLoad, 3> axes{};  // R, B, T in that order
    double payload_mass_kg = 0.0;
    double payload_margin_kg = 0.0;       // rated payload - mass
    bool mass_ok = false;
    bool pass = false;                    // mass and all three axes within rating
};

// Worst case over the gravity direction: for an axis with unit vector a and a
// payload centre of mass at r from the flange, the largest possible gravity
// moment about a is m g |r_perp|, with r_perp the component of r perpendicular
// to a, and the payload's moment of inertia about a is J_own + m |r_perp|^2.
// `b_angle` is the B-axis angle, which is what tilts the R axis away from the
// flange axis; at b_angle = 0 the R and T axes are collinear and see the same
// lever arm, exactly as the real wrist does.
[[nodiscard]] WristAssessment evaluate_wrist(double payload_mass_kg,
                                             const Eigen::Vector3d& offset_m,
                                             double payload_inertia_kgm2,
                                             double b_angle_rad);

// ---------------------------------------------------------------------------
// The committed asset, found relative to the repository root
// ---------------------------------------------------------------------------

// Walks up from the current working directory looking for
// kAssetRelativePath. Throws StudyError naming every directory it tried when
// the file is absent - it never crashes and never returns an empty path.
[[nodiscard]] std::string find_asset_path();

// Reads and parses the asset at an explicit path. Throws StudyError when the
// file is missing or is not valid JSON.
[[nodiscard]] json::Value load_asset(const std::string& path);

// find_asset_path() followed by load_asset().
[[nodiscard]] json::Value load_asset();

}  // namespace specification

class RobotSpecificationModule final : public StudyModule {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "robot_specification"; }
    [[nodiscard]] ModuleDescription describe() const override;
    [[nodiscard]] json::Value invoke(std::string_view op, const json::Value& args) const override;
};

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_ROBOT_SPECIFICATION_HPP
