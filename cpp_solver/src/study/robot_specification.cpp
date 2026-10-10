#include "study/robot_specification.hpp"

#include "robot_physical_tree.hpp"
#include "study/gp8_model.hpp"
#include "yaskawa_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iterator>
#include <numbers>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace yaskawa::study {

namespace specification {

namespace {

constexpr double kRadToDeg = 180.0 / std::numbers::pi;
constexpr double kDegToRad = std::numbers::pi / 180.0;

// How far up from the working directory the asset search walks before giving
// up. The repository root is one or two levels above any build directory.
constexpr int kAssetSearchDepth = 8;

constexpr std::array<DocumentRef, 2> kDocuments = {{
    {
        "DS-699-H",
        "GP7 and GP8 Robots - Compact, High-Speed Robots (product datasheet)",
        "",  // a datasheet carries no Yaskawa manual number
        "DS-699-H",
        "Yaskawa America, Inc. - Motoman Robotics Division",
        "https://cdn.logic-control.com/media/gp7_gp8.pdf",
        2,
    },
    {
        "HW1484385",
        "MOTOMAN-GP8, -GP7 Supplemental Instructions",
        "HW1484385",
        "180382-1CD",
        "Yaskawa Electric Corporation",
        "https://res.cloudinary.com/reshape-prod/image/upload/v1706300926/products/documents/"
        "o8go5smyhvld5yonsave.pdf",
        21,
    },
}};

// Every published field, in the order a panel should read them. Generated from
// assets/gp8_published_specification.json and asserted equal to it, field by
// field, in cpp_solver/src/test_study_specification.cpp.
constexpr SpecField kFields[] = {
    {"general.controlled_axes", "general", "Controlled axes", true, 6.0, "", "", "DS-699-H", 2, "", ""},
    {"general.payload", "general", "Maximum payload at the wrist", true, 8.0, "", "kg", "DS-699-H", 2, "", "Wrist payload. The manual repeats it on page 4 as Payload / Wrist part 8 kg."},
    {"general.payload_u_arm", "general", "Allowable U-arm load", true, 1.0, "", "kg", "HW1484385", 4, "", "The allowable U-arm load varies with the wrist load; the manual refers to chapter 7.1.1."},
    {"general.horizontal_reach", "general", "Horizontal reach", true, 727.0, "", "mm", "DS-699-H", 2, "", "The dimension drawing on the same page prints it as R727."},
    {"general.vertical_reach", "general", "Vertical reach", true, 1312.0, "", "mm", "DS-699-H", 2, "", ""},
    {"general.repeatability", "general", "Repeatability (datasheet)", true, 0.02, "", "mm", "DS-699-H", 2, "+/-0.02 mm", "Page 1 of the same datasheet repeats +/-0.02 mm repeatability for the GP8."},
    {"general.repeatability_manual", "general", "Repeatability (manual, ISO 9283)", true, 0.01, "", "mm", "HW1484385", 4, "", "Conformed to ISO 9283. This disagrees with DS-699-H, which prints +/-0.02 mm; both values are kept with their own citation rather than averaged."},
    {"general.mass", "general", "Robot mass (datasheet, GP8 column)", true, 32.0, "", "kg", "DS-699-H", 2, "", "GP8 column. The GP7 column on the same row reads 34 kg; this project used that GP7 figure as the GP8 mass until the datasheet was read, and cpp_solver/include/study/gp8_model.hpp now carries the 32 kg GP8 value."},
    {"general.mass_manual", "general", "Robot mass (manual, approximate)", true, 35.0, "", "kg", "HW1484385", 4, "", "Approximate mass of type YR-1-06VX8-F00. Disagrees with the 32 kg of DS-699-H."},
    {"general.noise", "general", "Noise, A-weighted sound pressure", true, 75.0, "", "dB", "HW1484385", 4, "or less", "Equivalent continuous A-weighted sound pressure level to ISO 11201, at maximum load and maximum speed."},
    {"general.controller", "general", "Applicable controller", false, 0.0, "YRC1000 (ERAR-1000-06VX8) or YRC1000micro (ERBR-100-06VX8)", "", "HW1484385", 4, "", ""},
    {"electrical.supply_voltage_min", "electrical", "Supply voltage, lower", true, 380.0, "", "VAC", "DS-699-H", 2, "", ""},
    {"electrical.supply_voltage_max", "electrical", "Supply voltage, upper", true, 480.0, "", "VAC", "DS-699-H", 2, "", ""},
    {"electrical.power_capacity", "electrical", "Power capacity", true, 1.0, "", "kVA", "DS-699-H", 2, "", "The manual prints the same 1 kVA on page 4 as Power Capacity."},
    {"electrical.internal_user_io_conductors", "electrical", "Internal user I/O conductors", true, 17.0, "", "", "DS-699-H", 2, "", "17 conductors plus ground, through connector LF13WBRB-20S / LF13WBRB-20P."},
    {"electrical.internal_user_air_lines", "electrical", "Internal user air lines", true, 2.0, "", "", "DS-699-H", 2, "", "Two 1/4 inch connections, tapped Rc1/4 with pipe plug."},
    {"protection.wrist", "protection", "Degree of protection, wrist", false, 0.0, "IP67", "", "DS-699-H", 1, "", ""},
    {"protection.body_standard", "protection", "Degree of protection, body (standard)", false, 0.0, "IP54", "", "DS-699-H", 1, "", ""},
    {"protection.body_xp_package", "protection", "Degree of protection, body (XP package)", false, 0.0, "IP65", "", "DS-699-H", 1, "", "Optional eXtra Protection package. Page 2 warns that specifications for the XP package may differ."},
    {"protection.enclosure_manual", "protection", "Protective enclosure (manual)", false, 0.0, "IP67", "", "HW1484385", 4, "", "The manual states IP67 for the whole manipulator, where the datasheet splits IP67 wrist from IP54 body."},
    {"mounting.options", "mounting", "Mounting options", false, 0.0, "Floor, wall, tilt or ceiling", "", "DS-699-H", 2, "", "The manual adds on page 4 that the S-axis operating range is limited for the tilt- and wall-mounted cases."},
    {"ambient.temperature_min", "ambient", "Ambient temperature, minimum", true, 15.0, "", "degC", "HW1484385", 3, "", ""},
    {"ambient.temperature_max", "ambient", "Ambient temperature, maximum", true, 45.0, "", "degC", "HW1484385", 3, "", ""},
    {"ambient.humidity_min", "ambient", "Relative humidity, minimum", true, 20.0, "", "%RH", "HW1484385", 3, "", ""},
    {"ambient.humidity_max", "ambient", "Relative humidity, maximum", true, 80.0, "", "%RH", "HW1484385", 3, "", "No condensation."},
    {"ambient.vibration_max", "ambient", "Vibration, maximum", true, 4.9, "", "m/s^2", "HW1484385", 3, "or less", "Printed as 4.9 m/s2 (0.5G) or less."},
    {"ambient.altitude_max", "ambient", "Altitude, maximum", true, 1000.0, "", "m", "HW1484385", 3, "or less", ""},
    {"ambient.installation_flatness_max", "ambient", "Flatness for installation", true, 0.5, "", "mm", "HW1484385", 3, "or less", ""},
    {"axes[0].motion_range_min", "axis_motion", "S-axis motion range, lower (datasheet)", true, -170.0, "", "deg", "DS-699-H", 2, "", ""},
    {"axes[0].motion_range_max", "axis_motion", "S-axis motion range, upper (datasheet)", true, 170.0, "", "deg", "DS-699-H", 2, "", ""},
    {"axes[0].motion_range_min_manual", "axis_motion", "S-axis motion range, lower (manual)", true, -170.0, "", "deg", "HW1484385", 4, "", ""},
    {"axes[0].motion_range_max_manual", "axis_motion", "S-axis motion range, upper (manual)", true, 170.0, "", "deg", "HW1484385", 4, "", ""},
    {"axes[0].max_speed", "axis_motion", "S-axis maximum speed (datasheet)", true, 455.0, "", "deg/s", "DS-699-H", 2, "", ""},
    {"axes[0].max_speed_manual", "axis_motion", "S-axis maximum speed (manual)", true, 7.94, "", "rad/s", "HW1484385", 4, "", ""},
    {"axes[1].motion_range_min", "axis_motion", "L-axis motion range, lower (datasheet)", true, -65.0, "", "deg", "DS-699-H", 2, "", ""},
    {"axes[1].motion_range_max", "axis_motion", "L-axis motion range, upper (datasheet)", true, 150.0, "", "deg", "DS-699-H", 2, "", ""},
    {"axes[1].motion_range_min_manual", "axis_motion", "L-axis motion range, lower (manual)", true, -65.0, "", "deg", "HW1484385", 4, "", ""},
    {"axes[1].motion_range_max_manual", "axis_motion", "L-axis motion range, upper (manual)", true, 145.0, "", "deg", "HW1484385", 4, "", ""},
    {"axes[1].max_speed", "axis_motion", "L-axis maximum speed (datasheet)", true, 385.0, "", "deg/s", "DS-699-H", 2, "", ""},
    {"axes[1].max_speed_manual", "axis_motion", "L-axis maximum speed (manual)", true, 6.72, "", "rad/s", "HW1484385", 4, "", ""},
    {"axes[2].motion_range_min", "axis_motion", "U-axis motion range, lower (datasheet)", true, -113.0, "", "deg", "DS-699-H", 2, "", ""},
    {"axes[2].motion_range_max", "axis_motion", "U-axis motion range, upper (datasheet)", true, 255.0, "", "deg", "DS-699-H", 2, "", ""},
    {"axes[2].motion_range_min_manual", "axis_motion", "U-axis motion range, lower (manual)", true, -70.0, "", "deg", "HW1484385", 4, "", ""},
    {"axes[2].motion_range_max_manual", "axis_motion", "U-axis motion range, upper (manual)", true, 190.0, "", "deg", "HW1484385", 4, "", ""},
    {"axes[2].max_speed", "axis_motion", "U-axis maximum speed (datasheet)", true, 520.0, "", "deg/s", "DS-699-H", 2, "", ""},
    {"axes[2].max_speed_manual", "axis_motion", "U-axis maximum speed (manual)", true, 9.07, "", "rad/s", "HW1484385", 4, "", ""},
    {"axes[3].motion_range_min", "axis_motion", "R-axis motion range, lower (datasheet)", true, -190.0, "", "deg", "DS-699-H", 2, "", ""},
    {"axes[3].motion_range_max", "axis_motion", "R-axis motion range, upper (datasheet)", true, 190.0, "", "deg", "DS-699-H", 2, "", ""},
    {"axes[3].motion_range_min_manual", "axis_motion", "R-axis motion range, lower (manual)", true, -190.0, "", "deg", "HW1484385", 4, "", ""},
    {"axes[3].motion_range_max_manual", "axis_motion", "R-axis motion range, upper (manual)", true, 190.0, "", "deg", "HW1484385", 4, "", ""},
    {"axes[3].max_speed", "axis_motion", "R-axis maximum speed (datasheet)", true, 550.0, "", "deg/s", "DS-699-H", 2, "", ""},
    {"axes[3].max_speed_manual", "axis_motion", "R-axis maximum speed (manual)", true, 9.59, "", "rad/s", "HW1484385", 4, "", ""},
    {"axes[3].allowable_moment", "wrist_rating", "R-axis allowable moment", true, 17.0, "", "N m", "DS-699-H", 2, "", "The manual prints the same value on page 4 as 17 N m (1.73 kgf m)."},
    {"axes[3].allowable_moment_of_inertia", "wrist_rating", "R-axis allowable moment of inertia", true, 0.5, "", "kg m^2", "DS-699-H", 2, "", "The manual prints it as Allowable Inertia (GD2/4) on page 4."},
    {"axes[4].motion_range_min", "axis_motion", "B-axis motion range, lower (datasheet)", true, -135.0, "", "deg", "DS-699-H", 2, "", ""},
    {"axes[4].motion_range_max", "axis_motion", "B-axis motion range, upper (datasheet)", true, 135.0, "", "deg", "DS-699-H", 2, "", ""},
    {"axes[4].motion_range_min_manual", "axis_motion", "B-axis motion range, lower (manual)", true, -135.0, "", "deg", "HW1484385", 4, "", ""},
    {"axes[4].motion_range_max_manual", "axis_motion", "B-axis motion range, upper (manual)", true, 135.0, "", "deg", "HW1484385", 4, "", ""},
    {"axes[4].max_speed", "axis_motion", "B-axis maximum speed (datasheet)", true, 550.0, "", "deg/s", "DS-699-H", 2, "", ""},
    {"axes[4].max_speed_manual", "axis_motion", "B-axis maximum speed (manual)", true, 9.59, "", "rad/s", "HW1484385", 4, "", ""},
    {"axes[4].allowable_moment", "wrist_rating", "B-axis allowable moment", true, 17.0, "", "N m", "DS-699-H", 2, "", "The manual prints the same value on page 4 as 17 N m (1.73 kgf m)."},
    {"axes[4].allowable_moment_of_inertia", "wrist_rating", "B-axis allowable moment of inertia", true, 0.5, "", "kg m^2", "DS-699-H", 2, "", "The manual prints it as Allowable Inertia (GD2/4) on page 4."},
    {"axes[5].motion_range_min", "axis_motion", "T-axis motion range, lower (datasheet)", true, -360.0, "", "deg", "DS-699-H", 2, "", ""},
    {"axes[5].motion_range_max", "axis_motion", "T-axis motion range, upper (datasheet)", true, 360.0, "", "deg", "DS-699-H", 2, "", ""},
    {"axes[5].motion_range_min_manual", "axis_motion", "T-axis motion range, lower (manual)", true, -360.0, "", "deg", "HW1484385", 4, "", ""},
    {"axes[5].motion_range_max_manual", "axis_motion", "T-axis motion range, upper (manual)", true, 360.0, "", "deg", "HW1484385", 4, "", ""},
    {"axes[5].max_speed", "axis_motion", "T-axis maximum speed (datasheet)", true, 1000.0, "", "deg/s", "DS-699-H", 2, "", ""},
    {"axes[5].max_speed_manual", "axis_motion", "T-axis maximum speed (manual)", true, 17.45, "", "rad/s", "HW1484385", 4, "", ""},
    {"axes[5].allowable_moment", "wrist_rating", "T-axis allowable moment", true, 10.0, "", "N m", "DS-699-H", 2, "", "The manual prints the same value on page 4 as 10 N m (1.02 kgf m)."},
    {"axes[5].allowable_moment_of_inertia", "wrist_rating", "T-axis allowable moment of inertia", true, 0.2, "", "kg m^2", "DS-699-H", 2, "", "The manual prints it as Allowable Inertia (GD2/4) on page 4."},
};

[[nodiscard]] const std::unordered_map<std::string, const SpecField*>& field_index() {
    static const std::unordered_map<std::string, const SpecField*> index = [] {
        std::unordered_map<std::string, const SpecField*> out;
        out.reserve(std::size(kFields));
        for (const SpecField& field : kFields) {
            out.emplace(field.path, &field);
        }
        return out;
    }();
    return index;
}

[[nodiscard]] std::string axis_path(std::size_t axis, const char* leaf) {
    return "axes[" + std::to_string(axis) + "]." + leaf;
}

[[nodiscard]] std::string fixed(double value, int digits) {
    std::ostringstream out;
    out.setf(std::ios::fixed, std::ios::floatfield);
    out.precision(digits);
    out << value;
    return out.str();
}

// ---------------------------------------------------------------------------
// The MODELLED component tree, indexed once
// ---------------------------------------------------------------------------

struct TreeCache {
    std::size_t total = 0;
    std::vector<bool> is_leaf;            // by component index
    std::vector<int> category;            // by component index
    std::vector<SubsystemMass> subsystems;  // by category id, ascending
    std::vector<std::string> options;     // "all" then every subsystem name
    double leaf_mass_kg = 0.0;            // MODELLED, every leaf in the tree
};

[[nodiscard]] const TreeCache& tree_cache() {
    static const TreeCache cache = [] {
        TreeCache out;
        out.total = physical::get_total_component_count();
        out.is_leaf.assign(out.total, true);
        out.category.assign(out.total, 0);

        // Pass 1: who is a parent, which subsystem each node belongs to, and
        // the shallowest level each subsystem reaches.
        std::unordered_set<std::string> parents;
        parents.reserve(out.total);
        int max_category = 0;
        std::vector<std::uint8_t> levels(out.total, 0);
        for (std::size_t i = 0; i < out.total; ++i) {
            const physical::ComponentPtr component = physical::get_component_at(i);
            if (component == nullptr) {
                continue;
            }
            if (!component->parent_id.empty()) {
                parents.insert(component->parent_id);
            }
            const int category = static_cast<int>(component->system_category_id);
            out.category[i] = category;
            levels[i] = static_cast<std::uint8_t>(component->level);
            max_category = std::max(max_category, category);
        }

        out.subsystems.resize(static_cast<std::size_t>(max_category) + 1);
        for (std::size_t id = 0; id < out.subsystems.size(); ++id) {
            out.subsystems[id].category_id = static_cast<int>(id);
        }
        std::vector<std::uint8_t> min_level(out.subsystems.size(), 255);
        for (std::size_t i = 0; i < out.total; ++i) {
            std::uint8_t& lowest = min_level[static_cast<std::size_t>(out.category[i])];
            lowest = std::min(lowest, levels[i]);
        }

        // Pass 2: counts, leaf masses, and the mass each subsystem declares for
        // itself on its own assembly node. The hierarchy puts a mass on
        // assemblies as well as on leaves, so the two must never be added
        // together: that would count every parent twice.
        for (std::size_t i = 0; i < out.total; ++i) {
            const physical::ComponentPtr component = physical::get_component_at(i);
            if (component == nullptr) {
                continue;
            }
            const std::size_t id = static_cast<std::size_t>(out.category[i]);
            SubsystemMass& subsystem = out.subsystems[id];
            if (subsystem.name.empty()) {
                subsystem.name = component->system_name;
            }
            ++subsystem.component_count;
            out.is_leaf[i] = (parents.find(component->id) == parents.end());
            if (out.is_leaf[i]) {
                ++subsystem.leaf_count;
                subsystem.leaf_mass_kg += component->mass_kg;
                out.leaf_mass_kg += component->mass_kg;
            }
            if (levels[i] == min_level[id]) {
                subsystem.declared_assembly_mass_kg += component->mass_kg;
            }
        }

        out.options.reserve(out.subsystems.size() + 1);
        out.options.emplace_back("all");
        for (const SubsystemMass& subsystem : out.subsystems) {
            if (!subsystem.name.empty()) {
                out.options.push_back(subsystem.name);
            }
        }
        return out;
    }();
    return cache;
}

[[nodiscard]] int category_of_option(const std::string& option) {
    for (const SubsystemMass& subsystem : tree_cache().subsystems) {
        if (subsystem.name == option) {
            return subsystem.category_id;
        }
    }
    return -1;  // "all"
}

// The six moving links of gp8_model.hpp, in the order the tree numbers their
// subsystems: category 1 is the S-axis assembly, 2 the L-axis, and so on.
constexpr std::array<int, GP8_DOF> kLinkCategory = {1, 2, 3, 4, 5, 6};

// The joint names as HW1484385 Table 5-1 prints them, mirroring the "name"
// field of each axis in the committed asset.
constexpr std::array<const char*, GP8_DOF> kAxisJointNames = {
    "Swivel base (turning)", "Lower arm",              "Upper arm",
    "Arm roll (wrist roll)", "Wrist bend (pitch/yaw)", "Tool flange twist",
};

// ---------------------------------------------------------------------------
// Ops
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value field_value_cell(const SpecField& field) {
    return field.numeric ? json::Value(field.value) : json::Value(std::string(field.text));
}

[[nodiscard]] json::Value sources_table() {
    std::vector<std::vector<json::Value>> rows;
    rows.reserve(kDocuments.size());
    for (const DocumentRef& document : kDocuments) {
        rows.push_back({
            json::Value(std::string(document.id)),
            json::Value(std::string(document.title)),
            json::Value(std::string(document.manual_number)),
            json::Value(std::string(document.document_number)),
            json::Value(std::string(document.publisher)),
            json::Value(document.page_count),
            json::Value(std::string(document.url)),
        });
    }
    return json::from_table(
        {"id", "title", "manual_number", "document_number", "publisher", "pages", "url"}, rows);
}

[[nodiscard]] json::Value op_datasheet(const json::Value& args) {
    std::vector<std::string> options;
    options.reserve(field_groups().size() + 1);
    options.emplace_back("all");
    for (const std::string& group : field_groups()) {
        options.push_back(group);
    }
    const std::string section = optional_enum(args, "section", "all", options);

    std::vector<std::vector<json::Value>> rows;
    rows.reserve(std::size(kFields));
    std::size_t numeric_rows = 0;
    for (const SpecField& field : kFields) {
        if (section != "all" && section != field.group) {
            continue;
        }
        numeric_rows += field.numeric ? 1U : 0U;
        rows.push_back({
            json::Value(std::string(field.parameter)),
            field_value_cell(field),
            json::Value(std::string(field.unit)),
            json::Value(std::string(field.tolerance)),
            json::Value(std::string(field.document)),
            json::Value(field.page),
            json::Value(std::string(field.group)),
            json::Value(std::string(field.note)),
        });
    }

    json::Value out = json::Value::object();
    out.set("specification",
            json::from_table({"parameter", "value", "unit", "tolerance", "document", "page",
                              "group", "note"},
                             rows));
    out.set("sources", sources_table());
    out.set("rows", json::Value(static_cast<int>(rows.size())));
    out.set("numeric_rows", json::Value(static_cast<int>(numeric_rows)));
    out.set("published_fields", json::Value(static_cast<int>(std::size(kFields))));
    const std::string provenance =
        std::string(
            "Every row is a value read off a published Yaskawa document, with the document id and "
            "the page it was read on. The GP7 shares both documents with the GP8 and only the GP8 "
            "column was read. Where the datasheet and the instruction manual disagree - robot "
            "mass, repeatability, the L- and U-axis ranges - both values appear with their own "
            "citation instead of one being silently preferred. The same table, with the same "
            "citations, is committed as ") +
        kAssetRelativePath + ".";
    out.set("provenance", json::Value(provenance));
    return out;
}

[[nodiscard]] json::Value op_axis_limits(const json::Value& args) {
    const double range_tolerance = optional_scalar(args, "range_tolerance", 1.0e-3, 0.0, 0.5);
    const double speed_tolerance = optional_scalar(args, "speed_tolerance", 0.025, 0.0, 1.0);

    const std::array<AxisPublished, 6>& axes = published_axes();

    std::vector<std::vector<json::Value>> range_rows;
    range_rows.reserve(GP8_DOF);
    std::vector<std::vector<json::Value>> speed_rows;
    speed_rows.reserve(GP8_DOF);
    int range_disagreements = 0;
    int speed_disagreements = 0;
    int datasheet_disagreements = 0;

    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const AxisPublished& axis = axes[i];
        const double engine_min = joint_min(i);
        const double engine_max = joint_max(i);
        const double manual_min = axis.manual_min_deg * kDegToRad;
        const double manual_max = axis.manual_max_deg * kDegToRad;
        const double datasheet_min = axis.datasheet_min_deg * kDegToRad;
        const double datasheet_max = axis.datasheet_max_deg * kDegToRad;
        const double diff_min = engine_min - manual_min;
        const double diff_max = engine_max - manual_max;
        const double datasheet_gap =
            std::max(std::abs(manual_min - datasheet_min), std::abs(manual_max - datasheet_max));
        const bool engine_agrees =
            std::abs(diff_min) <= range_tolerance && std::abs(diff_max) <= range_tolerance;
        const bool documents_agree = datasheet_gap <= range_tolerance;

        std::string verdict;
        if (engine_agrees && documents_agree) {
            verdict = "agree";
        } else if (engine_agrees) {
            ++datasheet_disagreements;
            verdict = "engine matches HW1484385 p4; DS-699-H p2 prints " +
                      fixed(axis.datasheet_min_deg, 0) + " to " + fixed(axis.datasheet_max_deg, 0) +
                      " deg, " + fixed(datasheet_gap, 3) + " rad away";
        } else {
            ++range_disagreements;
            verdict = "engine disagrees with HW1484385 p4 by " +
                      fixed(std::max(std::abs(diff_min), std::abs(diff_max)), 4) + " rad";
            if (!documents_agree) {
                ++datasheet_disagreements;
                verdict += "; DS-699-H p2 disagrees with the manual too";
            }
        }

        range_rows.push_back({
            json::Value(std::string(axis.axis)),
            json::Value(axis.manual_min_deg),
            json::Value(axis.manual_max_deg),
            json::Value(axis.datasheet_min_deg),
            json::Value(axis.datasheet_max_deg),
            json::Value(engine_min * kRadToDeg),
            json::Value(engine_max * kRadToDeg),
            json::Value(engine_min),
            json::Value(engine_max),
            json::Value(diff_min),
            json::Value(diff_max),
            json::Value(verdict),
        });

        const double engine_speed = joint_max_velocity(i);
        const double speed_diff = engine_speed - axis.manual_speed_rad_s;
        const bool speed_agrees = std::abs(speed_diff) <= speed_tolerance;
        if (!speed_agrees) {
            ++speed_disagreements;
        }
        speed_rows.push_back({
            json::Value(std::string(axis.axis)),
            json::Value(axis.datasheet_speed_deg_s),
            json::Value(axis.manual_speed_rad_s),
            json::Value(engine_speed),
            json::Value(engine_speed * kRadToDeg),
            json::Value(speed_diff),
            json::Value(std::string(speed_agrees
                                        ? "agree"
                                        : "engine is " + fixed(speed_diff, 3) + " rad/s off " +
                                              fixed(axis.manual_speed_rad_s, 2) + " rad/s")),
        });
    }

    const bool all_agree = (range_disagreements == 0) && (speed_disagreements == 0);
    std::string verdict =
        all_agree
            ? "GP8_JOINT_LIMITS reproduces HW1484385 Table 5-1 on every axis, within " +
                  fixed(range_tolerance, 4) + " rad and " + fixed(speed_tolerance, 4) + " rad/s."
            : "GP8_JOINT_LIMITS departs from HW1484385 Table 5-1 on " +
                  std::to_string(range_disagreements) + " range(s) and " +
                  std::to_string(speed_disagreements) + " speed(s).";
    if (datasheet_disagreements != 0) {
        verdict += " Separately, DS-699-H and HW1484385 do not print the same GP8 ranges on " +
                   std::to_string(datasheet_disagreements) +
                   " axis/axes: the datasheet gives the L-axis +150 deg and the U-axis +255/-113 "
                   "deg where the manual gives +145 deg and -70/+190 deg. The engine follows the "
                   "manual.";
    }

    json::Value out = json::Value::object();
    out.set("ranges", json::from_table({"axis", "HW1484385_min_deg", "HW1484385_max_deg",
                                        "DS-699-H_min_deg", "DS-699-H_max_deg", "engine_min_deg",
                                        "engine_max_deg", "engine_min_rad", "engine_max_rad",
                                        "diff_min_rad", "diff_max_rad", "verdict"},
                                       range_rows));
    out.set("speeds", json::from_table({"axis", "DS-699-H_deg_s", "HW1484385_rad_s",
                                        "engine_rad_s", "engine_deg_s", "diff_rad_s", "verdict"},
                                       speed_rows));
    out.set("range_disagreements", json::Value(range_disagreements));
    out.set("speed_disagreements", json::Value(speed_disagreements));
    out.set("document_disagreements", json::Value(datasheet_disagreements));
    out.set("all_agree", json::Value(all_agree));
    out.set("verdict", json::Value(verdict));
    return out;
}

[[nodiscard]] json::Value op_wrist_capacity(const json::Value& args) {
    const double mass = optional_scalar(args, "payload_mass", 8.0, 0.0, 30.0);
    const Eigen::Vector3d offset =
        optional_vec3(args, "payload_offset", Eigen::Vector3d(0.0, 0.0, 0.10), -1.0, 1.0);
    const double inertia = optional_scalar(args, "payload_inertia", 0.02, 0.0, 5.0);
    const double b_angle = optional_scalar(args, "b_angle", 0.0, -2.356, 2.356);

    const WristAssessment assessment = evaluate_wrist(mass, offset, inertia, b_angle);

    std::vector<std::vector<json::Value>> rows;
    rows.reserve(assessment.axes.size());
    for (const WristAxisLoad& axis : assessment.axes) {
        std::string verdict = "PASS";
        if (!axis.moment_ok && !axis.inertia_ok) {
            verdict = "FAIL: moment and moment of inertia over rating";
        } else if (!axis.moment_ok) {
            verdict = "FAIL: moment over rating by " +
                      fixed(-axis.moment_margin_nm, 2) + " N m";
        } else if (!axis.inertia_ok) {
            verdict = "FAIL: moment of inertia over rating by " +
                      fixed(-axis.inertia_margin_kgm2, 3) + " kg m^2";
        }
        rows.push_back({
            json::Value(std::string(axis.axis)),
            json::Value(axis.lever_arm_m),
            json::Value(axis.moment_nm),
            json::Value(axis.allowable_moment_nm),
            json::Value(axis.moment_margin_nm),
            json::Value(axis.moment_utilisation),
            json::Value(axis.inertia_kgm2),
            json::Value(axis.allowable_inertia_kgm2),
            json::Value(axis.inertia_margin_kgm2),
            json::Value(axis.inertia_utilisation),
            json::Value(verdict),
        });
    }

    std::string verdict;
    if (assessment.pass) {
        verdict = "This gripper is inside the published GP8 wrist rating on all three axes, with " +
                  fixed(assessment.payload_margin_kg, 2) + " kg of payload left.";
    } else {
        verdict = "This gripper is outside the published GP8 wrist rating:";
        if (!assessment.mass_ok) {
            verdict += " the payload exceeds the rated 8 kg by " +
                       fixed(-assessment.payload_margin_kg, 2) + " kg;";
        }
        for (const WristAxisLoad& axis : assessment.axes) {
            if (!axis.ok) {
                verdict += " ";
                verdict += axis.axis;
                verdict += "-axis";
                if (!axis.moment_ok) {
                    verdict += " moment " + fixed(axis.moment_nm, 2) + " N m > " +
                               fixed(axis.allowable_moment_nm, 0) + " N m";
                }
                if (!axis.inertia_ok) {
                    verdict += (axis.moment_ok ? " inertia " : " and inertia ") +
                               fixed(axis.inertia_kgm2, 3) + " kg m^2 > " +
                               fixed(axis.allowable_inertia_kgm2, 1) + " kg m^2";
                }
                verdict += ";";
            }
        }
    }

    json::Value out = json::Value::object();
    out.set("wrist", json::from_table({"axis", "lever_arm_m", "moment_Nm", "allowable_moment_Nm",
                                       "moment_margin_Nm", "moment_utilisation", "inertia_kgm2",
                                       "allowable_inertia_kgm2", "inertia_margin_kgm2",
                                       "inertia_utilisation", "verdict"},
                                      rows));
    out.set("pass", json::Value(assessment.pass));
    out.set("mass_ok", json::Value(assessment.mass_ok));
    out.set("payload_margin_kg", json::Value(assessment.payload_margin_kg));
    out.set("rated_payload_kg", json::Value(published_value("general.payload")));
    out.set("verdict", json::Value(verdict));
    out.set("provenance",
            json::Value(std::string("The three ratings are published: 17/17/10 N m and "
                                    "0.5/0.5/0.2 kg m^2 for R/B/T, DS-699-H page 2, repeated in "
                                    "HW1484385 page 4. The moment and the moment of inertia of "
                                    "your payload are computed here, not published.")));
    return out;
}

[[nodiscard]] json::Value op_component_tree(const json::Value& args) {
    const TreeCache& cache = tree_cache();
    const std::string subsystem = optional_enum(args, "subsystem", "all", subsystem_options());
    const int depth = optional_int(args, "depth", 3, 1, 5);
    const int max_rows = optional_int(args, "max_rows", 40, 1, 400);
    const int wanted_category = category_of_option(subsystem);

    std::vector<std::vector<json::Value>> rows;
    rows.reserve(static_cast<std::size_t>(max_rows));
    std::size_t matching = 0;
    double shown_leaf_mass = 0.0;

    for (std::size_t i = 0; i < cache.total; ++i) {
        const physical::ComponentPtr component = physical::get_component_at(i);
        if (component == nullptr) {
            continue;
        }
        if (wanted_category >= 0 && cache.category[i] != wanted_category) {
            continue;
        }
        if (static_cast<int>(component->level) > depth) {
            continue;
        }
        ++matching;
        if (rows.size() >= static_cast<std::size_t>(max_rows)) {
            continue;
        }
        if (cache.is_leaf[i]) {
            shown_leaf_mass += component->mass_kg;
        }
        rows.push_back({
            json::Value(component->id),
            json::Value(component->name),
            json::Value(static_cast<int>(component->level)),
            json::Value(component->material),
            json::Value(component->part_number),
            json::Value(component->mass_kg),
            json::Value(static_cast<bool>(cache.is_leaf[i])),
            json::Value(component->system_name),
        });
    }

    std::vector<std::vector<json::Value>> subsystem_rows;
    subsystem_rows.reserve(cache.subsystems.size());
    for (const SubsystemMass& entry : cache.subsystems) {
        if (entry.name.empty()) {
            continue;
        }
        subsystem_rows.push_back({
            json::Value(entry.name),
            json::Value(static_cast<int>(entry.component_count)),
            json::Value(static_cast<int>(entry.leaf_count)),
            json::Value(entry.declared_assembly_mass_kg),
            json::Value(entry.leaf_mass_kg),
        });
    }

    json::Value out = json::Value::object();
    out.set("components",
            json::from_table({"id", "name", "level", "material", "part_number",
                              "mass_kg_modelled", "is_leaf", "subsystem"},
                             rows));
    out.set("subsystem_mass",
            json::from_table({"subsystem", "components", "leaves",
                              "declared_assembly_mass_kg_modelled", "leaf_mass_kg_modelled"},
                             subsystem_rows));
    out.set("shown", json::Value(static_cast<int>(rows.size())));
    out.set("matching", json::Value(static_cast<int>(matching)));
    out.set("total", json::Value(static_cast<int>(cache.total)));
    out.set("shown_leaf_mass_kg_modelled", json::Value(shown_leaf_mass));
    out.set("total_leaf_mass_kg_modelled", json::Value(cache.leaf_mass_kg));
    out.set("provenance", json::Value(std::string(kComponentProvenance)));
    return out;
}

[[nodiscard]] json::Value op_mass_reconciliation(const json::Value& args) {
    const double tolerance = optional_scalar(args, "tolerance_kg", 1.0, 0.0, 20.0);
    const std::vector<std::string> reference_options = {"DS-699-H", "HW1484385", "gp8_model.hpp"};
    const std::string reference =
        optional_enum(args, "published_reference", "DS-699-H", reference_options);

    const TreeCache& cache = tree_cache();
    const double datasheet_mass = published_value("general.mass");
    const double manual_mass = published_value("general.mass_manual");
    double reference_mass = datasheet_mass;
    std::string reference_label = "DS-699-H page 2, GP8 column";
    bool reference_is_published = true;
    if (reference == "HW1484385") {
        reference_mass = manual_mass;
        reference_label = "HW1484385 page 4, Table 5-1 Approx. Mass";
    } else if (reference == "gp8_model.hpp") {
        reference_mass = GP8_ROBOT_MASS_KG;
        reference_label = "gp8_model.hpp GP8_ROBOT_MASS_KG (the GP8 datasheet column)";
        reference_is_published = false;
    }

    std::vector<std::vector<json::Value>> rows;
    rows.reserve(GP8_DOF);
    double tree_link_mass = 0.0;
    double tree_assembly_mass = 0.0;
    double header_link_mass = 0.0;
    double total_discrepancy = 0.0;
    int disagreements = 0;

    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const std::size_t category = static_cast<std::size_t>(kLinkCategory[i]);
        const SubsystemMass& subsystem = cache.subsystems[category];
        const double tree_leaf = subsystem.leaf_mass_kg;
        const double header = GP8_LINKS[i].mass;
        const double discrepancy = tree_leaf - header;
        const double relative = (header > 0.0) ? (100.0 * discrepancy / header) : 0.0;
        tree_link_mass += tree_leaf;
        tree_assembly_mass += subsystem.declared_assembly_mass_kg;
        header_link_mass += header;
        total_discrepancy += discrepancy;
        const bool agrees = std::abs(discrepancy) <= tolerance;
        if (!agrees) {
            ++disagreements;
        }
        rows.push_back({
            json::Value(std::string(GP8_AXIS_NAMES[i])),
            json::Value(subsystem.name),
            json::Value(tree_leaf),
            json::Value(subsystem.declared_assembly_mass_kg),
            json::Value(header),
            json::Value(discrepancy),
            json::Value(relative),
            json::Value(std::string(agrees ? "within " + fixed(tolerance, 2) + " kg"
                                           : "off by " + fixed(discrepancy, 3) + " kg")),
        });
    }

    std::vector<std::vector<json::Value>> totals;
    totals.reserve(8);
    totals.push_back({json::Value(std::string("Six link subsystems, leaf sum of the component "
                                              "tree")),
                      json::Value(tree_link_mass), json::Value(std::string("modelled"))});
    totals.push_back({json::Value(std::string("Six link subsystems, assembly masses the tree "
                                              "declares")),
                      json::Value(tree_assembly_mass), json::Value(std::string("modelled"))});
    totals.push_back({json::Value(std::string("Six moving links, GP8_LINKS in gp8_model.hpp")),
                      json::Value(header_link_mass), json::Value(std::string("modelled"))});
    totals.push_back({json::Value(std::string("Whole component tree, leaf sum of all subsystems "
                                              "including controller, pendant, gripper and "
                                              "workcell")),
                      json::Value(cache.leaf_mass_kg), json::Value(std::string("modelled"))});
    totals.push_back({json::Value(std::string("Robot mass, DS-699-H page 2, GP8 column")),
                      json::Value(datasheet_mass), json::Value(std::string("published"))});
    totals.push_back({json::Value(std::string("Robot mass, HW1484385 page 4, Table 5-1")),
                      json::Value(manual_mass), json::Value(std::string("published"))});
    totals.push_back({json::Value(std::string("Robot mass, gp8_model.hpp GP8_ROBOT_MASS_KG")),
                      json::Value(GP8_ROBOT_MASS_KG),
                      json::Value(std::string("modelled, and it is the GP7 datasheet weight"))});
    totals.push_back({json::Value(std::string("Reference selected for this run: ") +
                                  reference_label),
                      json::Value(reference_mass),
                      json::Value(std::string(reference_is_published ? "published" : "modelled"))});

    const double tree_vs_reference = tree_link_mass - reference_mass;
    std::string verdict =
        "The modelled breakdown and the published total do not agree, and that is the point of "
        "this table: the six link subsystems of the component tree add up to " +
        fixed(tree_link_mass, 2) + " kg over their leaves, GP8_LINKS in gp8_model.hpp adds up to " +
        fixed(header_link_mass, 2) + " kg, and the published robot mass is " +
        fixed(reference_mass, 2) + " kg (" + reference_label + "). The tree's link leaves are " +
        fixed(tree_vs_reference, 2) + " kg away from that published total";
    if (reference == "DS-699-H") {
        verdict += ", and the published total itself is disputed: DS-699-H page 2 says " +
                   fixed(datasheet_mass, 0) + " kg for the GP8 while HW1484385 page 4 says " +
                   fixed(manual_mass, 0) + " kg, and gp8_model.hpp uses " +
                   fixed(GP8_ROBOT_MASS_KG, 0) + " kg, the GP8 column, which the geometry-derived link masses are fitted to";
    }
    verdict += ". Per-link, " + std::to_string(disagreements) + " of 6 subsystems differ from " +
               "gp8_model.hpp by more than " + fixed(tolerance, 2) + " kg.";

    json::Value out = json::Value::object();
    out.set("links", json::from_table({"axis", "subsystem", "tree_leaf_mass_kg_modelled",
                                       "tree_assembly_mass_kg_modelled", "gp8_model_mass_kg",
                                       "discrepancy_kg", "relative_percent", "verdict"},
                                      rows));
    out.set("totals", json::from_table({"quantity", "mass_kg", "kind"}, totals));
    out.set("tree_link_mass_kg_modelled", json::Value(tree_link_mass));
    out.set("header_link_mass_kg_modelled", json::Value(header_link_mass));
    out.set("total_discrepancy_kg", json::Value(total_discrepancy));
    out.set("published_reference_kg", json::Value(reference_mass));
    out.set("tree_vs_published_kg", json::Value(tree_vs_reference));
    out.set("link_disagreements", json::Value(disagreements));
    out.set("agrees", json::Value(disagreements == 0));
    out.set("verdict", json::Value(verdict));
    out.set("provenance", json::Value(std::string(kComponentProvenance)));
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// Published data accessors
// ---------------------------------------------------------------------------

std::span<const DocumentRef> documents() noexcept {
    return std::span<const DocumentRef>(kDocuments.data(), kDocuments.size());
}

std::span<const SpecField> published_fields() noexcept {
    return std::span<const SpecField>(std::begin(kFields), std::end(kFields));
}

const std::vector<std::string>& field_groups() {
    static const std::vector<std::string> groups = [] {
        std::vector<std::string> out;
        for (const SpecField& field : kFields) {
            if (std::find(out.begin(), out.end(), field.group) == out.end()) {
                out.emplace_back(field.group);
            }
        }
        return out;
    }();
    return groups;
}

const SpecField& published_field(std::string_view path) {
    const auto& index = field_index();
    const auto it = index.find(std::string(path));
    if (it == index.end()) {
        throw StudyError("'" + std::string(path) +
                         "' is not a published field of the GP8 specification");
    }
    return *it->second;
}

double published_value(std::string_view path) {
    const SpecField& field = published_field(path);
    if (!field.numeric) {
        throw StudyError("published field '" + std::string(path) + "' is the text '" +
                         std::string(field.text) + "', not a number");
    }
    return field.value;
}

const std::array<AxisPublished, 6>& published_axes() {
    static const std::array<AxisPublished, 6> axes = [] {
        std::array<AxisPublished, 6> out{};
        for (std::size_t i = 0; i < GP8_DOF; ++i) {
            AxisPublished& axis = out[i];
            axis.axis = GP8_AXIS_NAMES[i];
            axis.name = kAxisJointNames[i];
            axis.datasheet_min_deg = published_value(axis_path(i, "motion_range_min"));
            axis.datasheet_max_deg = published_value(axis_path(i, "motion_range_max"));
            axis.manual_min_deg = published_value(axis_path(i, "motion_range_min_manual"));
            axis.manual_max_deg = published_value(axis_path(i, "motion_range_max_manual"));
            axis.datasheet_speed_deg_s = published_value(axis_path(i, "max_speed"));
            axis.manual_speed_rad_s = published_value(axis_path(i, "max_speed_manual"));
            const std::string moment = axis_path(i, "allowable_moment");
            axis.has_wrist_rating = field_index().find(moment) != field_index().end();
            if (axis.has_wrist_rating) {
                axis.allowable_moment_nm = published_value(moment);
                axis.allowable_inertia_kgm2 =
                    published_value(axis_path(i, "allowable_moment_of_inertia"));
            }
        }
        return out;
    }();
    return axes;
}

// ---------------------------------------------------------------------------
// Wrist capacity
// ---------------------------------------------------------------------------

WristAssessment evaluate_wrist(double payload_mass_kg, const Eigen::Vector3d& offset_m,
                               double payload_inertia_kgm2, double b_angle_rad) {
    if (!std::isfinite(payload_mass_kg) || payload_mass_kg < 0.0) {
        throw StudyError("payload mass must be a finite, non-negative number of kilograms");
    }
    if (!std::isfinite(payload_inertia_kgm2) || payload_inertia_kgm2 < 0.0) {
        throw StudyError("payload inertia must be a finite, non-negative number of kg m^2");
    }
    if (!offset_m.allFinite()) {
        throw StudyError("payload offset must be three finite numbers of metres");
    }
    if (!std::isfinite(b_angle_rad)) {
        throw StudyError("B-axis angle must be a finite number of radians");
    }

    // Flange frame: z out of the flange face along the T axis, y along the B
    // axis. The R axis leaves the flange axis as the B axis bends.
    const std::array<Eigen::Vector3d, 3> directions = {
        Eigen::Vector3d(std::sin(b_angle_rad), 0.0, std::cos(b_angle_rad)),  // R
        Eigen::Vector3d(0.0, 1.0, 0.0),                                      // B
        Eigen::Vector3d(0.0, 0.0, 1.0),                                      // T
    };
    const std::array<const char*, 3> names = {"R", "B", "T"};
    const std::array<std::size_t, 3> axis_of_name = {3, 4, 5};  // index into published_axes()

    WristAssessment assessment;
    assessment.payload_mass_kg = payload_mass_kg;
    const double rated_payload = published_value("general.payload");
    assessment.payload_margin_kg = rated_payload - payload_mass_kg;
    assessment.mass_ok = payload_mass_kg <= rated_payload;
    assessment.pass = assessment.mass_ok;

    for (std::size_t i = 0; i < 3; ++i) {
        const AxisPublished& published = published_axes()[axis_of_name[i]];
        const Eigen::Vector3d& direction = directions[i];
        const double along = offset_m.dot(direction);
        const double lever = std::sqrt(std::max(0.0, offset_m.squaredNorm() - along * along));

        WristAxisLoad& load = assessment.axes[i];
        load.axis = names[i];
        load.lever_arm_m = lever;
        load.moment_nm = payload_mass_kg * GP8_GRAVITY_MPS2 * lever;
        load.allowable_moment_nm = published.allowable_moment_nm;
        load.moment_margin_nm = load.allowable_moment_nm - load.moment_nm;
        load.moment_utilisation = load.moment_nm / load.allowable_moment_nm;
        load.inertia_kgm2 = payload_inertia_kgm2 + payload_mass_kg * lever * lever;
        load.allowable_inertia_kgm2 = published.allowable_inertia_kgm2;
        load.inertia_margin_kgm2 = load.allowable_inertia_kgm2 - load.inertia_kgm2;
        load.inertia_utilisation = load.inertia_kgm2 / load.allowable_inertia_kgm2;
        load.moment_ok = load.moment_nm <= load.allowable_moment_nm;
        load.inertia_ok = load.inertia_kgm2 <= load.allowable_inertia_kgm2;
        load.ok = load.moment_ok && load.inertia_ok;
        assessment.pass = assessment.pass && load.ok;
    }
    return assessment;
}

// ---------------------------------------------------------------------------
// MODELLED component tree
// ---------------------------------------------------------------------------

const std::vector<SubsystemMass>& subsystem_masses() { return tree_cache().subsystems; }

const std::vector<std::string>& subsystem_options() { return tree_cache().options; }

// ---------------------------------------------------------------------------
// The committed asset
// ---------------------------------------------------------------------------

std::string find_asset_path() {
    std::error_code error;
    std::filesystem::path directory = std::filesystem::current_path(error);
    if (error) {
        throw StudyError(std::string("cannot read the working directory while looking for ") +
                         kAssetRelativePath + ": " + error.message());
    }
    std::string tried;
    for (int level = 0; level <= kAssetSearchDepth; ++level) {
        const std::filesystem::path candidate = directory / kAssetRelativePath;
        if (std::filesystem::exists(candidate, error) &&
            std::filesystem::is_regular_file(candidate, error)) {
            return candidate.string();
        }
        tried += (tried.empty() ? "" : ", ") + candidate.string();
        if (!directory.has_parent_path() || directory.parent_path() == directory) {
            break;
        }
        directory = directory.parent_path();
    }
    throw StudyError(std::string("cannot find ") + kAssetRelativePath +
                     " above the working directory; tried " + tried);
}

json::Value load_asset(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        throw StudyError("cannot open the published specification asset '" + path + "'");
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    if (file.bad()) {
        throw StudyError("cannot read the published specification asset '" + path + "'");
    }
    try {
        return json::parse(buffer.str());
    } catch (const json::ParseError& error) {
        throw StudyError("the published specification asset '" + path + "' is not valid JSON: " +
                         error.what());
    }
}

json::Value load_asset() { return load_asset(find_asset_path()); }

}  // namespace specification

// ---------------------------------------------------------------------------
// Self-description
// ---------------------------------------------------------------------------

ModuleDescription RobotSpecificationModule::describe() const {
    ModuleDescription d;
    d.name = "robot_specification";
    d.title = "Published Specification and Modelled Part Breakdown of the GP8";
    d.course = CourseRef{3883, "M-407-01", "Robotics Modelling"};
    d.topics = {
        "Block 6 · Model validation against engineering specifications",
        "3884 (M-408-01) · Actuator and wrist load limits",
    };
    d.source = "cpp_solver/include/study/robot_specification.hpp";
    d.summary =
        "Shows what Yaskawa actually publishes about the GP8 - every number cited to a document "
        "and a page - next to what the engine and the project's own modelled part breakdown "
        "claim, and prints each disagreement instead of hiding it.";

    {
        OpSpec op;
        op.name = "datasheet";
        op.title = "The published GP8 specification, with a page citation per number";
        op.formula =
            "\\text{value} \\;\\mapsto\\; (\\text{unit},\\ \\text{document},\\ \\text{page})";
        op.explain =
            "This is the op that answers \"what is actually known about this robot\". Every row "
            "was read off one of two published documents: the Yaskawa America datasheet DS-699-H "
            "(2 pages) and the Yaskawa Electric instruction manual HW1484385 (21 pages). Both "
            "cover the GP7 and the GP8 together, so every row also records which column or table "
            "it came from - mixing the two robots up is the easy mistake, and the GP7 is the "
            "heavier, longer-reach, slower machine. Where the two documents disagree, both values "
            "are listed with their own citation: the datasheet gives the GP8 a 32 kg weight and "
            "+/-0.02 mm repeatability, the manual gives 35 kg and 0.01 mm to ISO 9283. A number "
            "without a page is not in this table.";
        op.params = {
            ParamSpec::enumeration("section", "Which part of the specification",
                                   [] {
                                       std::vector<std::string> options;
                                       options.emplace_back("all");
                                       for (const auto& group : specification::field_groups()) {
                                           options.push_back(group);
                                       }
                                       return options;
                                   }(),
                                   "all"),
        };
        op.outputs = {
            OutputSpec::make("specification", "table",
                             "Parameter, value, unit, tolerance, document and page"),
            OutputSpec::make("sources", "table", "The documents, with manual number and URL"),
            OutputSpec::make("rows", "int", "Rows shown"),
            OutputSpec::make("numeric_rows", "int", "How many of them are numeric"),
            OutputSpec::make("published_fields", "int", "Cited fields in the whole table"),
            OutputSpec::make("provenance", "text", "Where these numbers come from"),
        };
        d.ops.push_back(std::move(op));
    }

    {
        OpSpec op;
        op.name = "axis_limits";
        op.title = "Published motion range and speed against the limits the engine uses";
        op.formula =
            "\\Delta_i = q_i^{\\text{engine}} - \\frac{\\pi}{180}\\, q_i^{\\text{published}}";
        op.explain =
            "The engine clamps and plans against GP8_JOINT_LIMITS in yaskawa_kinematics.hpp, in "
            "radians. The documents publish degrees. This op converts the published numbers and "
            "subtracts, per axis, so a transcription error cannot hide behind a unit change. Three "
            "things show up. The engine follows the instruction manual, not the marketing "
            "datasheet: the manual's L-axis +145 deg and U-axis -70/+190 deg are what "
            "GP8_JOINT_LIMITS encodes, while DS-699-H prints +150 deg and +255/-113 deg for the "
            "same axes. The remaining range differences are the rounding of a three-decimal "
            "radian constant, under 0.001 rad. The S-axis speed is the real outlier: both "
            "documents say 455 deg/s = 7.94 rad/s and the engine stores 7.85 rad/s, so every "
            "S-axis time budget the engine computes is 1.1 percent pessimistic.";
        op.params = {
            ParamSpec::scalar("range_tolerance", "Range agreement tolerance", "rad", 0.0, 0.5,
                              1.0e-3),
            ParamSpec::scalar("speed_tolerance", "Speed agreement tolerance", "rad/s", 0.0, 1.0,
                              0.025),
        };
        op.outputs = {
            OutputSpec::make("ranges", "table", "Per-axis motion range, both documents and the "
                                                "engine, with the difference"),
            OutputSpec::make("speeds", "table", "Per-axis maximum speed, published and engine"),
            OutputSpec::make("range_disagreements", "int", "Axes whose range exceeds the tolerance"),
            OutputSpec::make("speed_disagreements", "int", "Axes whose speed exceeds the tolerance"),
            OutputSpec::make("document_disagreements", "int",
                             "Axes where the two documents differ from each other"),
            OutputSpec::make("all_agree", "bool", "Engine matches HW1484385 everywhere"),
            OutputSpec::make("verdict", "text", "What the differences mean"),
        };
        d.ops.push_back(std::move(op));
    }

    {
        OpSpec op;
        op.name = "wrist_capacity";
        op.title = "Test a gripper against the published R/B/T wrist rating";
        op.formula =
            "M_a = m g \\lVert r_\\perp(a) \\rVert,\\qquad "
            "J_a = J_{\\text{own}} + m \\lVert r_\\perp(a) \\rVert^2,\\qquad "
            "r_\\perp(a) = r - (r\\cdot a)\\,a";
        op.explain =
            "The datasheet's allowable moment and allowable moment of inertia - 17/17/10 N m and "
            "0.5/0.5/0.2 kg m^2 for R/B/T - are the numbers that decide whether a gripper can go "
            "on this robot, and they are a table until you put your own tool against them. Give "
            "the payload mass, where its centre of mass sits relative to the flange face, and its "
            "own moment of inertia about its centre of mass. For each wrist axis the op takes the "
            "part of the offset perpendicular to that axis, which is the lever arm, and reports "
            "the worst-case gravity moment m g r_perp - worst case over the gravity direction, so "
            "the answer does not depend on the arm's pose - and the payload inertia about the "
            "axis by the parallel-axis theorem. The B-axis angle is what tilts the R axis off the "
            "flange axis: at B = 0 the R and T axes are collinear and see the same lever arm, "
            "which is why a long, light tool runs out of T-axis moment (10 N m) long before it "
            "runs out of R-axis moment (17 N m).";
        op.params = {
            ParamSpec::scalar("payload_mass", "Payload mass", "kg", 0.0, 30.0, 8.0),
            ParamSpec::vec3("payload_offset", "Payload centre of mass from the flange face", "m",
                            -1.0, 1.0, Eigen::Vector3d(0.0, 0.0, 0.10)),
            ParamSpec::scalar("payload_inertia", "Payload inertia about its own centre of mass",
                              "kg m^2", 0.0, 5.0, 0.02),
            ParamSpec::scalar("b_angle", "B-axis angle", "rad", -2.356, 2.356, 0.0),
        };
        op.outputs = {
            OutputSpec::make("wrist", "table",
                             "Per-axis moment and inertia against the rating, with the margin"),
            OutputSpec::make("pass", "bool", "Payload and all three axes within rating"),
            OutputSpec::make("mass_ok", "bool", "Payload within the rated 8 kg"),
            OutputSpec::make("payload_margin_kg", "scalar", "Rated payload left", "kg"),
            OutputSpec::make("rated_payload_kg", "scalar", "Published rated payload", "kg"),
            OutputSpec::make("verdict", "text", "Pass or fail, and why"),
            OutputSpec::make("provenance", "text", "Which numbers are published"),
        };
        d.ops.push_back(std::move(op));
    }

    {
        OpSpec op;
        op.name = "component_tree";
        op.title = "Browse the modelled component breakdown";
        op.formula = "m_{\\text{subsystem}} = \\sum_{c \\in \\text{leaves}} m_c";
        op.explain =
            "This component data is a MODELLED breakdown, not Yaskawa data: the part numbers, "
            "materials and masses in cpp_solver/include/robot_physical_tree.tpp were authored for "
            "this project and no vendor parts list was used, so nothing in this panel should be "
            "quoted as a Yaskawa figure. With that said, it is a useful object: a five-level "
            "hierarchy of a GP8 workcell from the system root down to individual SMD resistors, "
            "grouped into thirteen subsystems. Pick a subsystem and a depth to browse it. Note "
            "that the hierarchy declares a mass on its assemblies as well as on its leaves, so "
            "the subsystem totals add up the leaves only - adding every node would count every "
            "parent twice - and the assembly mass the tree declares for itself is shown beside it "
            "so the two can be compared. Every mass in every output of this op is modelled, never "
            "published.";
        op.params = {
            ParamSpec::enumeration("subsystem", "Subsystem", specification::subsystem_options(),
                                   "all"),
            ParamSpec::integer("depth", "Deepest level to show (1 system .. 5 micro-part)", "", 1,
                               5, 3),
            ParamSpec::integer("max_rows", "Rows to return", "", 1, 400, 40),
        };
        op.outputs = {
            OutputSpec::make("components", "table",
                             "id, name, level, material, part number and MODELLED mass"),
            OutputSpec::make("subsystem_mass", "table",
                             "Per-subsystem component count and MODELLED mass totals"),
            OutputSpec::make("shown", "int", "Components in the table"),
            OutputSpec::make("matching", "int", "Components matching the filter"),
            OutputSpec::make("total", "int", "Components in the whole tree"),
            OutputSpec::make("shown_leaf_mass_kg_modelled", "scalar",
                             "MODELLED mass of the shown leaves", "kg"),
            OutputSpec::make("total_leaf_mass_kg_modelled", "scalar",
                             "MODELLED mass of every leaf in the tree", "kg"),
            OutputSpec::make("provenance", "text",
                             "That this breakdown is modelled, not vendor data"),
        };
        d.ops.push_back(std::move(op));
    }

    {
        OpSpec op;
        op.name = "mass_reconciliation";
        op.title = "Modelled part masses against the published robot mass";
        op.formula =
            "\\delta_i = m_i^{\\text{tree}} - m_i^{\\text{model}},\\qquad "
            "\\Delta = \\sum_i \\delta_i";
        op.explain =
            "Two invented mass breakdowns and one published total, in one table, so the "
            "disagreement is visible rather than buried. Per link it compares the leaf mass of "
            "that axis' subsystem in the modelled component tree against GP8_LINKS in "
            "gp8_model.hpp, and sums the per-link differences into one total. Then it puts both "
            "modelled totals next to the published robot mass - and the published mass is itself "
            "disputed: DS-699-H page 2 prints 32 kg for the GP8 and 34 kg for the GP7 in the "
            "adjacent column, while HW1484385 page 4 prints 35 kg. gp8_model.hpp is fitted to the "
            "32 kg GP8 column. Nothing here is reconciled by adjusting a number; the op "
            "reports the gap.";
        op.params = {
            ParamSpec::scalar("tolerance_kg", "Per-link agreement tolerance", "kg", 0.0, 20.0,
                              1.0),
            ParamSpec::enumeration("published_reference", "Published total to compare against",
                                   {"DS-699-H", "HW1484385", "gp8_model.hpp"}, "DS-699-H"),
        };
        op.outputs = {
            OutputSpec::make("links", "table",
                             "Per-link MODELLED tree mass, gp8_model.hpp mass and discrepancy"),
            OutputSpec::make("totals", "table", "Every total, each labelled modelled or published"),
            OutputSpec::make("tree_link_mass_kg_modelled", "scalar",
                             "Leaf mass of the six link subsystems", "kg"),
            OutputSpec::make("header_link_mass_kg_modelled", "scalar",
                             "Sum of GP8_LINKS masses", "kg"),
            OutputSpec::make("total_discrepancy_kg", "scalar",
                             "Sum of the per-link discrepancies", "kg"),
            OutputSpec::make("published_reference_kg", "scalar", "The published total used", "kg"),
            OutputSpec::make("tree_vs_published_kg", "scalar",
                             "Tree link mass minus that published total", "kg"),
            OutputSpec::make("link_disagreements", "int", "Links outside the tolerance"),
            OutputSpec::make("agrees", "bool", "Every link within the tolerance"),
            OutputSpec::make("verdict", "text", "What the gap means"),
            OutputSpec::make("provenance", "text",
                             "That the breakdown is modelled, not vendor data"),
        };
        d.ops.push_back(std::move(op));
    }

    return d;
}

json::Value RobotSpecificationModule::invoke(std::string_view op, const json::Value& args) const {
    if (op == "datasheet") {
        return specification::op_datasheet(args);
    }
    if (op == "axis_limits") {
        return specification::op_axis_limits(args);
    }
    if (op == "wrist_capacity") {
        return specification::op_wrist_capacity(args);
    }
    if (op == "component_tree") {
        return specification::op_component_tree(args);
    }
    if (op == "mass_reconciliation") {
        return specification::op_mass_reconciliation(args);
    }
    unknown_op(name(), op);
}

}  // namespace yaskawa::study
