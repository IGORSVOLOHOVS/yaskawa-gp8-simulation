// Test suite for the robot_specification study module. Plain asserts in the
// style of src/test_study_modules.cpp - there is no gtest in this project and
// there will not be one.
//
// What this suite is for: the module publishes numbers that a report will
// quote, so the checks are about provenance as much as arithmetic.
//   - every numeric field of assets/gp8_published_specification.json carries a
//     unit and a page citation, and the page exists in the document it names;
//   - the compiled table and that asset agree field by field, both ways, so the
//     committed JSON cannot drift away from the code;
//   - the published axis ranges agree with GP8_JOINT_LIMITS to a stated
//     tolerance in radians, and where they do not the discrepancy is asserted
//     explicitly with its magnitude - the suite stays green by documenting the
//     disagreement, not by widening the tolerance until it disappears;
//   - wrist_capacity accepts a payload inside the rating and rejects one
//     outside it on each of R, B and T independently;
//   - component_tree returns its provenance text and internally consistent
//     counts, and mass_reconciliation's per-link discrepancies sum to the total
//     it reports;
//   - every op runs from the defaults it declares and returns every output it
//     declares.

#include "study/gp8_model.hpp"
#include "study/json.hpp"
#include "study/robot_specification.hpp"
#include "study/study_module.hpp"
#include "yaskawa_kinematics.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

using yaskawa::study::json::Value;
namespace json = yaskawa::study::json;
namespace study = yaskawa::study;
namespace spec = yaskawa::study::specification;

namespace {

constexpr double kRadToDeg = 180.0 / std::numbers::pi;
constexpr double kDegToRad = std::numbers::pi / 180.0;

// The stated tolerances. The ranges in GP8_JOINT_LIMITS are three-decimal
// radian constants, so a degree value converted into radians can only be
// reproduced to about 1e-3 rad; the speeds are two-decimal rad/s constants.
constexpr double kRangeToleranceRad = 1.0e-3;
constexpr double kSpeedToleranceRadPerSec = 0.025;

int g_checks = 0;
int g_failures = 0;
std::vector<std::string> g_failed_names;

void check(bool ok, const std::string& what) {
    ++g_checks;
    if (ok) {
        std::cout << "[PASS] " << what << "\n";
    } else {
        ++g_failures;
        g_failed_names.push_back(what);
        std::cout << "[FAIL] " << what << "\n";
    }
}

void check_near(double actual, double expected, double tolerance, const std::string& what) {
    const double error = std::abs(actual - expected);
    const bool ok = std::isfinite(error) && error <= tolerance;
    if (!ok) {
        std::cout << "       expected " << expected << ", got " << actual << " (error " << error
                  << " > " << tolerance << ")\n";
    }
    check(ok, what);
}

// ---------------------------------------------------------------------------
// A path resolver for the asset, so a compiled field can name its own JSON slot
// ---------------------------------------------------------------------------

[[nodiscard]] const Value* resolve(const Value& root, const std::string& path) {
    const Value* node = &root;
    std::size_t i = 0;
    while (i < path.size()) {
        if (path[i] == '.') {
            ++i;
            continue;
        }
        if (path[i] == '[') {
            const std::size_t close = path.find(']', i);
            if (close == std::string::npos) {
                return nullptr;
            }
            const std::size_t index =
                static_cast<std::size_t>(std::stoul(path.substr(i + 1, close - i - 1)));
            if (!node->is_array() || index >= node->size()) {
                return nullptr;
            }
            node = &(*node)[index];
            i = close + 1;
            continue;
        }
        const std::size_t end = path.find_first_of(".[", i);
        const std::string key = path.substr(i, end == std::string::npos ? end : end - i);
        if (!node->is_object() || !node->contains(key)) {
            return nullptr;
        }
        node = &(*node)[key];
        i = (end == std::string::npos) ? path.size() : end;
    }
    return node;
}

[[nodiscard]] int document_page_count(const std::string& id) {
    for (const spec::DocumentRef& document : spec::documents()) {
        if (id == document.id) {
            return document.page_count;
        }
    }
    return 0;
}

// Walks the asset and visits every leaf object that carries a "value".
void walk_fields(const Value& node, const std::string& path,
                 std::vector<std::string>& found) {
    if (node.is_object()) {
        if (node.contains("value")) {
            found.push_back(path);
            return;
        }
        const std::vector<std::string>& keys = node.keys();
        for (std::size_t i = 0; i < keys.size(); ++i) {
            if (keys[i] == "sources") {
                continue;  // the document records, not specification fields
            }
            walk_fields(node.items()[i], path.empty() ? keys[i] : path + "." + keys[i], found);
        }
        return;
    }
    if (node.is_array()) {
        for (std::size_t i = 0; i < node.size(); ++i) {
            walk_fields(node[i], path + "[" + std::to_string(i) + "]", found);
        }
    }
}

// ---------------------------------------------------------------------------
// The committed asset
// ---------------------------------------------------------------------------

void test_asset(const Value& asset) {
    std::cout << "\n--- assets/gp8_published_specification.json ---\n";

    check(asset.is_object(), "the asset parses into a JSON object");
    check(asset["robot"].is_string() && asset["robot"].as_string() == "Yaskawa Motoman GP8",
          "the asset names the robot it describes");
    check(asset["column_read"].is_string() && asset["column_read"].as_string() == "GP8",
          "the asset records that the GP8 column was read, not the GP7");

    const Value& sources = asset["sources"];
    check(sources.is_object() && sources.size() == 2,
          "the asset carries a sources block naming both documents");
    bool sources_complete = sources.size() == 2;
    for (std::size_t i = 0; i < sources.size(); ++i) {
        const Value& document = sources.items()[i];
        sources_complete = sources_complete && document["title"].is_string() &&
                           !document["title"].as_string().empty() && document["url"].is_string() &&
                           document["url"].as_string().rfind("https://", 0) == 0 &&
                           document["page_count"].is_number() &&
                           document.contains("manual_number");
    }
    check(sources_complete,
          "every source carries a title, a URL, a page count and a manual number slot");
    check(sources["HW1484385"]["manual_number"].is_string() &&
              sources["HW1484385"]["manual_number"].as_string() == "HW1484385",
          "the instruction manual carries its Yaskawa manual number");
    check(sources["DS-699-H"]["manual_number"].is_null(),
          "the datasheet, which has no manual number, says so with null rather than inventing one");

    std::vector<std::string> paths;
    walk_fields(asset, "", paths);
    check(paths.size() == spec::published_fields().size(),
          "the asset holds exactly as many cited fields as the compiled table (" +
              std::to_string(paths.size()) + ")");

    bool every_field_cited = !paths.empty();
    bool every_page_exists = true;
    std::size_t numeric_fields = 0;
    for (const std::string& path : paths) {
        const Value* field = resolve(asset, path);
        if (field == nullptr) {
            every_field_cited = false;
            std::cout << "       " << path << " does not resolve\n";
            continue;
        }
        const bool has_unit = field->contains("unit") && (*field)["unit"].is_string();
        const Value& source = (*field)["source"];
        const bool has_source = source.is_object() && source["document"].is_string() &&
                                !source["document"].as_string().empty() &&
                                source["page"].is_number() && source["page"].as_double() >= 1.0;
        if ((*field)["value"].is_number()) {
            ++numeric_fields;
        }
        if (!has_unit || !has_source) {
            every_field_cited = false;
            std::cout << "       " << path << " is missing a unit or a document/page citation\n";
            continue;
        }
        const int pages = document_page_count(source["document"].as_string());
        if (pages == 0 || source["page"].as_double() > static_cast<double>(pages)) {
            every_page_exists = false;
            std::cout << "       " << path << " cites page " << source["page"].as_double()
                      << " of a " << pages << "-page document\n";
        }
    }
    check(every_field_cited,
          "every field in the asset carries a unit and a source document with a page number");
    check(every_page_exists, "every cited page exists in the document it names");
    check(numeric_fields >= 60,
          "the asset carries " + std::to_string(numeric_fields) +
              " numeric fields, all of them cited");

    const Value& disagreements = asset["known_disagreements"];
    check(disagreements.is_array() && disagreements.size() >= 4,
          "the asset lists the disagreements between its own sources instead of hiding them");
}

void test_asset_against_compiled_table(const Value& asset) {
    std::cout << "\n--- compiled table against the asset ---\n";

    bool all_agree = true;
    for (const spec::SpecField& field : spec::published_fields()) {
        const Value* node = resolve(asset, field.path);
        if (node == nullptr) {
            all_agree = false;
            std::cout << "       compiled field " << field.path << " is absent from the asset\n";
            continue;
        }
        const Value& value = (*node)["value"];
        bool agrees = (*node)["unit"].as_string() == field.unit &&
                      (*node)["source"]["document"].as_string() == field.document &&
                      static_cast<int>((*node)["source"]["page"].as_double()) == field.page;
        if (field.numeric) {
            agrees = agrees && value.is_number() && value.as_double() == field.value;
        } else {
            agrees = agrees && value.is_string() && value.as_string() == field.text;
        }
        const std::string tolerance =
            node->contains("tolerance") ? (*node)["tolerance"].as_string() : std::string();
        const std::string note =
            node->contains("note") ? (*node)["note"].as_string() : std::string();
        agrees = agrees && tolerance == field.tolerance && note == field.note;
        if (!agrees) {
            all_agree = false;
            std::cout << "       compiled field " << field.path << " differs from the asset\n";
        }
    }
    check(all_agree,
          "all " + std::to_string(spec::published_fields().size()) +
              " compiled fields match the asset in value, unit, tolerance, note, document and "
              "page");

    check(spec::published_value("general.payload") == 8.0,
          "payload 8 kg, DS-699-H page 2, GP8 column (the GP7 column says 7 kg)");
    check(spec::published_value("general.horizontal_reach") == 727.0 &&
              spec::published_value("general.vertical_reach") == 1312.0,
          "reach 727 mm horizontal and 1312 mm vertical, DS-699-H page 2 (GP7: 927 and 1693)");
    check(spec::published_value("general.mass") == 32.0 &&
              spec::published_value("general.mass_manual") == 35.0,
          "robot mass 32 kg on DS-699-H page 2 and 35 kg on HW1484385 page 4, both recorded");
    check(spec::published_field("general.mass").document == std::string("DS-699-H") &&
              spec::published_field("general.mass").page == 2,
          "the mass carries its page citation");
    check(spec::published_value("general.repeatability") == 0.02 &&
              spec::published_value("general.repeatability_manual") == 0.01,
          "repeatability +/-0.02 mm (datasheet) and 0.01 mm (manual, ISO 9283), both recorded");

    bool wrist_rating_right = true;
    const std::array<double, 3> moments = {17.0, 17.0, 10.0};
    const std::array<double, 3> inertias = {0.5, 0.5, 0.2};
    for (std::size_t i = 0; i < 3; ++i) {
        const spec::AxisPublished& axis = spec::published_axes()[i + 3];
        wrist_rating_right = wrist_rating_right && axis.has_wrist_rating &&
                             axis.allowable_moment_nm == moments[i] &&
                             axis.allowable_inertia_kgm2 == inertias[i];
    }
    check(wrist_rating_right,
          "allowable wrist moment 17/17/10 N m and inertia 0.5/0.5/0.2 kg m^2 for R/B/T");
    check(!spec::published_axes()[0].has_wrist_rating &&
              !spec::published_axes()[1].has_wrist_rating &&
              !spec::published_axes()[2].has_wrist_rating,
          "S, L and U carry no wrist rating, because the datasheet prints none");
}

void test_asset_absence() {
    std::cout << "\n--- a missing asset is a StudyError, not a crash ---\n";

    bool threw_study_error = false;
    std::string message;
    try {
        const Value ignored = spec::load_asset("gp8_published_specification_does_not_exist.json");
        (void)ignored;
    } catch (const study::StudyError& error) {
        threw_study_error = true;
        message = error.what();
    } catch (...) {
        threw_study_error = false;
    }
    check(threw_study_error && message.find("gp8_published_specification_does_not_exist") !=
                                   std::string::npos,
          "loading an absent asset throws StudyError naming the file it could not open");

    bool found = false;
    try {
        const std::string path = spec::find_asset_path();
        found = !path.empty() && path.find("gp8_published_specification.json") != std::string::npos;
    } catch (const study::StudyError& error) {
        std::cout << "       " << error.what() << "\n";
    }
    check(found, "find_asset_path() locates the asset by walking up to the repository root");
}

// ---------------------------------------------------------------------------
// Published axis limits against GP8_JOINT_LIMITS
// ---------------------------------------------------------------------------

void test_axis_limits() {
    std::cout << "\n--- published axis limits against GP8_JOINT_LIMITS ---\n";

    // Agreement with the instruction manual, which is what the engine encodes.
    for (std::size_t i = 0; i < study::GP8_DOF; ++i) {
        const spec::AxisPublished& axis = spec::published_axes()[i];
        const std::string where = std::string(axis.axis) + "-axis";
        check_near(study::joint_min(i), axis.manual_min_deg * kDegToRad, kRangeToleranceRad,
                   where + " lower limit matches HW1484385 page 4 within 1e-3 rad");
        check_near(study::joint_max(i), axis.manual_max_deg * kDegToRad, kRangeToleranceRad,
                   where + " upper limit matches HW1484385 page 4 within 1e-3 rad");
    }

    // The datasheet does not print the same GP8 L- and U-axis ranges as the
    // manual. That is a real disagreement between two Yaskawa documents, so it
    // is asserted with its magnitude rather than tolerated.
    const spec::AxisPublished& l_axis = spec::published_axes()[1];
    const spec::AxisPublished& u_axis = spec::published_axes()[2];
    check(l_axis.datasheet_max_deg == 150.0 && l_axis.manual_max_deg == 145.0,
          "documented discrepancy: DS-699-H page 2 gives the L-axis +150 deg, HW1484385 page 4 "
          "gives +145 deg");
    check_near(std::abs(study::joint_max(1) - l_axis.datasheet_max_deg * kDegToRad), 0.08799,
               1.0e-4,
               "the engine's L-axis upper limit is 0.088 rad (5.04 deg) below the datasheet, "
               "because it follows the manual");
    check(u_axis.datasheet_min_deg == -113.0 && u_axis.datasheet_max_deg == 255.0 &&
              u_axis.manual_min_deg == -70.0 && u_axis.manual_max_deg == 190.0,
          "documented discrepancy: DS-699-H page 2 gives the U-axis +255/-113 deg, HW1484385 "
          "page 4 gives -70/+190 deg");
    check_near(std::abs(study::joint_max(2) - u_axis.datasheet_max_deg * kDegToRad), 1.13457,
               1.0e-4, "the engine's U-axis upper limit is 1.1346 rad (65 deg) below the "
                       "datasheet, because it follows the manual");
    check_near(std::abs(study::joint_min(2) - u_axis.datasheet_min_deg * kDegToRad), 0.75123,
               1.0e-4, "the engine's U-axis lower limit is 0.7512 rad (43 deg) above the "
                       "datasheet, because it follows the manual");

    // Speeds: every axis but S is a rounding of the published figure.
    for (std::size_t i = 0; i < study::GP8_DOF; ++i) {
        const spec::AxisPublished& axis = spec::published_axes()[i];
        const double published = axis.manual_speed_rad_s;
        const double engine = study::joint_max_velocity(i);
        const std::string where = std::string(axis.axis) + "-axis";
        // The manual rounds its rad/s figure to two decimals, which is worth up
        // to 0.6 deg/s on the fastest axes; the two printed numbers are the
        // same speed to that rounding and no closer.
        check_near(axis.datasheet_speed_deg_s, published * kRadToDeg, 0.6,
                   where + " maximum speed is the same number in deg/s and in rad/s across the "
                           "two documents, to the manual's two-decimal rounding");
        if (i == 0) {
            check_near(published - engine, 0.09, 1.0e-9,
                       "documented discrepancy: the S-axis maximum speed is 7.94 rad/s "
                       "(455 deg/s) in both documents and 7.85 rad/s (449.84 deg/s) in "
                       "GP8_JOINT_LIMITS, 0.09 rad/s low");
        } else {
            check_near(engine, published, kSpeedToleranceRadPerSec,
                       where + " maximum speed matches HW1484385 page 4 within 0.025 rad/s");
        }
    }
}

// ---------------------------------------------------------------------------
// Ops
// ---------------------------------------------------------------------------

[[nodiscard]] Value args_from_defaults(const study::OpSpec& op) {
    Value args = Value::object();
    for (const study::ParamSpec& param : op.params) {
        args.set(param.name, param.default_value);
    }
    return args;
}

[[nodiscard]] std::size_t column_of(const Value& table, const std::string& name) {
    const Value& columns = table["columns"];
    for (std::size_t i = 0; i < columns.size(); ++i) {
        if (columns[i].as_string() == name) {
            return i;
        }
    }
    return columns.size();
}

void test_defaults(const study::RobotSpecificationModule& module) {
    std::cout << "\n--- every op runs from its declared defaults ---\n";

    const study::ModuleDescription description = module.describe();
    check(description.name == "robot_specification" && description.course.id == 3883 &&
              description.source == "cpp_solver/include/study/robot_specification.hpp",
          "the module describes itself with its name, course 3883 and its header path");
    check(description.ops.size() == 5 && description.has_op("datasheet") &&
              description.has_op("axis_limits") && description.has_op("wrist_capacity") &&
              description.has_op("component_tree") && description.has_op("mass_reconciliation"),
          "all five ops are declared");

    bool params_complete = true;
    for (const study::OpSpec& op : description.ops) {
        if (op.formula.empty() || op.explain.empty() || op.outputs.empty()) {
            params_complete = false;
        }
        for (const study::ParamSpec& param : op.params) {
            if (param.name.empty() || param.type.empty() || param.label.empty() ||
                param.default_value.is_null()) {
                params_complete = false;
            }
            if (param.type == "enum" && param.options.empty()) {
                params_complete = false;
            }
            if (param.type == "scalar" && !param.has_range) {
                params_complete = false;
            }
        }
    }
    check(params_complete,
          "every op carries a formula, an explanation, outputs, and parameters with a type, a "
          "label, a default and a range or options");

    for (const study::OpSpec& op : description.ops) {
        Value result;
        bool ran = false;
        try {
            result = module.invoke(op.name, args_from_defaults(op));
            ran = true;
        } catch (const std::exception& error) {
            std::cout << "       " << op.name << " threw " << error.what() << "\n";
        }
        bool complete = ran && result.is_object();
        for (const study::OutputSpec& output : op.outputs) {
            if (!result.contains(output.name)) {
                complete = false;
                std::cout << "       " << op.name << " did not return '" << output.name << "'\n";
            }
        }
        check(complete, op.name + " runs from its defaults and returns every declared output");
    }
}

void test_datasheet_op(const study::RobotSpecificationModule& module) {
    std::cout << "\n--- datasheet ---\n";

    const Value all = module.invoke("datasheet", Value::object());
    const Value& table = all["specification"];
    check(table["rows"].size() == spec::published_fields().size(),
          "the default datasheet table shows every published field");

    const std::size_t value_column = column_of(table, "value");
    const std::size_t unit_column = column_of(table, "unit");
    const std::size_t document_column = column_of(table, "document");
    const std::size_t page_column = column_of(table, "page");
    check(value_column < table["columns"].size() && unit_column < table["columns"].size() &&
              document_column < table["columns"].size() && page_column < table["columns"].size(),
          "the table carries parameter, value, unit, document and page columns");

    bool every_row_cited = true;
    for (std::size_t i = 0; i < table["rows"].size(); ++i) {
        const Value& row = table["rows"][i];
        const bool cited = row[document_column].is_string() &&
                           !row[document_column].as_string().empty() &&
                           row[page_column].is_number() && row[page_column].as_double() >= 1.0 &&
                           row[unit_column].is_string();
        if (!cited) {
            every_row_cited = false;
        }
    }
    check(every_row_cited, "every datasheet row carries a unit and a document with a page number");

    Value filtered_args = Value::object();
    filtered_args.set("section", Value(std::string("wrist_rating")));
    const Value filtered = module.invoke("datasheet", filtered_args);
    check(filtered["specification"]["rows"].size() == 6,
          "filtering to the wrist rating leaves the six R/B/T moment and inertia rows");
    check(all["sources"]["rows"].size() == 2,
          "the sources table names both documents with their manual number and URL");
}

void test_wrist_capacity(const study::RobotSpecificationModule& module) {
    std::cout << "\n--- wrist_capacity ---\n";

    struct Case {
        const char* name;
        double mass;
        Eigen::Vector3d offset;
        double inertia;
        double b_angle;
        int failing_axis;  // -1 for none, else index into R, B, T
    };

    // Each failing case loads one wrist axis past its rating while leaving the
    // other two inside theirs, which is only possible because the B-axis angle
    // decides how far the R axis has swung off the flange axis.
    const std::array<Case, 4> cases = {{
        {"a 3 kg tool 80 mm off the flange", 3.0, Eigen::Vector3d(0.0, 0.0, 0.08), 0.01, 0.0, -1},
        {"an 8 kg tool 250 mm along the flange axis, B bending it", 8.0,
         Eigen::Vector3d(0.0, 0.0, 0.25), 0.0, 0.0, 1},
        {"an 8 kg tool 140 mm radially off the flange axis", 8.0,
         Eigen::Vector3d(0.14, 0.0, 0.0), 0.0, 0.0, 2},
        {"an 8 kg tool offset in y and z with the B axis at 90 deg", 8.0,
         Eigen::Vector3d(0.0, 0.12, 0.20), 0.0, std::numbers::pi / 2.0, 0},
    }};

    for (const Case& test_case : cases) {
        Value args = Value::object();
        args.set("payload_mass", Value(test_case.mass));
        args.set("payload_offset", json::from_vec3(test_case.offset));
        args.set("payload_inertia", Value(test_case.inertia));
        args.set("b_angle", Value(test_case.b_angle));
        const Value result = module.invoke("wrist_capacity", args);
        const spec::WristAssessment assessment = spec::evaluate_wrist(
            test_case.mass, test_case.offset, test_case.inertia, test_case.b_angle);

        if (test_case.failing_axis < 0) {
            check(result["pass"].as_bool() && assessment.pass,
                  std::string("wrist_capacity passes ") + test_case.name);
            bool margins_positive = true;
            for (const spec::WristAxisLoad& axis : assessment.axes) {
                margins_positive = margins_positive && axis.moment_margin_nm > 0.0 &&
                                   axis.inertia_margin_kgm2 > 0.0 &&
                                   axis.moment_utilisation < 1.0;
            }
            check(margins_positive, "a passing payload reports a positive margin on all three axes");
            continue;
        }

        bool only_that_axis_failed = !result["pass"].as_bool() && !assessment.pass;
        for (std::size_t i = 0; i < assessment.axes.size(); ++i) {
            const bool should_fail = (static_cast<int>(i) == test_case.failing_axis);
            if (assessment.axes[i].ok == should_fail) {
                only_that_axis_failed = false;
                std::cout << "       " << assessment.axes[i].axis
                          << " ok=" << assessment.axes[i].ok << " moment "
                          << assessment.axes[i].moment_nm << " / "
                          << assessment.axes[i].allowable_moment_nm << " N m, inertia "
                          << assessment.axes[i].inertia_kgm2 << " / "
                          << assessment.axes[i].allowable_inertia_kgm2 << " kg m^2\n";
            }
        }
        check(only_that_axis_failed,
              std::string("wrist_capacity fails the ") +
                  assessment.axes[static_cast<std::size_t>(test_case.failing_axis)].axis +
                  "-axis alone for " + test_case.name);
    }

    // The physics the op claims: worst-case gravity moment about an axis is
    // m g times the offset component perpendicular to it.
    const spec::WristAssessment side = spec::evaluate_wrist(
        4.0, Eigen::Vector3d(0.10, 0.0, 0.0), 0.0, 0.0);
    check_near(side.axes[2].moment_nm, 4.0 * study::GP8_GRAVITY_MPS2 * 0.10, 1.0e-12,
               "T-axis moment is m g r_perp for a purely radial offset");
    check_near(side.axes[2].inertia_kgm2, 4.0 * 0.10 * 0.10, 1.0e-12,
               "T-axis payload inertia is the parallel-axis m r_perp^2");
    check_near(side.axes[0].moment_nm, side.axes[2].moment_nm, 1.0e-12,
               "with the B axis at zero the R and T axes see the same lever arm, as the real "
               "wrist does");

    Value over_payload = Value::object();
    over_payload.set("payload_mass", Value(9.0));
    const Value over = module.invoke("wrist_capacity", over_payload);
    check(!over["mass_ok"].as_bool() && !over["pass"].as_bool() &&
              over["payload_margin_kg"].as_double() < 0.0,
          "9 kg is rejected against the published 8 kg rated payload");
}

void test_component_tree(const study::RobotSpecificationModule& module) {
    std::cout << "\n--- component_tree ---\n";

    const study::ModuleDescription description = module.describe();
    const study::OpSpec* tree_op = nullptr;
    for (const study::OpSpec& op : description.ops) {
        if (op.name == "component_tree") {
            tree_op = &op;
        }
    }
    check(tree_op != nullptr, "component_tree is declared");
    if (tree_op == nullptr) {
        return;
    }
    const std::string first_sentence = tree_op->explain.substr(0, tree_op->explain.find('.') + 1);
    check(first_sentence.find("MODELLED") != std::string::npos &&
              first_sentence.find("not Yaskawa data") != std::string::npos,
          "the first sentence of the explanation says the breakdown is modelled, not Yaskawa data");

    const Value result = module.invoke("component_tree", args_from_defaults(*tree_op));
    check(result["provenance"].is_string() &&
              result["provenance"].as_string() == std::string(spec::kComponentProvenance),
          "component_tree returns the provenance text");
    check(result["provenance"].as_string().find("not taken from any vendor document") !=
              std::string::npos,
          "the provenance text states that no vendor document was used");

    const int shown = static_cast<int>(result["shown"].as_double());
    const int matching = static_cast<int>(result["matching"].as_double());
    const int total = static_cast<int>(result["total"].as_double());
    check(shown == static_cast<int>(result["components"]["rows"].size()),
          "the reported count of shown components equals the rows returned");
    check(shown <= matching && matching <= total && total > 0,
          "shown <= matching <= total (" + std::to_string(shown) + " <= " +
              std::to_string(matching) + " <= " + std::to_string(total) + ")");

    bool mass_column_labelled = false;
    const Value& columns = result["components"]["columns"];
    for (std::size_t i = 0; i < columns.size(); ++i) {
        if (columns[i].as_string().find("mass") != std::string::npos) {
            mass_column_labelled = columns[i].as_string().find("modelled") != std::string::npos;
        }
    }
    check(mass_column_labelled, "the component mass column is labelled modelled, not published");

    bool subsystem_masses_labelled = true;
    const Value& subsystem_columns = result["subsystem_mass"]["columns"];
    for (std::size_t i = 0; i < subsystem_columns.size(); ++i) {
        const std::string column = subsystem_columns[i].as_string();
        if (column.find("mass") != std::string::npos &&
            column.find("modelled") == std::string::npos) {
            subsystem_masses_labelled = false;
        }
    }
    check(subsystem_masses_labelled, "every subsystem mass column is labelled modelled");

    std::size_t component_sum = 0;
    double leaf_mass_sum = 0.0;
    bool leaves_are_a_subset = true;
    for (const spec::SubsystemMass& subsystem : spec::subsystem_masses()) {
        component_sum += subsystem.component_count;
        leaf_mass_sum += subsystem.leaf_mass_kg;
        leaves_are_a_subset = leaves_are_a_subset &&
                              subsystem.leaf_count <= subsystem.component_count &&
                              subsystem.leaf_mass_kg >= 0.0;
    }
    check(leaves_are_a_subset,
          "in every subsystem the leaves are a subset of its components and carry a non-negative "
          "modelled mass");
    check(component_sum == static_cast<std::size_t>(total),
          "the per-subsystem component counts sum to the whole tree");
    check_near(leaf_mass_sum, result["total_leaf_mass_kg_modelled"].as_double(), 1.0e-9,
               "the per-subsystem modelled leaf masses sum to the reported tree total");

    Value one_subsystem = Value::object();
    one_subsystem.set("subsystem", Value(std::string("R-Axis & Wrist")));
    one_subsystem.set("depth", Value(5));
    one_subsystem.set("max_rows", Value(400));
    const Value wrist = module.invoke("component_tree", one_subsystem);
    check(static_cast<int>(wrist["matching"].as_double()) ==
                  static_cast<int>(wrist["shown"].as_double()) &&
              wrist["shown"].as_double() > 0.0,
          "at full depth and row budget, every matching component of one subsystem is shown");
    check(wrist["matching"].as_double() < result["total"].as_double(),
          "one subsystem holds fewer components than the whole tree");
}

void test_mass_reconciliation(const study::RobotSpecificationModule& module) {
    std::cout << "\n--- mass_reconciliation ---\n";

    const Value result = module.invoke("mass_reconciliation", Value::object());
    const Value& links = result["links"];
    check(links["rows"].size() == study::GP8_DOF, "one row per moving link");

    const std::size_t discrepancy_column = column_of(links, "discrepancy_kg");
    const std::size_t tree_column = column_of(links, "tree_leaf_mass_kg_modelled");
    const std::size_t model_column = column_of(links, "gp8_model_mass_kg");
    check(discrepancy_column < links["columns"].size() && tree_column < links["columns"].size() &&
              model_column < links["columns"].size(),
          "the table carries the modelled tree mass, the gp8_model.hpp mass and the discrepancy");

    double discrepancy_sum = 0.0;
    double tree_sum = 0.0;
    double model_sum = 0.0;
    for (std::size_t i = 0; i < links["rows"].size(); ++i) {
        const Value& row = links["rows"][i];
        discrepancy_sum += row[discrepancy_column].as_double();
        tree_sum += row[tree_column].as_double();
        model_sum += row[model_column].as_double();
    }
    check_near(discrepancy_sum, result["total_discrepancy_kg"].as_double(), 1.0e-9,
               "the per-link discrepancies sum to the reported total discrepancy");
    check_near(tree_sum, result["tree_link_mass_kg_modelled"].as_double(), 1.0e-9,
               "the per-link modelled tree masses sum to the reported tree total");
    check_near(model_sum, result["header_link_mass_kg_modelled"].as_double(), 1.0e-9,
               "the per-link gp8_model.hpp masses sum to the reported header total");
    check_near(model_sum, study::total_moving_mass(), 1.0e-9,
               "those masses are GP8_LINKS from gp8_model.hpp, not a second copy");
    check_near(result["total_discrepancy_kg"].as_double(), tree_sum - model_sum, 1.0e-9,
               "the total discrepancy is the modelled tree total minus the gp8_model.hpp total");

    check_near(result["published_reference_kg"].as_double(), 32.0, 1.0e-12,
               "the default published reference is the 32 kg of DS-699-H page 2, the GP8 column");
    check_near(result["tree_vs_published_kg"].as_double(),
               result["tree_link_mass_kg_modelled"].as_double() -
                   result["published_reference_kg"].as_double(),
               1.0e-9,
               "the gap to the published total is the modelled tree total minus that published "
               "mass");

    Value manual_args = Value::object();
    manual_args.set("published_reference", Value(std::string("HW1484385")));
    const Value against_manual = module.invoke("mass_reconciliation", manual_args);
    check_near(against_manual["published_reference_kg"].as_double(), 35.0, 1.0e-12,
               "the reference can be switched to the 35 kg of HW1484385 page 4");

    Value model_args = Value::object();
    model_args.set("published_reference", Value(std::string("gp8_model.hpp")));
    const Value against_model = module.invoke("mass_reconciliation", model_args);
    check_near(against_model["published_reference_kg"].as_double(), study::GP8_ROBOT_MASS_KG,
               1.0e-12,
               "the reference can be switched to gp8_model.hpp's own GP8_ROBOT_MASS_KG");
    check(against_model["totals"]["rows"].size() >= 7,
          "the totals table labels every number modelled or published");

    bool totals_labelled = true;
    const std::size_t kind_column = column_of(result["totals"], "kind");
    for (std::size_t i = 0; i < result["totals"]["rows"].size(); ++i) {
        const std::string kind = result["totals"]["rows"][i][kind_column].as_string();
        if (kind.find("modelled") == std::string::npos &&
            kind.find("published") == std::string::npos) {
            totals_labelled = false;
        }
    }
    check(totals_labelled, "every total says whether it is modelled or published");
    check(result["provenance"].as_string() == std::string(spec::kComponentProvenance),
          "mass_reconciliation repeats the provenance of the breakdown it sums");

    std::cout << "       modelled tree link mass " << tree_sum << " kg, GP8_LINKS " << model_sum
              << " kg, published 32 kg (DS-699-H p2), 35 kg (HW1484385 p4)\n";
}

void test_axis_limits_op(const study::RobotSpecificationModule& module) {
    std::cout << "\n--- axis_limits ---\n";

    const Value result = module.invoke("axis_limits", Value::object());
    check(result["ranges"]["rows"].size() == study::GP8_DOF &&
              result["speeds"]["rows"].size() == study::GP8_DOF,
          "one range row and one speed row per axis");

    const Value& ranges = result["ranges"];
    const std::size_t engine_min = column_of(ranges, "engine_min_rad");
    const std::size_t engine_max = column_of(ranges, "engine_max_rad");
    const std::size_t diff_min = column_of(ranges, "diff_min_rad");
    const std::size_t diff_max = column_of(ranges, "diff_max_rad");
    bool engine_matches = true;
    for (std::size_t i = 0; i < ranges["rows"].size(); ++i) {
        const Value& row = ranges["rows"][i];
        engine_matches = engine_matches &&
                         row[engine_min].as_double() == study::joint_min(i) &&
                         row[engine_max].as_double() == study::joint_max(i) &&
                         std::abs(row[diff_min].as_double()) <= kRangeToleranceRad &&
                         std::abs(row[diff_max].as_double()) <= kRangeToleranceRad;
    }
    check(engine_matches,
          "the engine column is GP8_JOINT_LIMITS itself and every difference against HW1484385 "
          "is inside the declared 1e-3 rad");
    check(static_cast<int>(result["range_disagreements"].as_double()) == 0,
          "no axis range disagrees with the manual beyond the tolerance");
    check(static_cast<int>(result["speed_disagreements"].as_double()) == 1,
          "exactly one speed disagrees - the S-axis, 7.85 rad/s against a published 7.94 rad/s");
    check(static_cast<int>(result["document_disagreements"].as_double()) == 2,
          "the panel reports the two axes on which the datasheet and the manual disagree (L and "
          "U)");
    check(!result["all_agree"].as_bool(),
          "all_agree is false, because the S-axis speed really does not match the documents");
    check(result["verdict"].as_string().find("DS-699-H") != std::string::npos,
          "the verdict names the document that disagrees instead of hiding it");

    Value tight = Value::object();
    tight.set("range_tolerance", Value(1.0e-6));
    const Value strict = module.invoke("axis_limits", tight);
    check(static_cast<int>(strict["range_disagreements"].as_double()) > 0,
          "at a 1e-6 rad tolerance the rounding of the radian constants shows up, so the "
          "tolerance the suite states is doing real work");
}

}  // namespace

int main() {
    std::cout << "====================================================\n";
    std::cout << "   RUNNING GP8 SPECIFICATION STUDY TEST SUITE       \n";
    std::cout << "====================================================\n";

    const study::RobotSpecificationModule module;

    try {
        const Value asset = spec::load_asset();
        test_asset(asset);
        test_asset_against_compiled_table(asset);
    } catch (const study::StudyError& error) {
        check(false, std::string("the committed asset loads: ") + error.what());
    }
    test_asset_absence();
    test_axis_limits();
    test_defaults(module);
    test_datasheet_op(module);
    test_axis_limits_op(module);
    test_wrist_capacity(module);
    test_component_tree(module);
    test_mass_reconciliation(module);

    std::cout << "====================================================\n";
    std::cout << "checks run: " << g_checks << ", failed: " << g_failures << "\n";
    if (g_failures != 0) {
        for (const std::string& name : g_failed_names) {
            std::cout << "FAILED: " << name << "\n";
        }
        std::cout << "SPECIFICATION TESTS FAILED\n";
        return 1;
    }
    std::cout << "          ALL TESTS PASSED                          \n";
    std::cout << "====================================================\n";
    return 0;
}
