// Test suite for the study layer: JSON, the registry protocol and the
// spatial_math module. Plain asserts in the style of src/test_kinematics.cpp -
// there is no gtest in this project and there will not be one.
//
// Every check asserts a property (orthonormality, a round trip, agreement with
// the existing forward kinematics), never a printed string.

#include "study/gp8_model.hpp"
#include "study/json.hpp"
#include "study/module_registry.hpp"
#include "study/spatial_math.hpp"
#include "study/study_module.hpp"
#include "yaskawa_kinematics.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

using yaskawa::study::json::Value;
namespace json = yaskawa::study::json;
namespace study = yaskawa::study;

namespace {

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
// JSON
// ---------------------------------------------------------------------------

void expect_round_trip(const Value& value, const std::string& what) {
    const std::string text = json::dump(value);
    bool ok = false;
    try {
        ok = (json::parse(text) == value);
    } catch (const std::exception& error) {
        std::cout << "       dump was " << text << ", parse threw " << error.what() << "\n";
    }
    if (!ok) {
        std::cout << "       dump was " << text << "\n";
    }
    check(ok, "json round trip: " + what);
}

void expect_parse_error(const std::string& text, const std::string& what) {
    bool threw_parse_error = false;
    try {
        const Value parsed = json::parse(text);
        (void)parsed;
    } catch (const json::ParseError&) {
        threw_parse_error = true;
    } catch (...) {
        threw_parse_error = false;
    }
    check(threw_parse_error, "json rejects " + what);
}

void test_json() {
    std::cout << "\n--- json ---\n";

    expect_round_trip(Value(), "null");
    expect_round_trip(Value(true), "true");
    expect_round_trip(Value(false), "false");
    expect_round_trip(Value(0), "zero");
    expect_round_trip(Value(42), "integer");
    expect_round_trip(Value(-17.25), "negative fraction");
    expect_round_trip(Value(3.141592653589793), "pi");
    expect_round_trip(Value(1e30), "large exponent");
    expect_round_trip(Value(-2.5e-9), "small negative exponent");
    expect_round_trip(Value(std::numeric_limits<double>::min()), "smallest normal double");
    expect_round_trip(Value(""), "empty string");
    expect_round_trip(Value("plain"), "plain string");
    expect_round_trip(Value(std::string("quote \" backslash \\ slash / control \b\f\n\r\t")),
                      "string needing every escape");
    expect_round_trip(Value::array(), "empty array");
    expect_round_trip(Value::object(), "empty object");
    expect_round_trip(Value::array({Value(1), Value("two"), Value(true), Value()}), "mixed array");
    expect_round_trip(Value::array({Value::array({Value(1), Value(2)}),
                                    Value::object({{"deep", Value::array({Value(3)})}})}),
                      "nested array of objects");
    expect_round_trip(Value::object({{"a", Value(1)},
                                     {"b", Value::object({{"c", Value::array({Value(2), Value(3)})}})},
                                     {"d", Value()}}),
                      "nested object");

    Eigen::Matrix<double, 6, 1> q6;
    q6 << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
    expect_round_trip(json::from_vec6(q6), "vec6 helper");
    expect_round_trip(json::from_matrix(Eigen::Matrix4d::Identity()), "mat4 helper");
    expect_round_trip(json::from_isometry(Eigen::Isometry3d::Identity()), "isometry helper");
    expect_round_trip(json::from_series("step", std::vector<double>{0.0, 0.1},
                                        std::vector<double>{1.0, 2.0}),
                      "series helper");
    expect_round_trip(json::from_points({Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6)}),
                      "points helper");
    expect_round_trip(json::from_complex_set({{-1.0, 2.0}, {-1.0, -2.0}}), "complex_set helper");
    expect_round_trip(json::from_table({"a", "b"}, std::vector<std::vector<double>>{{1.0, 2.0}}),
                      "table helper");

    // Escapes decode to the right bytes.
    {
        const Value parsed = json::parse("\"a\\\"b\\\\c\\/d\\be\\ff\\ng\\rh\\ti\"");
        const std::string expected = "a\"b\\c/d\be\ff\ng\rh\ti";
        check(parsed.is_string() && parsed.as_string() == expected,
              "json decodes every two-character escape");
    }

    // \uXXXX, including a surrogate pair, becomes UTF-8.
    {
        const Value parsed = json::parse("\"\\u00e9\"");
        const std::string& text = parsed.as_string();
        check(text.size() == 2 && static_cast<unsigned char>(text[0]) == 0xC3 &&
                  static_cast<unsigned char>(text[1]) == 0xA9,
              "json decodes \\u00e9 to two UTF-8 bytes");
        expect_round_trip(parsed, "decoded \\u00e9");
    }
    {
        const Value parsed = json::parse("\"\\ud83d\\ude00\"");
        check(parsed.as_string().size() == 4, "json decodes a surrogate pair to four UTF-8 bytes");
        expect_round_trip(parsed, "decoded surrogate pair");
    }
    {
        const Value parsed = json::parse("\"\\u0007\"");
        check(parsed.as_string() == std::string(1, '\x07') && json::dump(parsed) == "\"\\u0007\"",
              "json re-escapes a control character it decoded");
    }

    // Non-finite numbers must never reach the wire as nan/inf.
    check(json::dump(Value(std::nan(""))) == "null", "json dumps NaN as null");
    check(json::dump(Value(std::numeric_limits<double>::infinity())) == "null",
          "json dumps +inf as null");
    check(json::dump(Value(-std::numeric_limits<double>::infinity())) == "null",
          "json dumps -inf as null");
    check(json::dump(json::from_vec3(Eigen::Vector3d(1.0, std::nan(""), 3.0))) == "[1,null,3]",
          "json dumps a NaN inside a vector as null");

    // Key order is insertion order, not sorted order: output is deterministic.
    {
        Value object = Value::object();
        object.set("zulu", Value(1));
        object.set("alpha", Value(2));
        object.set("mike", Value(3));
        check(json::dump(object) == "{\"zulu\":1,\"alpha\":2,\"mike\":3}",
              "json preserves object key order");
        object.set("alpha", Value(9));
        check(json::dump(object) == "{\"zulu\":1,\"alpha\":9,\"mike\":3}",
              "json overwrites a key in place without reordering");
    }

    // Accessors and lookups.
    {
        const Value object = Value::object({{"n", Value(7)}, {"s", Value("x")}});
        check(object.contains("n") && !object.contains("missing"), "json contains() works");
        check(object["missing"].is_null(), "json reads a missing key as null");
        check(object["n"].as_int() == 7 && object["s"].as_string() == "x", "json as_* accessors");
        check(object.size() == 2, "json object size");
        const Value list = Value::array({Value(1), Value(2), Value(3)});
        check(list.size() == 3 && list[2].as_double() == 3.0 && list[9].is_null(),
              "json array indexing");
    }

    expect_parse_error("", "an empty document");
    expect_parse_error("   ", "whitespace only");
    expect_parse_error("{", "an unterminated object");
    expect_parse_error("[1,2", "an unterminated array");
    expect_parse_error("{\"a\":}", "a missing value");
    expect_parse_error("{\"a\" 1}", "a missing colon");
    expect_parse_error("{a:1}", "an unquoted key");
    expect_parse_error("[1,]", "a trailing comma");
    expect_parse_error("[1 2]", "a missing comma");
    expect_parse_error("01", "a leading zero");
    expect_parse_error("+1", "a leading plus");
    expect_parse_error(".5", "a bare decimal point");
    expect_parse_error("1.", "a trailing decimal point");
    expect_parse_error("1e", "an empty exponent");
    expect_parse_error("nan", "the literal nan");
    expect_parse_error("tru", "a truncated literal");
    expect_parse_error("\"unterminated", "an unterminated string");
    expect_parse_error("\"\\q\"", "an unknown escape");
    expect_parse_error("\"\\u00g0\"", "a bad hex escape");
    expect_parse_error("\"\\ud83d\"", "a lone high surrogate");
    expect_parse_error("{\"a\":1} trailing", "trailing content");
}

// ---------------------------------------------------------------------------
// Registry protocol
// ---------------------------------------------------------------------------

[[nodiscard]] Value handle_text(const study::ModuleRegistry& registry, const std::string& text) {
    try {
        return registry.handle(json::parse(text));
    } catch (const std::exception& error) {
        std::cout << "       handle() threw on " << text << ": " << error.what() << "\n";
        return Value();
    }
}

[[nodiscard]] Eigen::Matrix4d read_mat4(const Value& value) {
    Eigen::Matrix4d out = Eigen::Matrix4d::Zero();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out(r, c) = value[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)].as_double();
        }
    }
    return out;
}

[[nodiscard]] Eigen::Matrix3d read_mat3(const Value& value) {
    Eigen::Matrix3d out = Eigen::Matrix3d::Zero();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out(r, c) = value[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)].as_double();
        }
    }
    return out;
}

[[nodiscard]] Eigen::Vector3d read_vec3(const Value& value) {
    return Eigen::Vector3d(value[0].as_double(), value[1].as_double(), value[2].as_double());
}

[[nodiscard]] std::string error_text(const Value& response) {
    const Value& error = response["error"];
    return error.is_string() ? error.as_string() : std::string("<no error field>");
}

[[nodiscard]] bool is_failure(const Value& response, const std::string& fragment) {
    if (!response.is_object() || !response["ok"].is_bool() || response["ok"].as_bool()) {
        return false;
    }
    if (!response["error"].is_string()) {
        return false;
    }
    return response["error"].as_string().find(fragment) != std::string::npos;
}

void test_registry() {
    std::cout << "\n--- registry ---\n";

    const study::ModuleRegistry registry = study::build_default_registry();
    check(registry.size() >= 1, "registry holds at least one module");
    check(registry.find("spatial_math") != nullptr, "registry finds spatial_math");
    check(registry.find("no_such_module") == nullptr, "registry returns nullptr for an unknown name");

    // Failure paths: all four must be ok:false, and none may throw.
    check(is_failure(handle_text(registry, "{\"id\":11,\"module\":\"nope\",\"op\":\"x\"}"),
                     "unknown module"),
          "registry rejects an unknown module");
    check(is_failure(handle_text(registry, "{\"id\":12,\"module\":\"spatial_math\",\"op\":\"nope\"}"),
                     "has no op"),
          "registry rejects an unknown op");
    check(is_failure(handle_text(registry,
                                 "{\"id\":13,\"module\":\"spatial_math\",\"op\":\"rotation_from_"
                                 "rpy\",\"args\":{}}"),
                     "'rpy'"),
          "registry rejects a missing argument and names it");
    check(is_failure(handle_text(registry,
                                 "{\"id\":14,\"module\":\"spatial_math\",\"op\":\"frame_of_joint\","
                                 "\"args\":{\"q\":[99,0,0,0,0,0]}}"),
                     "out of range"),
          "registry rejects an out-of-range argument");
    check(is_failure(handle_text(registry,
                                 "{\"id\":15,\"module\":\"spatial_math\",\"op\":\"frame_of_joint\","
                                 "\"args\":{\"q\":[0,0,0,0,0]}}"),
                     "must have 6 elements"),
          "registry rejects a wrong-length vector");
    check(is_failure(handle_text(registry, "{\"id\":16,\"module\":\"spatial_math\"}"), "'op'"),
          "registry rejects a request without an op");
    check(is_failure(handle_text(registry, "[1,2,3]"), "must be a JSON object"),
          "registry rejects a request that is not an object");
    check(is_failure(handle_text(registry, "{\"id\":17,\"op\":\"compose\"}"), "'module'"),
          "registry rejects a non-describe op without a module");

    // The id is echoed unchanged on both paths.
    {
        const Value failure = handle_text(registry, "{\"id\":4242,\"module\":\"x\",\"op\":\"y\"}");
        check(failure["id"].is_number() && failure["id"].as_int() == 4242,
              "registry echoes the id on a failure");
        const Value ok = handle_text(
            registry,
            "{\"id\":-7,\"module\":\"spatial_math\",\"op\":\"rotation_from_rpy\",\"args\":{\"rpy\":"
            "[0,0,0]}}");
        check(ok["id"].is_number() && ok["id"].as_int() == -7, "registry echoes a negative id");
        check(ok["ok"].is_bool() && ok["ok"].as_bool(), "registry answers a valid request with ok");
        check(ok["module"].as_string() == "spatial_math" &&
                  ok["op"].as_string() == "rotation_from_rpy",
              "registry echoes the module and the op");
        check(ok["us"].is_number() && ok["us"].as_int() >= 0, "registry reports us >= 0");
        check(ok["result"].is_object(), "registry wraps the result in an object");
    }

    // Reserved describe forms.
    {
        const Value whole = handle_text(registry, "{\"id\":1,\"op\":\"describe\"}");
        check(whole["ok"].as_bool() && whole["us"].is_number(), "describe with no module succeeds");
        const Value& catalogue = whole["result"];
        check(catalogue["modules"].size() == registry.size(),
              "describe_all lists every registered module");
        check(catalogue["courses"].size() >= 1, "describe_all lists the courses");

        bool every_module_described = true;
        for (std::size_t i = 0; i < catalogue["modules"].size(); ++i) {
            const Value& description = catalogue["modules"][i];
            every_module_described = every_module_described && description["name"].is_string() &&
                                     description["title"].is_string() &&
                                     description["course"]["id"].is_number() &&
                                     description["source"].is_string() &&
                                     description["summary"].is_string() &&
                                     description["ops"].size() > 0 &&
                                     registry.find(description["name"].as_string()) != nullptr;
        }
        check(every_module_described, "every described module is complete and resolvable");

        bool every_course_linked = true;
        for (std::size_t i = 0; i < catalogue["courses"].size(); ++i) {
            const Value& course = catalogue["courses"][i];
            every_course_linked = every_course_linked && course["id"].is_number() &&
                                  course["code"].is_string() && course["name"].is_string() &&
                                  course["modules"].size() > 0;
        }
        check(every_course_linked, "every course carries its module names");

        const Value one = handle_text(
            registry, "{\"id\":2,\"module\":\"spatial_math\",\"op\":\"describe\"}");
        check(one["ok"].as_bool() && one["result"]["name"].as_string() == "spatial_math",
              "describe with a module returns that module");
        check(one["result"]["course"]["id"].as_int() == 3883 &&
                  one["result"]["course"]["code"].as_string() == "M-407-01",
              "spatial_math points at Moodle course 3883");
        check(one["result"]["source"].as_string() == "cpp_solver/include/study/spatial_math.hpp",
              "the description carries a repository-relative source path");
    }

    // Every declared op must run from its own declared defaults: that is what
    // makes the generated UI trustworthy.
    {
        bool all_defaults_run = true;
        bool all_params_described = true;
        for (std::size_t m = 0; m < registry.size(); ++m) {
            const Value catalogue = registry.describe_all();
            const Value& description = catalogue["modules"][m];
            const std::string module_name = description["name"].as_string();
            for (std::size_t o = 0; o < description["ops"].size(); ++o) {
                const Value& op = description["ops"][o];
                Value args = Value::object();
                for (std::size_t p = 0; p < op["params"].size(); ++p) {
                    const Value& param = op["params"][p];
                    all_params_described = all_params_described && param["name"].is_string() &&
                                           param["type"].is_string() && param["label"].is_string() &&
                                           !param["default"].is_null();
                    if (param["type"].as_string() == "scalar" ||
                        param["type"].as_string() == "int" || param["type"].as_string() == "vec3" ||
                        param["type"].as_string() == "vec6") {
                        all_params_described = all_params_described && param["min"].is_number() &&
                                               param["max"].is_number() && param["unit"].is_string();
                    }
                    if (param["type"].as_string() == "enum") {
                        all_params_described = all_params_described && param["options"].size() > 0;
                    }
                    args.set(param["name"].as_string(), param["default"]);
                }
                Value request = Value::object();
                request.set("id", Value(100 + static_cast<int>(o)));
                request.set("module", Value(module_name));
                request.set("op", op["name"]);
                request.set("args", args);
                Value response;
                try {
                    response = registry.handle(request);
                } catch (const std::exception& error) {
                    std::cout << "       handle() threw for " << module_name << "."
                              << op["name"].as_string() << ": " << error.what() << "\n";
                }
                const bool ok = response.is_object() && response["ok"].is_bool() &&
                                response["ok"].as_bool() && response["result"].is_object();
                if (!ok) {
                    std::cout << "       default invocation failed for " << module_name << "."
                              << op["name"].as_string() << ": "
                              << error_text(response) << "\n";
                    all_defaults_run = false;
                }
                // Every declared output must actually be present in the result.
                for (std::size_t x = 0; ok && x < op["outputs"].size(); ++x) {
                    const std::string output_name = op["outputs"][x]["name"].as_string();
                    if (!response["result"].contains(output_name)) {
                        std::cout << "       missing declared output " << output_name << " of "
                                  << module_name << "." << op["name"].as_string() << "\n";
                        all_defaults_run = false;
                    }
                }
            }
        }
        check(all_params_described, "every param declares type, label, default and a range");
        check(all_defaults_run, "every op runs from its declared defaults and returns every output");
    }
}

// ---------------------------------------------------------------------------
// spatial_math
// ---------------------------------------------------------------------------

void test_spatial_math() {
    std::cout << "\n--- spatial_math ---\n";

    const study::ModuleRegistry registry = study::build_default_registry();

    const std::vector<Eigen::Vector3d> rpy_samples = {
        Eigen::Vector3d(0.0, 0.0, 0.0),      Eigen::Vector3d(0.1, 0.2, 0.3),
        Eigen::Vector3d(-0.5, 0.7, -1.2),    Eigen::Vector3d(1.0, -0.4, 2.5),
        Eigen::Vector3d(2.9, 1.1, -2.9),     Eigen::Vector3d(-3.0, -1.4, 0.8),
    };

    // R is a proper rotation and the module says so too.
    {
        double worst_orthonormality = 0.0;
        double worst_determinant = 0.0;
        for (const auto& rpy : rpy_samples) {
            const Eigen::Matrix3d R = study::spatial::rotation_from_rpy(rpy);
            worst_orthonormality =
                std::max(worst_orthonormality, study::spatial::orthonormality_error(R));
            worst_determinant = std::max(worst_determinant, std::abs(R.determinant() - 1.0));
        }
        check(worst_orthonormality < 1e-12,
              "rotation_from_rpy is orthonormal to 1e-12 (worst " +
                  json::number_to_string(worst_orthonormality) + ")");
        check(worst_determinant < 1e-12,
              "rotation_from_rpy has det(R) = 1 to 1e-12 (worst " +
                  json::number_to_string(worst_determinant) + ")");
    }

    // The same, through the wire protocol.
    {
        Value request = Value::object();
        request.set("id", Value(21));
        request.set("module", Value("spatial_math"));
        request.set("op", Value("rotation_from_rpy"));
        request.set("args", Value::object({{"rpy", json::from_vec3(rpy_samples[2])}}));
        const Value response = registry.handle(request);
        const Eigen::Matrix3d R = read_mat3(response["result"]["R"]);
        check_near((R - study::spatial::rotation_from_rpy(rpy_samples[2])).norm(), 0.0, 1e-15,
                   "rotation_from_rpy over JSON equals the in-process map");
        check_near(response["result"]["det_R"].as_double(), 1.0, 1e-12, "the op reports det(R) = 1");
        check(response["result"]["orthonormality_error"].as_double() < 1e-12,
              "the op reports an orthonormality error below 1e-12");
        const double angle = response["result"]["angle"].as_double();
        const Eigen::Vector3d axis = read_vec3(response["result"]["axis"]);
        const Eigen::Matrix3d from_axis_angle(Eigen::AngleAxisd(angle, axis.normalized()));
        check_near((from_axis_angle - R).norm(), 0.0, 1e-12,
                   "the reported axis-angle rebuilds the same rotation");
        const Value& quaternion = response["result"]["quaternion"];
        const Eigen::Quaterniond q(quaternion["rows"][0][0].as_double(),
                                   quaternion["rows"][0][1].as_double(),
                                   quaternion["rows"][0][2].as_double(),
                                   quaternion["rows"][0][3].as_double());
        check_near((q.toRotationMatrix() - R).norm(), 0.0, 1e-12,
                   "the reported quaternion rebuilds the same rotation");
    }

    // The inverse map round-trips on every non-degenerate sample.
    {
        double worst = 0.0;
        for (const auto& rpy : rpy_samples) {
            const Eigen::Matrix3d R = study::spatial::rotation_from_rpy(rpy);
            const study::spatial::RpyBranches branches = study::spatial::rpy_from_rotation(R);
            worst = std::max(worst, (branches.primary - rpy).cwiseAbs().maxCoeff());
            // The second branch must describe the same rotation, not the same angles.
            const Eigen::Matrix3d R2 = study::spatial::rotation_from_rpy(branches.alternate);
            worst = std::max(worst, (R2 - R).norm());
        }
        check(worst < 1e-12, "rpy_from_rotation(rotation_from_rpy(x)) == x and the second branch "
                             "rebuilds R (worst " + json::number_to_string(worst) + ")");
    }

    // Gimbal lock is flagged with a numeric margin, not guessed at.
    {
        Value request = Value::object();
        request.set("module", Value("spatial_math"));
        request.set("op", Value("rpy_from_rotation"));
        const Eigen::Matrix3d locked =
            study::spatial::rotation_from_rpy(Eigen::Vector3d(0.3, std::numbers::pi / 2.0, 0.0));
        request.set("args", Value::object({{"R", json::from_matrix(locked)}}));
        const Value response = registry.handle(request);
        check(response["ok"].as_bool() && response["result"]["gimbal_lock"].as_bool() &&
                  response["result"]["gimbal_margin"].as_double() < 1e-9,
              "rpy_from_rotation flags gimbal lock with a margin below 1e-9");

        const Eigen::Matrix3d healthy =
            study::spatial::rotation_from_rpy(Eigen::Vector3d(0.3, 0.4, 0.5));
        request.set("args", Value::object({{"R", json::from_matrix(healthy)}}));
        const Value healthy_response = registry.handle(request);
        check(!healthy_response["result"]["gimbal_lock"].as_bool() &&
                  healthy_response["result"]["gimbal_margin"].as_double() > 0.9,
              "rpy_from_rotation reports a healthy margin away from gimbal lock");
        const Eigen::Vector3d recovered = read_vec3(healthy_response["result"]["rpy"]);
        check_near((recovered - Eigen::Vector3d(0.3, 0.4, 0.5)).norm(), 0.0, 1e-12,
                   "rpy_from_rotation over JSON recovers the angles");
    }

    // A non-orthonormal input is reported, never silently fixed.
    {
        Eigen::Matrix3d skewed = Eigen::Matrix3d::Identity();
        skewed(0, 1) = 0.01;
        Value request = Value::object();
        request.set("module", Value("spatial_math"));
        request.set("op", Value("rpy_from_rotation"));
        request.set("args", Value::object({{"R", json::from_matrix(skewed)}}));
        const Value response = registry.handle(request);
        check(response["ok"].as_bool() &&
                  response["result"]["orthonormality_error"].as_double() > 1e-3,
              "a drifting matrix is reported through orthonormality_error");
    }

    // ZYX Euler angles are fixed-axis RPY with the order reversed.
    {
        const Eigen::Vector3d angles(0.5, -0.3, 0.9);
        Value request = Value::object();
        request.set("module", Value("spatial_math"));
        request.set("op", Value("rotation_from_euler"));
        request.set("args", Value::object({{"angles", json::from_vec3(angles)},
                                           {"convention", Value("zyx")}}));
        const Value response = registry.handle(request);
        const Eigen::Matrix3d R = read_mat3(response["result"]["R"]);
        const Eigen::Matrix3d expected =
            study::spatial::rotation_from_rpy(Eigen::Vector3d(angles.z(), angles.y(), angles.x()));
        check_near((R - expected).norm(), 0.0, 1e-14,
                   "rotation_from_euler zyx equals fixed-axis rpy reversed");
        check(response["result"]["convention"].as_string() == "zyx" &&
                  !response["result"]["note"].as_string().empty(),
              "rotation_from_euler reports the convention it applied");

        request.set("args", Value::object({{"angles", json::from_vec3(angles)},
                                           {"convention", Value("zyz")}}));
        const Value zyz = registry.handle(request);
        const Eigen::Matrix3d R_zyz = read_mat3(zyz["result"]["R"]);
        check_near((R_zyz - study::spatial::rotation_from_euler_zyz(angles)).norm(), 0.0, 1e-14,
                   "rotation_from_euler zyz matches Rz Ry Rz");
        check((R_zyz - R).norm() > 1e-3, "the two conventions are genuinely different rotations");

        request.set("args", Value::object({{"angles", json::from_vec3(angles)},
                                           {"convention", Value("xyz")}}));
        check(is_failure(registry.handle(request), "must be one of"),
              "rotation_from_euler rejects an unknown convention");
    }

    // Composing a transform with its own inverse is the identity.
    {
        const Eigen::Vector3d rpy(0.3, -0.2, 0.5);
        const Eigen::Vector3d p(0.1, -0.2, 0.3);
        const Eigen::Matrix3d R = study::spatial::rotation_from_rpy(rpy);
        const Eigen::Vector3d p_inverse = -R.transpose() * p;
        const study::spatial::RpyBranches inverse_rpy =
            study::spatial::rpy_from_rotation(R.transpose());

        Value chain = Value::array();
        chain.push_back(Value::object({{"rpy", json::from_vec3(rpy)}, {"p", json::from_vec3(p)}}));
        chain.push_back(Value::object({{"rpy", json::from_vec3(inverse_rpy.primary)},
                                       {"p", json::from_vec3(p_inverse)}}));

        Value request = Value::object();
        request.set("module", Value("spatial_math"));
        request.set("op", Value("compose"));
        request.set("args", Value::object({{"transforms", chain}}));
        const Value response = registry.handle(request);
        check(response["ok"].as_bool(), "compose accepts a list of {rpy, p}");
        const Eigen::Matrix4d T = read_mat4(response["result"]["T"]);
        check_near((T - Eigen::Matrix4d::Identity()).norm(), 0.0, 1e-12,
                   "compose of a transform with its inverse is the identity to 1e-12");
        check(response["result"]["partials"].size() == 2 &&
                  response["result"]["count"].as_int() == 2,
              "compose returns one partial product per transform");
        const Eigen::Matrix4d first_partial = read_mat4(response["result"]["partials"][0]);
        check_near((first_partial - study::spatial::transform_from_rpy_p(rpy, p).matrix()).norm(),
                   0.0, 1e-15, "the first partial product is the first transform");
        check(response["result"]["inverse_residual"].as_double() < 1e-12,
              "compose reports || T T^-1 - I || below 1e-12");

        // The table row shape must compose to the same answer.
        Value table_request = Value::object();
        table_request.set("module", Value("spatial_math"));
        table_request.set("op", Value("compose"));
        table_request.set(
            "args",
            Value::object({{"transforms",
                            json::from_table({"roll", "pitch", "yaw", "x", "y", "z"},
                                             std::vector<std::vector<double>>{
                                                 {rpy.x(), rpy.y(), rpy.z(), p.x(), p.y(), p.z()}})}}));
        const Value table_response = registry.handle(table_request);
        check(table_response["ok"].as_bool() &&
                  (read_mat4(table_response["result"]["T"]) -
                   study::spatial::transform_from_rpy_p(rpy, p).matrix())
                          .norm() < 1e-15,
              "compose accepts the table row shape as well");
        check(is_failure(handle_text(registry,
                                     "{\"module\":\"spatial_math\",\"op\":\"compose\",\"args\":{\"t"
                                     "ransforms\":[]}}"),
                         "at least 1"),
              "compose rejects an empty chain");
    }

    // transform_point: the arithmetic in the table must add up to the answer.
    {
        const Eigen::Vector3d rpy(0.2, 0.4, -0.6);
        const Eigen::Vector3d p(0.04, 0.0, 0.33);
        const Eigen::Vector3d point(0.12, -0.05, 0.2);
        Value request = Value::object();
        request.set("module", Value("spatial_math"));
        request.set("op", Value("transform_point"));
        request.set("args", Value::object({{"rpy", json::from_vec3(rpy)},
                                           {"p", json::from_vec3(p)},
                                           {"point", json::from_vec3(point)}}));
        const Value response = registry.handle(request);
        const Eigen::Vector3d expected = study::spatial::rotation_from_rpy(rpy) * point + p;
        const Eigen::Vector3d actual = read_vec3(response["result"]["p_out"]);
        check_near((actual - expected).norm(), 0.0 ,1e-15, "transform_point applies R p + t");

        const Value& steps = response["result"]["steps"];
        check(steps["columns"].size() == 6 && steps["rows"].size() == 4,
              "transform_point shows four homogeneous rows of six columns");
        double worst_row = 0.0;
        for (std::size_t r = 0; r < 3; ++r) {
            const Value& row = steps["rows"][r];
            const double sum = row[1].as_double() + row[2].as_double() + row[3].as_double() +
                               row[4].as_double();
            worst_row = std::max(worst_row, std::abs(sum - row[5].as_double()));
        }
        check(worst_row < 1e-15, "every step row sums to its own result");
        check(steps["rows"][3][4].as_double() == 1.0 && steps["rows"][3][5].as_double() == 1.0,
              "the homogeneous row keeps the trailing 1");
    }

    // frame_of_joint against the engine's forward kinematics.
    {
        const yaskawa::YaskawaKinematics solver;
        std::vector<Eigen::Matrix<double, 6, 1>> configurations;
        configurations.push_back(Eigen::Matrix<double, 6, 1>::Zero());
        Eigen::Matrix<double, 6, 1> q;
        q << 0.2, -0.3, 0.4, -0.1, 0.2, -0.5;
        configurations.push_back(q);
        q << -1.1, 0.9, -0.7, 2.0, -1.3, 3.0;
        configurations.push_back(q);
        q << 2.9, 2.5, 3.3, -3.3, 2.3, -6.2;
        configurations.push_back(q);

        double worst_dh = 0.0;
        double worst_module = 0.0;
        double worst_correction = 0.0;
        for (const auto& configuration : configurations) {
            const Eigen::Isometry3d reference = solver.forwardKinematics(configuration);
            worst_dh = std::max(
                worst_dh,
                (study::forward_kinematics_dh(configuration).matrix() - reference.matrix()).norm());
            // The frame-6 orientation differs from the flange by exactly the
            // constant correction, and by nothing else.
            const Eigen::Matrix3d relabelling =
                study::dh_chain(configuration).linear().transpose() * reference.linear();
            worst_correction = std::max(
                worst_correction, (relabelling - study::flange_correction().linear()).norm());

            Value request = Value::object();
            request.set("module", Value("spatial_math"));
            request.set("op", Value("frame_of_joint"));
            request.set("args", Value::object({{"q", json::from_vec6(configuration)}}));
            const Value response = registry.handle(request);
            if (!response["ok"].is_bool() || !response["ok"].as_bool()) {
                std::cout << "       frame_of_joint failed: " << error_text(response)
                          << "\n";
                worst_module = 1.0;
                continue;
            }
            worst_module = std::max(
                worst_module,
                (read_mat4(response["result"]["T_flange"]) - reference.matrix()).norm());
        }

        check(worst_dh < 1e-12,
              "the gp8_model DH chain reproduces YaskawaKinematics::forwardKinematics (worst " +
                  json::number_to_string(worst_dh) + ")");
        check(worst_correction < 1e-12,
              "the DH frame 6 differs from the flange by exactly GP8_FLANGE_CORRECTION (worst " +
                  json::number_to_string(worst_correction) + ")");
        check(worst_module < 1e-9,
              "frame_of_joint reproduces the engine forward kinematics to 1e-9 (worst " +
                  json::number_to_string(worst_module) + ")");

        // Shape of the answer the 3D view consumes.
        Value request = Value::object();
        request.set("module", Value("spatial_math"));
        request.set("op", Value("frame_of_joint"));
        request.set("args",
                    Value::object({{"q", json::from_vec6(Eigen::Matrix<double, 6, 1>::Zero())}}));
        const Value response = registry.handle(request);
        const Value& result = response["result"];
        check(result["frames"].size() == study::GP8_DOF, "frame_of_joint returns six link frames");
        check(result["origins"].size() == study::GP8_DOF + 2,
              "frame_of_joint returns the base, six frames and the flange as points");
        check(result["dh"]["rows"].size() == study::GP8_DOF, "frame_of_joint returns the DH table");
        check(result["within_limits"].as_bool(), "the zero configuration is inside every limit");
        const Eigen::Vector3d home = read_vec3(result["flange_position"]);
        check_near(home.x(), 0.38, 1e-12, "home flange x is 0.380 m");
        check_near(home.y(), 0.0, 1e-12, "home flange y is 0 m");
        check_near(home.z(), 0.715, 1e-12, "home flange z is 0.715 m");
        const Eigen::Vector3d wrist = read_vec3(result["origins"][4]);
        check_near((wrist - read_vec3(result["origins"][6])).norm(), 0.0, 1e-12,
                   "the wrist axes 4, 5 and 6 share one origin (spherical wrist)");
        check(result["orthonormality_error"].as_double() < 1e-12,
              "the flange rotation stays orthonormal through the DH chain");
    }

    // The model's own invariants.
    {
        check(study::GP8_PAYLOAD_KG == 8.0, "gp8_model rates the payload at 8 kg");
        check(study::gravity_vector().z() < -9.8 && study::gravity_vector().head<2>().norm() == 0.0,
              "gravity points down the base z axis");
        bool drive_train_is_plausible = true;
        for (std::size_t i = 0; i < study::GP8_DOF; ++i) {
            const auto& link = study::GP8_LINKS[i];
            drive_train_is_plausible =
                drive_train_is_plausible && link.mass > 0.0 && link.gear_ratio > 1.0 &&
                link.rotor_inertia > 0.0 && link.viscous_friction > 0.0 &&
                link.coulomb_friction > 0.0 && link.max_torque > 0.0 &&
                study::link_inertia(i).determinant() > 0.0 &&
                study::reflected_rotor_inertia(i) > 0.0 &&
                study::joint_max_velocity(i) == yaskawa::GP8_JOINT_LIMITS[i].max_vel;
        }
        check(drive_train_is_plausible,
              "every link carries a positive mass, inertia, gear ratio and friction, and reuses "
              "the engine's joint speeds");
    }
}

}  // namespace

int main() {
    std::cout << "====================================================\n";
    std::cout << "      RUNNING YASKAWA STUDY MODULE TEST SUITE       \n";
    std::cout << "====================================================\n";

    test_json();
    test_registry();
    test_spatial_math();

    std::cout << "====================================================\n";
    std::cout << "checks run: " << g_checks << ", failed: " << g_failures << "\n";
    if (g_failures != 0) {
        for (const auto& name : g_failed_names) {
            std::cout << "FAILED: " << name << "\n";
        }
        std::cout << "STUDY TESTS FAILED\n";
        return 1;
    }
    std::cout << "          ALL TESTS PASSED                          \n";
    std::cout << "====================================================\n";
    return 0;
}
