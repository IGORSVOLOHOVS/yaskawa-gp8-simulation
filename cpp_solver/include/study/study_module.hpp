#ifndef YASKAWA_STUDY_STUDY_MODULE_HPP
#define YASKAWA_STUDY_STUDY_MODULE_HPP

// The study-module interface of docs/STUDY_MODULE_CONTRACT.md.
//
// A module does three things: it names itself, it describes itself well enough
// for the browser to generate its whole panel, and it answers invoke(). Bad
// input is reported by throwing StudyError from the require_* helpers; the
// registry turns that into an `ok: false` line and the process stays alive.
//
// Type vocabulary for ParamSpec::type and OutputSpec::type (contract section 3)
//   scalar, bool, int, enum, vec3, vec6, mat3, mat4, matrix,
//   series, series_set, points, complex_set, table, text
// plus mat4_set ([mat4, ...]) for a list of frames, which the 3D view needs.

#include "study/json.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace yaskawa::study {

// The only exception type a module throws for bad input.
class StudyError final : public std::runtime_error {
public:
    explicit StudyError(const std::string& message) : std::runtime_error(message) {}
};

// ---------------------------------------------------------------------------
// Self-description (contract section 2)
// ---------------------------------------------------------------------------

struct CourseRef {
    int id = 0;             // Moodle course id, so the UI can link back
    std::string code;       // e.g. "M-407-01"
    std::string name;       // e.g. "Robotics Modelling"

    [[nodiscard]] json::Value to_json() const {
        return json::Value::object({
            {"id", json::Value(id)},
            {"code", json::Value(code)},
            {"name", json::Value(name)},
        });
    }
};

struct ParamSpec {
    std::string name;
    std::string type;                   // type vocabulary
    std::string unit;                   // SI unit, empty for bool/enum
    std::string label;                  // human label for the control
    double min = 0.0;
    double max = 0.0;
    bool has_range = false;             // false for bool and enum
    json::Value default_value;          // shape follows `type`
    std::vector<std::string> options;   // enum only, non-empty for enum

    [[nodiscard]] static ParamSpec scalar(std::string name, std::string label, std::string unit,
                                          double min, double max, double default_value) {
        ParamSpec p;
        p.name = std::move(name);
        p.type = "scalar";
        p.unit = std::move(unit);
        p.label = std::move(label);
        p.min = min;
        p.max = max;
        p.has_range = true;
        p.default_value = json::Value(default_value);
        return p;
    }

    [[nodiscard]] static ParamSpec integer(std::string name, std::string label, std::string unit,
                                           int min, int max, int default_value) {
        ParamSpec p;
        p.name = std::move(name);
        p.type = "int";
        p.unit = std::move(unit);
        p.label = std::move(label);
        p.min = static_cast<double>(min);
        p.max = static_cast<double>(max);
        p.has_range = true;
        p.default_value = json::Value(default_value);
        return p;
    }

    [[nodiscard]] static ParamSpec boolean(std::string name, std::string label, bool default_value) {
        ParamSpec p;
        p.name = std::move(name);
        p.type = "bool";
        p.label = std::move(label);
        p.default_value = json::Value(default_value);
        return p;
    }

    [[nodiscard]] static ParamSpec enumeration(std::string name, std::string label,
                                               std::vector<std::string> options,
                                               std::string default_value) {
        ParamSpec p;
        p.name = std::move(name);
        p.type = "enum";
        p.label = std::move(label);
        p.options = std::move(options);
        p.default_value = json::Value(std::move(default_value));
        return p;
    }

    [[nodiscard]] static ParamSpec vec3(std::string name, std::string label, std::string unit,
                                        double min, double max, const Eigen::Vector3d& default_value) {
        ParamSpec p;
        p.name = std::move(name);
        p.type = "vec3";
        p.unit = std::move(unit);
        p.label = std::move(label);
        p.min = min;
        p.max = max;
        p.has_range = true;
        p.default_value = json::from_vec3(default_value);
        return p;
    }

    [[nodiscard]] static ParamSpec vec6(std::string name, std::string label, std::string unit,
                                        double min, double max,
                                        const Eigen::Matrix<double, 6, 1>& default_value) {
        ParamSpec p;
        p.name = std::move(name);
        p.type = "vec6";
        p.unit = std::move(unit);
        p.label = std::move(label);
        p.min = min;
        p.max = max;
        p.has_range = true;
        p.default_value = json::from_vec6(default_value);
        return p;
    }

    // Any other vocabulary type (mat3, matrix, table, ...) with a given default.
    [[nodiscard]] static ParamSpec structured(std::string name, std::string label, std::string type,
                                              std::string unit, json::Value default_value) {
        ParamSpec p;
        p.name = std::move(name);
        p.type = std::move(type);
        p.unit = std::move(unit);
        p.label = std::move(label);
        p.default_value = std::move(default_value);
        return p;
    }

    [[nodiscard]] json::Value to_json() const {
        json::Value out = json::Value::object();
        out.set("name", json::Value(name));
        out.set("type", json::Value(type));
        if (!unit.empty()) {
            out.set("unit", json::Value(unit));
        }
        if (has_range) {
            out.set("min", json::Value(min));
            out.set("max", json::Value(max));
        }
        out.set("default", default_value);
        out.set("label", json::Value(label));
        if (!options.empty()) {
            out.set("options", json::from_strings(options));
        }
        return out;
    }
};

struct OutputSpec {
    std::string name;
    std::string type;    // type vocabulary
    std::string label;
    std::string unit;    // optional

    [[nodiscard]] static OutputSpec make(std::string name, std::string type, std::string label,
                                         std::string unit = {}) {
        OutputSpec o;
        o.name = std::move(name);
        o.type = std::move(type);
        o.label = std::move(label);
        o.unit = std::move(unit);
        return o;
    }

    [[nodiscard]] json::Value to_json() const {
        json::Value out = json::Value::object();
        out.set("name", json::Value(name));
        out.set("type", json::Value(type));
        out.set("label", json::Value(label));
        if (!unit.empty()) {
            out.set("unit", json::Value(unit));
        }
        return out;
    }
};

struct OpSpec {
    std::string name;
    std::string title;
    std::string formula;     // LaTeX without surrounding dollar signs
    std::string explain;     // written for a person revising the night before
    std::vector<ParamSpec> params;
    std::vector<OutputSpec> outputs;

    [[nodiscard]] json::Value to_json() const {
        json::Value param_list = json::Value::array();
        param_list.reserve(params.size());
        for (const auto& p : params) {
            param_list.push_back(p.to_json());
        }
        json::Value output_list = json::Value::array();
        output_list.reserve(outputs.size());
        for (const auto& o : outputs) {
            output_list.push_back(o.to_json());
        }
        json::Value out = json::Value::object();
        out.set("name", json::Value(name));
        out.set("title", json::Value(title));
        out.set("formula", json::Value(formula));
        out.set("explain", json::Value(explain));
        out.set("params", std::move(param_list));
        out.set("outputs", std::move(output_list));
        return out;
    }
};

struct ModuleDescription {
    std::string name;
    std::string title;
    CourseRef course;
    std::vector<std::string> topics;
    std::string source;     // repository-relative path to the header
    std::string summary;
    std::vector<OpSpec> ops;

    [[nodiscard]] json::Value to_json() const {
        json::Value op_list = json::Value::array();
        op_list.reserve(ops.size());
        for (const auto& op : ops) {
            op_list.push_back(op.to_json());
        }
        json::Value out = json::Value::object();
        out.set("name", json::Value(name));
        out.set("title", json::Value(title));
        out.set("course", course.to_json());
        out.set("topics", json::from_strings(topics));
        out.set("source", json::Value(source));
        out.set("summary", json::Value(summary));
        out.set("ops", std::move(op_list));
        return out;
    }

    [[nodiscard]] bool has_op(std::string_view op_name) const noexcept {
        for (const auto& op : ops) {
            if (op.name == op_name) {
                return true;
            }
        }
        return false;
    }
};

// ---------------------------------------------------------------------------
// The module interface (contract section 4)
// ---------------------------------------------------------------------------

class StudyModule {
public:
    StudyModule() = default;
    StudyModule(const StudyModule&) = delete;
    StudyModule& operator=(const StudyModule&) = delete;
    virtual ~StudyModule() = default;

    [[nodiscard]] virtual std::string_view name() const noexcept = 0;
    [[nodiscard]] virtual ModuleDescription describe() const = 0;
    [[nodiscard]] virtual json::Value invoke(std::string_view op, const json::Value& args) const = 0;
};

// ---------------------------------------------------------------------------
// Validation helpers. Every message names the offending parameter; every range
// message states the allowed range and the value that was received.
// ---------------------------------------------------------------------------

namespace detail {

[[nodiscard]] inline std::string quote(std::string_view key) {
    return "'" + std::string(key) + "'";
}

[[nodiscard]] inline std::string num(double value) { return json::number_to_string(value); }

[[nodiscard]] inline std::string options_list(const std::vector<std::string>& options) {
    std::string out;
    for (std::size_t i = 0; i < options.size(); ++i) {
        if (i != 0) {
            out += ", ";
        }
        out += options[i];
    }
    return out;
}

[[nodiscard]] inline double checked_range(std::string_view key, double value, double min, double max,
                                          std::string_view element = {}) {
    if (!std::isfinite(value)) {
        throw StudyError("parameter " + quote(key) + " must be a finite number");
    }
    if (value < min || value > max) {
        std::string where = quote(key);
        if (!element.empty()) {
            where += " element " + std::string(element);
        }
        throw StudyError("parameter " + where + " = " + num(value) + " is out of range [" + num(min) +
                         ", " + num(max) + "]");
    }
    return value;
}

}  // namespace detail

constexpr double kNoLimit = std::numeric_limits<double>::max();

// The whole args payload (or a nested object) must be a JSON object.
[[nodiscard]] inline const json::Value& require_object(const json::Value& value,
                                                       std::string_view what = "args") {
    if (!value.is_object()) {
        throw StudyError(std::string(what) + " must be a JSON object, found " + value.type_name());
    }
    return value;
}

[[nodiscard]] inline const json::Value& require_present(const json::Value& args,
                                                        std::string_view key) {
    (void)require_object(args);
    if (!args.contains(key)) {
        throw StudyError("missing required parameter " + detail::quote(key));
    }
    const json::Value& value = args[key];
    if (value.is_null()) {
        throw StudyError("parameter " + detail::quote(key) + " must not be null");
    }
    return value;
}

[[nodiscard]] inline const json::Value& require_array(const json::Value& args, std::string_view key,
                                                      std::size_t min_size = 0) {
    const json::Value& value = require_present(args, key);
    if (!value.is_array()) {
        throw StudyError("parameter " + detail::quote(key) + " must be an array, found " +
                         value.type_name());
    }
    if (value.size() < min_size) {
        throw StudyError("parameter " + detail::quote(key) + " needs at least " +
                         std::to_string(min_size) + " elements, received " +
                         std::to_string(value.size()));
    }
    return value;
}

[[nodiscard]] inline double require_scalar(const json::Value& args, std::string_view key,
                                           double min = -kNoLimit, double max = kNoLimit) {
    const json::Value& value = require_present(args, key);
    if (!value.is_number()) {
        throw StudyError("parameter " + detail::quote(key) + " must be a number, found " +
                         value.type_name());
    }
    return detail::checked_range(key, value.as_double(), min, max);
}

[[nodiscard]] inline int require_int(const json::Value& args, std::string_view key,
                                     int min = -1000000000, int max = 1000000000) {
    const json::Value& value = require_present(args, key);
    if (!value.is_number()) {
        throw StudyError("parameter " + detail::quote(key) + " must be an integer, found " +
                         value.type_name());
    }
    const double raw = value.as_double();
    if (!std::isfinite(raw)) {
        throw StudyError("parameter " + detail::quote(key) + " must be a finite integer");
    }
    if (std::abs(raw - std::round(raw)) > 1e-9) {
        throw StudyError("parameter " + detail::quote(key) + " = " + detail::num(raw) +
                         " must be an integer");
    }
    const double rounded = std::round(raw);
    (void)detail::checked_range(key, rounded, static_cast<double>(min), static_cast<double>(max));
    return static_cast<int>(rounded);
}

[[nodiscard]] inline bool require_bool(const json::Value& args, std::string_view key) {
    const json::Value& value = require_present(args, key);
    if (!value.is_bool()) {
        throw StudyError("parameter " + detail::quote(key) + " must be true or false, found " +
                         value.type_name());
    }
    return value.as_bool();
}

[[nodiscard]] inline std::string require_enum(const json::Value& args, std::string_view key,
                                              const std::vector<std::string>& options) {
    const json::Value& value = require_present(args, key);
    if (!value.is_string()) {
        throw StudyError("parameter " + detail::quote(key) + " must be one of [" +
                         detail::options_list(options) + "], found " + value.type_name());
    }
    const std::string& selected = value.as_string();
    for (const auto& option : options) {
        if (option == selected) {
            return selected;
        }
    }
    throw StudyError("parameter " + detail::quote(key) + " must be one of [" +
                     detail::options_list(options) + "], received '" + selected + "'");
}

[[nodiscard]] inline std::string require_enum(const json::Value& args, std::string_view key,
                                              std::initializer_list<std::string_view> options) {
    std::vector<std::string> as_strings;
    as_strings.reserve(options.size());
    for (const auto& option : options) {
        as_strings.emplace_back(option);
    }
    return require_enum(args, key, as_strings);
}

[[nodiscard]] inline std::string require_string(const json::Value& args, std::string_view key) {
    const json::Value& value = require_present(args, key);
    if (!value.is_string()) {
        throw StudyError("parameter " + detail::quote(key) + " must be a string, found " +
                         value.type_name());
    }
    return value.as_string();
}

[[nodiscard]] inline Eigen::VectorXd require_vec(const json::Value& args, std::string_view key,
                                                 std::size_t n, double min = -kNoLimit,
                                                 double max = kNoLimit) {
    const json::Value& value = require_present(args, key);
    if (!value.is_array()) {
        throw StudyError("parameter " + detail::quote(key) + " must be an array of " +
                         std::to_string(n) + " numbers, found " + value.type_name());
    }
    if (value.size() != n) {
        throw StudyError("parameter " + detail::quote(key) + " must have " + std::to_string(n) +
                         " elements, received " + std::to_string(value.size()));
    }
    Eigen::VectorXd out(static_cast<Eigen::Index>(n));
    for (std::size_t i = 0; i < n; ++i) {
        const json::Value& element = value[i];
        if (!element.is_number()) {
            throw StudyError("parameter " + detail::quote(key) + " element " + std::to_string(i) +
                             " must be a number, found " + element.type_name());
        }
        out(static_cast<Eigen::Index>(i)) =
            detail::checked_range(key, element.as_double(), min, max, std::to_string(i));
    }
    return out;
}

[[nodiscard]] inline Eigen::Vector3d require_vec3(const json::Value& args, std::string_view key,
                                                  double min = -kNoLimit, double max = kNoLimit) {
    const Eigen::VectorXd v = require_vec(args, key, 3, min, max);
    return Eigen::Vector3d(v(0), v(1), v(2));
}

[[nodiscard]] inline Eigen::Matrix<double, 6, 1> require_vec6(const json::Value& args,
                                                              std::string_view key,
                                                              double min = -kNoLimit,
                                                              double max = kNoLimit) {
    const Eigen::VectorXd v = require_vec(args, key, 6, min, max);
    Eigen::Matrix<double, 6, 1> out;
    for (Eigen::Index i = 0; i < 6; ++i) {
        out(i) = v(i);
    }
    return out;
}

// Row-major nested arrays, e.g. a mat3 param coming back from the UI.
[[nodiscard]] inline Eigen::MatrixXd require_matrix(const json::Value& args, std::string_view key,
                                                    std::size_t rows, std::size_t cols) {
    const json::Value& value = require_present(args, key);
    if (!value.is_array() || value.size() != rows) {
        throw StudyError("parameter " + detail::quote(key) + " must be a row-major array of " +
                         std::to_string(rows) + " rows, received " + std::to_string(value.size()));
    }
    Eigen::MatrixXd out(static_cast<Eigen::Index>(rows), static_cast<Eigen::Index>(cols));
    for (std::size_t r = 0; r < rows; ++r) {
        const json::Value& row = value[r];
        if (!row.is_array() || row.size() != cols) {
            throw StudyError("parameter " + detail::quote(key) + " row " + std::to_string(r) +
                             " must hold " + std::to_string(cols) + " numbers, received " +
                             std::to_string(row.size()));
        }
        for (std::size_t c = 0; c < cols; ++c) {
            const json::Value& cell = row[c];
            if (!cell.is_number() || !std::isfinite(cell.as_double())) {
                throw StudyError("parameter " + detail::quote(key) + " element [" +
                                 std::to_string(r) + "][" + std::to_string(c) +
                                 "] must be a finite number");
            }
            out(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = cell.as_double();
        }
    }
    return out;
}

// ---- optional forms: absent or null falls back, present is still validated --

[[nodiscard]] inline bool is_absent(const json::Value& args, std::string_view key) {
    return !args.is_object() || !args.contains(key) || args[key].is_null();
}

[[nodiscard]] inline double optional_scalar(const json::Value& args, std::string_view key,
                                            double fallback, double min = -kNoLimit,
                                            double max = kNoLimit) {
    return is_absent(args, key) ? fallback : require_scalar(args, key, min, max);
}

[[nodiscard]] inline int optional_int(const json::Value& args, std::string_view key, int fallback,
                                      int min = -1000000000, int max = 1000000000) {
    return is_absent(args, key) ? fallback : require_int(args, key, min, max);
}

[[nodiscard]] inline bool optional_bool(const json::Value& args, std::string_view key,
                                        bool fallback) {
    return is_absent(args, key) ? fallback : require_bool(args, key);
}

[[nodiscard]] inline std::string optional_enum(const json::Value& args, std::string_view key,
                                               std::string fallback,
                                               const std::vector<std::string>& options) {
    return is_absent(args, key) ? std::move(fallback) : require_enum(args, key, options);
}

[[nodiscard]] inline std::string optional_string(const json::Value& args, std::string_view key,
                                                 std::string fallback) {
    return is_absent(args, key) ? std::move(fallback) : require_string(args, key);
}

[[nodiscard]] inline Eigen::VectorXd optional_vec(const json::Value& args, std::string_view key,
                                                  const Eigen::VectorXd& fallback,
                                                  double min = -kNoLimit, double max = kNoLimit) {
    if (is_absent(args, key)) {
        return fallback;
    }
    return require_vec(args, key, static_cast<std::size_t>(fallback.size()), min, max);
}

[[nodiscard]] inline Eigen::Vector3d optional_vec3(const json::Value& args, std::string_view key,
                                                   const Eigen::Vector3d& fallback,
                                                   double min = -kNoLimit, double max = kNoLimit) {
    return is_absent(args, key) ? fallback : require_vec3(args, key, min, max);
}

[[nodiscard]] inline Eigen::Matrix<double, 6, 1> optional_vec6(
    const json::Value& args, std::string_view key, const Eigen::Matrix<double, 6, 1>& fallback,
    double min = -kNoLimit, double max = kNoLimit) {
    return is_absent(args, key) ? fallback : require_vec6(args, key, min, max);
}

[[nodiscard]] inline Eigen::MatrixXd optional_matrix(const json::Value& args, std::string_view key,
                                                     const Eigen::MatrixXd& fallback) {
    if (is_absent(args, key)) {
        return fallback;
    }
    return require_matrix(args, key, static_cast<std::size_t>(fallback.rows()),
                          static_cast<std::size_t>(fallback.cols()));
}

// Rejects an op name a module does not implement, with the same wording the
// registry uses for an unknown module.
[[noreturn]] inline void unknown_op(std::string_view module_name, std::string_view op) {
    throw StudyError("module '" + std::string(module_name) + "' has no op '" + std::string(op) + "'");
}

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_STUDY_MODULE_HPP
