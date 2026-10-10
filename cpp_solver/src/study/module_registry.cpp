#include "study/module_registry.hpp"

#include "study/spatial_math.hpp"
#include "study/dh_kinematics.hpp"
#include "study/jacobian_statics.hpp"
#include "study/dynamics.hpp"
#include "study/trajectory_profiles.hpp"
#include "study/control_system.hpp"
#include "study/digital_control.hpp"
#include "study/dsp_system.hpp"
#include "study/sensing_models.hpp"
#include "study/vision_camera.hpp"
#include "study/learning_models.hpp"
#include "study/robot_specification.hpp"

#include <chrono>
#include <exception>
#include <string>
#include <utility>

namespace yaskawa::study {

namespace {

using Clock = std::chrono::steady_clock;

[[nodiscard]] const json::Value& empty_args() {
    static const json::Value instance = json::Value::object();
    return instance;
}

[[nodiscard]] long long elapsed_us(Clock::time_point start, Clock::time_point stop) noexcept {
    const auto us = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();
    return (us < 0) ? 0 : static_cast<long long>(us);
}

// Field order follows the contract's example response, so a diff of two runs
// is a diff of values and never of key order.
[[nodiscard]] json::Value success_response(long long id, const std::string& module_name,
                                           const std::string& op_name, long long us,
                                           json::Value result) {
    json::Value out = json::Value::object();
    out.set("id", json::Value(id));
    out.set("ok", json::Value(true));
    out.set("module", json::Value(module_name));
    out.set("op", json::Value(op_name));
    out.set("us", json::Value(us));
    out.set("result", std::move(result));
    return out;
}

[[nodiscard]] json::Value failure_response(long long id, const std::string& module_name,
                                           const std::string& op_name, const std::string& error) {
    json::Value out = json::Value::object();
    out.set("id", json::Value(id));
    out.set("ok", json::Value(false));
    out.set("module", json::Value(module_name));
    out.set("op", json::Value(op_name));
    out.set("error", json::Value(error));
    return out;
}

}  // namespace

void ModuleRegistry::add(std::unique_ptr<StudyModule> module) {
    if (!module) {
        return;
    }
    modules_.push_back(std::move(module));
}

const StudyModule* ModuleRegistry::find(std::string_view name) const {
    for (const auto& module : modules_) {
        if (module->name() == name) {
            return module.get();
        }
    }
    return nullptr;
}

json::Value ModuleRegistry::describe_all() const {
    struct CourseEntry {
        CourseRef ref;
        std::vector<std::string> module_names;
    };

    json::Value module_list = json::Value::array();
    module_list.reserve(modules_.size());

    std::vector<CourseEntry> courses;
    courses.reserve(modules_.size());

    for (const auto& module : modules_) {
        const ModuleDescription description = module->describe();
        module_list.push_back(description.to_json());

        // Deduplicate by Moodle course id, keeping first-seen order.
        CourseEntry* entry = nullptr;
        for (auto& candidate : courses) {
            if (candidate.ref.id == description.course.id) {
                entry = &candidate;
                break;
            }
        }
        if (entry == nullptr) {
            courses.push_back(CourseEntry{description.course, {}});
            entry = &courses.back();
        }
        entry->module_names.push_back(description.name);
    }

    json::Value course_list = json::Value::array();
    course_list.reserve(courses.size());
    for (const auto& course : courses) {
        json::Value entry = course.ref.to_json();
        entry.set("modules", json::from_strings(course.module_names));
        course_list.push_back(std::move(entry));
    }

    json::Value out = json::Value::object();
    out.set("modules", std::move(module_list));
    out.set("courses", std::move(course_list));
    return out;
}

json::Value ModuleRegistry::handle(const json::Value& request) const {
    long long id = 0;
    std::string module_name;
    std::string op_name;

    try {
        if (!request.is_object()) {
            throw StudyError(std::string("request must be a JSON object, found ") +
                             request.type_name());
        }

        if (request.contains("id") && !request["id"].is_null()) {
            if (!request["id"].is_number()) {
                throw StudyError("request field 'id' must be an integer");
            }
            id = request["id"].as_int();
        }

        if (request.contains("module") && !request["module"].is_null()) {
            if (!request["module"].is_string()) {
                throw StudyError("request field 'module' must be a string");
            }
            module_name = request["module"].as_string();
        }

        if (!request.contains("op") || request["op"].is_null()) {
            throw StudyError("request field 'op' is required");
        }
        if (!request["op"].is_string()) {
            throw StudyError("request field 'op' must be a string");
        }
        op_name = request["op"].as_string();

        // Reserved op 1: the whole catalogue.
        if (op_name == "describe" && module_name.empty()) {
            const auto start = Clock::now();
            json::Value result = describe_all();
            const auto stop = Clock::now();
            return success_response(id, module_name, op_name, elapsed_us(start, stop),
                                    std::move(result));
        }

        if (module_name.empty()) {
            throw StudyError("request field 'module' is required for op '" + op_name + "'");
        }

        const StudyModule* module = find(module_name);
        if (module == nullptr) {
            throw StudyError("unknown module '" + module_name + "'");
        }

        // Reserved op 2: one module's description.
        if (op_name == "describe") {
            const auto start = Clock::now();
            json::Value result = module->describe().to_json();
            const auto stop = Clock::now();
            return success_response(id, module_name, op_name, elapsed_us(start, stop),
                                    std::move(result));
        }

        const json::Value& args =
            (request.contains("args") && !request["args"].is_null()) ? request["args"]
                                                                     : empty_args();

        // The contract measures the op itself: nothing above this line counts.
        const auto start = Clock::now();
        json::Value result = module->invoke(op_name, args);
        const auto stop = Clock::now();
        return success_response(id, module_name, op_name, elapsed_us(start, stop),
                                std::move(result));
    } catch (const StudyError& error) {
        return failure_response(id, module_name, op_name, error.what());
    } catch (const json::ParseError& error) {
        return failure_response(id, module_name, op_name, error.what());
    } catch (const json::TypeError& error) {
        return failure_response(id, module_name, op_name, error.what());
    } catch (const std::exception& error) {
        return failure_response(id, module_name, op_name,
                                std::string("internal error: ") + error.what());
    } catch (...) {
        return failure_response(id, module_name, op_name, "internal error: unknown exception");
    }
}

ModuleRegistry build_default_registry() {
    ModuleRegistry registry;
    // Robotics Modelling (3883)
    registry.add(std::make_unique<SpatialMathModule>());
    registry.add(std::make_unique<DhKinematicsModule>());
    registry.add(std::make_unique<JacobianStaticsModule>());
    registry.add(std::make_unique<DynamicsModule>());
    registry.add(std::make_unique<TrajectoryProfilesModule>());
    // Robot Control and Feedback Systems (3884)
    registry.add(std::make_unique<ControlSystemModule>());
    registry.add(std::make_unique<DigitalControlModule>());
    // Digital Signal Processing (3882)
    registry.add(std::make_unique<DspSystemModule>());
    // Sensing, vision and Intelligent Systems 1 (3884 sessions 27-28, 2953)
    registry.add(std::make_unique<SensingModelsModule>());
    registry.add(std::make_unique<VisionCameraModule>());
    registry.add(std::make_unique<LearningModelsModule>());
    // The robot itself: what the datasheet publishes, and the modelled part tree
    registry.add(std::make_unique<RobotSpecificationModule>());
    return registry;
}

}  // namespace yaskawa::study
