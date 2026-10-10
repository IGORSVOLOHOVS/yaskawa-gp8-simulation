#ifndef YASKAWA_STUDY_MODULE_REGISTRY_HPP
#define YASKAWA_STUDY_MODULE_REGISTRY_HPP

// The registry owns every study module and implements the wire protocol of
// docs/STUDY_MODULE_CONTRACT.md section 1. It is the only place that turns an
// exception into a response: no module failure escapes handle().

#include "study/json.hpp"
#include "study/study_module.hpp"

#include <cstddef>
#include <memory>
#include <string_view>
#include <vector>

namespace yaskawa::study {

class ModuleRegistry {
public:
    ModuleRegistry() = default;
    ModuleRegistry(const ModuleRegistry&) = delete;
    ModuleRegistry& operator=(const ModuleRegistry&) = delete;
    ModuleRegistry(ModuleRegistry&&) noexcept = default;
    ModuleRegistry& operator=(ModuleRegistry&&) noexcept = default;

    void add(std::unique_ptr<StudyModule> module);

    [[nodiscard]] const StudyModule* find(std::string_view name) const;

    [[nodiscard]] std::size_t size() const noexcept { return modules_.size(); }

    // {"modules": [ModuleDescription, ...], "courses": [CourseRef + modules]}
    // The courses array is deduplicated from the modules themselves so the UI
    // can build its navigation from one request.
    [[nodiscard]] json::Value describe_all() const;

    // One request object in, one response object out. Never throws.
    [[nodiscard]] json::Value handle(const json::Value& request) const;

private:
    std::vector<std::unique_ptr<StudyModule>> modules_;
};

// Registers every module that ships in this build. Later blocks of the
// semester append one line each - keep it a single obvious list.
[[nodiscard]] ModuleRegistry build_default_registry();

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_MODULE_REGISTRY_HPP
