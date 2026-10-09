#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <memory>
#include <expected>

namespace yaskawa::physical {

#pragma pack(push, 1)
struct alignas(8) ComponentSpec {
    std::string id;
    std::string name;
    std::string parent_id;
    uint8_t level : 8; // 1: System, 2: Assembly, 3: Sub-assembly, 4: Component, 5: Micro-part
    uint8_t system_category_id : 8; // 0..12
    std::string system_name;
    std::string subsystem;
    std::string material;
    std::string manufacturer;
    std::string part_number;
    std::string specs_summary;
    std::string tolerances;
    double mass_kg;
};
#pragma pack(pop)

// Smart pointer alias for safe non-owning component references
using ComponentPtr = std::shared_ptr<const ComponentSpec>;

// Returns total count of registered physical components in the tree (1000+)
size_t get_total_component_count() noexcept;

// Access component by index (0 .. count-1) using smart pointer
ComponentPtr get_component_at(size_t index) noexcept;

// Find component by ID (Smart pointer-based)
ComponentPtr find_component_by_id(const std::string& id) noexcept;

// C++23 Monadic Expected-based Component Lookup using smart pointer
std::expected<ComponentPtr, std::string> find_component(const std::string& id) noexcept;

// Export full tree to JSON string
std::string export_tree_to_json();

// Export component details to JSON string
std::string export_component_json(const std::string& id);

// C++23 Monadic Expected-based Verification Routine
std::expected<size_t, std::string> verify_tree_integrity() noexcept;

// Verification routine verifying all 1000+ components are valid and non-empty
bool verify_physical_tree_integrity(size_t& verified_count, std::string& error_msg);

} // namespace yaskawa::physical
