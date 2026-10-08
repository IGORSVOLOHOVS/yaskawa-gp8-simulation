#ifndef ROBOT_PHYSICAL_TREE_HPP
#define ROBOT_PHYSICAL_TREE_HPP

#include <string>
#include <vector>
#include <cstddef>

namespace yaskawa::physical {

struct ComponentSpec {
    std::string id;
    std::string name;
    std::string parent_id;
    int level; // 1: System, 2: Assembly, 3: Sub-assembly, 4: Component, 5: Micro-part
    int system_category_id; // 1..10
    std::string system_name;
    std::string subsystem;
    std::string material;
    std::string manufacturer;
    std::string part_number;
    std::string specs_summary;
    std::string tolerances;
    double mass_kg;
};

// Returns total count of registered physical components in the tree (1000+)
size_t get_total_component_count() noexcept;

// Access component by index (0 .. count-1)
const ComponentSpec* get_component_at(size_t index) noexcept;

// Find component by ID
const ComponentSpec* find_component_by_id(const std::string& id) noexcept;

// Export full tree to JSON string
std::string export_tree_to_json();

// Export component details to JSON string
std::string export_component_json(const std::string& id);

// Verification routine verifying all 1000+ components are valid and non-empty
bool verify_physical_tree_integrity(size_t& verified_count, std::string& error_msg);

} // namespace yaskawa::physical

#endif // ROBOT_PHYSICAL_TREE_HPP
