#include "robot_physical_tree.hpp"
#include "inplace_vector.hpp"
#include "robot_physical_tree.tpp"
#include <algorithm>
#include <expected>
#include <unordered_set>
#include <ranges>
#include <sstream>
#include <string>

namespace yaskawa::physical {

namespace {

const inplace_vector<ComponentSpec, 1400>& get_all_components_internal() {
    static const inplace_vector<ComponentSpec, 1400> components = [] {
        inplace_vector<ComponentSpec, 1400> list;
        constexpr size_t ESTIMATED_TOTAL_PARTS = 1400;
        list.reserve(ESTIMATED_TOTAL_PARTS);
        detail::populate_catalog_meta(list);
        return list;
    }();
    return components;
}

std::string escape_json(const std::string& sv) {
    std::string out;
    constexpr size_t JSON_PADDING = 10;
    out.reserve(sv.size() + JSON_PADDING);
    for (char c : sv) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:   out += c; break;
        }
    }
    return out;
}

} // namespace

size_t get_total_component_count() noexcept {
    return get_all_components_internal().size();
}

ComponentPtr get_component_at(size_t index) noexcept {
    const auto& all = get_all_components_internal();
    if (index < all.size()) {
        return ComponentPtr(&all[index], [](const ComponentSpec*) {});
    }
    return nullptr;
}

ComponentPtr find_component_by_id(const std::string& id) noexcept {
    const auto& all = get_all_components_internal();
    auto it = std::ranges::find_if(all, [&](const auto& item) { return item.id == id; });
    if (it != all.end()) {
        return ComponentPtr(&*it, [](const ComponentSpec*) {});
    }
    return nullptr;
}

std::expected<ComponentPtr, std::string> find_component(const std::string& id) noexcept {
    auto comp = find_component_by_id(id);
    if (comp != nullptr) {
        return comp;
    }
    return std::unexpected("Component with ID '" + id + "' not found");
}

std::string export_component_json(const std::string& id) {
    return find_component(id)
        .transform([](const ComponentPtr& comp) -> std::string {
            std::ostringstream ss;
            ss << "{\n"
               << "  \"id\": \"" << escape_json(comp->id) << "\",\n"
               << "  \"name\": \"" << escape_json(comp->name) << "\",\n"
               << "  \"parent_id\": \"" << escape_json(comp->parent_id) << "\",\n"
               << "  \"level\": " << static_cast<int>(comp->level) << ",\n"
               << "  \"system_category_id\": " << static_cast<int>(comp->system_category_id) << ",\n"
               << "  \"system_name\": \"" << escape_json(comp->system_name) << "\",\n"
               << "  \"subsystem\": \"" << escape_json(comp->subsystem) << "\",\n"
               << "  \"material\": \"" << escape_json(comp->material) << "\",\n"
               << "  \"manufacturer\": \"" << escape_json(comp->manufacturer) << "\",\n"
               << "  \"part_number\": \"" << escape_json(comp->part_number) << "\",\n"
               << "  \"specs_summary\": \"" << escape_json(comp->specs_summary) << "\",\n"
               << "  \"tolerances\": \"" << escape_json(comp->tolerances) << "\",\n"
               << "  \"mass_kg\": " << comp->mass_kg << "\n"
               << "}";
            return ss.str();
        })
        .or_else([](const std::string& err) -> std::expected<std::string, std::string> {
            return "{\"status\":\"error\",\"message\":\"" + err + "\"}";
        })
        .value();
}

std::string export_tree_to_json() {
    const auto& all = get_all_components_internal();
    std::ostringstream ss;
    ss << "{\n"
       << "  \"status\": \"success\",\n"
       << "  \"total_count\": " << all.size() << ",\n"
       << "  \"components\": [\n";

    for (size_t i : std::views::iota(size_t{0}, all.size())) {
        const auto& item = all[i];
        ss << "    {\n"
           << "      \"id\": \"" << escape_json(item.id) << "\",\n"
           << "      \"name\": \"" << escape_json(item.name) << "\",\n"
           << "      \"parent_id\": \"" << escape_json(item.parent_id) << "\",\n"
           << "      \"level\": " << static_cast<int>(item.level) << ",\n"
           << "      \"system_category_id\": " << static_cast<int>(item.system_category_id) << ",\n"
           << "      \"system_name\": \"" << escape_json(item.system_name) << "\",\n"
           << "      \"subsystem\": \"" << escape_json(item.subsystem) << "\",\n"
           << "      \"material\": \"" << escape_json(item.material) << "\",\n"
           << "      \"manufacturer\": \"" << escape_json(item.manufacturer) << "\",\n"
           << "      \"part_number\": \"" << escape_json(item.part_number) << "\",\n"
           << "      \"specs_summary\": \"" << escape_json(item.specs_summary) << "\",\n"
           << "      \"tolerances\": \"" << escape_json(item.tolerances) << "\",\n"
           << "      \"mass_kg\": " << item.mass_kg << "\n"
           << "    }" << (i + 1 < all.size() ? "," : "") << "\n";
    }

    ss << "  ]\n"
       << "}";
    return ss.str();
}

std::expected<size_t, std::string> verify_tree_integrity() noexcept {
    const auto& all = get_all_components_internal();
    constexpr size_t MIN_VERIFIED_COUNT = 1000;
    if (all.size() < MIN_VERIFIED_COUNT) {
        return std::unexpected("Component count is less than 1000: count = " + std::to_string(all.size()));
    }

    std::unordered_set<std::string> id_map;
    id_map.reserve(all.size());

    for (const auto& comp : all) {
        if (comp.id.empty()) {
            return std::unexpected("Found component with empty ID");
        }
        if (comp.name.empty()) {
            return std::unexpected("Component " + comp.id + " has empty name");
        }
        if (id_map.contains(comp.id)) {
            return std::unexpected("Duplicate component ID: " + comp.id);
        }
        id_map.insert(comp.id);
    }

    for (const auto& comp : all) {
        if (!comp.parent_id.empty() && !id_map.contains(comp.parent_id)) {
            return std::unexpected("Missing parent ID '" + comp.parent_id + "' for component " + comp.id);
        }
    }

    return all.size();
}

bool verify_physical_tree_integrity(size_t& verified_count, std::string& error_msg) {
    return verify_tree_integrity()
        .transform([&verified_count, &error_msg](size_t count) {
            verified_count = count;
            error_msg = "OK: Tree integrity verified across " + std::to_string(count) + " components.";
            return true;
        })
        .or_else([&verified_count, &error_msg](const std::string& err) {
            verified_count = 0;
            error_msg = "ERROR: " + err;
            return std::expected<bool, std::string>(false);
        })
        .value_or(false);
}

} // namespace yaskawa::physical
