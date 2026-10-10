// study_api: the C++23 side of the study layer.
//
// One JSON request object per line on stdin, one JSON response object per line
// on stdout, flushed immediately so the Python relay can stream it to the
// browser. A malformed line is answered with ok:false and the loop continues;
// EOF exits 0. `study_api --describe` prints the whole catalogue and exits, so
// ./robot and CI can smoke-test the binary without building a pipe.

#include "study/json.hpp"
#include "study/module_registry.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {

[[nodiscard]] bool is_blank(std::string_view line) noexcept {
    for (const char c : line) {
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
            return false;
        }
    }
    return true;
}

// A parse failure happens before any id is known, so the response echoes 0.
[[nodiscard]] yaskawa::study::json::Value parse_failure(const std::string& error) {
    using yaskawa::study::json::Value;
    Value out = Value::object();
    out.set("id", Value(0));
    out.set("ok", Value(false));
    out.set("module", Value(""));
    out.set("op", Value(""));
    out.set("error", Value(error));
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    using namespace yaskawa::study;

    std::ios::sync_with_stdio(false);
    const ModuleRegistry registry = build_default_registry();

    if (argc > 1) {
        const std::string_view first_argument(argv[1]);
        if (first_argument == "--describe") {
            std::cout << json::dump(registry.describe_all()) << '\n';
            std::cout.flush();
            return 0;
        }
        std::cerr << "study_api: unknown argument '" << first_argument
                  << "'; expected --describe or no argument at all\n";
        return 2;
    }

    std::string line;
    while (std::getline(std::cin, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();  // a CRLF pipe is still one line
        }
        if (is_blank(line)) {
            continue;
        }

        json::Value response;
        try {
            response = registry.handle(json::parse(line));
        } catch (const json::ParseError& error) {
            response = parse_failure(error.what());
        } catch (const std::exception& error) {
            response = parse_failure(std::string("internal error: ") + error.what());
        } catch (...) {
            response = parse_failure("internal error: unknown exception");
        }

        std::cout << json::dump(response) << '\n';
        std::cout.flush();
    }

    return 0;
}
