#ifndef YASKAWA_STUDY_JSON_HPP
#define YASKAWA_STUDY_JSON_HPP

// Self-contained JSON value, parser and serialiser for the study layer.
//
// No third-party dependency: the wire protocol of docs/STUDY_MODULE_CONTRACT.md
// is one compact JSON object per line, which is small enough to own outright.
//
// Design notes
//  - Object member order is preserved (parallel key/value vectors, never a
//    std::map) so dump() is byte-for-byte deterministic.
//  - Numbers are IEEE-754 doubles. Non-finite values serialise as `null`
//    because `nan`/`inf` are not JSON and would poison the browser parser.
//  - dump() uses the shortest round-trip representation (std::to_chars), so
//    parse(dump(v)) == v holds for every finite shape.

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <charconv>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace yaskawa::study::json {

// Thrown by parse() on malformed input. The registry turns it into ok:false.
class ParseError final : public std::runtime_error {
public:
    explicit ParseError(const std::string& message) : std::runtime_error(message) {}
};

// Thrown by the as_*() accessors when the stored type is not the asked type.
class TypeError final : public std::runtime_error {
public:
    explicit TypeError(const std::string& message) : std::runtime_error(message) {}
};

enum class Type : std::uint8_t { Null, Bool, Number, String, Array, Object };

[[nodiscard]] std::string number_to_string(double value);

class Value {
public:
    Value() noexcept = default;
    Value(std::nullptr_t) noexcept {}

    Value(bool value) noexcept : type_(Type::Bool), bool_(value) {}

    template <typename T>
        requires std::integral<T> && (!std::same_as<T, bool>)
    Value(T value) noexcept : type_(Type::Number), number_(static_cast<double>(value)) {}

    template <typename T>
        requires std::floating_point<T>
    Value(T value) noexcept : type_(Type::Number), number_(static_cast<double>(value)) {}

    Value(const char* value) : type_(Type::String), string_(value ? value : "") {}
    Value(std::string value) : type_(Type::String), string_(std::move(value)) {}
    Value(std::string_view value) : type_(Type::String), string_(value) {}

    // ---- builders -------------------------------------------------------
    [[nodiscard]] static Value object() {
        Value v;
        v.type_ = Type::Object;
        return v;
    }

    [[nodiscard]] static Value object(std::initializer_list<std::pair<std::string, Value>> members) {
        Value v = object();
        v.keys_.reserve(members.size());
        v.items_.reserve(members.size());
        for (const auto& member : members) {
            v.keys_.push_back(member.first);
            v.items_.push_back(member.second);
        }
        return v;
    }

    [[nodiscard]] static Value array() {
        Value v;
        v.type_ = Type::Array;
        return v;
    }

    [[nodiscard]] static Value array(std::initializer_list<Value> elements) {
        Value v = array();
        v.items_.assign(elements.begin(), elements.end());
        return v;
    }

    [[nodiscard]] static Value array(std::vector<Value> elements) {
        Value v = array();
        v.items_ = std::move(elements);
        return v;
    }

    // ---- interrogation --------------------------------------------------
    [[nodiscard]] Type type() const noexcept { return type_; }
    [[nodiscard]] bool is_null() const noexcept { return type_ == Type::Null; }
    [[nodiscard]] bool is_bool() const noexcept { return type_ == Type::Bool; }
    [[nodiscard]] bool is_number() const noexcept { return type_ == Type::Number; }
    [[nodiscard]] bool is_string() const noexcept { return type_ == Type::String; }
    [[nodiscard]] bool is_array() const noexcept { return type_ == Type::Array; }
    [[nodiscard]] bool is_object() const noexcept { return type_ == Type::Object; }

    [[nodiscard]] const char* type_name() const noexcept {
        switch (type_) {
            case Type::Null: return "null";
            case Type::Bool: return "bool";
            case Type::Number: return "number";
            case Type::String: return "string";
            case Type::Array: return "array";
            case Type::Object: return "object";
        }
        return "null";
    }

    [[nodiscard]] double as_double() const {
        if (type_ != Type::Number) {
            throw TypeError(std::string("expected a number, found ") + type_name());
        }
        return number_;
    }

    // Rounds, so 17.0000000001 coming back from a browser still reads as 17.
    [[nodiscard]] long long as_int() const {
        const double d = as_double();
        if (!std::isfinite(d)) {
            throw TypeError("expected an integral number, found a non-finite value");
        }
        return static_cast<long long>(std::llround(d));
    }

    [[nodiscard]] bool as_bool() const {
        if (type_ != Type::Bool) {
            throw TypeError(std::string("expected a bool, found ") + type_name());
        }
        return bool_;
    }

    [[nodiscard]] const std::string& as_string() const {
        if (type_ != Type::String) {
            throw TypeError(std::string("expected a string, found ") + type_name());
        }
        return string_;
    }

    // Element count: array elements, object members, string characters, else 0.
    [[nodiscard]] std::size_t size() const noexcept {
        switch (type_) {
            case Type::Array:
            case Type::Object: return items_.size();
            case Type::String: return string_.size();
            default: return 0;
        }
    }

    [[nodiscard]] bool empty() const noexcept { return size() == 0; }

    [[nodiscard]] bool contains(std::string_view key) const noexcept {
        return index_of(key) != npos;
    }

    [[nodiscard]] const std::vector<std::string>& keys() const noexcept { return keys_; }
    [[nodiscard]] const std::vector<Value>& items() const noexcept { return items_; }

    // A missing key or an out-of-range index reads as null instead of throwing:
    // module code checks with the require_* helpers, which report far better.
    [[nodiscard]] const Value& operator[](std::string_view key) const noexcept {
        const std::size_t i = index_of(key);
        return (i == npos) ? null_value() : items_[i];
    }

    [[nodiscard]] const Value& operator[](std::size_t index) const noexcept {
        return (type_ == Type::Array && index < items_.size()) ? items_[index] : null_value();
    }

    [[nodiscard]] const Value& at(std::size_t index) const {
        if (index >= items_.size()) {
            throw TypeError("array index " + std::to_string(index) + " is out of range");
        }
        return items_[index];
    }

    // ---- mutation (building a response) ---------------------------------
    Value& operator[](std::string_view key) {
        if (type_ == Type::Null) {
            type_ = Type::Object;
        }
        if (type_ != Type::Object) {
            throw TypeError(std::string("cannot index a ") + type_name() + " by key");
        }
        const std::size_t i = index_of(key);
        if (i != npos) {
            return items_[i];
        }
        keys_.emplace_back(key);
        items_.emplace_back();
        return items_.back();
    }

    Value& set(std::string_view key, Value value) {
        Value& slot = (*this)[key];
        slot = std::move(value);
        return slot;
    }

    Value& push_back(Value value) {
        if (type_ == Type::Null) {
            type_ = Type::Array;
        }
        if (type_ != Type::Array) {
            throw TypeError(std::string("cannot push_back onto a ") + type_name());
        }
        items_.push_back(std::move(value));
        return items_.back();
    }

    void reserve(std::size_t n) { items_.reserve(n); }

    [[nodiscard]] bool operator==(const Value& other) const {
        if (type_ != other.type_) {
            return false;
        }
        switch (type_) {
            case Type::Null: return true;
            case Type::Bool: return bool_ == other.bool_;
            case Type::Number: return number_ == other.number_;
            case Type::String: return string_ == other.string_;
            case Type::Array: return items_ == other.items_;
            case Type::Object: return keys_ == other.keys_ && items_ == other.items_;
        }
        return false;
    }

    [[nodiscard]] bool operator!=(const Value& other) const { return !(*this == other); }

private:
    static constexpr std::size_t npos = static_cast<std::size_t>(-1);

    [[nodiscard]] static const Value& null_value() noexcept {
        static const Value instance;
        return instance;
    }

    [[nodiscard]] std::size_t index_of(std::string_view key) const noexcept {
        if (type_ != Type::Object) {
            return npos;
        }
        for (std::size_t i = 0; i < keys_.size(); ++i) {
            if (keys_[i] == key) {
                return i;
            }
        }
        return npos;
    }

    Type type_ = Type::Null;
    bool bool_ = false;
    double number_ = 0.0;
    std::string string_;
    std::vector<Value> items_;       // array elements, or object values
    std::vector<std::string> keys_;  // object keys, parallel to items_
};

// ---------------------------------------------------------------------------
// Serialisation
// ---------------------------------------------------------------------------

inline std::string number_to_string(double value) {
    if (!std::isfinite(value)) {
        return "null";
    }
    char buffer[48];
    const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);
    return std::string(buffer, result.ptr);
}

namespace detail {

inline void append_escaped(std::string& out, std::string_view text) {
    out.push_back('"');
    for (const char raw : text) {
        const auto byte = static_cast<unsigned char>(raw);
        switch (byte) {
            case 0x22: out += "\\\""; break;
            case 0x5C: out += "\\\\"; break;
            case 0x08: out += "\\b"; break;
            case 0x0C: out += "\\f"; break;
            case 0x0A: out += "\\n"; break;
            case 0x0D: out += "\\r"; break;
            case 0x09: out += "\\t"; break;
            default:
                if (byte < 0x20) {
                    static const char kHex[] = "0123456789abcdef";
                    out += "\\u00";
                    out.push_back(kHex[(byte >> 4) & 0x0F]);
                    out.push_back(kHex[byte & 0x0F]);
                } else {
                    out.push_back(raw);  // UTF-8 multi-byte sequences pass through
                }
                break;
        }
    }
    out.push_back('"');
}

inline void dump_into(std::string& out, const Value& value) {
    switch (value.type()) {
        case Type::Null:
            out += "null";
            break;
        case Type::Bool:
            out += value.as_bool() ? "true" : "false";
            break;
        case Type::Number:
            out += number_to_string(value.as_double());
            break;
        case Type::String:
            append_escaped(out, value.as_string());
            break;
        case Type::Array: {
            out.push_back('[');
            const auto& elements = value.items();
            for (std::size_t i = 0; i < elements.size(); ++i) {
                if (i != 0) {
                    out.push_back(',');
                }
                dump_into(out, elements[i]);
            }
            out.push_back(']');
            break;
        }
        case Type::Object: {
            out.push_back('{');
            const auto& keys = value.keys();
            const auto& values = value.items();
            for (std::size_t i = 0; i < keys.size(); ++i) {
                if (i != 0) {
                    out.push_back(',');
                }
                append_escaped(out, keys[i]);
                out.push_back(':');
                dump_into(out, values[i]);
            }
            out.push_back('}');
            break;
        }
    }
}

}  // namespace detail

// Compact, single-line, deterministic JSON text.
[[nodiscard]] inline std::string dump(const Value& value) {
    std::string out;
    out.reserve(256);
    detail::dump_into(out, value);
    return out;
}

// ---------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------

namespace detail {

class Parser {
public:
    explicit Parser(std::string_view text) noexcept : text_(text) {}

    [[nodiscard]] Value run() {
        skip_whitespace();
        Value value = parse_value(0);
        skip_whitespace();
        if (pos_ != text_.size()) {
            fail("trailing characters after the JSON value");
        }
        return value;
    }

private:
    static constexpr int kMaxDepth = 200;

    [[noreturn]] void fail(const std::string& message) const {
        throw ParseError("JSON parse error at offset " + std::to_string(pos_) + ": " + message);
    }

    [[nodiscard]] bool done() const noexcept { return pos_ >= text_.size(); }

    [[nodiscard]] char peek() const {
        if (done()) {
            fail("unexpected end of input");
        }
        return text_[pos_];
    }

    void skip_whitespace() noexcept {
        while (pos_ < text_.size()) {
            const char c = text_[pos_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                ++pos_;
            } else {
                break;
            }
        }
    }

    void expect(char c) {
        if (done() || text_[pos_] != c) {
            fail(std::string("expected '") + c + "'");
        }
        ++pos_;
    }

    void expect_literal(std::string_view literal) {
        if (text_.substr(pos_, literal.size()) != literal) {
            fail("invalid literal, expected " + std::string(literal));
        }
        pos_ += literal.size();
    }

    [[nodiscard]] Value parse_value(int depth) {
        if (depth > kMaxDepth) {
            fail("nesting is deeper than " + std::to_string(kMaxDepth) + " levels");
        }
        switch (peek()) {
            case '{': return parse_object(depth);
            case '[': return parse_array(depth);
            case '"': return Value(parse_string());
            case 't': expect_literal("true"); return Value(true);
            case 'f': expect_literal("false"); return Value(false);
            case 'n': expect_literal("null"); return Value();
            default: return parse_number();
        }
    }

    [[nodiscard]] Value parse_object(int depth) {
        expect('{');
        Value result = Value::object();
        skip_whitespace();
        if (!done() && peek() == '}') {
            ++pos_;
            return result;
        }
        while (true) {
            skip_whitespace();
            if (peek() != '"') {
                fail("object keys must be strings");
            }
            const std::string key = parse_string();
            skip_whitespace();
            expect(':');
            skip_whitespace();
            result.set(key, parse_value(depth + 1));
            skip_whitespace();
            const char c = peek();
            if (c == ',') {
                ++pos_;
                continue;
            }
            if (c == '}') {
                ++pos_;
                return result;
            }
            fail("expected ',' or '}' in object");
        }
    }

    [[nodiscard]] Value parse_array(int depth) {
        expect('[');
        Value result = Value::array();
        skip_whitespace();
        if (!done() && peek() == ']') {
            ++pos_;
            return result;
        }
        while (true) {
            skip_whitespace();
            result.push_back(parse_value(depth + 1));
            skip_whitespace();
            const char c = peek();
            if (c == ',') {
                ++pos_;
                continue;
            }
            if (c == ']') {
                ++pos_;
                return result;
            }
            fail("expected ',' or ']' in array");
        }
    }

    [[nodiscard]] unsigned parse_hex4() {
        if (pos_ + 4 > text_.size()) {
            fail("truncated unicode escape");
        }
        unsigned code = 0;
        for (int i = 0; i < 4; ++i) {
            const char c = text_[pos_ + static_cast<std::size_t>(i)];
            unsigned digit = 0;
            if (c >= '0' && c <= '9') {
                digit = static_cast<unsigned>(c - '0');
            } else if (c >= 'a' && c <= 'f') {
                digit = static_cast<unsigned>(c - 'a') + 10U;
            } else if (c >= 'A' && c <= 'F') {
                digit = static_cast<unsigned>(c - 'A') + 10U;
            } else {
                fail("a unicode escape needs four hexadecimal digits");
            }
            code = (code << 4) | digit;
        }
        pos_ += 4;
        return code;
    }

    static void append_utf8(std::string& out, unsigned code_point) {
        if (code_point < 0x80U) {
            out.push_back(static_cast<char>(code_point));
        } else if (code_point < 0x800U) {
            out.push_back(static_cast<char>(0xC0U | (code_point >> 6)));
            out.push_back(static_cast<char>(0x80U | (code_point & 0x3FU)));
        } else if (code_point < 0x10000U) {
            out.push_back(static_cast<char>(0xE0U | (code_point >> 12)));
            out.push_back(static_cast<char>(0x80U | ((code_point >> 6) & 0x3FU)));
            out.push_back(static_cast<char>(0x80U | (code_point & 0x3FU)));
        } else {
            out.push_back(static_cast<char>(0xF0U | (code_point >> 18)));
            out.push_back(static_cast<char>(0x80U | ((code_point >> 12) & 0x3FU)));
            out.push_back(static_cast<char>(0x80U | ((code_point >> 6) & 0x3FU)));
            out.push_back(static_cast<char>(0x80U | (code_point & 0x3FU)));
        }
    }

    [[nodiscard]] std::string parse_string() {
        expect('"');
        std::string out;
        while (true) {
            if (done()) {
                fail("unterminated string");
            }
            const char c = text_[pos_++];
            if (c == '"') {
                return out;
            }
            if (c != '\\') {
                if (static_cast<unsigned char>(c) < 0x20) {
                    fail("raw control character in string");
                }
                out.push_back(c);
                continue;
            }
            if (done()) {
                fail("unterminated escape sequence");
            }
            const char esc = text_[pos_++];
            switch (esc) {
                case '"': out.push_back('"'); break;
                case '\\': out.push_back('\\'); break;
                case '/': out.push_back('/'); break;
                case 'b': out.push_back('\b'); break;
                case 'f': out.push_back('\f'); break;
                case 'n': out.push_back('\n'); break;
                case 'r': out.push_back('\r'); break;
                case 't': out.push_back('\t'); break;
                case 'u': {
                    unsigned code = parse_hex4();
                    if (code >= 0xD800U && code <= 0xDBFFU) {
                        if (pos_ + 1 >= text_.size() || text_[pos_] != '\\' || text_[pos_ + 1] != 'u') {
                            fail("high surrogate without a low surrogate");
                        }
                        pos_ += 2;
                        const unsigned low = parse_hex4();
                        if (low < 0xDC00U || low > 0xDFFFU) {
                            fail("invalid low surrogate");
                        }
                        code = 0x10000U + ((code - 0xD800U) << 10) + (low - 0xDC00U);
                    } else if (code >= 0xDC00U && code <= 0xDFFFU) {
                        fail("unpaired low surrogate");
                    }
                    append_utf8(out, code);
                    break;
                }
                default:
                    fail("unknown escape sequence");
            }
        }
    }

    [[nodiscard]] Value parse_number() {
        const std::size_t start = pos_;
        if (!done() && text_[pos_] == '-') {
            ++pos_;
        }
        if (done() || text_[pos_] < '0' || text_[pos_] > '9') {
            fail("invalid number: an integer part is required");
        }
        if (text_[pos_] == '0') {
            ++pos_;
            if (!done() && text_[pos_] >= '0' && text_[pos_] <= '9') {
                fail("invalid number: leading zeros are not allowed");
            }
        } else {
            while (!done() && text_[pos_] >= '0' && text_[pos_] <= '9') {
                ++pos_;
            }
        }
        if (!done() && text_[pos_] == '.') {
            ++pos_;
            if (done() || text_[pos_] < '0' || text_[pos_] > '9') {
                fail("invalid number: a digit must follow the decimal point");
            }
            while (!done() && text_[pos_] >= '0' && text_[pos_] <= '9') {
                ++pos_;
            }
        }
        if (!done() && (text_[pos_] == 'e' || text_[pos_] == 'E')) {
            ++pos_;
            if (!done() && (text_[pos_] == '+' || text_[pos_] == '-')) {
                ++pos_;
            }
            if (done() || text_[pos_] < '0' || text_[pos_] > '9') {
                fail("invalid number: a digit must follow the exponent");
            }
            while (!done() && text_[pos_] >= '0' && text_[pos_] <= '9') {
                ++pos_;
            }
        }
        const std::string_view token = text_.substr(start, pos_ - start);
        double parsed = 0.0;
        const auto result = std::from_chars(token.data(), token.data() + token.size(), parsed);
        if (result.ec == std::errc::result_out_of_range) {
            // Overflow and underflow of a syntactically valid literal: saturate
            // the way a browser's JSON.parse does rather than reject the line.
            parsed = (token.front() == '-') ? -HUGE_VAL : HUGE_VAL;
        } else if (result.ec != std::errc{} || result.ptr != token.data() + token.size()) {
            fail("invalid number");
        }
        return Value(parsed);
    }

    std::string_view text_;
    std::size_t pos_ = 0;
};

}  // namespace detail

[[nodiscard]] inline Value parse(std::string_view text) {
    return detail::Parser(text).run();
}

// ---------------------------------------------------------------------------
// Contract type vocabulary helpers (docs/STUDY_MODULE_CONTRACT.md section 3)
// ---------------------------------------------------------------------------

// vec3 / vec6 / any length: a flat array of numbers.
template <typename Derived>
[[nodiscard]] Value from_vector(const Eigen::MatrixBase<Derived>& v) {
    Value out = Value::array();
    out.reserve(static_cast<std::size_t>(v.size()));
    for (Eigen::Index i = 0; i < v.size(); ++i) {
        out.push_back(Value(static_cast<double>(v(i))));
    }
    return out;
}

[[nodiscard]] inline Value from_vec3(const Eigen::Vector3d& v) { return from_vector(v); }

[[nodiscard]] inline Value from_vec6(const Eigen::Matrix<double, 6, 1>& v) { return from_vector(v); }

// mat3 / mat4 / matrix: row-major nested arrays.
template <typename Derived>
[[nodiscard]] Value from_matrix(const Eigen::MatrixBase<Derived>& m) {
    Value rows = Value::array();
    rows.reserve(static_cast<std::size_t>(m.rows()));
    for (Eigen::Index r = 0; r < m.rows(); ++r) {
        Value row = Value::array();
        row.reserve(static_cast<std::size_t>(m.cols()));
        for (Eigen::Index c = 0; c < m.cols(); ++c) {
            row.push_back(Value(static_cast<double>(m(r, c))));
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

// mat4 from a rigid transform.
[[nodiscard]] inline Value from_isometry(const Eigen::Isometry3d& T) {
    return from_matrix(T.matrix());
}

// series: {"x": [...], "y": [...], "label": "..."}
[[nodiscard]] inline Value from_series(std::string_view label,
                                       const std::vector<double>& x,
                                       const std::vector<double>& y) {
    Value xs = Value::array();
    xs.reserve(x.size());
    for (const double v : x) {
        xs.push_back(Value(v));
    }
    Value ys = Value::array();
    ys.reserve(y.size());
    for (const double v : y) {
        ys.push_back(Value(v));
    }
    Value out = Value::object();
    out.set("x", std::move(xs));
    out.set("y", std::move(ys));
    out.set("label", Value(std::string(label)));
    return out;
}

template <typename DerivedX, typename DerivedY>
[[nodiscard]] Value from_series(std::string_view label,
                                const Eigen::MatrixBase<DerivedX>& x,
                                const Eigen::MatrixBase<DerivedY>& y) {
    Value out = Value::object();
    out.set("x", from_vector(x));
    out.set("y", from_vector(y));
    out.set("label", Value(std::string(label)));
    return out;
}

// points: [[x, y, z], ...]
[[nodiscard]] inline Value from_points(const std::vector<Eigen::Vector3d>& points) {
    Value out = Value::array();
    out.reserve(points.size());
    for (const auto& p : points) {
        out.push_back(from_vec3(p));
    }
    return out;
}

// complex_set: [[re, im], ...]
[[nodiscard]] inline Value from_complex_set(const std::vector<std::complex<double>>& values) {
    Value out = Value::array();
    out.reserve(values.size());
    for (const auto& z : values) {
        out.push_back(Value::array({Value(z.real()), Value(z.imag())}));
    }
    return out;
}

// table: {"columns": [...], "rows": [[...], ...]}
[[nodiscard]] inline Value from_table(const std::vector<std::string>& columns,
                                      const std::vector<std::vector<Value>>& rows) {
    Value column_names = Value::array();
    column_names.reserve(columns.size());
    for (const auto& name : columns) {
        column_names.push_back(Value(name));
    }
    Value row_values = Value::array();
    row_values.reserve(rows.size());
    for (const auto& row : rows) {
        Value out_row = Value::array();
        out_row.reserve(row.size());
        for (const auto& cell : row) {
            out_row.push_back(cell);
        }
        row_values.push_back(std::move(out_row));
    }
    Value out = Value::object();
    out.set("columns", std::move(column_names));
    out.set("rows", std::move(row_values));
    return out;
}

[[nodiscard]] inline Value from_table(const std::vector<std::string>& columns,
                                      const std::vector<std::vector<double>>& rows) {
    std::vector<std::vector<Value>> converted;
    converted.reserve(rows.size());
    for (const auto& row : rows) {
        std::vector<Value> out_row;
        out_row.reserve(row.size());
        for (const double cell : row) {
            out_row.emplace_back(cell);
        }
        converted.push_back(std::move(out_row));
    }
    return from_table(columns, converted);
}

// mat4_set: [mat4, ...] - a list of homogeneous transforms, one per link frame.
// The contract names `series_set` for a list of series; this follows the same
// idiom, because the 3D view needs every frame of one configuration at once.
[[nodiscard]] inline Value from_isometry_list(const std::vector<Eigen::Isometry3d>& transforms) {
    Value out = Value::array();
    out.reserve(transforms.size());
    for (const auto& T : transforms) {
        out.push_back(from_isometry(T));
    }
    return out;
}

[[nodiscard]] inline Value from_strings(const std::vector<std::string>& values) {
    Value out = Value::array();
    out.reserve(values.size());
    for (const auto& v : values) {
        out.push_back(Value(v));
    }
    return out;
}

}  // namespace yaskawa::study::json

#endif  // YASKAWA_STUDY_JSON_HPP
