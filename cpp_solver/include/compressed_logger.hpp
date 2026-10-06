#ifndef COMPRESSED_LOGGER_HPP
#define COMPRESSED_LOGGER_HPP

#include "yaskawa_kinematics.hpp"
#include <cstdint>
#include <vector>
#include <fstream>
#include <source_location>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <algorithm>

namespace yaskawa {

// =============================================================================
// Ultra-Compressed 16-Byte Bit-Field Frame (10x Compression vs Raw Text Log)
// =============================================================================
// Total Size: 128 bits = EXACTLY 16 BYTES
#pragma pack(push, 1)
struct UltraCompactBitFrame {
    // 64-bit Header Word (64 bits)
    uint64_t timestamp_delta_us : 24; // 24 bits: Delta time up to 16.7 seconds
    uint64_t line_id            : 12; // 12 bits: Line number __LINE__ (up to 4095)
    uint64_t event_code         : 8;  // 8 bits: Event type ID (256 codes)
    uint64_t flags              : 4;  // 4 bits: Status flags
    uint64_t q0_bit             : 16; // 16 bits: Joint 1 angle

    // 64-bit Payload Word (64 bits)
    uint64_t q1_bit             : 12; // 12 bits: Joint 2 angle
    uint64_t q2_bit             : 12; // 12 bits: Joint 3 angle
    uint64_t q3_bit             : 12; // 12 bits: Joint 4 angle
    uint64_t q4_bit             : 14; // 14 bits: Joint 5 angle
    uint64_t q5_bit             : 14; // 14 bits: Joint 6 angle
};
#pragma pack(pop)

static_assert(sizeof(UltraCompactBitFrame) == 16, "UltraCompactBitFrame must be EXACTLY 16 bytes!");

class BitFieldLogger {
public:
    explicit BitFieldLogger(const std::string& bin_filename = "profiling/ultra_compact_trace.bin")
        : bin_filename_(bin_filename), start_time_(std::chrono::high_resolution_clock::now()) {
        buffer_.reserve(100000);
    }

    ~BitFieldLogger() {
        flushToFile();
    }

    // Pack 6 joint angles and execution metadata into 16-byte bit-fields
    void logBitFrame(
        uint8_t event_code,
        const Eigen::Matrix<double, DOF, 1>& joints,
        uint8_t flags = 0,
        std::source_location loc = std::source_location::current()
    ) noexcept {
        auto now = std::chrono::high_resolution_clock::now();
        uint32_t delta_us = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_).count()
        );

        UltraCompactBitFrame frame{};
        frame.timestamp_delta_us = delta_us & 0xFFFFFF; // 24-bit mask
        frame.line_id = loc.line() & 0xFFF;            // 12-bit mask
        frame.event_code = event_code & 0xFF;
        frame.flags = flags & 0xF;

        auto encode = [](double rad, uint64_t max_val) -> uint64_t {
            double norm = (rad + 6.283185) / 12.56637;
            norm = std::max(0.0, std::min(1.0, norm));
            return static_cast<uint64_t>(norm * max_val);
        };

        frame.q0_bit = encode(joints[0], 65535);
        frame.q1_bit = encode(joints[1], 4095);
        frame.q2_bit = encode(joints[2], 4095);
        frame.q3_bit = encode(joints[3], 4095);
        frame.q4_bit = encode(joints[4], 16383);
        frame.q5_bit = encode(joints[5], 16383);

        buffer_.push_back(frame);
    }

    void flushToFile() {
        if (buffer_.empty()) return;
        std::ofstream out(bin_filename_, std::ios::binary | std::ios::out);
        if (out.is_open()) {
            out.write(reinterpret_cast<const char*>(buffer_.data()), buffer_.size() * sizeof(UltraCompactBitFrame));
            out.close();
        }
    }

    size_t getFrameCount() const { return buffer_.size(); }
    size_t getCompressedSizeBytes() const { return buffer_.size() * sizeof(UltraCompactBitFrame); }

private:
    std::string bin_filename_;
    std::chrono::high_resolution_clock::time_point start_time_;
    std::vector<UltraCompactBitFrame> buffer_;
};

} // namespace yaskawa

#endif // COMPRESSED_LOGGER_HPP
