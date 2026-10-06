#ifndef RT_LOGGER_HPP
#define RT_LOGGER_HPP

#include "yaskawa_kinematics.hpp"
#include <string>
#include <fstream>
#include <sstream>
#include <chrono>
#include <mutex>
#include <unordered_map>
#include <iostream>

namespace yaskawa {

// =============================================================================
// Real-Time High-Speed String Telemetry Database & Logger
// =============================================================================
class RealTimeLogger {
public:
    explicit RealTimeLogger(const std::string& log_filename = "profiling/rt_telemetry.log") 
        : log_file_(log_filename, std::ios::out | std::ios::app) {}

    ~RealTimeLogger() {
        if (log_file_.is_open()) {
            log_file_.flush();
            log_file_.close();
        }
    }

    // Log joint telemetry with nanosecond precision timestamp into string database
    void logJointState(
        const std::string& key_prefix,
        const Eigen::Matrix<double, DOF, 1>& joints,
        const Eigen::Isometry3d& pose
    ) {
        auto now = std::chrono::high_resolution_clock::now();
        uint64_t timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()
        ).count();

        std::ostringstream ss;
        ss << timestamp_ns << " | " << key_prefix << " | q=[";
        for (size_t i = 0; i < DOF; ++i) {
            ss << joints[i] << (i + 1 < DOF ? ", " : "");
        }
        ss << "] | Pose_XYZ=[" 
           << pose.translation().x() << ", "
           << pose.translation().y() << ", "
           << pose.translation().z() << "]";

        std::string record = ss.str();

        std::lock_guard<std::mutex> lock(mutex_);
        // In-Memory Key-Value String Database
        string_db_[key_prefix + "_" + std::to_string(timestamp_ns)] = record;

        // Persistent Append-Only Telemetry Log
        if (log_file_.is_open()) {
            log_file_ << record << "\n";
        }
    }

    // Retrieve last logged string entry from database
    std::string getLatestEntry(const std::string& key) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = string_db_.find(key);
        if (it != string_db_.end()) {
            return it->second;
        }
        return "KeyNotSet";
    }

private:
    std::ofstream log_file_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::string> string_db_;
};

} // namespace yaskawa

#endif // RT_LOGGER_HPP
