#ifndef MY_CR5_CONTROL_RESULT_UTILS_HPP
#define MY_CR5_CONTROL_RESULT_UTILS_HPP

#include "my_cr5_control/env_utils.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>

namespace my_cr5_control::results {

inline std::filesystem::path ensureTestResultsDirPath() {
    const std::string override_dir =
        my_cr5_control::env::getString("MY_CR5_CONTROL_RESULTS_DIR", "");
    std::filesystem::path dir;

    if (!override_dir.empty()) {
        dir = override_dir;
    }
#ifdef MY_CR5_CONTROL_SOURCE_DIR
    else {
        dir = std::filesystem::path(MY_CR5_CONTROL_SOURCE_DIR) / "test_results";
    }
#else
    else {
        dir = "test_results";
    }
#endif

    std::filesystem::create_directories(dir);
    return dir;
}

inline std::string ensureTestResultsDir() {
    return ensureTestResultsDirPath().string();
}

inline std::string makeTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    const std::time_t time_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now {};
    localtime_r(&time_now, &tm_now);

    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S")
        << "_" << std::setw(3) << std::setfill('0') << milliseconds.count();
    return oss.str();
}

inline std::filesystem::path categorizeResultFile(const std::string& filename) {
    if (filename.find("_planner_comparison_simple_") != std::string::npos) {
        return std::filesystem::path("benchmarks") / "simple" / "raw" / filename;
    }

    if (filename.find("_planner_comparison_v2_") != std::string::npos) {
        return std::filesystem::path("benchmarks") / "v2" / "raw" / filename;
    }

    if (filename.find("_simple_random_task_dataset_") != std::string::npos) {
        return std::filesystem::path("datasets") / "simple_random" / "raw" / filename;
    }

    if (filename.find("_guide_ranking_simple_dataset_") != std::string::npos) {
        return std::filesystem::path("datasets") / "guide_ranking_simple" / "raw" / filename;
    }

    if (filename.find("_learned_guidance_simple_ablation_") != std::string::npos) {
        return std::filesystem::path("benchmarks") / "simple_guidance" / "raw" / filename;
    }

    if (filename == "planner_comparison_simple_plot_data_metrics.csv" ||
        filename == "planner_comparison_simple_plot_summary_metrics.csv") {
        return std::filesystem::path("benchmarks") / "simple" / "aggregates" / filename;
    }

    if (filename == "planner_comparison_v2_plot_data_metrics.csv" ||
        filename == "planner_comparison_v2_plot_summary_metrics.csv") {
        return std::filesystem::path("benchmarks") / "v2" / "aggregates" / filename;
    }

    if (filename == "planner_comparison_simple_plot_data.csv" ||
        filename == "planner_comparison_simple_plot_summary.csv" ||
        filename == "planner_comparison_simple_results.csv" ||
        filename == "planner_comparison_simple_summary.csv") {
        return std::filesystem::path("benchmarks") / "simple" / "legacy" / filename;
    }

    if (filename == "planner_comparison_v2_plot_data.csv" ||
        filename == "planner_comparison_v2_plot_summary.csv") {
        return std::filesystem::path("benchmarks") / "v2" / "legacy" / filename;
    }

    if (filename == "benchmark_training_dataset.csv") {
        return std::filesystem::path("exports") / filename;
    }

    if (filename == "dataset_manifest.csv") {
        return filename;
    }

    if (filename.find("box_measurement_log") != std::string::npos ||
        filename.find("box_rviz_teaching_log") != std::string::npos) {
        return std::filesystem::path("operations") / "measurement" / filename;
    }

    if (filename.find("tcp_calibration") != std::string::npos) {
        return std::filesystem::path("operations") / "calibration" / filename;
    }

    return filename;
}

inline std::filesystem::path resolveOutputPath(const std::string& filename) {
    const std::filesystem::path base_dir = ensureTestResultsDirPath();
    const std::filesystem::path relative_path = categorizeResultFile(filename);
    const std::filesystem::path full_path = base_dir / relative_path;
    std::filesystem::create_directories(full_path.parent_path());
    return full_path;
}

inline std::string makeOutputPath(const std::string& timestamp, const std::string& filename) {
    return resolveOutputPath(timestamp + "_" + filename).string();
}

inline std::string makeOutputPath(const std::string& filename) {
    return makeOutputPath(makeTimestamp(), filename);
}

inline std::string makeSharedOutputPath(const std::string& filename) {
    return resolveOutputPath(filename).string();
}

}  // namespace my_cr5_control::results

#endif  // MY_CR5_CONTROL_RESULT_UTILS_HPP
