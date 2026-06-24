#ifndef MY_CR5_CONTROL_PLANNER_SELECTION_UTILS_HPP_
#define MY_CR5_CONTROL_PLANNER_SELECTION_UTILS_HPP_

#include "my_cr5_control/env_utils.hpp"

#include <algorithm>
#include <cctype>
#include <initializer_list>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace my_cr5_control::benchmarks {

inline const char* getFirstPlannerEnvValue(std::initializer_list<const char*> env_keys) {
    return my_cr5_control::env::firstValue(env_keys);
}

inline bool parseEnvBool(const char* raw, bool default_value = false) {
    return my_cr5_control::env::parseBool(raw, default_value);
}

inline bool getEnvBool(const char* key, bool default_value = false) {
    return my_cr5_control::env::getBool(key, default_value);
}

inline std::optional<double> getEnvClampedDouble(const char* key, double min_value, double max_value) {
    return my_cr5_control::env::getOptionalDoubleClamped(key, min_value, max_value);
}

struct AdaptiveEllipsoidEnvConfig {
    bool enabled{true};
    std::optional<double> fixed_difficulty;
};

inline AdaptiveEllipsoidEnvConfig getAdaptiveEllipsoidEnvConfig() {
    AdaptiveEllipsoidEnvConfig config;
    // Paper-mainline benchmark behavior defaults to adaptive ellipsoid enabled.
    // Use MY_CR5_CONTROL_ADAPTIVE_ELLIPSOID=0 to opt out for ablations.
    config.enabled = getEnvBool("MY_CR5_CONTROL_ADAPTIVE_ELLIPSOID", true);
    config.fixed_difficulty = getEnvClampedDouble(
        "MY_CR5_CONTROL_ADAPTIVE_ELLIPSOID_FIXED_DIFFICULTY", 0.0, 1.0);
    if (!config.fixed_difficulty.has_value()) {
        config.fixed_difficulty = getEnvClampedDouble(
            "MY_CR5_CONTROL_ADAPTIVE_ELLIPSOID_DIFFICULTY", 0.0, 1.0);
    }
    return config;
}

inline double resolveAdaptiveEllipsoidDifficulty(const AdaptiveEllipsoidEnvConfig& config,
                                                 double scene_difficulty) {
    if (config.fixed_difficulty.has_value()) {
        return *config.fixed_difficulty;
    }
    return std::clamp(scene_difficulty, 0.0, 1.0);
}

inline std::string joinTokens(const std::vector<std::string>& tokens) {
    std::ostringstream oss;
    for (std::size_t i = 0; i < tokens.size(); ++i) {
        if (i != 0) {
            oss << ",";
        }
        oss << tokens[i];
    }
    return oss.str();
}

inline std::vector<std::string> splitCommaTokens(const std::string& raw) {
    std::vector<std::string> tokens;
    std::stringstream ss(raw);
    std::string token;
    while (std::getline(ss, token, ',')) {
        token.erase(
            std::remove_if(token.begin(), token.end(), [](unsigned char ch) { return std::isspace(ch); }),
            token.end());
        if (!token.empty()) {
            tokens.push_back(token);
        }
    }
    return tokens;
}

template <typename PlannerConfig>
std::string joinPlannerNames(const std::vector<PlannerConfig>& planners) {
    std::vector<std::string> names;
    names.reserve(planners.size());
    for (const auto& planner : planners) {
        names.push_back(planner.name);
    }
    return joinTokens(names);
}

template <typename PlannerConfig>
std::vector<PlannerConfig> selectPlannerConfigs(
    const std::vector<PlannerConfig>& defaults,
    const std::vector<PlannerConfig>& supported,
    std::initializer_list<const char*> env_keys,
    std::vector<std::string>* unknown_tokens = nullptr) {
    const char* raw = getFirstPlannerEnvValue(env_keys);
    if (raw == nullptr) {
        return defaults;
    }

    std::vector<PlannerConfig> selected;
    for (const auto& token : splitCommaTokens(raw)) {
        const auto it = std::find_if(supported.begin(), supported.end(), [&](const PlannerConfig& config) {
            return config.name == token;
        });
        if (it == supported.end()) {
            if (unknown_tokens != nullptr &&
                std::find(unknown_tokens->begin(), unknown_tokens->end(), token) == unknown_tokens->end()) {
                unknown_tokens->push_back(token);
            }
            continue;
        }

        const auto duplicate_it =
            std::find_if(selected.begin(), selected.end(), [&](const PlannerConfig& config) {
                return config.name == it->name;
            });
        if (duplicate_it == selected.end()) {
            selected.push_back(*it);
        }
    }

    return selected.empty() ? defaults : selected;
}

}  // namespace my_cr5_control::benchmarks

#endif  // MY_CR5_CONTROL_PLANNER_SELECTION_UTILS_HPP_
