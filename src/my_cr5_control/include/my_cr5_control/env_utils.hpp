#ifndef MY_CR5_CONTROL_ENV_UTILS_HPP
#define MY_CR5_CONTROL_ENV_UTILS_HPP

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <limits>
#include <optional>
#include <string>

namespace my_cr5_control::env {

inline const char* firstValue(std::initializer_list<const char*> keys) {
    for (const char* key : keys) {
        const char* raw = std::getenv(key);
        if (raw != nullptr && raw[0] != '\0') {
            return raw;
        }
    }
    return nullptr;
}

inline bool parseBool(const char* raw, bool default_value = false) {
    if (raw == nullptr || raw[0] == '\0') {
        return default_value;
    }

    std::string token(raw);
    std::transform(token.begin(), token.end(), token.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });

    if (token == "1" || token == "true" || token == "yes" || token == "on") {
        return true;
    }
    if (token == "0" || token == "false" || token == "no" || token == "off") {
        return false;
    }
    return default_value;
}

inline bool getBool(const char* key, bool default_value = false) {
    return parseBool(std::getenv(key), default_value);
}

inline std::string getString(const char* key, const std::string& default_value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr || raw[0] == '\0') {
        return default_value;
    }
    return raw;
}

inline int getIntClamped(const char* key, int default_value, int min_value, int max_value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr || raw[0] == '\0') {
        return default_value;
    }

    char* end = nullptr;
    const long parsed = std::strtol(raw, &end, 10);
    if (end == raw) {
        return default_value;
    }
    return static_cast<int>(
        std::clamp(parsed, static_cast<long>(min_value), static_cast<long>(max_value)));
}

inline unsigned long getUnsignedLongClamped(const char* key,
                                           unsigned long default_value,
                                           unsigned long min_value,
                                           unsigned long max_value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr || raw[0] == '\0') {
        return default_value;
    }

    char* end = nullptr;
    const unsigned long parsed = std::strtoul(raw, &end, 10);
    if (end == raw) {
        return default_value;
    }
    return std::clamp(parsed, min_value, max_value);
}

inline double getDoubleClamped(const char* key,
                               double default_value,
                               double min_value,
                               double max_value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr || raw[0] == '\0') {
        return default_value;
    }

    char* end = nullptr;
    const double parsed = std::strtod(raw, &end);
    if (end == raw) {
        return default_value;
    }
    return std::clamp(parsed, min_value, max_value);
}

inline std::optional<double> getOptionalDoubleClamped(const char* key,
                                                      double min_value,
                                                      double max_value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr || raw[0] == '\0') {
        return std::nullopt;
    }

    char* end = nullptr;
    const double parsed = std::strtod(raw, &end);
    if (end == raw) {
        return std::nullopt;
    }
    return std::clamp(parsed, min_value, max_value);
}

inline std::optional<std::uint32_t> parseUint32(const char* raw) {
    if (raw == nullptr || raw[0] == '\0') {
        return std::nullopt;
    }

    char* end = nullptr;
    const unsigned long parsed = std::strtoul(raw, &end, 10);
    if (end == raw) {
        return std::nullopt;
    }
    return static_cast<std::uint32_t>(
        std::min<unsigned long>(parsed, std::numeric_limits<std::uint32_t>::max()));
}

inline std::optional<std::uint32_t> getUint32(const char* key) {
    return parseUint32(std::getenv(key));
}

}  // namespace my_cr5_control::env

#endif  // MY_CR5_CONTROL_ENV_UTILS_HPP
