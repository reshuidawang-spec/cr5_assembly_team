#ifndef MY_CR5_CONTROL_PAPER_MAINLINE_GUIDE_GEOMETRY_HPP
#define MY_CR5_CONTROL_PAPER_MAINLINE_GUIDE_GEOMETRY_HPP

#include <geometry_msgs/msg/point.hpp>

#include <cmath>

namespace my_cr5_control::paper_mainline {

inline double pointDistance(const geometry_msgs::msg::Point& a,
                            const geometry_msgs::msg::Point& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline geometry_msgs::msg::Point midpoint(const geometry_msgs::msg::Point& a,
                                          const geometry_msgs::msg::Point& b) {
    geometry_msgs::msg::Point mid;
    mid.x = 0.5 * (a.x + b.x);
    mid.y = 0.5 * (a.y + b.y);
    mid.z = 0.5 * (a.z + b.z);
    return mid;
}

inline double normalizedGapPenalty(double value, double preferred_value) {
    if (preferred_value <= 1e-9 || value >= preferred_value) {
        return 0.0;
    }
    const double gap = (preferred_value - value) / preferred_value;
    return gap * gap;
}

}  // namespace my_cr5_control::paper_mainline

#endif  // MY_CR5_CONTROL_PAPER_MAINLINE_GUIDE_GEOMETRY_HPP
