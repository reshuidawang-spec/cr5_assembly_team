#ifndef MY_CR5_CONTROL_PROBE_PARAMS_HPP
#define MY_CR5_CONTROL_PROBE_PARAMS_HPP

namespace my_cr5_control::probe {

// 测头主体参数（与 setupCalibrationScene 的碰撞模型保持一致）
inline constexpr double kProbeBodyHeight = 0.076;
inline constexpr double kProbeBodyRadius = 0.0315;
inline constexpr double kProbeStemHeight = 0.050;
inline constexpr double kProbeStemRadius = 0.003;

// 星形测针参数
inline constexpr double kStarStylusLength = 0.040;
inline constexpr double kStarStylusTipRadius = 0.0005;
inline constexpr double kStarStylusZOffset = 0.126;
inline constexpr double kStarStylusReach = 0.020;  // 旋转后等效外扩半径

// 测针尖端参数
inline constexpr double kProbeTipHeight = 0.020;
inline constexpr double kProbeTipRadius = 0.0005;
inline constexpr double kProbeTipZOffset = 0.136;

// 法兰到测针尖端的总长度（用于垂直触碰）
inline constexpr double kProbeLength = 0.146;

}  // namespace my_cr5_control::probe

#endif  // MY_CR5_CONTROL_PROBE_PARAMS_HPP
