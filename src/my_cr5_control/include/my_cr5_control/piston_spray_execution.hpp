#ifndef MY_CR5_CONTROL_PISTON_SPRAY_EXECUTION_HPP
#define MY_CR5_CONTROL_PISTON_SPRAY_EXECUTION_HPP

#include "my_cr5_control/piston_spray_planner.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

class CR5Robot;

namespace my_cr5_control::piston {

enum class ExecutionState {
    Idle,
    Ready,
    Running,
    Paused,
    Completed,
    Stopped,
    EmergencyStopped,
    Fault,
};

std::string executionStateToString(ExecutionState state);

struct ExecutionEvent {
    ExecutionState state{ExecutionState::Idle};
    int current_step_index{0};
    int total_step_count{0};
    bool success{true};
    std::string message;
};

class MotionInterface {
public:
    virtual ~MotionInterface() = default;

    virtual bool ensureReady(std::string* error_message) = 0;
    virtual bool moveToPose(const geometry_msgs::msg::Pose& flange_pose,
                            double speed_mm_s,
                            bool linear_motion,
                            std::string* error_message) = 0;
    virtual void stopMotion() = 0;
    virtual geometry_msgs::msg::Pose currentPose() const = 0;
};

class PeripheralInterface {
public:
    virtual ~PeripheralInterface() = default;

    virtual bool ensureReady(std::string* error_message) = 0;
    virtual bool setTurntableRpm(double rpm, std::string* error_message) = 0;
    virtual bool startTurntable(std::string* error_message) = 0;
    virtual bool stopTurntable(std::string* error_message) = 0;
    virtual bool openSpray(double flow_rate_ml_min, std::string* error_message) = 0;
    virtual bool closeSpray(std::string* error_message) = 0;
    virtual void emergencyStop() = 0;
};

class MockMotionInterface final : public MotionInterface {
public:
    MockMotionInterface();

    bool ensureReady(std::string* error_message) override;
    bool moveToPose(const geometry_msgs::msg::Pose& flange_pose,
                    double speed_mm_s,
                    bool linear_motion,
                    std::string* error_message) override;
    void stopMotion() override;
    geometry_msgs::msg::Pose currentPose() const override;

private:
    mutable std::mutex mutex_;
    geometry_msgs::msg::Pose current_pose_;
};

class MockPeripheralInterface final : public PeripheralInterface {
public:
    bool ensureReady(std::string* error_message) override;
    bool setTurntableRpm(double rpm, std::string* error_message) override;
    bool startTurntable(std::string* error_message) override;
    bool stopTurntable(std::string* error_message) override;
    bool openSpray(double flow_rate_ml_min, std::string* error_message) override;
    bool closeSpray(std::string* error_message) override;
    void emergencyStop() override;

private:
    double current_rpm_{0.0};
    double current_flow_rate_ml_min_{0.0};
    bool turntable_running_{false};
    bool spray_open_{false};
};

class CR5MotionInterface final : public MotionInterface {
public:
    explicit CR5MotionInterface(std::string node_name = "piston_spray_executor",
                                bool attach_probe_model = true);

    bool ensureReady(std::string* error_message) override;
    bool moveToPose(const geometry_msgs::msg::Pose& flange_pose,
                    double speed_mm_s,
                    bool linear_motion,
                    std::string* error_message) override;
    void stopMotion() override;
    geometry_msgs::msg::Pose currentPose() const override;

private:
    std::string node_name_;
    bool attach_probe_model_{true};
    std::shared_ptr<CR5Robot> robot_;
    bool initialized_{false};
};

class SprayExecutionEngine {
public:
    using EventCallback = std::function<void(const ExecutionEvent&)>;

    SprayExecutionEngine(std::shared_ptr<MotionInterface> motion_interface,
                         std::shared_ptr<PeripheralInterface> peripheral_interface);
    ~SprayExecutionEngine();

    bool loadPlan(const SprayPlan& plan, std::string* error_message);
    bool start(EventCallback callback, std::string* error_message);
    bool pause(std::string* error_message);
    bool resume(std::string* error_message);
    bool stop(std::string* error_message);
    void emergencyStop();

    ExecutionState state() const;

private:
    bool executeStep(const ExecutionStep& step, std::string* error_message);
    void publishEvent(ExecutionState state,
                      int current_step_index,
                      bool success,
                      const std::string& message);
    void workerLoop();
    void joinFinishedWorker();

    std::shared_ptr<MotionInterface> motion_interface_;
    std::shared_ptr<PeripheralInterface> peripheral_interface_;

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::thread worker_;

    std::optional<SprayPlan> plan_;
    std::unordered_map<std::string, KeyPose> key_pose_map_;
    EventCallback callback_;
    ExecutionState state_{ExecutionState::Idle};

    bool worker_running_{false};
    bool pause_requested_{false};
    bool stop_requested_{false};
    bool emergency_requested_{false};
};

}  // namespace my_cr5_control::piston

#endif  // MY_CR5_CONTROL_PISTON_SPRAY_EXECUTION_HPP
