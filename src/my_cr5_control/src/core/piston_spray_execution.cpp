#include "my_cr5_control/piston_spray_execution.hpp"

#include "my_cr5_control/cr5_robot.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace my_cr5_control::piston {
namespace {

constexpr double kMmToM = 0.001;

double poseDistanceM(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b) {
    const double dx = a.position.x - b.position.x;
    const double dy = a.position.y - b.position.y;
    const double dz = a.position.z - b.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

std::string executionStateToString(ExecutionState state) {
    switch (state) {
        case ExecutionState::Idle:
            return "Idle";
        case ExecutionState::Ready:
            return "Ready";
        case ExecutionState::Running:
            return "Running";
        case ExecutionState::Paused:
            return "Paused";
        case ExecutionState::Completed:
            return "Completed";
        case ExecutionState::Stopped:
            return "Stopped";
        case ExecutionState::EmergencyStopped:
            return "EmergencyStopped";
        case ExecutionState::Fault:
            return "Fault";
    }
    return "Unknown";
}

MockMotionInterface::MockMotionInterface() {
    current_pose_.orientation.w = 1.0;
}

bool MockMotionInterface::ensureReady(std::string* error_message) {
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

bool MockMotionInterface::moveToPose(const geometry_msgs::msg::Pose& flange_pose,
                                     double speed_mm_s,
                                     bool /*linear_motion*/,
                                     std::string* error_message) {
    if (speed_mm_s <= 0.0) {
        if (error_message != nullptr) {
            *error_message = "Mock motion speed must be > 0.";
        }
        return false;
    }

    geometry_msgs::msg::Pose start_pose;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        start_pose = current_pose_;
    }

    const double distance_mm = poseDistanceM(start_pose, flange_pose) / kMmToM;
    const double duration_s = std::clamp(distance_mm / speed_mm_s, 0.05, 2.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(duration_s * 1000.0)));

    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_pose_ = flange_pose;
    }
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

void MockMotionInterface::stopMotion() {
}

geometry_msgs::msg::Pose MockMotionInterface::currentPose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_pose_;
}

bool MockPeripheralInterface::ensureReady(std::string* error_message) {
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

bool MockPeripheralInterface::setTurntableRpm(double rpm, std::string* error_message) {
    current_rpm_ = rpm;
    if (error_message != nullptr) {
        error_message->clear();
    }
    return rpm > 0.0;
}

bool MockPeripheralInterface::startTurntable(std::string* error_message) {
    turntable_running_ = true;
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

bool MockPeripheralInterface::stopTurntable(std::string* error_message) {
    turntable_running_ = false;
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

bool MockPeripheralInterface::openSpray(double flow_rate_ml_min, std::string* error_message) {
    current_flow_rate_ml_min_ = flow_rate_ml_min;
    spray_open_ = true;
    if (error_message != nullptr) {
        error_message->clear();
    }
    return flow_rate_ml_min > 0.0;
}

bool MockPeripheralInterface::closeSpray(std::string* error_message) {
    spray_open_ = false;
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

void MockPeripheralInterface::emergencyStop() {
    spray_open_ = false;
    turntable_running_ = false;
}

CR5MotionInterface::CR5MotionInterface(std::string node_name, bool attach_probe_model)
    : node_name_(std::move(node_name)),
      attach_probe_model_(attach_probe_model) {
}

bool CR5MotionInterface::ensureReady(std::string* error_message) {
    if (initialized_ && robot_ && robot_->isReady()) {
        if (error_message != nullptr) {
            error_message->clear();
        }
        return true;
    }

    robot_ = std::make_shared<CR5Robot>(node_name_, attach_probe_model_);
    if (!robot_->init()) {
        if (error_message != nullptr) {
            *error_message = "CR5Robot initialization failed.";
        }
        robot_.reset();
        initialized_ = false;
        return false;
    }
    initialized_ = true;
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

bool CR5MotionInterface::moveToPose(const geometry_msgs::msg::Pose& flange_pose,
                                    double speed_mm_s,
                                    bool linear_motion,
                                    std::string* error_message) {
    if (!ensureReady(error_message) || !robot_) {
        return false;
    }

    const double scaling = std::clamp(speed_mm_s / 100.0, 0.05, 1.0);
    robot_->setSpeed(scaling);

    const bool success = linear_motion
        ? robot_->moveLine(flange_pose) > 0.95
        : robot_->moveToPose(flange_pose);
    if (!success && error_message != nullptr) {
        *error_message = linear_motion
            ? "CR5 Cartesian motion failed."
            : "CR5 planned motion failed.";
    }
    return success;
}

void CR5MotionInterface::stopMotion() {
    if (robot_) {
        robot_->stopMotion();
    }
}

geometry_msgs::msg::Pose CR5MotionInterface::currentPose() const {
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    if (robot_ && robot_->isReady()) {
        return robot_->getCurrentPose();
    }
    return pose;
}

SprayExecutionEngine::SprayExecutionEngine(
    std::shared_ptr<MotionInterface> motion_interface,
    std::shared_ptr<PeripheralInterface> peripheral_interface)
    : motion_interface_(std::move(motion_interface)),
      peripheral_interface_(std::move(peripheral_interface)) {
}

SprayExecutionEngine::~SprayExecutionEngine() {
    emergencyStop();
    joinFinishedWorker();
}

void SprayExecutionEngine::joinFinishedWorker() {
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool SprayExecutionEngine::loadPlan(const SprayPlan& plan, std::string* error_message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (worker_running_) {
        if (error_message != nullptr) {
            *error_message = "Cannot load a new plan while execution is running.";
        }
        return false;
    }

    plan_ = plan;
    key_pose_map_.clear();
    for (const auto& key_pose : plan.key_poses) {
        key_pose_map_[key_pose.name] = key_pose;
    }
    state_ = ExecutionState::Ready;
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

bool SprayExecutionEngine::start(EventCallback callback, std::string* error_message) {
    joinFinishedWorker();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!plan_.has_value()) {
            if (error_message != nullptr) {
                *error_message = "No spray plan loaded.";
            }
            return false;
        }
        if (worker_running_) {
            if (error_message != nullptr) {
                *error_message = "Execution is already running.";
            }
            return false;
        }
        if (!motion_interface_ || !peripheral_interface_) {
            if (error_message != nullptr) {
                *error_message = "Execution interfaces are not configured.";
            }
            return false;
        }
    }

    std::string readiness_error;
    if (!motion_interface_->ensureReady(&readiness_error)) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = ExecutionState::Fault;
        }
        if (error_message != nullptr) {
            *error_message = readiness_error;
        }
        return false;
    }
    if (!peripheral_interface_->ensureReady(&readiness_error)) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = ExecutionState::Fault;
        }
        if (error_message != nullptr) {
            *error_message = readiness_error;
        }
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        callback_ = std::move(callback);
        pause_requested_ = false;
        stop_requested_ = false;
        emergency_requested_ = false;
        worker_running_ = true;
        state_ = ExecutionState::Running;
        worker_ = std::thread([this]() { workerLoop(); });
    }
    publishEvent(ExecutionState::Running, 0, true, "Execution started.");
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

bool SprayExecutionEngine::pause(std::string* error_message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != ExecutionState::Running) {
        if (error_message != nullptr) {
            *error_message = "Pause is only available while running.";
        }
        return false;
    }
    pause_requested_ = true;
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

bool SprayExecutionEngine::resume(std::string* error_message) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ != ExecutionState::Paused) {
            if (error_message != nullptr) {
                *error_message = "Resume is only available while paused.";
            }
            return false;
        }
        pause_requested_ = false;
    }
    condition_.notify_all();
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

bool SprayExecutionEngine::stop(std::string* error_message) {
    bool stop_before_start = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ != ExecutionState::Running &&
            state_ != ExecutionState::Paused &&
            state_ != ExecutionState::Ready) {
            if (error_message != nullptr) {
                *error_message = "Stop is not available in the current state.";
            }
            return false;
        }
        stop_requested_ = true;
        pause_requested_ = false;
        if (state_ == ExecutionState::Ready) {
            state_ = ExecutionState::Stopped;
            stop_before_start = true;
            if (error_message != nullptr) {
                error_message->clear();
            }
        }
    }
    if (stop_before_start) {
        publishEvent(ExecutionState::Stopped, 0, true, "Execution stopped before start.");
        return true;
    }
    motion_interface_->stopMotion();
    peripheral_interface_->closeSpray(nullptr);
    peripheral_interface_->stopTurntable(nullptr);
    condition_.notify_all();
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

void SprayExecutionEngine::emergencyStop() {
    bool publish_immediately = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        emergency_requested_ = true;
        pause_requested_ = false;
        if (!worker_running_) {
            state_ = ExecutionState::EmergencyStopped;
            publish_immediately = true;
        }
    }
    if (motion_interface_) {
        motion_interface_->stopMotion();
    }
    if (peripheral_interface_) {
        peripheral_interface_->emergencyStop();
    }
    condition_.notify_all();
    if (publish_immediately) {
        publishEvent(ExecutionState::EmergencyStopped, 0, false, "Emergency stop latched.");
    }
}

ExecutionState SprayExecutionEngine::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void SprayExecutionEngine::publishEvent(ExecutionState state,
                                        int current_step_index,
                                        bool success,
                                        const std::string& message) {
    EventCallback callback_copy;
    int total_step_count = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        callback_copy = callback_;
        total_step_count = plan_.has_value()
            ? static_cast<int>(plan_->execution_steps.size())
            : 0;
        state_ = state;
    }

    if (callback_copy) {
        callback_copy(ExecutionEvent{
            state,
            current_step_index,
            total_step_count,
            success,
            message,
        });
    }
}

bool SprayExecutionEngine::executeStep(const ExecutionStep& step, std::string* error_message) {
    if (!plan_.has_value()) {
        if (error_message != nullptr) {
            *error_message = "No plan available during execution.";
        }
        return false;
    }

    if (step.command == "MOVE_NOZZLE") {
        const auto iter = key_pose_map_.find(step.pose_name);
        if (iter == key_pose_map_.end()) {
            if (error_message != nullptr) {
                *error_message = "Missing key pose: " + step.pose_name;
            }
            return false;
        }
        const bool linear_motion = (step.motion_mode == "cartesian");
        return motion_interface_->moveToPose(
            iter->second.flange_pose,
            step.speed_mm_s,
            linear_motion,
            error_message);
    }
    if (step.command == "SET_TURNTABLE_RPM") {
        return peripheral_interface_->setTurntableRpm(step.numeric_value, error_message);
    }
    if (step.command == "START_TURNTABLE") {
        return peripheral_interface_->startTurntable(error_message);
    }
    if (step.command == "STOP_TURNTABLE") {
        return peripheral_interface_->stopTurntable(error_message);
    }
    if (step.command == "OPEN_SPRAY") {
        return peripheral_interface_->openSpray(step.numeric_value, error_message);
    }
    if (step.command == "CLOSE_SPRAY") {
        return peripheral_interface_->closeSpray(error_message);
    }

    if (error_message != nullptr) {
        *error_message = "Unsupported command: " + step.command;
    }
    return false;
}

void SprayExecutionEngine::workerLoop() {
    int current_step_index = 0;
    for (std::size_t index = 0; plan_.has_value() && index < plan_->execution_steps.size(); ++index) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (emergency_requested_) {
                worker_running_ = false;
                lock.unlock();
                publishEvent(ExecutionState::EmergencyStopped,
                             current_step_index,
                             false,
                             "Emergency stop triggered.");
                return;
            }
            if (stop_requested_) {
                worker_running_ = false;
                lock.unlock();
                publishEvent(ExecutionState::Stopped,
                             current_step_index,
                             true,
                             "Execution stopped.");
                return;
            }
            if (pause_requested_) {
                lock.unlock();
                publishEvent(ExecutionState::Paused,
                             current_step_index,
                             true,
                             "Execution paused at step boundary.");
                lock.lock();
                condition_.wait(lock, [this]() {
                    return !pause_requested_ || stop_requested_ || emergency_requested_;
                });
                if (emergency_requested_) {
                    worker_running_ = false;
                    lock.unlock();
                    publishEvent(ExecutionState::EmergencyStopped,
                                 current_step_index,
                                 false,
                                 "Emergency stop triggered while paused.");
                    return;
                }
                if (stop_requested_) {
                    worker_running_ = false;
                    lock.unlock();
                    publishEvent(ExecutionState::Stopped,
                                 current_step_index,
                                 true,
                                 "Execution stopped while paused.");
                    return;
                }
                lock.unlock();
                publishEvent(ExecutionState::Running,
                             current_step_index,
                             true,
                             "Execution resumed.");
            }
        }

        current_step_index = static_cast<int>(index) + 1;
        const auto& step = plan_->execution_steps[index];
        publishEvent(ExecutionState::Running,
                     current_step_index,
                     true,
                     "Executing step " + std::to_string(step.step_index) + ": " + step.command);

        std::string error_message;
        if (!executeStep(step, &error_message)) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                worker_running_ = false;
            }
            publishEvent(ExecutionState::Fault,
                         current_step_index,
                         false,
                         error_message.empty() ? "Step execution failed." : error_message);
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        worker_running_ = false;
    }
    publishEvent(ExecutionState::Completed,
                 plan_.has_value() ? static_cast<int>(plan_->execution_steps.size()) : 0,
                 true,
                 "Execution completed.");
}

}  // namespace my_cr5_control::piston
