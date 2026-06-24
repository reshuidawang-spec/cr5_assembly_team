#ifndef MY_CR5_CONTROL_PISTON_SPRAY_GUI_WINDOW_HPP
#define MY_CR5_CONTROL_PISTON_SPRAY_GUI_WINDOW_HPP

#include <QDoubleSpinBox>
#include <QComboBox>
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTextEdit>
#include <QLabel>
#include <QWidget>

#include <optional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "my_cr5_control/cr5_robot.hpp"
#include "my_cr5_control/piston_spray_execution.hpp"
#include "my_cr5_control/piston_spray_planner.hpp"

class PistonSprayGuiWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit PistonSprayGuiWindow(QWidget* parent = nullptr);
    ~PistonSprayGuiWindow() override;

private slots:
    void onGeneratePlan();
    void onLoadJobJson();
    void onExportPlanJson();
    void onStartExecution();
    void onPauseResumeExecution();
    void onStopExecution();
    void onEmergencyStopExecution();
    void onGenerateRvizPiston();
    void onClearRvizPiston();
    void onPublishWorkpieceFrame();
    void onClearWorkpieceFrame();
    void onCaptureTeachPose();
    void onMoveToAdjustedTeachPose();
    void onResetTeachOffsets();

private:
    void setupUi();
    QWidget* createControlPanel();
    void appendLog(const QString& message, const QString& level = "INFO");
    void resetToDefaults();
    void applyLoadedJson(const QByteArray& content);
    void updateSummary();
    void updatePoseTable();
    void updateExecutionUi();
    void updateManualTeachUi();
    void updateWorkpieceFramePreview();
    void updateAdjustedTeachPose();
    void saveTeachState() const;
    void loadTeachState();
    void ensureExecutionBackend();
    bool ensureManualTeachBackend(std::string* error_message);
    bool ensureRvizSceneRobot(std::string* error_message);
    bool ensureWorkpieceMarkerPublisher(std::string* error_message);
    bool publishWorkpieceFrameMarkers(const my_cr5_control::piston::PistonSpecMm& piston,
                                      const my_cr5_control::piston::WorkpieceFrame& frame,
                                      double reference_distance_mm,
                                      std::string* error_message);
    void clearWorkpieceFrameMarkers();
    void handleExecutionEvent(const my_cr5_control::piston::ExecutionEvent& event);
    bool collectInputs(my_cr5_control::piston::PistonSpecMm* piston,
                       my_cr5_control::piston::SprayProcessMm* process,
                       my_cr5_control::piston::WorkpieceFrame* frame,
                       my_cr5_control::piston::ToolTcpOffsetMm* tool_offset) const;

    QString jobName() const;
    QString defaultExportPath() const;

    QLineEdit* job_name_edit_{nullptr};

    QDoubleSpinBox* piston_diameter_spin_{nullptr};
    QDoubleSpinBox* spray_length_spin_{nullptr};
    QDoubleSpinBox* spray_distance_spin_{nullptr};
    QDoubleSpinBox* spray_width_spin_{nullptr};
    QDoubleSpinBox* overlap_spin_{nullptr};
    QDoubleSpinBox* turntable_rpm_spin_{nullptr};
    QDoubleSpinBox* flow_rate_spin_{nullptr};
    QDoubleSpinBox* lead_in_spin_{nullptr};
    QDoubleSpinBox* lead_out_spin_{nullptr};
    QDoubleSpinBox* radial_clearance_spin_{nullptr};
    QDoubleSpinBox* approach_speed_spin_{nullptr};
    QSpinBox* sample_count_spin_{nullptr};

    QDoubleSpinBox* frame_origin_x_spin_{nullptr};
    QDoubleSpinBox* frame_origin_y_spin_{nullptr};
    QDoubleSpinBox* frame_origin_z_spin_{nullptr};
    QDoubleSpinBox* axial_x_spin_{nullptr};
    QDoubleSpinBox* axial_y_spin_{nullptr};
    QDoubleSpinBox* axial_z_spin_{nullptr};
    QDoubleSpinBox* radial_x_spin_{nullptr};
    QDoubleSpinBox* radial_y_spin_{nullptr};
    QDoubleSpinBox* radial_z_spin_{nullptr};
    QDoubleSpinBox* tool_roll_spin_{nullptr};

    QDoubleSpinBox* tcp_offset_x_spin_{nullptr};
    QDoubleSpinBox* tcp_offset_y_spin_{nullptr};
    QDoubleSpinBox* tcp_offset_z_spin_{nullptr};
    QLabel* teach_live_pose_label_{nullptr};
    QLabel* teach_saved_pose_label_{nullptr};
    QLabel* teach_target_pose_label_{nullptr};
    QDoubleSpinBox* teach_delta_x_spin_{nullptr};
    QDoubleSpinBox* teach_delta_y_spin_{nullptr};
    QDoubleSpinBox* teach_delta_z_spin_{nullptr};
    QDoubleSpinBox* teach_delta_roll_spin_{nullptr};
    QDoubleSpinBox* teach_delta_pitch_spin_{nullptr};
    QDoubleSpinBox* teach_delta_yaw_spin_{nullptr};
    QDoubleSpinBox* teach_move_speed_spin_{nullptr};
    QComboBox* teach_motion_mode_combo_{nullptr};
    QPushButton* capture_teach_pose_btn_{nullptr};
    QPushButton* reset_teach_offset_btn_{nullptr};
    QPushButton* move_adjusted_teach_pose_btn_{nullptr};
    QDoubleSpinBox* rviz_piston_distance_spin_{nullptr};
    QLabel* workpiece_frame_preview_label_{nullptr};
    QPushButton* publish_workpiece_frame_btn_{nullptr};
    QPushButton* clear_workpiece_frame_btn_{nullptr};
    QLabel* workpiece_frame_status_label_{nullptr};
    QPushButton* generate_rviz_piston_btn_{nullptr};
    QPushButton* clear_rviz_piston_btn_{nullptr};
    QLabel* rviz_piston_status_label_{nullptr};

    QTextEdit* summary_text_{nullptr};
    QTextEdit* log_text_{nullptr};
    QTableWidget* pose_table_{nullptr};
    QWidget* preview_widget_{nullptr};
    QComboBox* execution_backend_combo_{nullptr};
    QPushButton* start_execution_btn_{nullptr};
    QPushButton* pause_resume_btn_{nullptr};
    QPushButton* stop_execution_btn_{nullptr};
    QPushButton* emergency_stop_btn_{nullptr};
    QLabel* execution_state_label_{nullptr};
    QLabel* execution_backend_label_{nullptr};

    std::optional<my_cr5_control::piston::SprayPlan> current_plan_;
    std::optional<geometry_msgs::msg::Pose> current_tcp_pose_;
    std::optional<geometry_msgs::msg::Pose> saved_teach_tcp_pose_;
    std::optional<geometry_msgs::msg::Pose> adjusted_teach_tcp_pose_;
    std::shared_ptr<CR5Robot> rviz_scene_robot_;
    std::shared_ptr<my_cr5_control::piston::MotionInterface> manual_motion_interface_;
    std::shared_ptr<my_cr5_control::piston::MotionInterface> motion_interface_;
    std::shared_ptr<my_cr5_control::piston::PeripheralInterface> peripheral_interface_;
    std::shared_ptr<my_cr5_control::piston::SprayExecutionEngine> execution_engine_;
    rclcpp::Node::SharedPtr workpiece_marker_node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr workpiece_marker_pub_;
    int execution_backend_index_{-1};
    bool suppress_teach_state_save_{false};
};

#endif  // MY_CR5_CONTROL_PISTON_SPRAY_GUI_WINDOW_HPP
