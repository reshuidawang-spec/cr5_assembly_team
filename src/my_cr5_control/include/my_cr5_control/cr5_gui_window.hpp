#ifndef CR5_GUI_WINDOW_HPP
#define CR5_GUI_WINDOW_HPP

#include <QMainWindow>
#include <QTextEdit>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QProcess>
#include <QSpinBox>
#include <QTimer>
#include <atomic>
#include <memory>
#include "my_cr5_control/cr5_robot.hpp"

class CR5GuiWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit CR5GuiWindow(QWidget* parent = nullptr);
    ~CR5GuiWindow() override;

private slots:
    void onRecordPoint();
    void onClearPoints();
    void onAddPoint();
    void onRemovePoint();
    void onStartMeasurement();
    void onStartCalibration();
    void onUpdateRobotState();
    void onEmergencyStop();
    void onToggleCalibrationScene();
    void onToggleBox();
    void onRunSimpleBenchmark();
    void onRunV2Benchmark();
    void onRunRandomDataset();
    void onPlotSimpleBenchmark();
    void onPlotSimpleRandomDataset();
    void onExportBenchmarkDataset();
    void onRefreshDatasetManifest();
    void onClearCurrentState();
    void onExternalTaskOutput();
    void onExternalTaskFinished(int exit_code, QProcess::ExitStatus exit_status);
    void onExternalTaskErrorOccurred(QProcess::ProcessError error);

private:
    void setupUI();
    QGroupBox* createPointInputGroup();
    QGroupBox* createControlGroup();
    QGroupBox* createAutomationGroup();
    QGroupBox* createStatusGroup();
    QGroupBox* createLogGroup();
    void appendLog(const QString& message, const QString& level = "INFO");
    void appendLogAsync(const QString& message, const QString& level = "INFO");
    bool validatePointInput(int index);
    void clearTeachPoints();
    void rebuildPointRows();
    void updateActionAvailability();
    bool isBusy() const;
    bool startExternalTask(const QString& task_name,
                           const QString& program,
                           const QStringList& arguments,
                           const QList<QPair<QString, QString>>& env_overrides = {});
    void stopExternalTask(bool force_kill = false);
    void resetUiState(bool clear_logs);

    // TCP标定相关
    struct CalibrationPoint {
        std::string name;
        geometry_msgs::msg::Pose target_pose;
        geometry_msgs::msg::Pose approach_pose;
    };
    geometry_msgs::msg::Quaternion calculateLookAt(
        const geometry_msgs::msg::Point& source,
        const geometry_msgs::msg::Point& target);
    geometry_msgs::msg::Quaternion getVerticalDownOrientation();
    std::vector<CalibrationPoint> generateConePoints();

    // ROS相关
    std::shared_ptr<CR5Robot> robot_;
    QTimer* state_timer_;

    // 界面组件
    QTextEdit* log_text_;

    // 点坐标输入（动态列表）
    std::vector<QLineEdit*> point_x_;
    std::vector<QLineEdit*> point_y_;
    std::vector<QLineEdit*> point_z_;
    std::vector<QPushButton*> record_btn_;
    std::vector<QLabel*> point_status_;
    QPushButton* add_point_btn_;
    QPushButton* remove_point_btn_;
    QGridLayout* point_grid_layout_;

    // 控制按钮
    QPushButton* clear_btn_;
    QPushButton* start_measure_btn_;
    QPushButton* start_calib_btn_;
    QPushButton* emergency_stop_btn_;
    QPushButton* toggle_calib_scene_btn_;
    QPushButton* toggle_box_btn_;
    QPushButton* clear_state_btn_;

    // 自动化测试 / 绘图
    QSpinBox* benchmark_repeats_spin_;
    QSpinBox* random_task_count_spin_;
    QPushButton* run_simple_benchmark_btn_;
    QPushButton* run_v2_benchmark_btn_;
    QPushButton* run_random_dataset_btn_;
    QPushButton* plot_simple_benchmark_btn_;
    QPushButton* plot_simple_random_btn_;
    QPushButton* export_benchmark_dataset_btn_;
    QPushButton* refresh_manifest_btn_;
    QLabel* automation_status_label_;

    // 状态显示
    QLabel* robot_status_label_;
    QLabel* current_pose_label_;
    QLabel* connection_status_label_;

    // 数据存储
    struct TeachPoint {
        double x, y, z;
        bool recorded;
    };
    std::vector<TeachPoint> teach_points_;
    int recorded_count_;
    std::atomic_bool is_running_;
    bool robot_connected_;
    bool calibration_scene_visible_;
    bool box_visible_;
    QProcess* task_process_;
    QString current_external_task_name_;
    bool suppress_external_task_notifications_;
};

#endif // CR5_GUI_WINDOW_HPP
