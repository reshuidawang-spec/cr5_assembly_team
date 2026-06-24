#include "my_cr5_control/cr5_gui_window.hpp"
#include "my_cr5_control/probe_params.hpp"
#include "my_cr5_control/result_utils.hpp"
#include "my_cr5_control/scene_utils.hpp"
#include <QMessageBox>
#include <QApplication>
#include <QDateTime>
#include <QScrollBar>
#include <QDir>
#include <QProcessEnvironment>
#include <fstream>
#include <thread>
#include <cmath>
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

CR5GuiWindow::CR5GuiWindow(QWidget* parent)
    : QMainWindow(parent), recorded_count_(0), is_running_(false), robot_connected_(false),
      calibration_scene_visible_(true), box_visible_(true), task_process_(nullptr),
      suppress_external_task_notifications_(false) {

    // 初始化3个默认示教点
    for (int i = 0; i < 3; ++i) {
        teach_points_.push_back({0.0, 0.0, 0.0, false});
    }

    robot_ = std::make_shared<CR5Robot>("cr5_gui_robot");

    setupUI();

    task_process_ = new QProcess(this);
    task_process_->setProcessChannelMode(QProcess::MergedChannels);
    connect(task_process_, &QProcess::readyRead, this, &CR5GuiWindow::onExternalTaskOutput);
    connect(task_process_,
            qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this,
            &CR5GuiWindow::onExternalTaskFinished);
    connect(task_process_,
            &QProcess::errorOccurred,
            this,
            &CR5GuiWindow::onExternalTaskErrorOccurred);

    // 初始化机器人
    appendLog("正在初始化机器人...", "INFO");
    if (robot_->init()) {
        robot_connected_ = true;
        appendLog("✓ 机器人初始化成功", "SUCCESS");
        robot_->addSimulationEnvironment();
        appendLog("✓ 仿真环境加载完成", "SUCCESS");
        connection_status_label_->setText("状态: 已连接");
        connection_status_label_->setStyleSheet("color: green; font-weight: bold;");
    } else {
        appendLog("✗ 机器人初始化失败", "ERROR");
        connection_status_label_->setText("状态: 连接失败");
        connection_status_label_->setStyleSheet("color: red; font-weight: bold;");
    }

    // 状态更新定时器
    state_timer_ = new QTimer(this);
    connect(state_timer_, &QTimer::timeout, this, &CR5GuiWindow::onUpdateRobotState);
    state_timer_->start(500);

    updateActionAvailability();
}

CR5GuiWindow::~CR5GuiWindow() {
    is_running_.store(false);
    stopExternalTask(true);
    if (state_timer_) state_timer_->stop();
}

void CR5GuiWindow::setupUI() {
    setWindowTitle("CR5 机器人控制系统");
    resize(1000, 700);

    QWidget* central = new QWidget(this);
    setCentralWidget(central);

    QHBoxLayout* main_layout = new QHBoxLayout(central);

    // 左侧：控制面板
    QVBoxLayout* left_layout = new QVBoxLayout();
    left_layout->addWidget(createPointInputGroup());
    left_layout->addWidget(createControlGroup());
    left_layout->addWidget(createAutomationGroup());
    left_layout->addWidget(createStatusGroup());
    left_layout->addStretch();

    // 右侧：日志显示
    QVBoxLayout* right_layout = new QVBoxLayout();
    right_layout->addWidget(createLogGroup());

    main_layout->addLayout(left_layout, 1);
    main_layout->addLayout(right_layout, 2);
}

QGroupBox* CR5GuiWindow::createPointInputGroup() {
    QGroupBox* group = new QGroupBox("示教点坐标输入");
    QVBoxLayout* outer = new QVBoxLayout(group);

    // 表头
    QGridLayout* header = new QGridLayout();
    header->addWidget(new QLabel("<b>点</b>"), 0, 0);
    header->addWidget(new QLabel("<b>X (m)</b>"), 0, 1);
    header->addWidget(new QLabel("<b>Y (m)</b>"), 0, 2);
    header->addWidget(new QLabel("<b>Z (m)</b>"), 0, 3);
    header->addWidget(new QLabel("<b>操作</b>"), 0, 4);
    header->addWidget(new QLabel("<b>状态</b>"), 0, 5);
    outer->addLayout(header);

    // 动态行区域
    point_grid_layout_ = new QGridLayout();
    outer->addLayout(point_grid_layout_);

    // 添加/删除按钮
    QHBoxLayout* btn_row = new QHBoxLayout();
    add_point_btn_ = new QPushButton("+ 添加点");
    add_point_btn_->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 6px;");
    connect(add_point_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onAddPoint);
    btn_row->addWidget(add_point_btn_);

    remove_point_btn_ = new QPushButton("- 移除最后点");
    remove_point_btn_->setStyleSheet("background-color: #F44336; color: white; font-weight: bold; padding: 6px;");
    connect(remove_point_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onRemovePoint);
    btn_row->addWidget(remove_point_btn_);
    btn_row->addStretch();
    outer->addLayout(btn_row);

    // 初始行
    rebuildPointRows();

    return group;
}

QGroupBox* CR5GuiWindow::createControlGroup() {
    QGroupBox* group = new QGroupBox("控制面板");
    QVBoxLayout* layout = new QVBoxLayout(group);

    clear_btn_ = new QPushButton("清除示教点");
    clear_btn_->setStyleSheet("background-color: #FFA500; color: white; font-weight: bold; padding: 8px;");
    connect(clear_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onClearPoints);
    layout->addWidget(clear_btn_);

    start_measure_btn_ = new QPushButton("开始箱体测量");
    start_measure_btn_->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 12px; font-size: 14px;");
    start_measure_btn_->setEnabled(false);
    connect(start_measure_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onStartMeasurement);
    layout->addWidget(start_measure_btn_);

    start_calib_btn_ = new QPushButton("开始TCP标定");
    start_calib_btn_->setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 12px; font-size: 14px;");
    connect(start_calib_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onStartCalibration);
    layout->addWidget(start_calib_btn_);

    emergency_stop_btn_ = new QPushButton("紧急停止");
    emergency_stop_btn_->setStyleSheet("background-color: #F44336; color: white; font-weight: bold; padding: 12px; font-size: 14px;");
    emergency_stop_btn_->setEnabled(false);
    connect(emergency_stop_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onEmergencyStop);
    layout->addWidget(emergency_stop_btn_);

    toggle_calib_scene_btn_ = new QPushButton("清除标定杆");
    toggle_calib_scene_btn_->setStyleSheet("background-color: #9C27B0; color: white; font-weight: bold; padding: 8px;");
    connect(toggle_calib_scene_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onToggleCalibrationScene);
    layout->addWidget(toggle_calib_scene_btn_);

    toggle_box_btn_ = new QPushButton("清除箱体");
    toggle_box_btn_->setStyleSheet("background-color: #FF9800; color: white; font-weight: bold; padding: 8px;");
    connect(toggle_box_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onToggleBox);
    layout->addWidget(toggle_box_btn_);

    clear_state_btn_ = new QPushButton("清除当前所有状态");
    clear_state_btn_->setStyleSheet("background-color: #607D8B; color: white; font-weight: bold; padding: 8px;");
    connect(clear_state_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onClearCurrentState);
    layout->addWidget(clear_state_btn_);

    return group;
}

QGroupBox* CR5GuiWindow::createAutomationGroup() {
    QGroupBox* group = new QGroupBox("测试与绘图");
    QVBoxLayout* layout = new QVBoxLayout(group);

    QGridLayout* config_layout = new QGridLayout();
    config_layout->addWidget(new QLabel("Benchmark重复次数"), 0, 0);
    benchmark_repeats_spin_ = new QSpinBox();
    benchmark_repeats_spin_->setRange(1, 100);
    benchmark_repeats_spin_->setValue(10);
    config_layout->addWidget(benchmark_repeats_spin_, 0, 1);

    config_layout->addWidget(new QLabel("随机任务数"), 1, 0);
    random_task_count_spin_ = new QSpinBox();
    random_task_count_spin_->setRange(10, 2000);
    random_task_count_spin_->setSingleStep(10);
    random_task_count_spin_->setValue(300);
    config_layout->addWidget(random_task_count_spin_, 1, 1);
    layout->addLayout(config_layout);

    run_simple_benchmark_btn_ = new QPushButton("运行 Simple Benchmark");
    run_simple_benchmark_btn_->setStyleSheet("background-color: #3F51B5; color: white; font-weight: bold; padding: 8px;");
    connect(run_simple_benchmark_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onRunSimpleBenchmark);
    layout->addWidget(run_simple_benchmark_btn_);

    run_v2_benchmark_btn_ = new QPushButton("运行 V2 Benchmark");
    run_v2_benchmark_btn_->setStyleSheet("background-color: #5C6BC0; color: white; font-weight: bold; padding: 8px;");
    connect(run_v2_benchmark_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onRunV2Benchmark);
    layout->addWidget(run_v2_benchmark_btn_);

    run_random_dataset_btn_ = new QPushButton("采集 Simple 随机任务");
    run_random_dataset_btn_->setStyleSheet("background-color: #00897B; color: white; font-weight: bold; padding: 8px;");
    connect(run_random_dataset_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onRunRandomDataset);
    layout->addWidget(run_random_dataset_btn_);

    plot_simple_benchmark_btn_ = new QPushButton("绘制 Simple Benchmark 图");
    plot_simple_benchmark_btn_->setStyleSheet("background-color: #6A1B9A; color: white; font-weight: bold; padding: 8px;");
    connect(plot_simple_benchmark_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onPlotSimpleBenchmark);
    layout->addWidget(plot_simple_benchmark_btn_);

    plot_simple_random_btn_ = new QPushButton("绘制 Simple Random 图");
    plot_simple_random_btn_->setStyleSheet("background-color: #8E24AA; color: white; font-weight: bold; padding: 8px;");
    connect(plot_simple_random_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onPlotSimpleRandomDataset);
    layout->addWidget(plot_simple_random_btn_);

    export_benchmark_dataset_btn_ = new QPushButton("导出 Benchmark 训练表");
    export_benchmark_dataset_btn_->setStyleSheet("background-color: #3949AB; color: white; font-weight: bold; padding: 8px;");
    connect(export_benchmark_dataset_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onExportBenchmarkDataset);
    layout->addWidget(export_benchmark_dataset_btn_);

    refresh_manifest_btn_ = new QPushButton("刷新 Dataset Manifest");
    refresh_manifest_btn_->setStyleSheet("background-color: #546E7A; color: white; font-weight: bold; padding: 8px;");
    connect(refresh_manifest_btn_, &QPushButton::clicked, this, &CR5GuiWindow::onRefreshDatasetManifest);
    layout->addWidget(refresh_manifest_btn_);

    automation_status_label_ = new QLabel("自动任务状态: 空闲");
    automation_status_label_->setWordWrap(true);
    layout->addWidget(automation_status_label_);

    return group;
}

QGroupBox* CR5GuiWindow::createStatusGroup() {
    QGroupBox* group = new QGroupBox("机器人状态");
    QVBoxLayout* layout = new QVBoxLayout(group);

    connection_status_label_ = new QLabel("状态: 初始化中...");
    layout->addWidget(connection_status_label_);

    robot_status_label_ = new QLabel("运行状态: 空闲");
    layout->addWidget(robot_status_label_);

    current_pose_label_ = new QLabel("当前位姿: --");
    current_pose_label_->setWordWrap(true);
    layout->addWidget(current_pose_label_);

    return group;
}

QGroupBox* CR5GuiWindow::createLogGroup() {
    QGroupBox* group = new QGroupBox("运行日志");
    QVBoxLayout* layout = new QVBoxLayout(group);

    log_text_ = new QTextEdit();
    log_text_->setReadOnly(true);
    log_text_->setStyleSheet("background-color: #1E1E1E; color: #D4D4D4; font-family: 'Courier New'; font-size: 10pt;");
    layout->addWidget(log_text_);

    return group;
}

void CR5GuiWindow::appendLog(const QString& message, const QString& level) {
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString color = "white";

    if (level == "SUCCESS") color = "#4CAF50";
    else if (level == "ERROR") color = "#F44336";
    else if (level == "WARN") color = "#FFA500";
    else if (level == "INFO") color = "#2196F3";

    QString html = QString("<span style='color: gray;'>[%1]</span> "
                          "<span style='color: %2; font-weight: bold;'>[%3]</span> "
                          "<span style='color: white;'>%4</span>")
                      .arg(timestamp, color, level, message);

    log_text_->append(html);
    log_text_->verticalScrollBar()->setValue(log_text_->verticalScrollBar()->maximum());
}

void CR5GuiWindow::appendLogAsync(const QString& message, const QString& level) {
    QMetaObject::invokeMethod(
        this,
        [this, message, level]() { appendLog(message, level); },
        Qt::QueuedConnection);
}

void CR5GuiWindow::clearTeachPoints() {
    for (size_t i = 0; i < teach_points_.size(); ++i) {
        teach_points_[i].recorded = false;
    }
    recorded_count_ = 0;
    rebuildPointRows();
}

bool CR5GuiWindow::isBusy() const {
    return is_running_.load() ||
           (task_process_ != nullptr && task_process_->state() != QProcess::NotRunning);
}

void CR5GuiWindow::updateActionAvailability() {
    const bool busy = isBusy();
    const bool robot_available = robot_connected_;

    for (size_t i = 0; i < teach_points_.size(); ++i) {
        const bool editable = robot_available && !busy && !teach_points_[i].recorded;
        if (i < record_btn_.size()) record_btn_[i]->setEnabled(editable);
        if (i < point_x_.size()) point_x_[i]->setEnabled(editable);
        if (i < point_y_.size()) point_y_[i]->setEnabled(editable);
        if (i < point_z_.size()) point_z_[i]->setEnabled(editable);
    }

    clear_btn_->setEnabled(!busy);
    start_measure_btn_->setEnabled(robot_available && !busy && recorded_count_ >= 1);
    start_calib_btn_->setEnabled(robot_available && !busy);
    add_point_btn_->setEnabled(!busy);
    remove_point_btn_->setEnabled(!busy && teach_points_.size() > 1);
    emergency_stop_btn_->setEnabled(busy);
    toggle_calib_scene_btn_->setEnabled(robot_available && !busy);
    toggle_box_btn_->setEnabled(robot_available && !busy);
    clear_state_btn_->setEnabled(true);

    benchmark_repeats_spin_->setEnabled(!busy);
    random_task_count_spin_->setEnabled(!busy);
    run_simple_benchmark_btn_->setEnabled(robot_available && !busy);
    run_v2_benchmark_btn_->setEnabled(robot_available && !busy);
    run_random_dataset_btn_->setEnabled(robot_available && !busy);
    plot_simple_benchmark_btn_->setEnabled(!busy);
    plot_simple_random_btn_->setEnabled(!busy);
    export_benchmark_dataset_btn_->setEnabled(!busy);
    refresh_manifest_btn_->setEnabled(!busy);
}

bool CR5GuiWindow::startExternalTask(
    const QString& task_name,
    const QString& program,
    const QStringList& arguments,
    const QList<QPair<QString, QString>>& env_overrides) {
    if (isBusy()) {
        QMessageBox::warning(this, "任务繁忙", "当前已有任务在运行，请先停止或等待完成。");
        return false;
    }

    current_external_task_name_ = task_name;
    automation_status_label_->setText(QString("自动任务状态: 运行中 (%1)").arg(task_name));
    automation_status_label_->setStyleSheet("color: orange; font-weight: bold;");

    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert("PYTHONUNBUFFERED", "1");
    for (const auto& pair : env_overrides) {
        env.insert(pair.first, pair.second);
    }
    task_process_->setProcessEnvironment(env);
    task_process_->setWorkingDirectory(QStringLiteral(MY_CR5_CONTROL_SOURCE_DIR));

    appendLog(
        QString("启动外部任务: %1 | 命令: %2 %3")
            .arg(task_name, program, arguments.join(" ")),
        "INFO");

    task_process_->start(program, arguments);
    if (!task_process_->waitForStarted(3000)) {
        automation_status_label_->setText("自动任务状态: 启动失败");
        automation_status_label_->setStyleSheet("color: red; font-weight: bold;");
        appendLog(QString("外部任务启动失败: %1").arg(task_name), "ERROR");
        current_external_task_name_.clear();
        updateActionAvailability();
        return false;
    }

    updateActionAvailability();
    return true;
}

void CR5GuiWindow::stopExternalTask(bool force_kill) {
    if (task_process_ == nullptr || task_process_->state() == QProcess::NotRunning) {
        return;
    }

    if (force_kill) {
        suppress_external_task_notifications_ = true;
        task_process_->kill();
        task_process_->waitForFinished(1000);
        return;
    }

    task_process_->terminate();
    if (!task_process_->waitForFinished(2000)) {
        task_process_->kill();
        task_process_->waitForFinished(1000);
    }
}

void CR5GuiWindow::resetUiState(bool clear_logs) {
    clearTeachPoints();

    if (clear_logs) {
        log_text_->clear();
    }

    if (robot_connected_ && robot_) {
        if (!calibration_scene_visible_) {
            robot_->restoreCalibrationScene();
            calibration_scene_visible_ = true;
        }
        if (!box_visible_) {
            robot_->restoreBox();
            box_visible_ = true;
        }
    }

    toggle_calib_scene_btn_->setText("清除标定杆");
    toggle_box_btn_->setText("清除箱体");

    robot_status_label_->setText("运行状态: 空闲");
    robot_status_label_->setStyleSheet("color: green; font-weight: bold;");
    automation_status_label_->setText("自动任务状态: 空闲");
    automation_status_label_->setStyleSheet("");

    if (!robot_connected_) {
        connection_status_label_->setText("状态: 连接失败");
        connection_status_label_->setStyleSheet("color: red; font-weight: bold;");
    }

    updateActionAvailability();
}

bool CR5GuiWindow::validatePointInput(int index) {
    bool ok_x, ok_y, ok_z;
    double x = point_x_[index]->text().toDouble(&ok_x);
    double y = point_y_[index]->text().toDouble(&ok_y);
    double z = point_z_[index]->text().toDouble(&ok_z);

    if (!ok_x || !ok_y || !ok_z) {
        QMessageBox::warning(this, "输入错误", QString("点 P%1 的坐标格式不正确！").arg(index + 1));
        return false;
    }

    if (z < 0.05 || z > 1.0) {
        QMessageBox::warning(this, "输入错误", QString("点 P%1 的Z坐标超出合理范围 (0.05-1.0m)！").arg(index + 1));
        return false;
    }

    teach_points_[index].x = x;
    teach_points_[index].y = y;
    teach_points_[index].z = z;
    return true;
}

void CR5GuiWindow::onRecordPoint() {
    QPushButton* btn = qobject_cast<QPushButton*>(sender());
    int index = -1;

    for (size_t i = 0; i < record_btn_.size(); ++i) {
        if (btn == record_btn_[i]) {
            index = static_cast<int>(i);
            break;
        }
    }

    if (index == -1) return;

    if (validatePointInput(index)) {
        teach_points_[index].recorded = true;
        point_status_[index]->setText("✓ 已记录");
        point_status_[index]->setStyleSheet("color: green; font-weight: bold;");

        // 禁用已记录行的输入
        point_x_[index]->setEnabled(false);
        point_y_[index]->setEnabled(false);
        point_z_[index]->setEnabled(false);
        record_btn_[index]->setEnabled(false);

        recorded_count_++;
        appendLog(QString("已记录 P%1: (%.4f, %.4f, %.4f)")
                     .arg(index + 1)
                     .arg(teach_points_[index].x)
                     .arg(teach_points_[index].y)
                     .arg(teach_points_[index].z), "SUCCESS");

        if (recorded_count_ >= 1) {
            appendLog(QString("已记录 %1 个示教点，可以开始测量").arg(recorded_count_), "INFO");
        }
        updateActionAvailability();
    }
}

void CR5GuiWindow::onClearPoints() {
    clearTeachPoints();
    updateActionAvailability();
    appendLog("已清除所有示教点", "INFO");
}

void CR5GuiWindow::onAddPoint() {
    teach_points_.push_back({0.0, 0.0, 0.0, false});
    rebuildPointRows();
    updateActionAvailability();
    appendLog(QString("已添加 P%1").arg(teach_points_.size()), "INFO");
}

void CR5GuiWindow::onRemovePoint() {
    if (teach_points_.size() <= 1) return;

    // 如果最后一点已记录，减少计数
    if (teach_points_.back().recorded) {
        recorded_count_--;
    }
    teach_points_.pop_back();
    rebuildPointRows();
    updateActionAvailability();
    appendLog(QString("已移除 P%1").arg(teach_points_.size() + 1), "INFO");
}

void CR5GuiWindow::rebuildPointRows() {
    // 清除旧行控件
    point_x_.clear();
    point_y_.clear();
    point_z_.clear();
    record_btn_.clear();
    point_status_.clear();

    // 清除布局中的旧控件
    QLayoutItem* item;
    while ((item = point_grid_layout_->takeAt(0)) != nullptr) {
        if (item->widget()) {
            item->widget()->deleteLater();
        }
        delete item;
    }

    // 重建行
    for (size_t i = 0; i < teach_points_.size(); ++i) {
        auto* label = new QLabel(QString("P%1").arg(i + 1));
        point_grid_layout_->addWidget(label, static_cast<int>(i), 0);

        auto* x_edit = new QLineEdit();
        x_edit->setPlaceholderText("0.500");
        if (teach_points_[i].recorded) {
            x_edit->setText(QString::number(teach_points_[i].x, 'f', 4));
        }
        point_grid_layout_->addWidget(x_edit, static_cast<int>(i), 1);
        point_x_.push_back(x_edit);

        auto* y_edit = new QLineEdit();
        y_edit->setPlaceholderText("0.000");
        if (teach_points_[i].recorded) {
            y_edit->setText(QString::number(teach_points_[i].y, 'f', 4));
        }
        point_grid_layout_->addWidget(y_edit, static_cast<int>(i), 2);
        point_y_.push_back(y_edit);

        auto* z_edit = new QLineEdit();
        z_edit->setPlaceholderText("0.300");
        if (teach_points_[i].recorded) {
            z_edit->setText(QString::number(teach_points_[i].z, 'f', 4));
        }
        point_grid_layout_->addWidget(z_edit, static_cast<int>(i), 3);
        point_z_.push_back(z_edit);

        auto* btn = new QPushButton("记录");
        connect(btn, &QPushButton::clicked, this, &CR5GuiWindow::onRecordPoint);
        point_grid_layout_->addWidget(btn, static_cast<int>(i), 4);
        record_btn_.push_back(btn);

        auto* status = new QLabel(teach_points_[i].recorded ? "✓ 已记录" : "未记录");
        status->setStyleSheet(teach_points_[i].recorded
            ? "color: green; font-weight: bold;" : "color: gray;");
        point_grid_layout_->addWidget(status, static_cast<int>(i), 5);
        point_status_.push_back(status);
    }
}

void CR5GuiWindow::onStartMeasurement() {
    if (isBusy()) {
        QMessageBox::warning(this, "警告", "任务正在执行中，请等待完成！");
        return;
    }

    if (recorded_count_ < 1) {
        QMessageBox::warning(this, "警告", "请先记录至少1个示教点！");
        return;
    }

    is_running_.store(true);
    robot_status_label_->setText("运行状态: 执行测量中...");
    robot_status_label_->setStyleSheet("color: orange; font-weight: bold;");
    updateActionAvailability();

    appendLog("========================================", "INFO");
    appendLog(QString("开始测量流程 (%1 个点)").arg(recorded_count_), "INFO");
    appendLog("========================================", "INFO");

    // 收集已记录的点
    std::vector<std::pair<int, geometry_msgs::msg::Point>> targets;
    for (size_t i = 0; i < teach_points_.size(); ++i) {
        if (teach_points_[i].recorded) {
            geometry_msgs::msg::Point pt;
            pt.x = teach_points_[i].x;
            pt.y = teach_points_[i].y;
            pt.z = teach_points_[i].z;
            targets.push_back({static_cast<int>(i) + 1, pt});
        }
    }

    // 在新线程中执行测量任务
    std::thread([this, targets]() {
        const std::string timestamp = my_cr5_control::results::makeTimestamp();
        const std::string log_path =
            my_cr5_control::results::makeOutputPath(timestamp, "box_measurement_log.csv");
        std::ofstream log_file(log_path);
        log_file << "Point,TipX,TipY,TipZ,PlanType,Fraction,FlangeX,FlangeY,FlangeZ,QX,QY,QZ,QW\n";

        const geometry_msgs::msg::Quaternion vertical_down_orientation = getVerticalDownOrientation();
        bool interrupted = false;

        for (const auto& [idx, tip_point] : targets) {
            if (!is_running_.load()) {
                interrupted = true;
                break;
            }

            appendLogAsync(QString(">>> 执行路径点 P%1").arg(idx), "INFO");

            auto result = robot_->measureTipPoint(tip_point, vertical_down_orientation, 0.08);

            log_file << "P" << idx << ","
                     << tip_point.x << "," << tip_point.y << "," << tip_point.z << ","
                     << "cartesian," << result.touch_fraction << ","
                     << result.final_flange_pose.position.x << ","
                     << result.final_flange_pose.position.y << ","
                     << result.final_flange_pose.position.z << ","
                     << result.final_flange_pose.orientation.x << ","
                     << result.final_flange_pose.orientation.y << ","
                     << result.final_flange_pose.orientation.z << ","
                     << result.final_flange_pose.orientation.w << "\n";

            if (!result.reached_approach) {
                appendLogAsync(QString("⚠ 预备点规划失败，跳过 P%1").arg(idx), "WARN");
            } else if (result.touch_fraction > 0.95) {
                appendLogAsync(QString("✓ P%1 触碰成功 (%.1f%%)").arg(idx).arg(result.touch_fraction * 100.0), "SUCCESS");
            } else {
                appendLogAsync(QString("⚠ P%1 触碰不完整 (%.1f%%)").arg(idx).arg(result.touch_fraction * 100.0), "WARN");
            }
        }

        log_file.close();
        appendLogAsync("========================================", "INFO");
        if (interrupted) {
            appendLogAsync(QString("⚠ 测量流程已中断，日志已保存到 %1").arg(QString::fromStdString(log_path)), "WARN");
        } else {
            appendLogAsync(QString("✓ 测量流程完成，日志已保存到 %1").arg(QString::fromStdString(log_path)), "SUCCESS");
        }
        appendLogAsync("========================================", "INFO");

        is_running_.store(false);
        QMetaObject::invokeMethod(this, [this]() {
            robot_status_label_->setText("运行状态: 空闲");
            robot_status_label_->setStyleSheet("color: green; font-weight: bold;");
            updateActionAvailability();
        });
    }).detach();
}

void CR5GuiWindow::onStartCalibration() {
    if (isBusy()) {
        QMessageBox::warning(this, "警告", "任务正在执行中，请等待完成！");
        return;
    }

    QMessageBox::StandardButton reply = QMessageBox::question(
        this, "确认", "即将开始TCP标定流程，请确保标定球已正确放置。\n\n是否继续？",
        QMessageBox::Yes | QMessageBox::No);

    if (reply != QMessageBox::Yes) return;

    is_running_.store(true);
    robot_status_label_->setText("运行状态: TCP标定中...");
    robot_status_label_->setStyleSheet("color: orange; font-weight: bold;");
    updateActionAvailability();

    appendLog("========================================", "INFO");
    appendLog("开始TCP标定流程", "INFO");
    appendLog("========================================", "INFO");

    // 在新线程中执行标定任务
    std::thread([this]() {
        robot_->setupCalibrationScene();
        appendLogAsync("✓ 标定场景加载完成", "SUCCESS");

        // 打开日志文件
        const std::string timestamp = my_cr5_control::results::makeTimestamp();
        const std::string log_path =
            my_cr5_control::results::makeOutputPath(timestamp, "tcp_calibration_log.csv");
        std::ofstream log_file(log_path);
        log_file << "Point,X,Y,Z,QX,QY,QZ,QW\n";

        // 生成标定点
        auto points = generateConePoints();
        appendLogAsync(QString("生成了 %1 个标定点").arg(points.size()), "INFO");
        bool interrupted = false;

        // 移动到安全高度
        const double SAFE_HEIGHT = 0.75;
        const double SPHERE_X = 0.50;
        const double SPHERE_Y = 0.00;

        appendLogAsync(">>> 移动到安全高度...", "INFO");
        geometry_msgs::msg::Pose safe_pose;
        safe_pose.position.x = SPHERE_X;
        safe_pose.position.y = SPHERE_Y;
        safe_pose.position.z = SAFE_HEIGHT;
        safe_pose.orientation = calculateLookAt(safe_pose.position, points[0].target_pose.position);

        robot_->setSpeed(0.5);
        if (!robot_->moveToPose(safe_pose)) {
            appendLogAsync("⚠ 无法移动到安全高度，标定终止", "ERROR");
            log_file.close();
            is_running_.store(false);
            QMetaObject::invokeMethod(this, [this]() {
                robot_status_label_->setText("运行状态: 空闲");
                robot_status_label_->setStyleSheet("color: green; font-weight: bold;");
                updateActionAvailability();
            });
            return;
        }
        appendLogAsync("✓ 已到达安全高度", "SUCCESS");

        // 执行标定点循环
        for (const auto& pt : points) {
            if (!is_running_.load()) {
                interrupted = true;
                appendLogAsync("标定流程被中断", "WARN");
                break;
            }

            appendLogAsync(QString(">>> 执行标定点: %1").arg(QString::fromStdString(pt.name)), "INFO");

            // 移动到预备点
            robot_->setSpeed(0.5);
            if (!robot_->moveToPose(pt.approach_pose)) {
                appendLogAsync(QString("⚠ 预备点规划失败: %1").arg(QString::fromStdString(pt.name)), "WARN");
                continue;
            }

            // 直线触碰
            appendLogAsync("    触碰中...", "INFO");
            robot_->setSpeed(0.1);
            double fraction = robot_->moveLine(pt.target_pose);

            if (fraction > 0.95) {
                appendLogAsync(QString("✓ 触碰成功 (%.1f%%)").arg(fraction * 100.0), "SUCCESS");
                auto p = robot_->getCurrentPose();
                log_file << pt.name << ","
                         << std::fixed << std::setprecision(6)
                         << p.position.x << "," << p.position.y << "," << p.position.z << ","
                         << p.orientation.x << "," << p.orientation.y << ","
                         << p.orientation.z << "," << p.orientation.w << "\n";
            } else {
                appendLogAsync(QString("⚠ 触碰未完全到达 (%.1f%%)").arg(fraction * 100.0), "WARN");
            }

            // 后退
            appendLogAsync("    后退中...", "INFO");
            robot_->setSpeed(0.5);
            double retract_fraction = robot_->moveLine(pt.approach_pose);
            if (retract_fraction < 0.95) {
                appendLogAsync(QString("⚠ 后退不完整 (%.1f%%)，使用RRT规划返回").arg(retract_fraction * 100.0), "WARN");
                robot_->moveToPose(pt.approach_pose);
            }
        }

        // 返回安全点
        appendLogAsync(">>> 返回安全点...", "INFO");
        robot_->moveToPose(safe_pose);

        log_file.close();
        appendLogAsync("========================================", "INFO");
        if (interrupted) {
            appendLogAsync(QString("⚠ TCP 标定流程已中断，日志已保存到 %1").arg(QString::fromStdString(log_path)), "WARN");
        } else {
            appendLogAsync(QString("✓ TCP标定流程完成，日志已保存到 %1").arg(QString::fromStdString(log_path)), "SUCCESS");
        }
        appendLogAsync("========================================", "INFO");

        is_running_.store(false);
        QMetaObject::invokeMethod(this, [this]() {
            robot_status_label_->setText("运行状态: 空闲");
            robot_status_label_->setStyleSheet("color: green; font-weight: bold;");
            updateActionAvailability();
        });
    }).detach();
}

void CR5GuiWindow::onEmergencyStop() {
    bool handled = false;

    if (task_process_ != nullptr && task_process_->state() != QProcess::NotRunning) {
        appendLog("正在停止外部任务...", "WARN");
        stopExternalTask(false);
        handled = true;
    }

    if (is_running_.exchange(false)) {
        appendLog("机器人任务已收到停止请求，将在当前动作结束后退出", "WARN");
        handled = true;
    }

    if (!handled) {
        appendLog("当前没有正在运行的任务", "INFO");
        return;
    }

    robot_status_label_->setText("运行状态: 停止请求已发送");
    robot_status_label_->setStyleSheet("color: red; font-weight: bold;");
    automation_status_label_->setText("自动任务状态: 停止中");
    automation_status_label_->setStyleSheet("color: red; font-weight: bold;");
    updateActionAvailability();
}

void CR5GuiWindow::onUpdateRobotState() {
    if (!robot_ || !robot_connected_ || isBusy()) return;

    try {
        auto pose = robot_->getCurrentPose();
        QString pose_text = QString("当前位姿:\nX: %1 m\nY: %2 m\nZ: %3 m")
                               .arg(pose.position.x, 0, 'f', 4)
                               .arg(pose.position.y, 0, 'f', 4)
                               .arg(pose.position.z, 0, 'f', 4);
        current_pose_label_->setText(pose_text);
    } catch (...) {
        // 忽略错误
    }
}

void CR5GuiWindow::onToggleCalibrationScene() {
    if (!robot_) return;

    if (calibration_scene_visible_) {
        robot_->removeCalibrationScene();
        calibration_scene_visible_ = false;
        toggle_calib_scene_btn_->setText("恢复标定杆");
        appendLog("标定场景已移除", "INFO");
    } else {
        robot_->restoreCalibrationScene();
        calibration_scene_visible_ = true;
        toggle_calib_scene_btn_->setText("清除标定杆");
        appendLog("标定场景已恢复", "SUCCESS");
    }
    updateActionAvailability();
}

void CR5GuiWindow::onToggleBox() {
    if (!robot_) return;

    if (box_visible_) {
        robot_->removeBox();
        box_visible_ = false;
        toggle_box_btn_->setText("恢复箱体");
        appendLog("箱体已移除", "INFO");
    } else {
        robot_->restoreBox();
        box_visible_ = true;
        toggle_box_btn_->setText("清除箱体");
        appendLog("箱体已恢复", "SUCCESS");
    }
    updateActionAvailability();
}

void CR5GuiWindow::onRunSimpleBenchmark() {
    startExternalTask(
        "Simple Benchmark",
        "ros2",
        {"run", "my_cr5_control", "planner_comparison_simple_node"},
        {
            {"MY_CR5_CONTROL_SIMPLE_REPEATS", QString::number(benchmark_repeats_spin_->value())},
        });
}

void CR5GuiWindow::onRunV2Benchmark() {
    startExternalTask(
        "V2 Benchmark",
        "ros2",
        {"run", "my_cr5_control", "planner_comparison_v2_node"},
        {
            {"MY_CR5_CONTROL_V2_REPEATS", QString::number(benchmark_repeats_spin_->value())},
        });
}

void CR5GuiWindow::onRunRandomDataset() {
    startExternalTask(
        "Simple Random Dataset",
        "ros2",
        {"run", "my_cr5_control", "random_task_dataset_simple_node"},
        {
            {"MY_CR5_CONTROL_RANDOM_SIMPLE_TASKS", QString::number(random_task_count_spin_->value())},
        });
}

void CR5GuiWindow::onPlotSimpleBenchmark() {
    const QString script_path =
        QDir(QStringLiteral(MY_CR5_CONTROL_SOURCE_DIR)).filePath("scripts/benchmarks/plot_simple_benchmark.py");
    startExternalTask(
        "Simple Benchmark Plot",
        "python3",
        {script_path});
}

void CR5GuiWindow::onPlotSimpleRandomDataset() {
    const QString script_path =
        QDir(QStringLiteral(MY_CR5_CONTROL_SOURCE_DIR)).filePath("scripts/datasets/plot_simple_random_dataset.py");
    startExternalTask(
        "Simple Random Plot",
        "python3",
        {script_path});
}

void CR5GuiWindow::onExportBenchmarkDataset() {
    const QString script_path =
        QDir(QStringLiteral(MY_CR5_CONTROL_SOURCE_DIR)).filePath("scripts/benchmarks/export_benchmark_dataset.py");
    startExternalTask(
        "Export Benchmark Dataset",
        "python3",
        {script_path});
}

void CR5GuiWindow::onRefreshDatasetManifest() {
    const QString script_path =
        QDir(QStringLiteral(MY_CR5_CONTROL_SOURCE_DIR)).filePath("scripts/maintenance/build_dataset_manifest.py");
    startExternalTask(
        "Refresh Dataset Manifest",
        "python3",
        {script_path});
}

void CR5GuiWindow::onClearCurrentState() {
    if (isBusy()) {
        const auto reply = QMessageBox::question(
            this,
            "确认清除",
            "当前仍有任务在运行。是否停止任务并清除当前 GUI 状态？",
            QMessageBox::Yes | QMessageBox::No);
        if (reply != QMessageBox::Yes) {
            return;
        }
    }

    is_running_.store(false);
    stopExternalTask(true);
    resetUiState(true);
    appendLog("GUI 状态已重置，已恢复到默认空闲状态", "INFO");
}

void CR5GuiWindow::onExternalTaskOutput() {
    if (task_process_ == nullptr) {
        return;
    }

    const QString text = QString::fromLocal8Bit(task_process_->readAll()).trimmed();
    if (text.isEmpty()) {
        return;
    }

    const QStringList lines = text.split('\n', Qt::SkipEmptyParts);
    for (const QString& line : lines) {
        appendLog(QString("[%1] %2").arg(current_external_task_name_, line.trimmed()), "INFO");
    }
}

void CR5GuiWindow::onExternalTaskFinished(int exit_code, QProcess::ExitStatus exit_status) {
    if (suppress_external_task_notifications_) {
        suppress_external_task_notifications_ = false;
        current_external_task_name_.clear();
        updateActionAvailability();
        return;
    }

    const bool normal_exit = exit_status == QProcess::NormalExit;
    const bool success = normal_exit && exit_code == 0;
    const QString finished_task = current_external_task_name_.isEmpty()
        ? "外部任务"
        : current_external_task_name_;

    automation_status_label_->setText(
        success
            ? QString("自动任务状态: 已完成 (%1)").arg(finished_task)
            : QString("自动任务状态: 已结束 (%1, exit=%2)").arg(finished_task).arg(exit_code));
    automation_status_label_->setStyleSheet(
        success ? "color: green; font-weight: bold;" : "color: red; font-weight: bold;");

    appendLog(
        success
            ? QString("外部任务完成: %1").arg(finished_task)
            : QString("外部任务结束: %1 | exit_code=%2").arg(finished_task).arg(exit_code),
        success ? "SUCCESS" : "WARN");

    current_external_task_name_.clear();
    updateActionAvailability();
}

void CR5GuiWindow::onExternalTaskErrorOccurred(QProcess::ProcessError error) {
    if (suppress_external_task_notifications_) {
        return;
    }
    if (error == QProcess::Crashed) {
        appendLog(QString("外部任务崩溃: %1").arg(current_external_task_name_), "ERROR");
        return;
    }
    if (task_process_ != nullptr && task_process_->state() != QProcess::NotRunning) {
        appendLog(QString("外部任务错误: %1").arg(task_process_->errorString()), "ERROR");
    }
}

// TCP标定辅助函数
geometry_msgs::msg::Quaternion CR5GuiWindow::calculateLookAt(
    const geometry_msgs::msg::Point& source,
    const geometry_msgs::msg::Point& target)
{
    tf2::Vector3 position(source.x, source.y, source.z);
    tf2::Vector3 target_pos(target.x, target.y, target.z);
    tf2::Vector3 z_axis = (target_pos - position).normalized();

    tf2::Vector3 up(0, 0, 1);
    if (std::abs(z_axis.dot(up)) > 0.99) {
        up = tf2::Vector3(1, 0, 0);
    }

    tf2::Vector3 x_axis = up.cross(z_axis).normalized();
    tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();

    tf2::Matrix3x3 mat(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z()
    );
    tf2::Quaternion q;
    mat.getRotation(q);
    return tf2::toMsg(q);
}

geometry_msgs::msg::Quaternion CR5GuiWindow::getVerticalDownOrientation() {
    tf2::Quaternion q;
    q.setRPY(0, M_PI, 0);
    return tf2::toMsg(q);
}

std::vector<CR5GuiWindow::CalibrationPoint> CR5GuiWindow::generateConePoints() {
    using namespace my_cr5_control::probe;

    const double SPHERE_X = 0.50;
    const double SPHERE_Y = 0.00;
    const double SPHERE_Z = 0.45;
    const double SPHERE_RADIUS = 0.0125;
    const double APPROACH_RETRACT_DIST = 0.10;

    std::vector<CalibrationPoint> points;
    geometry_msgs::msg::Point center;
    center.x = SPHERE_X; center.y = SPHERE_Y; center.z = SPHERE_Z;

    // 1. 顶点：测针尖端向下触碰球顶
    {
        CalibrationPoint pt;
        pt.name = "Top";
        double flange_dist = SPHERE_RADIUS + kProbeLength;
        pt.target_pose.position.x = center.x;
        pt.target_pose.position.y = center.y;
        pt.target_pose.position.z = center.z + flange_dist;
        pt.target_pose.orientation = calculateLookAt(pt.target_pose.position, center);

        pt.approach_pose = pt.target_pose;
        pt.approach_pose.position.z += APPROACH_RETRACT_DIST;
        points.push_back(pt);
    }

    // 2. 赤道4点：使用星形测针侧面触碰
    struct EquatorConfig { std::string name; double azimuth; };
    std::vector<EquatorConfig> equator_configs = {
        {"Equator_Front", 0.0},
        {"Equator_Left",  90.0},
        {"Equator_Back",  180.0},
        {"Equator_Right", 270.0}
    };

    for (const auto& cfg : equator_configs) {
        CalibrationPoint pt;
        pt.name = cfg.name;

        double horizontal_dist = kStarStylusReach + SPHERE_RADIUS;
        double theta = cfg.azimuth * M_PI / 180.0;

        pt.target_pose.position.x = center.x + horizontal_dist * std::cos(theta);
        pt.target_pose.position.y = center.y + horizontal_dist * std::sin(theta);
        pt.target_pose.position.z = center.z + kStarStylusZOffset;
        pt.target_pose.orientation = getVerticalDownOrientation();

        pt.approach_pose = pt.target_pose;
        pt.approach_pose.position.x += APPROACH_RETRACT_DIST * std::cos(theta);
        pt.approach_pose.position.y += APPROACH_RETRACT_DIST * std::sin(theta);

        points.push_back(pt);
    }

    return points;
}
