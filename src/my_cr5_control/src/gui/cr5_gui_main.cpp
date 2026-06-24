#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "my_cr5_control/cr5_gui_window.hpp"

int main(int argc, char** argv) {
    // 初始化ROS 2
    rclcpp::init(argc, argv);

    // 初始化Qt应用
    QApplication app(argc, argv);

    // 创建并显示主窗口
    CR5GuiWindow window;
    window.show();

    // 运行Qt事件循环
    int result = app.exec();

    // 清理ROS 2
    rclcpp::shutdown();

    return result;
}
