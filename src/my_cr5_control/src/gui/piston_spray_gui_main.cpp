#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "my_cr5_control/piston_spray_gui_window.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    PistonSprayGuiWindow window;
    window.show();

    const int exit_code = app.exec();
    rclcpp::shutdown();
    return exit_code;
}
