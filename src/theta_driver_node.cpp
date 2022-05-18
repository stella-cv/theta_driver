#include "theta_driver/theta_driver_lib.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    // Init rclcpp
    rclcpp::init(argc, argv);
    // Create executor
    rclcpp::executors::SingleThreadedExecutor exec;
    // Create empty options
    rclcpp::NodeOptions options;
    // Create the theta_driver
    auto theta_driver = std::make_shared<theta_driver::ThetaDriver>(options);
    // Add the node to the executor
    exec.add_node(theta_driver);
    // Spin the node
    exec.spin();
    // Shutdown rclcpp
    rclcpp::shutdown();
    return 0;
}
