#include "theta_driver/theta_driver_nodelet.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "theta_driver_node");
    theta_driver::ThetaDriverNodelet driver;
    bool ok = driver.init();
    if (!ok) {
        ROS_FATAL("Initialization failed");
        return 1;
    }
    ros::spin();
    return 0;
}
