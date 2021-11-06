#include "theta_driver/theta_driver_nodelet.hpp"
#include <nodelet/loader.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "theta_driver_node");
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "theta_driver/ThetaDriverNodelet", remap, nargv);
    ros::spin();
    return 0;
}
