#include "sp_hw/hardware_interface/hardware_interface.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sp_hw");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    std::shared_ptr<sp_hw::SpRobotHW> hardware_interface_ = std::make_shared<sp_hw::SpRobotHW>();
    hardware_interface_->init(nh, nh_p);
}