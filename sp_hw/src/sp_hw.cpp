#include <ros/ros.h>
#include "sp_hw/hardware_interface_loader.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sp_hw");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();
    try
    {
        std::shared_ptr<sp_hw::SpRobotHW> hardware_interface = std::make_shared<sp_hw::SpRobotHW>();
        sp_hw::SpRobotHWLoader control_loop(nh, hardware_interface);

        ros::waitForShutdown();
    }
    catch (const ros::Exception &e)
    {
        ROS_FATAL_STREAM("Error in the SP_HW : \n"
                         << "\t" << e.what());
        return 1;
    }
    return 0;
}