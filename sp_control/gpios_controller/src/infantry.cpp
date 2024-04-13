//
// Created by CherryBlossomNight on 2024/1/19
//

#include "gpios_controller/infantry.h"
#include <angles/angles.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace gpios_controller
{

    template <typename T>
    T getParam(ros::NodeHandle &pnh, const std::string &param_name, const T &default_val)
    {
        T param_val;
        pnh.param<T>(param_name, param_val, default_val);
        return param_val;
    }


    bool Infantry::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {

        cmd_cover_sub_ = root_nh.subscribe<std_msgs::Bool>("/cmd_cover", 1,  &Infantry::cmdCoverCallback, this);
       
        ros::NodeHandle nh_cover = ros::NodeHandle(controller_nh, "cover");

        gpio_command_interface_ = robot_hw->get<sp_control::GpioCommandInterface>();

        if (!ctrl_cover_.init(gpio_command_interface_, nh_cover))
            return false;
        ROS_INFO_STREAM("COVER : Initializing Completed");
        return true;
    }


    void Infantry::update(const ros::Time& time, const ros::Duration& period)
    {
        ctrl_cover_.setCommand(cover_msg_);
        ctrl_cover_.update(time, period);     
    }


    void Infantry::cmdCoverCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        cover_msg_ = msg->data;
    }


    PLUGINLIB_EXPORT_CLASS(gpios_controller::Infantry, controller_interface::ControllerBase);

}// namespace gpios_controller

