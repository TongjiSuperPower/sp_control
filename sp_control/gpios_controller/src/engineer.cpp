//
// Created by CherryBlossomNight on 2024/2/19
//

#include "gpios_controller/engineer.h"
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


    bool Engineer::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {

        cmd_pump_sub_ = root_nh.subscribe<std_msgs::Bool>("/cmd_pump", 1,  &Engineer::cmdPumpCallback, this);
        cmd_rod_sub_ = root_nh.subscribe<std_msgs::Bool>("/cmd_rod", 1,  &Engineer::cmdRodCallback, this);
       
        ros::NodeHandle nh_pump = ros::NodeHandle(controller_nh, "pump");
        ros::NodeHandle nh_rod = ros::NodeHandle(controller_nh, "rod");

        gpio_command_interface_ = robot_hw->get<sp_control::GpioCommandInterface>();

        if (!ctrl_pump_.init(gpio_command_interface_, nh_pump) || !ctrl_rod_.init(gpio_command_interface_, nh_rod))
            return false;
        ROS_INFO_STREAM("GPIOS : Initializing Completed");
        return true;
    }


    void Engineer::update(const ros::Time& time, const ros::Duration& period)
    {
        ctrl_pump_.setCommand(pump_msg_);
        ctrl_rod_.setCommand(rod_msg_);

        ctrl_pump_.update(time, period);   
        ctrl_rod_.update(time, period);   

    }


    void Engineer::cmdPumpCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        pump_msg_ = msg->data;
    }

    void Engineer::cmdRodCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        rod_msg_ = msg->data;
    }


    PLUGINLIB_EXPORT_CLASS(gpios_controller::Engineer, controller_interface::ControllerBase);

}// namespace cover_controller

