#include "rotate_ore_controller/rotate_ore_controller.h"
#include <pluginlib/class_list_macros.hpp>


namespace rotate_ore_controller
{

    bool RotateOreController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        ROS_INFO("RotateOre: START TO INIT ...");
        ros::NodeHandle nh;
        ros::NodeHandle nh_left = ros::NodeHandle(controller_nh, "ore_left");
        ros::NodeHandle nh_right = ros::NodeHandle(controller_nh, "ore_right");

        effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (!ctrl_left_.init(effort_joint_interface_, nh_left) || !ctrl_right_.init(effort_joint_interface_, nh_right) )
            return false;

        effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

        vel_ = sp_common::getParam(controller_nh, "vel", 3.14);

        cmd_ore_sub_ = root_nh.subscribe<std_msgs::Int8>("/cmd_ore", 1, &RotateOreController::cmdOreCallback, this);
        

        ROS_INFO("ROTATEORE: INIT SUCCESS !");

        return true;
    }

    void RotateOreController::update(const ros::Time &time, const ros::Duration &period)
    {
        ore_cmd_ = cmd_rt_buffer_.readFromRT()->ore_cmd_;
        ROS_INFO_STREAM("ore_cmd_:" << ore_cmd_);
        moveJoint(time, period);
    }

    void RotateOreController::moveJoint(const ros::Time &time, const ros::Duration &period)
    {
        if (ore_cmd_ == 1)
        {
            ctrl_left_.setCommand(vel_);
            ctrl_left_.update(time, period);
            ctrl_right_.setCommand(vel_);
            ctrl_right_.update(time, period);
        }
        else if (ore_cmd_ == 2)
        {
            ctrl_left_.setCommand(-vel_);
            ctrl_left_.update(time, period);
            ctrl_right_.setCommand(-vel_);
            ctrl_right_.update(time, period);
        }
        else if (ore_cmd_ == 3)
        {
            ctrl_left_.setCommand(-vel_);
            ctrl_left_.update(time, period);
            ctrl_right_.setCommand(vel_);
            ctrl_right_.update(time, period);
        }      
        else if (ore_cmd_ == 4)
        {
            ctrl_left_.setCommand(vel_);
            ctrl_left_.update(time, period);
            ctrl_right_.setCommand(-vel_);
            ctrl_right_.update(time, period);
        }
        else 
        {
            ctrl_left_.setCommand(0);
            ctrl_left_.update(time, period);
            ctrl_right_.setCommand(0);
            ctrl_right_.update(time, period);
        }

    }


    void RotateOreController::cmdOreCallback(const std_msgs::Int8::ConstPtr &msg)
    {
        cmd_struct_.ore_cmd_ = msg->data;
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }

}

PLUGINLIB_EXPORT_CLASS(rotate_ore_controller::RotateOreController, controller_interface::ControllerBase)