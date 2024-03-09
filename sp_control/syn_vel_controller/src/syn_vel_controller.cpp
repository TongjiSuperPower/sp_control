//
// Created by CherryBlossomNight on 2023/10/24.
//

#include "syn_vel_controller/syn_vel_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <sp_common/base_utilities.h>

namespace syn_vel_controller
{
  bool SynVelController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {
    ros::NodeHandle nh_left = ros::NodeHandle(controller_nh, "left_joint");
    ros::NodeHandle nh_right = ros::NodeHandle(controller_nh, "right_joint");

    effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (!ctrl_left_.init(effort_joint_interface_, nh_left) || !ctrl_right_.init(effort_joint_interface_, nh_right) )
        return false;
    vel_ = (ctrl_left_.joint_.getVelocity() + ctrl_right_.joint_.getVelocity()) / 2;

    ROS_INFO("SYN : Initializing Completed");

    
    return true;
  }

  void SynVelController::update(const ros::Time &time, const ros::Duration &period)
  { 
    vel_ = (ctrl_left_.joint_.getVelocity() + ctrl_right_.joint_.getVelocity()) / 2;

    ctrl_left_.setCommand(cmd_);
    ctrl_right_.setCommand(cmd_);
    ctrl_left_.update(time, period);
    ctrl_right_.update(time, period);

  }

  float SynVelController::getVelocity()
  {
    return vel_;
  }

  void SynVelController::setCommand(float cmd)
  {
    cmd_ = cmd;
  }

  void SynVelController::setCommandCB(const std_msgs::Float64::ConstPtr &msg)
  {
    setCommand(msg->data);
  }

} 

PLUGINLIB_EXPORT_CLASS(syn_vel_controller::SynVelController, controller_interface::ControllerBase)