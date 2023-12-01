//
// Created by CherryBlossomNight on 2023/10/24.
//

#include "syn_controller/syn_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <sp_common/base_utilities.h>

namespace syn_controller
{
  bool SynController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {
    ros::NodeHandle nh_left = ros::NodeHandle(controller_nh, "left_joint");
    ros::NodeHandle nh_right = ros::NodeHandle(controller_nh, "right_joint");
    syn_cmd_sub_ = nh.subscribe<std_msgs::Float64>("syn_cmd", 1, &SynController::setCommandCB, this);

    has_feedforward_ = sp_common::getParam(controller_nh, "has_feedforward", false);
    feedforward_ = sp_common::getParam(controller_nh, "feedforward", 0.0);

    effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (!ctrl_left_.init(effort_joint_interface_, nh_left) || !ctrl_right_.init(effort_joint_interface_, nh_right) )
        return false;
    pos_ = (ctrl_left_.joint_.getPosition() + ctrl_right_.joint_.getPosition()) / 2;

    ROS_INFO("SYN : Initializing Completed");

    
    return true;
  }

  void SynController::update(const ros::Time &time, const ros::Duration &period)
  { 
    pos_ = (ctrl_left_.joint_.getPosition() + ctrl_right_.joint_.getPosition()) / 2;
    //if (ctrl_left_.joint_.getPosition() - ctrl_right_.joint_.getPosition())

    ctrl_left_.setCommand(cmd_);
    ctrl_right_.setCommand(cmd_);
    ctrl_left_.update(time, period);
    ctrl_right_.update(time, period);
    if (has_feedforward_)
    {
      if ((ctrl_left_.joint_.getCommand() - ctrl_left_.joint_.getPosition())>0)
      {
        ctrl_left_.joint_.setCommand(ctrl_left_.joint_.getCommand() + feedforward_ - 75);
        ctrl_right_.joint_.setCommand(ctrl_right_.joint_.getCommand() + feedforward_ - 75);
        }
      else if ((ctrl_left_.joint_.getCommand()- ctrl_left_.joint_.getPosition())<=0)
      {
        ctrl_left_.joint_.setCommand(ctrl_left_.joint_.getCommand() - feedforward_ - 75);
        ctrl_right_.joint_.setCommand(ctrl_right_.joint_.getCommand() - feedforward_ - 75);
      }
    }

  }

  double SynController::getPosition()
  {
    return pos_;
  }

  void SynController::setCommand(double cmd)
  {
    cmd_ = cmd;
  }

  void SynController::setCommandCB(const std_msgs::Float64::ConstPtr &msg)
  {
    setCommand(msg->data);
  }

} // namespace syn_controller

PLUGINLIB_EXPORT_CLASS(syn_controller::SynController, controller_interface::ControllerBase)