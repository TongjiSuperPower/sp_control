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

    has_friction_ = sp_common::getParam(controller_nh, "has_friction", false);
    friction_ = sp_common::getParam(controller_nh, "friction", 0.0);
    has_gravity_ = sp_common::getParam(controller_nh, "has_gravity", false);
    gravity_ = sp_common::getParam(controller_nh, "gravity", 0.0);

    effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (!ctrl_left_.init(effort_joint_interface_, nh_left) || !ctrl_right_.init(effort_joint_interface_, nh_right) )
        return false;
    pos_ = (ctrl_left_.joint_.getPosition() + ctrl_right_.joint_.getPosition()) / 2;

    ROS_INFO("SYN : Initializing Completed");

    
    return true;
  }

  void SynController::update(const ros::Time &time, const ros::Duration &period)
  { 
    i++;
    pos_ = (ctrl_left_.joint_.getPosition() + ctrl_right_.joint_.getPosition()) / 2;
    //if (ctrl_left_.joint_.getPosition() - ctrl_right_.joint_.getPosition())
   
     

    ctrl_left_.setCommand(cmd_);
    ctrl_right_.setCommand(cmd_);
    ctrl_left_.update(time, period);
    ctrl_right_.update(time, period);
     if (i == 50)
    {
      // ROS_INFO_STREAM(cmd_);
      // ROS_INFO_STREAM(ctrl_left_.joint_.getPosition());
      // ROS_INFO_STREAM(ctrl_right_.joint_.getPosition());
      // ROS_INFO_STREAM("------------");
      // ROS_INFO_STREAM(ctrl_left_.joint_.getEffort());
      // ROS_INFO_STREAM(ctrl_right_.joint_.getEffort());
      // ROS_INFO_STREAM(ctrl_left_.joint_.getCommand());
      // ROS_INFO_STREAM(ctrl_right_.joint_.getCommand());
      // ROS_INFO_STREAM("---------------------------------------");
      i = 0;
    }
    if (has_friction_)
    {
      if (has_gravity_)
      {
        if ((cmd_ - ctrl_left_.joint_.getPosition()) > 0.008)
        {
          ctrl_left_.joint_.setCommand(ctrl_left_.joint_.getCommand() + friction_ + gravity_);
          ctrl_right_.joint_.setCommand(ctrl_right_.joint_.getCommand() + friction_ + gravity_);
        }
        else if ((cmd_ - ctrl_left_.joint_.getPosition()) < -0.008)
        {
          ctrl_left_.joint_.setCommand(ctrl_left_.joint_.getCommand() - friction_ + gravity_);
          ctrl_right_.joint_.setCommand(ctrl_right_.joint_.getCommand() - friction_ + gravity_);
        }
        else
        {
          double eff = (cmd_ - ctrl_left_.joint_.getPosition()) / 0.008;
          ctrl_left_.joint_.setCommand(ctrl_left_.joint_.getCommand() + eff * friction_ + gravity_);
          ctrl_right_.joint_.setCommand(ctrl_right_.joint_.getCommand() + eff * friction_ + gravity_);

        }
      }
      else
      {
        if ((cmd_ - ctrl_left_.joint_.getPosition()) > 0.02)
        {
          ctrl_left_.joint_.setCommand(ctrl_left_.joint_.getCommand() + friction_);
          ctrl_right_.joint_.setCommand(ctrl_right_.joint_.getCommand() + friction_);
        }
        else if ((cmd_ - ctrl_left_.joint_.getPosition()) < -0.02)
        {
          ctrl_left_.joint_.setCommand(ctrl_left_.joint_.getCommand() - friction_);
          ctrl_right_.joint_.setCommand(ctrl_right_.joint_.getCommand() - friction_);
        }
        else
        {
          double eff = (cmd_ - ctrl_left_.joint_.getPosition()) / 0.02;
          ctrl_left_.joint_.setCommand(ctrl_left_.joint_.getCommand() + eff * friction_);
          ctrl_right_.joint_.setCommand(ctrl_right_.joint_.getCommand() + eff * friction_);

        }
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