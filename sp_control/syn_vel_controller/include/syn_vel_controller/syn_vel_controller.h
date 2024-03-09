//
// Created by Cherryblossomnight on 2023/10/24.
//

#pragma once

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/Float64.h>
namespace syn_vel_controller
{
  class SynVelController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                          hardware_interface::JointStateInterface>
  {
  public:
    SynVelController() = default;

    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

    void update(const ros::Time &time, const ros::Duration &period) override;

    float getVelocity();
    
    void setCommand(float cmd);

      

  private:
    void initPos();

    void setCommandCB(const std_msgs::Float64::ConstPtr &msg);

    ros::NodeHandle nh;
    ros::Subscriber syn_cmd_sub_;

    hardware_interface::EffortJointInterface *effort_joint_interface_{};

    effort_controllers::JointVelocityController ctrl_left_;
    effort_controllers::JointVelocityController ctrl_right_; 

    float vel_{};
    float cmd_{};

    bool has_friction_{};
    float friction_{};
    bool has_gravity_{};
    float gravity_{};
  };
} // namespace syn_controller
