//
// Created by Cherryblossomnight on 2023/11/28.
//

#pragma once

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_position_controller.h>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/Float64.h>
namespace differential_controller
{
  class DifferentialController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>
  {
  public:
    DifferentialController() = default;

    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
      
    void update(const ros::Time &time, const ros::Duration &period) override;

    double getPitchPosition();

    double getRollPosition();
    
    void setPitchCommand(double pitch_cmd);

    void setRollCommand(double roll_cmd);
     

  private:

    void setPitchCommandCB(const std_msgs::Float64::ConstPtr &pitch_cmd);

    void setRollCommandCB(const std_msgs::Float64::ConstPtr &roll_cmd);


    ros::NodeHandle nh;

    ros::Subscriber pitch_cmd_sub_;
    ros::Subscriber roll_cmd_sub_;

    ros::Publisher pitch_pos_pub_;


    hardware_interface::EffortJointInterface *effort_joint_interface_{};

    effort_controllers::JointPositionController ctrl_positive_;
    effort_controllers::JointPositionController ctrl_negative_; 

    double pitch_pos_{};
    double roll_pos_{};
    double pitch_cmd_{};
    double roll_cmd_{};

    double differential_reduction_{};
  };
} // namespace differential_controller
