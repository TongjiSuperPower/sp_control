#pragma once

#include "chassis_controller/chassis_base.h"
#include <effort_controllers/joint_position_controller.h>

namespace chassis_controller
{
    class Swerve : public ChassisBase
    {
    public:
        Swerve() = default;
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    private:
        effort_controllers::JointVelocityController ctrl_lf_driving_, ctrl_rf_driving_, ctrl_lb_driving_, ctrl_rb_driving_;
        effort_controllers::JointPositionController ctrl_lf_heading_, ctrl_rf_heading_, ctrl_lb_heading_, ctrl_rb_heading_;

        void moveJoint(const ros::Time &time, const ros::Duration &period) override;
        geometry_msgs::Twist forwardKinematics() override;
    };
}