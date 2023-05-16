#pragma once

#include "chassis_controller/chassis_base.h"

namespace chassis_controller
{
    class RollerWheel : public ChassisBase
    {
    public:
        RollerWheel() = default;
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    private:
        effort_controllers::JointVelocityController ctrl_lf_, ctrl_rf_, ctrl_lb_, ctrl_rb_;

        void moveJoint(const ros::Time &time, const ros::Duration &period) override;
        geometry_msgs::Twist forwardKinematics() override;
    };
}