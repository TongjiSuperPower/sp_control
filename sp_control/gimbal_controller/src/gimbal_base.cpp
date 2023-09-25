#include "gimbal_controller/gimbal_base.h"
#include <pluginlib/class_list_macros.hpp>
namespace gimbal_controller
{

    bool GimbalBase::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        ROS_INFO("GIMBAL: START TO INIT ...");

        if (!controller_nh.getParam("publish_rate", publish_rate_) || !controller_nh.getParam("timeout", timeout_))
        {
            ROS_ERROR("Some gimbal params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
            return false;
        }
        
        cmd_pos_sub_ = root_nh.subscribe<geometry_msgs::Vector3>("cmd_pos", 1, &GimbalBase::cmdPosCallback, this);
        return true;

    }

    void GimbalBase::update(const ros::Time &time, const ros::Duration &period)
    {
        geometry_msgs::Vector3 cmd_pos = cmd_rt_buffer_.readFromRT()->cmd_pos_;

        pos_cmd_.x = cmd_pos.x;
        pos_cmd_.y = cmd_pos.y;
        pos_cmd_.z = cmd_pos.z;

        moveJoint(time, period);
    }

    void GimbalBase::cmdPosCallback(const geometry_msgs::Vector3::ConstPtr &msg)
    {
        cmd_struct_.cmd_pos_ = *msg;
        cmd_struct_.stamp_ = ros::Time::now();
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }
}