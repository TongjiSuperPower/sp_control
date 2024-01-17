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
        
        cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Vector3>("cmd_gimbal_vel", 1, &GimbalBase::cmdVelCallback, this);
        return true;

    }


    void GimbalBase::cmdVelCallback(const geometry_msgs::Vector3::ConstPtr &msg)
    {
        cmd_struct_.cmd_gimbal_vel_ = *msg;
        cmd_struct_.stamp_ = ros::Time::now();
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }
}