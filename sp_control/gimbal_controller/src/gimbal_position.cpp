#include "gimbal_controller/gimbal_position.h"
#include <angles/angles.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace gimbal_controller
{
    template <typename T>
    T getParam(ros::NodeHandle &pnh, const std::string &param_name, const T &default_val)
    {
        T param_val;
        pnh.param<T>(param_name, param_val, default_val);
        return param_val;
    }

    bool GimbalPosition::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        GimbalBase::init(robot_hw, root_nh, controller_nh);
        ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw");
        ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
        // get the position joint interface
        position_joint_interface_ = robot_hw->get<hardware_interface::PositionJointInterface>();

        // subsribe the topic "cmd_pos"
       
        if (!ctrl_yaw_.init(position_joint_interface_, nh_yaw) || !ctrl_pitch_.init(position_joint_interface_, nh_pitch))
            return false;
        // construct vector<joint> to control the acuators directly.
        ROS_INFO("GIMBAL: INIT SUCCESS !");

        return true;
    }


    void GimbalPosition::moveJoint(const ros::Time &time, const ros::Duration &period)
    {
        // ctrl_yaw_.joint_.setCommand(pos_cmd_.z);
        // ctrl_pitch_.joint_.setCommand(pos_cmd_.y);
    }
    PLUGINLIB_EXPORT_CLASS(gimbal_controller::GimbalPosition, controller_interface::ControllerBase);
} // namespace gimbal_controller
