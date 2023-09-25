#include "gimbal_controller/gimbal_effort.h"
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

    bool GimbalEffort::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        GimbalBase::init(robot_hw, root_nh, controller_nh);
        ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw");
        ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
        // get the position joint interface
        effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

        // subsribe the topic "cmd_pos"
       
        if (!ctrl_yaw_.init(effort_joint_interface_, nh_yaw) || !ctrl_pitch_.init(effort_joint_interface_, nh_pitch))
            return false;
        // construct vector<joint> to control the acuators directly.

        ROS_INFO("GIMBAL: INIT SUCCESS !");

        return true;
    }


    void GimbalEffort::moveJoint(const ros::Time &time, const ros::Duration &period)
    {
        ctrl_yaw_.setCommand(pos_cmd_.z);
        ctrl_pitch_.setCommand(pos_cmd_.y);
        ctrl_yaw_.update(time, period);
        ctrl_pitch_.update(time, period);
    }
    PLUGINLIB_EXPORT_CLASS(gimbal_controller::GimbalEffort, controller_interface::ControllerBase);
} // namespace gimbal_controller
