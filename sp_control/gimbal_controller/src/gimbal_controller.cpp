#include "gimbal_controller/gimbal_controller.h"
#include <angles/angles.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace gimbal_controller
{
    template <typename T>
    T getParam(ros::NodeHandle& pnh, const std::string& param_name, const T& default_val)
    {
	T param_val;
	pnh.param<T>(param_name, param_val, default_val);
	return param_val;
    }

    bool GimbalController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
	ROS_INFO("GIMBAL: START TO INIT ...");

        if (!controller_nh.getParam("publish_rate", publish_rate_) || !controller_nh.getParam("timeout", timeout_)) 
        {
            ROS_ERROR("Some gimbal params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
            return false;
        }

	// get the position joint interface
        position_joint_interface_ = robot_hw->get<hardware_interface::PositionJointInterface>();
	
	// subsribe the topic "cmd_pos"
        cmd_pos_sub_ = root_nh.subscribe<geometry_msgs::Vector3>("cmd_pos", 1, &GimbalController::cmdPosCallback, this);

        ros::NodeHandle nh_yaw 	 = ros::NodeHandle(controller_nh, "yaw");
        ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
        ros::NodeHandle nh_height= ros::NodeHandle(controller_nh, "height");
        if (!ctrl_yaw_.init(position_joint_interface_, nh_yaw) || !ctrl_pitch_.init(position_joint_interface_, nh_pitch) ||
            !ctrl_height_.init(position_joint_interface_, nh_height) )
            return false;
	// construct vector<joint> to control the acuators directly.
        joint_handles_.push_back(ctrl_yaw_.joint_);
        joint_handles_.push_back(ctrl_pitch_.joint_);
        joint_handles_.push_back(ctrl_height_.joint_);

	ROS_INFO("GIMBAL: INIT SUCCESS !");

        return true;
    }

    void GimbalController::update(const ros::Time& time, const ros::Duration& period)
    {
        geometry_msgs::Vector3 cmd_pos = cmd_rt_buffer_.readFromRT()->cmd_pos_;

        pos_cmd_.x = cmd_pos.x;
        pos_cmd_.y = cmd_pos.y;
        pos_cmd_.z = cmd_pos.z;
        
        moveJoint(time, period);
    }

    void GimbalController::cmdPosCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        cmd_struct_.cmd_pos_ = *msg;
        cmd_struct_.stamp_ = ros::Time::now();
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }

    void GimbalController::moveJoint(const ros::Time& time, const ros::Duration& period)
    {
	joint_handles_[0].setCommand(pos_cmd_.x);
	joint_handles_[1].setCommand(pos_cmd_.y);
	joint_handles_[2].setCommand(pos_cmd_.z);

	/*
        ctrl_yaw_.commandCB(float(pos_cmd_.x));
        ctrl_pitch_.commandCB(float(pos_cmd_.y));
        ctrl_height_.commandCB(float(pos_cmd_.z));
        ctrl_yaw_.update(time, period);
        ctrl_pitch_.update(time, period);
        ctrl_height_.update(time, period);*/
    }
	PLUGINLIB_EXPORT_CLASS(gimbal_controller::GimbalController, controller_interface::ControllerBase);
}  // namespace gimbal_controller
