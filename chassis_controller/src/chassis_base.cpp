#include "chassis_controller/chassis_base.h"
#include <angles/angles.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace chassis_controller
{
	template <typename T>
	T getParam(ros::NodeHandle& pnh, const std::string& param_name, const T& default_val)
	{
		T param_val;
		pnh.param<T>(param_name, param_val, default_val);
		return param_val;
	}

    bool ChassisBase::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
		std::cout<<"CHASSIS: READY TO INIT"<<std::endl;
		ROS_INFO("CHASSIS: READY TO INIT");
        if (!controller_nh.getParam("publish_rate", publish_rate_) || !controller_nh.getParam("timeout", timeout_) ||
            !controller_nh.getParam("power/vel_coeff", velocity_coeff_) ||
            !controller_nh.getParam("power/effort_coeff", effort_coeff_) ||
            !controller_nh.getParam("power/power_offset", power_offset_))
        {
            ROS_ERROR("Some chassis params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
			std::cout<<"Some chassis params doesn't given"<<controller_nh.getNamespace().c_str()<<std::endl;
            return false;
        }
        wheel_radius_ = getParam(controller_nh, "wheel_radius", 0.02);
        wheel_track_ = getParam(controller_nh, "wheel_track", 0.410);
        wheel_base_ = getParam(controller_nh, "wheel_base", 0.320);
        twist_angular_ = getParam(controller_nh, "twist_angular", M_PI / 6);

        effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

        // Setup odometry realtime publisher + odom message constant fields
       cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisBase::cmdVelCallback, this);

       ros::NodeHandle nh_lf = ros::NodeHandle(controller_nh, "left_front");
       ros::NodeHandle nh_rf = ros::NodeHandle(controller_nh, "right_front");
       ros::NodeHandle nh_lb = ros::NodeHandle(controller_nh, "left_back");
       ros::NodeHandle nh_rb = ros::NodeHandle(controller_nh, "right_back");
       if (!ctrl_lf_.init(effort_joint_interface_, nh_lf) || !ctrl_rf_.init(effort_joint_interface_, nh_rf) ||
           !ctrl_lb_.init(effort_joint_interface_, nh_lb) || !ctrl_rb_.init(effort_joint_interface_, nh_rb))
           return false;
       joint_handles_.push_back(ctrl_lf_.joint_);
       joint_handles_.push_back(ctrl_rf_.joint_);
       joint_handles_.push_back(ctrl_lb_.joint_);
       joint_handles_.push_back(ctrl_rb_.joint_);

        return true;
    }

    void ChassisBase::update(const ros::Time& time, const ros::Duration& period)
    {
        geometry_msgs::Twist cmd_vel = cmd_rt_buffer_.readFromRT()->cmd_vel_;

        if ((time - cmd_rt_buffer_.readFromRT()->stamp_).toSec() > timeout_)
        {
            vel_cmd_.x = 0.;
            vel_cmd_.y = 0.;
            vel_cmd_.z = 0.;
        }
        else
        {
            vel_cmd_.x = cmd_vel.linear.x;
            vel_cmd_.y = cmd_vel.linear.y;
            vel_cmd_.z = cmd_vel.angular.z;
        }

        moveJoint(time, period);
    }


    void ChassisBase::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        cmd_struct_.cmd_vel_ = *msg;
        cmd_struct_.stamp_ = ros::Time::now();
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }

    void ChassisBase::moveJoint(const ros::Time& time, const ros::Duration& period)
    {
        double a = (wheel_base_ + wheel_track_) / 2.0;
        ctrl_lf_.setCommand((vel_cmd_.x - vel_cmd_.y - vel_cmd_.z * a) / wheel_radius_);
        ctrl_rf_.setCommand((vel_cmd_.x + vel_cmd_.y + vel_cmd_.z * a) / wheel_radius_);
        ctrl_lb_.setCommand((vel_cmd_.x + vel_cmd_.y - vel_cmd_.z * a) / wheel_radius_);
        ctrl_rb_.setCommand((vel_cmd_.x - vel_cmd_.y + vel_cmd_.z * a) / wheel_radius_);
        ctrl_lf_.update(time, period);
        ctrl_rf_.update(time, period);
        ctrl_lb_.update(time, period);
        ctrl_rb_.update(time, period);
    }

    geometry_msgs::Twist ChassisBase::forwardKinematics()
    {
        geometry_msgs::Twist vel_data;
        double k = wheel_radius_ / 4.0;
        double lf_velocity = ctrl_lf_.joint_.getVelocity();
        double rf_velocity = ctrl_rf_.joint_.getVelocity();
        double lb_velocity = ctrl_lb_.joint_.getVelocity();
        double rb_velocity = ctrl_rb_.joint_.getVelocity();
        vel_data.linear.x = (rf_velocity + lf_velocity + lb_velocity + rb_velocity) * k;
        vel_data.linear.y = (rf_velocity - lf_velocity + lb_velocity - rb_velocity) * k;
        vel_data.angular.z = 2 * (rf_velocity - lf_velocity - lb_velocity + rb_velocity) * k / (wheel_base_ + wheel_track_);
        return vel_data;
    }
	PLUGINLIB_EXPORT_CLASS(chassis_controller::ChassisBase, controller_interface::ControllerBase);
}  // namespace rm_chassis_controllers
