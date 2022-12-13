#include "chassis_controller/swerve.h"
#include <pluginlib/class_list_macros.hpp>

namespace chassis_controller
{
    bool Swerve::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        ChassisBase::init(robot_hw, root_nh, controller_nh);

        ros::NodeHandle nh_lf_driving = ros::NodeHandle(controller_nh, "left_front_driving");
        ros::NodeHandle nh_rf_driving = ros::NodeHandle(controller_nh, "right_front_driving");
        ros::NodeHandle nh_lb_driving = ros::NodeHandle(controller_nh, "left_back_driving");
        ros::NodeHandle nh_rb_driving = ros::NodeHandle(controller_nh, "right_back_driving");
        if (!ctrl_lf_driving_.init(effort_joint_interface_, nh_lf_driving) || !ctrl_rf_driving_.init(effort_joint_interface_, nh_rf_driving) ||
            !ctrl_lb_driving_.init(effort_joint_interface_, nh_lb_driving) || !ctrl_rb_driving_.init(effort_joint_interface_, nh_rb_driving))
            return false;

        ros::NodeHandle nh_lf_heading = ros::NodeHandle(controller_nh, "left_front_heading");
        ros::NodeHandle nh_rf_heading = ros::NodeHandle(controller_nh, "right_front_heading");
        ros::NodeHandle nh_lb_heading = ros::NodeHandle(controller_nh, "left_back_heading");
        ros::NodeHandle nh_rb_heading = ros::NodeHandle(controller_nh, "right_back_heading");
        if (!ctrl_lf_heading_.init(effort_joint_interface_, nh_lf_heading) || !ctrl_rf_heading_.init(effort_joint_interface_, nh_rf_heading) ||
            !ctrl_lb_heading_.init(effort_joint_interface_, nh_lb_heading) || !ctrl_rb_heading_.init(effort_joint_interface_, nh_rb_heading))
            return false;

        ROS_INFO("CHASSIS : Initializing Completed");
        return true;
    }

    void Swerve::moveJoint(const ros::Time &time, const ros::Duration &period)
    {
        double a = (wheel_base_ + wheel_track_) / 2.0;
        ctrl_lf_heading_.setCommand((vel_cmd_.x - vel_cmd_.y - vel_cmd_.z * a) / wheel_radius_);
        ctrl_rf_heading_.setCommand((vel_cmd_.x + vel_cmd_.y + vel_cmd_.z * a) / wheel_radius_);
        ctrl_lb_heading_.setCommand((vel_cmd_.x + vel_cmd_.y - vel_cmd_.z * a) / wheel_radius_);
        ctrl_rb_heading_.setCommand((vel_cmd_.x - vel_cmd_.y + vel_cmd_.z * a) / wheel_radius_);
        ctrl_lf_heading_.update(time, period);
        ctrl_rf_heading_.update(time, period);
        ctrl_lb_heading_.update(time, period);
        ctrl_rb_heading_.update(time, period);

        ctrl_lf_driving_.setCommand((vel_cmd_.x - vel_cmd_.y - vel_cmd_.z * a) / wheel_radius_);
        ctrl_rf_driving_.setCommand((vel_cmd_.x + vel_cmd_.y + vel_cmd_.z * a) / wheel_radius_);
        ctrl_lb_driving_.setCommand((vel_cmd_.x + vel_cmd_.y - vel_cmd_.z * a) / wheel_radius_);
        ctrl_rb_driving_.setCommand((vel_cmd_.x - vel_cmd_.y + vel_cmd_.z * a) / wheel_radius_);
        ctrl_lf_driving_.update(time, period);
        ctrl_rf_driving_.update(time, period);
        ctrl_lb_driving_.update(time, period);
        ctrl_rb_driving_.update(time, period);
    }

    geometry_msgs::Twist Swerve::forwardKinematics()
    {
        geometry_msgs::Twist vel_data;
        double k = wheel_radius_ / 4.0;
        double lf_velocity = ctrl_lf_driving_.joint_.getVelocity();
        double rf_velocity = ctrl_rf_driving_.joint_.getVelocity();
        double lb_velocity = ctrl_lb_driving_.joint_.getVelocity();
        double rb_velocity = ctrl_rb_driving_.joint_.getVelocity();
        vel_data.linear.x = (rf_velocity + lf_velocity + lb_velocity + rb_velocity) * k;
        vel_data.linear.y = (rf_velocity - lf_velocity + lb_velocity - rb_velocity) * k;
        vel_data.angular.z = 2 * (rf_velocity - lf_velocity - lb_velocity + rb_velocity) * k / (wheel_base_ + wheel_track_);
        return vel_data;
    }
    PLUGINLIB_EXPORT_CLASS(chassis_controller::Swerve, controller_interface::ControllerBase)
}
