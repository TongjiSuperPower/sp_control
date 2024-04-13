#include "calibration_controller/calibration_controller.h"
#include <angles/angles.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace calibration_controller
{

    bool CalibrationController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        ROS_INFO("CALIBRATION : Initializing Started");
        ros::NodeHandle nh_z = ros::NodeHandle(controller_nh, "joint_z");
        ros::NodeHandle nh_x = ros::NodeHandle(controller_nh, "joint_x");
        ros::NodeHandle nh_y = ros::NodeHandle(controller_nh, "joint_y");
        ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "joint_pitch");
        
        gpio_command_interface_ = robot_hw->get<sp_control::GpioCommandInterface>();
           
        if (!ctrl_joint_z_.init(gpio_command_interface_, nh_z) || !ctrl_joint_x_.init(gpio_command_interface_, nh_x) ||
            !ctrl_joint_y_.init(gpio_command_interface_, nh_y) || !ctrl_joint_pitch_.init(gpio_command_interface_, nh_pitch))
        {
            ROS_ERROR("Some calibration params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
            return false;
        }

        joint_z_msg_pub_ = root_nh.advertise<std_msgs::Bool>("/cali_msg/joint_z",10);
        joint_x_msg_pub_ = root_nh.advertise<std_msgs::Bool>("/cali_msg/joint_x",10);
        joint_y_msg_pub_ = root_nh.advertise<std_msgs::Bool>("/cali_msg/joint_y",10);
        joint_pitch_msg_pub_ = root_nh.advertise<std_msgs::Bool>("/cali_msg/joint_pitch",10);

        return true;
    }
    
    void CalibrationController::starting(const ros::Time& time)
    {
        ROS_INFO_STREAM("Successfully started calibration_controller!");
    }


    void CalibrationController::update(const ros::Time &time, const ros::Duration &period)
    {
        ctrl_joint_z_.update(time, period);
        ctrl_joint_x_.update(time, period);
        ctrl_joint_y_.update(time, period);
        ctrl_joint_pitch_.update(time, period);

        std_msgs::Bool joint_z_msg, joint_x_msg, joint_y_msg, joint_pitch_msg;
        joint_z_msg.data = ctrl_joint_z_.getState();
        joint_x_msg.data = ctrl_joint_x_.getState();
        joint_y_msg.data = ctrl_joint_y_.getState();
        joint_pitch_msg.data = ctrl_joint_pitch_.getState();

        joint_z_msg_pub_.publish(joint_z_msg);
        joint_x_msg_pub_.publish(joint_x_msg);
        joint_y_msg_pub_.publish(joint_y_msg);
        joint_pitch_msg_pub_.publish(joint_pitch_msg);
    }

    void CalibrationController::stopping(const ros::Time& time)
    {
        ROS_INFO_STREAM("Successfully stopped calibration_controller!");
    }

    
    PLUGINLIB_EXPORT_CLASS(calibration_controller::CalibrationController, controller_interface::ControllerBase);
} // namespace calibration_controller
