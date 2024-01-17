//
// Created by CherryBlossomNight on 2023/12/13
//

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

        if (!yaw_pid_.init(ros::NodeHandle(nh_yaw, "pid")) || !pitch_pid_.init(ros::NodeHandle(nh_pitch, "pid")))
            return false;

        ROS_INFO("GIMBAL: INIT SUCCESS !");
        imu_sub_ = root_nh.subscribe<sensor_msgs::Imu>("imu_msg", 1, &GimbalEffort::cmdIMUCallback, this);
        angle_pub_ = root_nh.advertise<std_msgs::Float64>("yaw_angle", 10);

        return true;
    }

    void GimbalEffort::update(const ros::Time &time, const ros::Duration &period)
    {
        if (!initiated_)
        {
            initCmd(time, period);
        }
        
        cmd_vel_ = cmd_rt_buffer_.readFromRT()->cmd_gimbal_vel_;
        getPosition(time, period);
        moveJoint(time, period);
    }

    void GimbalEffort::initCmd(const ros::Time &time, const ros::Duration &period)
    {   
        getAngle();
        yaw_cmd_ = yaw_pos_;
        pitch_cmd_ = pitch_pos_;
        initiated_ = true;
    }

    void GimbalEffort::getPosition(const ros::Time &time, const ros::Duration &period)
    {
        getAngle();
    }

    void GimbalEffort::getAngle()
    {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(quat_msg_, quat);
        quat.normalize();
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        yaw_pos_ = yaw;
        pitch_pos_ = pitch;
    }

    double GimbalEffort::posLimit(double pos)
    {
        while (pos > M_PI || pos <= -M_PI)
        {
            if (pos > M_PI)
                pos -= 2 * M_PI;
            else 
                pos += 2 * M_PI;
        }
        return pos;
    }


    void GimbalEffort::moveJoint(const ros::Time &time, const ros::Duration &period)
    {
        yaw_cmd_ += cmd_vel_.z;
        if ((ctrl_pitch_.joint_.getPosition() < -0.39 && cmd_vel_.y <= 0) ||
            (ctrl_pitch_.joint_.getPosition() > 0.39 && cmd_vel_.y >= 0) ||
            (ctrl_pitch_.joint_.getPosition() >= -0.39 && ctrl_pitch_.joint_.getPosition() <= 0.39))
            pitch_cmd_ += cmd_vel_.y;
        // ROS_INFO_STREAM("yaw_cmd_"<<yaw_cmd_);
        // ROS_INFO_STREAM("yaw_pos_"<<yaw_pos_);

       
        double yaw_error = angles::shortest_angular_distance(yaw_cmd_, yaw_pos_);
        double pitch_error = angles::shortest_angular_distance(pitch_cmd_, pitch_pos_);
        yaw_pid_.computeCommand(yaw_error, period);
        pitch_pid_.computeCommand(pitch_error, period);
     
        ctrl_yaw_.joint_.setCommand(yaw_pid_.getCurrentCmd());
        ctrl_pitch_.joint_.setCommand(pitch_pid_.getCurrentCmd());

        pubYawAngle();
        // ctrl_yaw_.update(time, period);
        // ctrl_pitch_.update(time, period);
    }

    void GimbalEffort::pubYawAngle()
    {
        double yaw_angle = ctrl_yaw_.joint_.getPosition();
        yaw_angle = posLimit(yaw_angle);
        std_msgs::Float64 yaw;
        yaw.data = yaw_angle;
        angle_pub_.publish(yaw);

        // geometry_msgs::Twist chassis_vel_cmd;
        // chassis_vel_cmd.linear.x = cos(yaw_angle) * cmd_vel_.linear.x + sin(yaw_angle) * cmd_vel_.linear.y;
        // chassis_vel_cmd.linear.y = -sin(yaw_angle) * cmd_vel_.linear.x + cos(yaw_angle) * cmd_vel_.linear.y;
        // double follow_error = angles::shortest_angular_distance(0, yaw_angle);
        // follow_pid_.computeCommand(-follow_error, period);  
        // chassis_vel_cmd.angular.z = follow_pid_.getCurrentCmd();
        // cmd_vel_pub_.publish(chassis_vel_cmd);
        
    }

    void GimbalEffort::cmdIMUCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        quat_msg_ = msg->orientation;
    }
    PLUGINLIB_EXPORT_CLASS(gimbal_controller::GimbalEffort, controller_interface::ControllerBase);
} // namespace gimbal_controller
