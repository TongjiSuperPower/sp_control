#include "manipulator_controller/manipulator_controller.h"
#include <pluginlib/class_list_macros.hpp>
namespace manipulator_controller
{

    bool ManipulatorController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        ROS_INFO("MANIPULATOR: START TO INIT ...");

        // if (!controller_nh.getParam("publish_rate", publish_rate_) || !controller_nh.getParam("timeout", timeout_))
        // {
        //     ROS_ERROR("Some gimbal params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
        //     return false;
        // }
        ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
        ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw");
        ros::NodeHandle nh_roll = ros::NodeHandle(controller_nh, "roll");
        // get the position joint interface
        effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

        // subsribe the topic "cmd_pos"
       
        if (!ctrl_pitch_.init(effort_joint_interface_, nh_pitch) || !ctrl_yaw_.init(effort_joint_interface_, nh_yaw) || !ctrl_roll_.init(effort_joint_interface_, nh_roll))
            return false;
        // construct vector<joint> to control the acuators directly.

        

        cmd_quat_sub_ = root_nh.subscribe<geometry_msgs::Quaternion>("cmd_quat", 1, &ManipulatorController::cmdQuatCallback, this);
        ROS_INFO("GIMBAL: INIT SUCCESS !");
        return true;

    }

    void ManipulatorController::update(const ros::Time &time, const ros::Duration &period)
    {
        geometry_msgs::Quaternion cmd_quat = cmd_rt_buffer_.readFromRT()->cmd_quat_;
        sp_common::ManipulatorCmd cmd_manipulator_ = cmd_rt_buffer_.readFromRT()->cmd_manipulator_;
        quat_cmd_ = Eigen::Quaterniond(cmd_quat.w, cmd_quat.x, cmd_quat.y, cmd_quat.z);	
        quat_cmd_.normalized();
        euler_cmd_ = quat_cmd_.matrix().eulerAngles(1,2,0);

        if (mode_ != cmd_manipulator_.control_mode)
        {
            mode_ = cmd_manipulator_.control_mode;
            mode_changed_ = true;
        }

        if (process_ != cmd_manipulator_.control_process)
        {
            process_ = cmd_manipulator_.control_process;
            process_changed_ = true;
        }

        // switch (mode_)
        // {
            
        //     case AUTO:
        //     {
        //         autoMode(time, period);
        //         break;
        //     }
        //     case MUAL:
        //     {
        //         mual(time, period);
        //         break;
        //     }
        //     case JOINT:
        //     {
        //         joint(time, period);
        //         break;
        //     }
        // }

        // switch (process_)
        // {       
        //     case STOP:
        //     {
        //         stopProcess(time, period);
        //         break;
        //     }
        //     case REDAY:
        //     {
        //         readyProcess(time, period);
        //         break;
        //     }
        // }

        moveJoint(time, period);
    }

    void ManipulatorController::moveJoint(const ros::Time &time, const ros::Duration &period)
    {
        if (process_ == READY || process_ == MOVE || process_ == DONE)
        {
            ctrl_pitch_.setCommand(euler_cmd_[0]);
            ctrl_yaw_.setCommand(euler_cmd_[1]);
            ctrl_roll_.setCommand(euler_cmd_[2]);

            ctrl_pitch_.update(time, period);
            ctrl_yaw_.update(time, period);
            ctrl_roll_.update(time, period); 

            process_ = MOVE;
        }
    }



    void ManipulatorController::cmdQuatCallback(const geometry_msgs::Quaternion::ConstPtr &msg)
    {
        cmd_struct_.cmd_quat_ = *msg;
        cmd_struct_.stamp_ = ros::Time::now();
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }

    void ManipulatorController::cmdManipulatorCallback(const sp_common::ManipulatorCmd::ConstPtr &msg)
    {
        cmd_struct_.cmd_manipulator_ = *msg;
        cmd_struct_.stamp_ = ros::Time::now();
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }
}