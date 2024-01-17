#include "shooter_controller/shooter_controller.h"
#include <angles/angles.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace shooter_controller
{

    template <typename T>
    T getParam(ros::NodeHandle &pnh, const std::string &param_name, const T &default_val)
    {
        T param_val;
        pnh.param<T>(param_name, param_val, default_val);
        return param_val;
    }


    bool ShooterController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        if (!controller_nh.getParam("push_per_rotation", push_per_rotation_)
        ||  !controller_nh.getParam("fric_speed", fric_speed_)
        ||  !controller_nh.getParam("continuous_speed", continuous_speed_))
            return false;

        cmd_subscriber_ = root_nh.subscribe<sp_common::ShooterCmd>("/cmd_shooter", 1,  &ShooterController::cmdShooterCallback, this);
        ros::NodeHandle nh_fric_l = ros::NodeHandle(controller_nh, "friction_left");
        ros::NodeHandle nh_fric_r = ros::NodeHandle(controller_nh, "friction_right");
        ros::NodeHandle nh_trigger = ros::NodeHandle(controller_nh, "trigger");
        effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (!ctrl_fric_l_.init(effort_joint_interface_, nh_fric_l) ||
           !ctrl_fric_r_.init(effort_joint_interface_, nh_fric_r) ||
           !ctrl_trigger_.init(effort_joint_interface_, nh_trigger))        
                return false;
        ROS_INFO_STREAM("SHOOTER : Initializing Completed");
        return true;
    }

    void ShooterController::starting(const ros::Time& time)
    {
        fric_mode_ = FRIC_ON;
        shoot_process_ = SHOOT_STOP;
        ROS_INFO_STREAM("[Shooter] Enter SHOOT_STOP");
        shoot_process_changed_ = false;
    }

    void ShooterController::update(const ros::Time& time, const ros::Duration& period)
    {
        shooter_cmd_ = cmd_rt_buffer_.readFromRT()->shooter_cmd_;

        if (fric_mode_ != shooter_cmd_.fric_mode)
        {
            fric_mode_ = shooter_cmd_.fric_mode;
            fric_mode_changed_ = true;
        }

        if (fric_mode_changed_)
        {
            if (fric_mode_ == FRIC_ON)
                ROS_INFO_STREAM("[Fric] Enter FRIC_ON");
            else
                ROS_INFO_STREAM("[Fric] Enter FRIC_OFF");
            fric_mode_changed_ = false;
        }

        if (shoot_process_ != shooter_cmd_.shoot_process)
        {
            shoot_process_ = shooter_cmd_.shoot_process;
            shoot_process_changed_ = true;
        }

        setFricSpeed(shooter_cmd_, time, period);
            
        switch (shoot_process_)
        {
            case SHOOT_STOP:
            {
                stop(time, period);
                break;
            }
            case SHOOT_READY:
            {
                ready(time, period);
                break;
            }
            case SHOOT_SINGLE:
            {
                single(time, period);
                break;
            }
            case SHOOT_CONTINUOUS:
            {
                continuous(time, period);
                break;
            }
        }
        if (shoot_process_ != SHOOT_STOP)
            ctrl_trigger_.update(time, period);

       
    }



    void ShooterController::stop(const ros::Time& time, const ros::Duration& period)
    {
        if (shoot_process_changed_)
        {  
            shoot_process_changed_ = false;
            ROS_INFO_STREAM("[Shooter] Enter SHOOT_STOP");
        }
        ctrl_trigger_.joint_.setCommand(0.0);
       
    }

    void ShooterController::ready(const ros::Time& time, const ros::Duration& period)
    {
        if (shoot_process_changed_)
        {  
            shoot_process_changed_ = false;
            ROS_INFO("[Shooter] Enter SHOOT_READY");
            //normalize();
            trigger_cmd_ = ctrl_trigger_.joint_.getPosition();
            ctrl_trigger_.setCommand(trigger_cmd_);
           
        }
    }

    void ShooterController::single(const ros::Time& time, const ros::Duration& period)
    {
        if (shoot_process_changed_)
        {  
            shoot_process_changed_ = false;
            ROS_INFO("[Shooter] Enter SHOOT_SINGLE");
            is_shot_ = false;
        }
        if (!is_shot_)
        {
            trigger_cmd_ += 2. * M_PI / static_cast<double>(push_per_rotation_);
            ctrl_trigger_.setCommand(trigger_cmd_);
            is_shot_ = true;          
        }
    }

    void ShooterController::continuous(const ros::Time& time, const ros::Duration& period)
    {
        if (shoot_process_changed_)
        {  
            shoot_process_changed_ = false;
            ROS_INFO("[Shooter] Enter SHOOT_CONTINUOUS");
        }
            trigger_cmd_ += continuous_speed_;
            ctrl_trigger_.setCommand(trigger_cmd_);    
        
    }




    void ShooterController::normalize()
    {
        double push_angle = 2. * M_PI / static_cast<double>(push_per_rotation_);
        ctrl_trigger_.setCommand(push_angle * std::floor((ctrl_trigger_.joint_.getPosition() + 0.01) / push_angle));
    }

    void ShooterController::setFricSpeed(const sp_common::ShooterCmd& cmd, const ros::Time& time, const ros::Duration& period)
    {
        if (shooter_cmd_.fric_mode == FRIC_ON)
        {
            ctrl_fric_l_.setCommand(fric_speed_);
            ctrl_fric_r_.setCommand(-fric_speed_);
            ctrl_fric_l_.update(time, period);
            ctrl_fric_r_.update(time, period);
        }
        else
        {
            // ctrl_fric_l_.joint_.setCommand(0.0);
            // ctrl_fric_r_.joint_.setCommand(0.0);
            ctrl_fric_l_.setCommand(0.0);
            ctrl_fric_r_.setCommand(0.0);
            ctrl_fric_l_.update(time, period);
            ctrl_fric_r_.update(time, period);
        }

    }

    void ShooterController::cmdShooterCallback(const sp_common::ShooterCmd::ConstPtr &msg)
    {
        cmd_struct_.shooter_cmd_ = *msg;
        cmd_struct_.stamp_ = ros::Time::now();
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }
    PLUGINLIB_EXPORT_CLASS(shooter_controller::ShooterController, controller_interface::ControllerBase);

}// namespace shooter_controller

