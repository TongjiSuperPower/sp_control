#include "adjustment_controller/adjustment_controller.h"
#include <angles/angles.h>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace adjustment_controller
{
    template <typename T>
    T getParam(ros::NodeHandle &pnh, const std::string &param_name, const T &default_val)
    {
        T param_val;
        pnh.param<T>(param_name, param_val, default_val);
        return param_val;
    }

    bool AdjustmentController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        ROS_INFO_STREAM("ADJUSTMENT : Initializing Started");
        ros::NodeHandle ctrl_nh = ros::NodeHandle(controller_nh, "actuator1");

        if (!controller_nh.getParam("cmd_vel", cmd_vel_))
        {
            ROS_ERROR("Velocity value was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
            return false;
        }

        effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

       
        if (!ctrl_vel_.init(effort_joint_interface_, ctrl_nh))
        {
            return false;
        }
        ROS_INFO_STREAM("ADJUSTMENT: INIT SUCCESS !");

        return true;
    }
    void AdjustmentController::starting(const ros::Time& time)
    {
        ROS_INFO_STREAM("Successfully started adjustment_controller!");
    }



    void AdjustmentController::update(const ros::Time &time, const ros::Duration &period)
    {
        ctrl_vel_.setCommand(cmd_vel_);
        ctrl_vel_.update(time, period);  
    }

    void AdjustmentController::stopping(const ros::Time& time)
    {
        ROS_INFO_STREAM("Successfully stopped adjustment_controller!");
    }

    
    PLUGINLIB_EXPORT_CLASS(adjustment_controller::AdjustmentController, controller_interface::ControllerBase);
} // namespace adjustment_controller
