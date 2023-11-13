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
        ros::NodeHandle nh_z = ros::NodeHandle(controller_nh, "z");
        ros::NodeHandle nh_x1 = ros::NodeHandle(controller_nh, "x1");
        ros::NodeHandle nh_x2 = ros::NodeHandle(controller_nh, "x2");
        ros::NodeHandle nh_y = ros::NodeHandle(controller_nh, "y");
        ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
        ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw");
        ros::NodeHandle nh_roll = ros::NodeHandle(controller_nh, "roll");
        // get the position joint interface
        effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

        // subsribe the topic "cmd_pos"
        if (!ctrl_z_.init(robot_hw, controller_nh, nh_z) || !ctrl_x1_.init(robot_hw, controller_nh, nh_x1) 
         || !ctrl_x2_.init(robot_hw, controller_nh, nh_x2) || !ctrl_y_.init(effort_joint_interface_, nh_y))
            return false;
       
        if (!ctrl_pitch_.init(effort_joint_interface_, nh_pitch) || !ctrl_yaw_.init(effort_joint_interface_, nh_yaw) || !ctrl_roll_.init(effort_joint_interface_, nh_roll))
            return false;
        // construct vector<joint> to control the acuators directly.

        

        cmd_quat_sub_ = root_nh.subscribe<geometry_msgs::Quaternion>("cmd_quat", 1, &ManipulatorController::cmdQuatCallback, this);
        cmd_twist_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_twist", 1, &ManipulatorController::cmdTwistCallback, this);
        cmd_joint_sub_ = root_nh.subscribe<std_msgs::Float64MultiArray>("cmd_joint", 1, &ManipulatorController::cmdJointCallback, this);
        cmd_manipulator_sub_ = root_nh.subscribe<sp_common::ManipulatorCmd>("cmd_manipulator", 1, &ManipulatorController::cmdManipulatorCallback, this);
        
        twist_cmd_ = Eigen::VectorXd::Zero(6);

        joint_cmd_ = joint_vel_cmd_ = std::vector<double>(7, 0);

        ROS_INFO("MANIPULATOR: INIT SUCCESS !");
        //initPosition();
        initiated_ = false;

        return true;
    }

    void ManipulatorController::update(const ros::Time &time, const ros::Duration &period)
    {
        geometry_msgs::Quaternion cmd_quat = cmd_rt_buffer_.readFromRT()->cmd_quat_;
        geometry_msgs::Twist cmd_twist = cmd_rt_buffer_.readFromRT()->cmd_twist_;
        std_msgs::Float64MultiArray cmd_joint = cmd_rt_buffer_.readFromRT()->cmd_joint_;
        sp_common::ManipulatorCmd manipulator_cmd_ = cmd_rt_buffer_.readFromRT()->cmd_manipulator_;

        if (!cmd_joint.data.empty())
        {
            for (int i = 0; i < 7; i++)
                joint_vel_cmd_[i] = cmd_joint.data[i];
        }
        if (!initiated_)
        {
            ctrl_z_.update(time, period);
            ctrl_x1_.update(time, period);
            ctrl_x2_.update(time, period); 
            ctrl_y_.update(time, period);


            ctrl_pitch_.update(time, period);
            ctrl_yaw_.update(time, period);
            ctrl_roll_.update(time, period); 

            joint_cmd_[0] = ctrl_z_.getPosition();
            joint_cmd_[1] = ctrl_x1_.getPosition();
            joint_cmd_[2] = ctrl_x2_.getPosition();
            joint_cmd_[3] = ctrl_y_.joint_.getPosition();
            joint_cmd_[4] = ctrl_pitch_.joint_.getPosition();
            joint_cmd_[5] = ctrl_yaw_.joint_.getPosition();
            joint_cmd_[6] = ctrl_roll_.joint_.getPosition();
            initiated_ = true;
            ROS_INFO_STREAM("INIT");
            ROS_INFO_STREAM("joint_cmd_[0]:" << joint_cmd_[0]);
        }
        ROS_INFO_STREAM("joint_cmd_[0]:" << joint_cmd_[0]);

        //joint_cmd_[0] = cmd_joint.data.at(0);
        quat_cmd_ = Eigen::Quaterniond(cmd_quat.w, cmd_quat.x, cmd_quat.y, cmd_quat.z);	
        twist_cmd_[0] = cmd_twist.linear.x;
        twist_cmd_[1] = cmd_twist.linear.y;
        twist_cmd_[2] = cmd_twist.linear.z;
        twist_cmd_[3] = cmd_twist.angular.x;
        twist_cmd_[4] = cmd_twist.angular.y;
        twist_cmd_[5] = cmd_twist.angular.z;
      

        // Change mode
        if (mode_ != manipulator_cmd_.control_mode)
        {
            mode_ = manipulator_cmd_.control_mode;
            ROS_INFO_STREAM("Change mode" << mode_);
            mode_changed_ = true;
        }

        if (process_ != manipulator_cmd_.control_process)
        {
            process_ = manipulator_cmd_.control_process;
            process_changed_ = true;
        }

        switch (mode_)
        {
            
            case AUTO:
            {
                autoMode();       
                break;
            }
            case MAUL:
            {
                maulMode();
                break;
            }
            case JOINT:
            {
                jointMode();
                break;
            }
        }

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
        // if (process_ == READY || process_ == MOVE || process_ == DONE)
        // {

            for (int i = 0; i <  7; i++)
                joint_cmd_[i] += joint_vel_cmd_[i];
            jointPosConstraint();
            ctrl_z_.setCommand(joint_cmd_[0]);
            ctrl_x1_.setCommand(joint_cmd_[1]);
            ctrl_x2_.setCommand(joint_cmd_[2]);
            ctrl_y_.setCommand(joint_cmd_[3]);

            ctrl_pitch_.setCommand(joint_cmd_[4]);
            ctrl_yaw_.setCommand(joint_cmd_[5]);
            ctrl_roll_.setCommand(joint_cmd_[6]);


            // ctrl_z_.setCommand(xyz_cmd_[0]);
            // ctrl_x1_.setCommand(xyz_cmd_[1]);
            // ctrl_x2_.setCommand(xyz_cmd_[2]);
            // ctrl_y_.setCommand(xyz_cmd_[3]);

            // ctrl_pitch_.setCommand(rpy_cmd_[0]);
            // ctrl_yaw_.setCommand(rpy_cmd_[1]);
            // ctrl_roll_.setCommand(rpy_cmd_[2]);


            ctrl_z_.update(time, period);
            ctrl_x1_.update(time, period);
            ctrl_x2_.update(time, period); 
            ctrl_y_.update(time, period);


            ctrl_pitch_.update(time, period);
            ctrl_yaw_.update(time, period);
            ctrl_roll_.update(time, period); 

        //     process_ = MOVE;
        // }
    }

    void ManipulatorController::autoMode()
    {

        quat2Euler(); 
        cartesian_cmd_[0] = 0.0;
        cartesian_cmd_[1] = 0.4;
        cartesian_cmd_[2] = 0.4;
        cartesian_cmd_[3] = 0.3;


        //if (manipulator_cmd_.is_start_vision_exchange)
        {
            // if (!z_completed_)
            // {
            //     xyz_cmd_[0] = 0.0;
            //     //ROS_INFO_STREAM("Move Z");
            //     if (abs(xyz_cmd_[0] - ctrl_z_.joint_.getPosition()) < 0.04)
            //         z_completed_ = true;
            // }
            // if (z_completed_)
            // {
            //     xyz_cmd_[1] = cartesian_cmd_[1];
            //     xyz_cmd_[2] = cartesian_cmd_[2];
            //     //ROS_INFO_STREAM("Move X");
            //     if (abs(xyz_cmd_[1] - ctrl_x1_.joint_.getPosition()) < 0.002 && abs(xyz_cmd_[2] - ctrl_x2_.joint_.getPosition()))
            //         x_completed_ = true;             
            // }
            // if (x_completed_ && z_completed_)
            // {
            //     rpy_cmd_ = euler_cmd_;
            //     //ROS_INFO_STREAM("Move RPY");
            //     rpy_completed_ = true;

            // }
            // if (rpy_completed_ && x_completed_ && z_completed_)
            // {
            //     //ROS_INFO_STREAM("Done!");
            //     //z_completed_ = x_completed_ = y_completed_ = x_completed_ = false;
            // }
        }

    } 

    void ManipulatorController::maulMode()
    {
        //euler_cmd_[0] += joints

    }

    void ManipulatorController::jointMode()
    {
        //euler_cmd_[0] = manipulator_cmd_.joint5;

    }


    void ManipulatorController::quat2Euler()
    {
        quat_cmd_.normalized();
        Eigen::Matrix3d rot = quat_cmd_.matrix();
        euler_cmd_[0] = std::atan2(-rot(1, 2), rot(1, 1));
        euler_cmd_[1] = std::atan2(rot(1, 0), std::sqrt(rot(1, 1) * rot(1, 1) + rot(1, 2) * rot(1, 2)));
        euler_cmd_[2] = std::atan2(-rot(2, 0), rot(0, 0));
    }

    void ManipulatorController::initPosition()
    {
        xyz_cmd_[0] = ctrl_z_.getPosition();
        xyz_cmd_[1] = ctrl_x1_.getPosition();
        xyz_cmd_[2] = ctrl_x2_.getPosition();
        xyz_cmd_[3] = ctrl_y_.joint_.getPosition();
        rpy_cmd_[0] = ctrl_pitch_.joint_.getPosition();
        rpy_cmd_[1] = ctrl_yaw_.joint_.getPosition();
        rpy_cmd_[2] = ctrl_roll_.joint_.getPosition();

        //ROS_INFO_STREAM(euler_cmd_);

    }

    void ManipulatorController::jointPosConstraint()
    {
        if (joint_cmd_[0] > 0.00)
            joint_cmd_[0] = 0.00;
        else if (joint_cmd_[0] < -0.15)
            joint_cmd_[0] = -0.15;

        if (joint_cmd_[1] > 0.50)
            joint_cmd_[1] = 0.50;
        else if (joint_cmd_[1] < 0.00)
            joint_cmd_[1] = 0.00;

        if (joint_cmd_[2] > 0.45)
            joint_cmd_[2] = 0.45;
        else if (joint_cmd_[2] < 0.00)
            joint_cmd_[2] = 0.00;

        if (joint_cmd_[3] > 0.15)
            joint_cmd_[3] = 0.15;
        else if (joint_cmd_[3] < -0.15)
            joint_cmd_[3] = -0.15;

        if (joint_cmd_[4] > 1.57)
            joint_cmd_[4] = 1.57;
        else if (joint_cmd_[4] < 0.00)
            joint_cmd_[4] = 0.00;

        if (joint_cmd_[5] > 1.57)
            joint_cmd_[5] = 1.57;
        else if (joint_cmd_[5] < -1.57)
            joint_cmd_[5] = -1.57;

        if (joint_cmd_[6] > 1.57)
            joint_cmd_[6] = 1.57;
        else if (joint_cmd_[6] < -1.57)
            joint_cmd_[6] = -1.57;
    }



    void ManipulatorController::cmdQuatCallback(const geometry_msgs::Quaternion::ConstPtr &msg)
    {
        cmd_struct_.cmd_quat_ = *msg;
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }

    void ManipulatorController::cmdTwistCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        cmd_struct_.cmd_twist_ = *msg;
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }

    void ManipulatorController::cmdJointCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        cmd_struct_.cmd_joint_ = *msg;
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }

    void ManipulatorController::cmdManipulatorCallback(const sp_common::ManipulatorCmd::ConstPtr &msg)
    {
        cmd_struct_.cmd_manipulator_ = *msg;
        cmd_struct_.stamp_ = ros::Time::now();
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }
}

PLUGINLIB_EXPORT_CLASS(manipulator_controller::ManipulatorController, controller_interface::ControllerBase)