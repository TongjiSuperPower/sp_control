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

        y_has_friction_ = sp_common::getParam(controller_nh, "y/has_friction", false);
        y_friction_ = sp_common::getParam(controller_nh, "y/friction", 0.0);

        vel_limit_.push_back(sp_common::getParam(controller_nh, "vel_limit/z", 0.1));
        vel_limit_.push_back(sp_common::getParam(controller_nh, "vel_limit/x1", 0.5));
        vel_limit_.push_back(sp_common::getParam(controller_nh, "vel_limit/x2", 0.5));
        vel_limit_.push_back(sp_common::getParam(controller_nh, "vel_limit/y", 0.3));
        vel_limit_.push_back(sp_common::getParam(controller_nh, "vel_limit/pitch", 1.57));
        vel_limit_.push_back(sp_common::getParam(controller_nh, "vel_limit/yaw", 1.57));
        vel_limit_.push_back(sp_common::getParam(controller_nh, "vel_limit/roll", 1.57));

        upper_pos_limit_.push_back(sp_common::getParam(controller_nh, "upper_pos_limit/z", 0.0));
        upper_pos_limit_.push_back(sp_common::getParam(controller_nh, "upper_pos_limit/x1", 0.45));
        upper_pos_limit_.push_back(sp_common::getParam(controller_nh, "upper_pos_limit/x2", 0.45));
        upper_pos_limit_.push_back(sp_common::getParam(controller_nh, "upper_pos_limit/y", 0.15));
        upper_pos_limit_.push_back(sp_common::getParam(controller_nh, "upper_pos_limit/pitch", 1.57));
        upper_pos_limit_.push_back(sp_common::getParam(controller_nh, "upper_pos_limit/yaw", 1.57));
        upper_pos_limit_.push_back(sp_common::getParam(controller_nh, "upper_pos_limit/roll", 1.57));

        lower_pos_limit_.push_back(sp_common::getParam(controller_nh, "lower_pos_limit/z", -0.15));
        lower_pos_limit_.push_back(sp_common::getParam(controller_nh, "lower_pos_limit/x1", 0.00));
        lower_pos_limit_.push_back(sp_common::getParam(controller_nh, "lower_pos_limit/x2", 0.00));
        lower_pos_limit_.push_back(sp_common::getParam(controller_nh, "lower_pos_limit/y", -0.15));
        lower_pos_limit_.push_back(sp_common::getParam(controller_nh, "lower_pos_limit/pitch", -1.57));
        lower_pos_limit_.push_back(sp_common::getParam(controller_nh, "lower_pos_limit/yaw", -1.57));
        lower_pos_limit_.push_back(sp_common::getParam(controller_nh, "lower_pos_limit/roll", -1.57));

        structure_coeff_.l1_ = sp_common::getParam(controller_nh, "structure_coeff/l1", 0.6);
        structure_coeff_.l2_ = sp_common::getParam(controller_nh, "structure_coeff/l2", 0.25);
        structure_coeff_.l3_ = sp_common::getParam(controller_nh, "structure_coeff/l3", 0.1);
        structure_coeff_.l4_ = sp_common::getParam(controller_nh, "structure_coeff/l4", 0.1);

        

        cmd_quat_sub_ = root_nh.subscribe<geometry_msgs::Quaternion>("cmd_quat", 1, &ManipulatorController::cmdQuatCallback, this);
        cmd_twist_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_twist", 1, &ManipulatorController::cmdTwistCallback, this);
        cmd_joint_sub_ = root_nh.subscribe<std_msgs::Float64MultiArray>("cmd_joint", 1, &ManipulatorController::cmdJointCallback, this);
        cmd_manipulator_sub_ = root_nh.subscribe<sp_common::ManipulatorCmd>("cmd_manipulator", 1, &ManipulatorController::cmdManipulatorCallback, this);
        
        twist_cmd_ = Eigen::Matrix<double, 6, 1>::Zero();

        joint_cmd_ = joint_vel_cmd_= joint_pos_ = Eigen::Matrix<double, 7, 1>::Zero();

        ROS_INFO("MANIPULATOR: INIT SUCCESS !");
        //initPosition();
        initiated_ = false;
        jacobian = Eigen::Matrix3d::Zero();

        return true;
    }

    void ManipulatorController::update(const ros::Time &time, const ros::Duration &period)
    {
        if (!initiated_)
        {
            initPosition(time, period);
            
            initiated_ = true;
            ROS_INFO_STREAM("INIT JOINT POSITION");
        }
        //ROS_INFO_STREAM("joint_cmd_[5]:" << joint_cmd_[5]);


        geometry_msgs::Quaternion cmd_quat = cmd_rt_buffer_.readFromRT()->cmd_quat_;
        geometry_msgs::Twist cmd_twist = cmd_rt_buffer_.readFromRT()->cmd_twist_;
        std_msgs::Float64MultiArray cmd_joint = cmd_rt_buffer_.readFromRT()->cmd_joint_;
        sp_common::ManipulatorCmd manipulator_cmd_ = cmd_rt_buffer_.readFromRT()->cmd_manipulator_;

        quat_cmd_ = Eigen::Quaterniond(cmd_quat.w, cmd_quat.x, cmd_quat.y, cmd_quat.z);	

        //Update joint_vel_cmd_ message and twist_cmd_ message
        if (!cmd_joint.data.empty())
        {
            for (int i = 0; i < 7; i++)
                joint_vel_cmd_[i] = cmd_joint.data[i];
        }
        else
        {
            for (int i = 0; i < 7; i++)
                joint_vel_cmd_[i] = 0.0;
        }


        
        twist_cmd_[0] = cmd_twist.angular.x;
        twist_cmd_[1] = cmd_twist.angular.y;
        twist_cmd_[2] = cmd_twist.angular.z;
        twist_cmd_[3] = cmd_twist.linear.x;
        twist_cmd_[4] = cmd_twist.linear.y;
        twist_cmd_[5] = cmd_twist.linear.z;
      

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

        //Choose mode
        //AUTO MODE: auto control, be used when camera is vaild.
        //MAUL MODE: maul control by customer controller (twist_cmd)
        //JOINT MODE: joint control by remote_controller (joint_vel_cmd)
        //maulMode(); 
        // switch (mode_)
        // {
            
        //     case AUTO:
        //     {
        //         autoMode();       
        //         break;
        //     }
        //     case MAUL:
        //     {
        //         maulMode();
        //         break;
        //     }
        //     case JOINT:
        //     {
        //         jointMode();
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
        ROS_INFO_STREAM("joint_cmd_[0]:" << joint_cmd_[0]);
        ROS_INFO_STREAM("joint_err_[0]:" << joint_cmd_[0] - ctrl_z_.getPosition());
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

        ctrl_z_.update(time, period);
        ctrl_x1_.update(time, period);
        ctrl_x2_.update(time, period); 
        ctrl_y_.update(time, period);

        ctrl_pitch_.update(time, period);
        ctrl_yaw_.update(time, period);
        ctrl_roll_.update(time, period); 

        if (y_has_friction_)
        {
            if ((joint_cmd_[3] - ctrl_y_.joint_.getPosition()) > 0.001)
                ctrl_y_.joint_.setCommand(ctrl_y_.joint_.getCommand() + y_friction_);
            else if ((joint_cmd_[3] - ctrl_y_.joint_.getPosition()) < -0.001)
                ctrl_y_.joint_.setCommand(ctrl_y_.joint_.getCommand() - y_friction_);
        }

    }

    void ManipulatorController::autoMode()
    {
        if (mode_changed_)
        {
            ROS_INFO_STREAM("Enter auto mode");
            mode_changed_ = false;
        }

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
        if (mode_changed_)
        {
            ROS_INFO_STREAM("Enter maul mode");
            mode_changed_ = false;
        }
        //getPosition();
        updateJacobian();
        calJointVel();
    }

    void ManipulatorController::jointMode()
    {
        if (mode_changed_)
        {
            ROS_INFO_STREAM("Enter joiot mode");
            mode_changed_ = false;
        }
    }

    void ManipulatorController::quat2Euler()
    {
        quat_cmd_.normalized();
        Eigen::Matrix3d rot = quat_cmd_.matrix();
        euler_cmd_[0] = std::atan2(-rot(1, 2), rot(1, 1));
        euler_cmd_[1] = std::atan2(rot(1, 0), std::sqrt(rot(1, 1) * rot(1, 1) + rot(1, 2) * rot(1, 2)));
        euler_cmd_[2] = std::atan2(-rot(2, 0), rot(0, 0));
    }

    void ManipulatorController::initPosition(const ros::Time &time, const ros::Duration &period)
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

    }


    void ManipulatorController::getPosition()
    {   
        joint_pos_[0] = ctrl_z_.getPosition();
        joint_pos_[1] = ctrl_x1_.getPosition();
        joint_pos_[2] = ctrl_x2_.getPosition();
        joint_pos_[3] = ctrl_y_.joint_.getPosition();
        joint_pos_[4] = ctrl_pitch_.joint_.getPosition();
        joint_pos_[5] = ctrl_yaw_.joint_.getPosition();
        joint_pos_[6] = ctrl_roll_.joint_.getPosition();
    }

    void ManipulatorController::jointPosConstraint()
    {
        for (int i = 0; i < 7; i++)
        {
            if (joint_cmd_[i] > upper_pos_limit_[i])
                joint_cmd_[i] = upper_pos_limit_[i];
            else if (joint_cmd_[i] < lower_pos_limit_[i])
                joint_cmd_[i] = lower_pos_limit_[i];
        }
    }

    void ManipulatorController::calJointVel()
    {
        //Calculate Jacobian's (pseudo_)inverse.
        // Eigen::JacobiSVD<Eigen::MatrixXd> svd =
        // Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
        // Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();
        // Change joint_vel_cmd_ by using Jacobian
        Eigen::Vector3d omega = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        omega[0] = twist_cmd_[0];
        omega[1] = twist_cmd_[1];
        omega[2] = twist_cmd_[2];
        vel = jacobian.inverse() * omega;
        joint_vel_cmd_[4] = 0.4 * vel[0];
        joint_vel_cmd_[5] = 0.4 * vel[1];
        joint_vel_cmd_[6] = 0.4 * vel[2];

        joint_vel_cmd_[0] = twist_cmd_[5];
        joint_vel_cmd_[1] = twist_cmd_[3] / 2;
        joint_vel_cmd_[2] = twist_cmd_[3] / 2;
        joint_vel_cmd_[3] = twist_cmd_[4];
        // joint_vel_cmd_[4] = twist_cmd_[1];
        // joint_vel_cmd_[5] = twist_cmd_[2];
        // joint_vel_cmd_[6] = twist_cmd_[0];
    }


    void ManipulatorController::updateJacobian()
    {
        jacobian(0, 0) = sin(joint_pos_[5]);
        jacobian(0, 1) = 0;
        jacobian(0, 2) = 1;

        jacobian(1, 0) = cos(joint_pos_[5]) * cos(joint_pos_[6]);
        jacobian(1, 1) = sin(joint_pos_[6]);
        jacobian(1, 2) = 0;

        jacobian(2, 0) = -cos(joint_pos_[5]) * sin(joint_pos_[6]);
        jacobian(2, 1) = cos(joint_pos_[6]);
        jacobian(2, 2) = 0;
        // jacobian(1, 4) = jacobian(3, 1) = jacobian(3, 2) = jacobian(4, 3) = jacobian(5, 0) = 1.0;

        // jacobian(0, 5) = sin(joint_pos_[4]);
        // jacobian(0, 6) = cos(joint_pos_[4]) * cos(joint_pos_[5]);
        // jacobian(1, 6) = sin(joint_pos_[5]);
        // jacobian(2, 5) = cos(joint_pos_[4]);
        // jacobian(2, 6) = -sin(joint_pos_[4]) * cos(joint_pos_[5]);

        // jacobian(3, 4) = -sin(joint_pos_[4]) * cos(joint_pos_[5]) * structure_coeff_.l3_ - (sin(joint_pos_[4]) * sin(joint_pos_[5]) * sin(joint_pos_[6]) + cos(joint_pos_[4]) * cos(joint_pos_[6]) ) * structure_coeff_.l4_;
        // jacobian(5, 4) = -cos(joint_pos_[4]) * cos(joint_pos_[5]) * structure_coeff_.l3_ - (cos(joint_pos_[4]) * sin(joint_pos_[5]) * sin(joint_pos_[6]) + sin(joint_pos_[4]) * cos(joint_pos_[6]) ) * structure_coeff_.l4_;

        // jacobian(3, 5) = -cos(joint_pos_[4]) * sin(joint_pos_[5]) * structure_coeff_.l3_  + cos(joint_pos_[4]) * cos(joint_pos_[5]) * sin(joint_pos_[6]) * structure_coeff_.l4_;
        // jacobian(4, 5) = cos(joint_pos_[5]) * structure_coeff_.l3_  + sin(joint_pos_[5]) * sin(joint_pos_[6]) * structure_coeff_.l4_;
        // jacobian(5, 5) = sin(joint_pos_[4]) * sin(joint_pos_[5]) * structure_coeff_.l3_  - sin(joint_pos_[4]) * cos(joint_pos_[5]) * sin(joint_pos_[6]) * structure_coeff_.l4_;
        
        // jacobian(3, 6) = (-sin(joint_pos_[4]) * sin(joint_pos_[6]) + cos(joint_pos_[4]) * sin(joint_pos_[5]) * cos(joint_pos_[6])) * structure_coeff_.l4_;
        // jacobian(4, 6) = -cos(joint_pos_[5]) * cos(joint_pos_[6]) * structure_coeff_.l4_;
        // jacobian(5, 6) = (-cos(joint_pos_[4]) * sin(joint_pos_[6]) - sin(joint_pos_[4]) * sin(joint_pos_[5]) * cos(joint_pos_[6])) * structure_coeff_.l4_;

    }

    void ManipulatorController::finalPush()
    {


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