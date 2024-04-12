#include "manipulator_controller/manipulator_controller.h"
#include <pluginlib/class_list_macros.hpp>
namespace manipulator_controller
{

    bool ManipulatorController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        ROS_INFO("MANIPULATOR: START TO INIT ...");
        ros::NodeHandle nh;

        // if (!controller_nh.getParam("publish_rate", publish_rate_) || !controller_nh.getParam("timeout", timeout_))
        // {
        //     ROS_ERROR("Some gimbal params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
        //     return false;
        // }

        // Determine whether it is in simulation environment or real environment
        // simulated engineer uses series pitch and little roll axes, while real engineer uses differential pitch and little roll axes. 
        if (!nh.getParam("simulate", simulate_))
        {
            ROS_ERROR_STREAM("Don't know simulate or real.");
            return false;
        }
        ROS_INFO_STREAM("SIMULATE:" << (simulate_?"TRUE":"FALSE"));




        ros::NodeHandle nh_z = ros::NodeHandle(controller_nh, "z");
        ros::NodeHandle nh_x = ros::NodeHandle(controller_nh, "x");
        ros::NodeHandle nh_y = ros::NodeHandle(controller_nh, "y");
      
        ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw");
        ros::NodeHandle nh_roll1 = ros::NodeHandle(controller_nh, "roll1");

        ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
        ros::NodeHandle nh_roll2 = ros::NodeHandle(controller_nh, "roll2");
        ros::NodeHandle nh_diff = ros::NodeHandle(controller_nh, "diff");

        // get the effort joint interface
        effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();


        if (!ctrl_z_.init(robot_hw, controller_nh, nh_z) || 
            !ctrl_x_.init(robot_hw, controller_nh, nh_x) || !ctrl_y_.init(effort_joint_interface_, nh_y))
            return false;
       
        if (!ctrl_yaw_.init(effort_joint_interface_, nh_yaw) || !ctrl_roll1_.init(effort_joint_interface_, nh_roll1))
            return false;

        if (simulate_)
        {
            if (!ctrl_pitch_.init(effort_joint_interface_, nh_pitch) || !ctrl_roll2_.init(effort_joint_interface_, nh_roll2))
                return false;
        }
        else 
        {
            if (!ctrl_diff_.init(robot_hw, controller_nh, nh_diff))
                return false;
        }


        y_has_friction_ = sp_common::getParam(controller_nh, "y/has_friction", false);
        y_friction_ = sp_common::getParam(controller_nh, "y/friction", 0.0);
        XmlRpc::XmlRpcValue xml_rpc_value;

        controller_nh.getParam("vel_limit", xml_rpc_value);
        for (auto it = xml_rpc_value.begin(); it != xml_rpc_value.end(); ++it)
            vel_limit_.push_back(it->second);
       
        controller_nh.getParam("upper_pos_limit", xml_rpc_value);
        for (auto it = xml_rpc_value.begin(); it != xml_rpc_value.end(); ++it)
            upper_pos_limit_.push_back(it->second);

        controller_nh.getParam("lower_pos_limit", xml_rpc_value);
        for (auto it = xml_rpc_value.begin(); it != xml_rpc_value.end(); ++it)
            lower_pos_limit_.push_back(it->second);

        // controller_nh.getParam("structure_coeff", xml_rpc_value);
        // for (auto it = xml_rpc_value.begin(); it != xml_rpc_value.end(); ++it)
        //     structure_coeff_.push_back(it->second);

        controller_nh.getParam("destination", xml_rpc_value);
        for (auto des = xml_rpc_value.begin(); des != xml_rpc_value.end(); ++des)
        {
            std::string destination_name = des->first;
            std::vector<double> destinations;

            for (auto it = des->second.begin(); it != des->second.end(); ++it)
            {
                destinations.push_back(it->second);
            }
            joint_destination_.emplace(std::make_pair(destination_name, destinations));         
        }

        
        cmd_quat_sub_ = root_nh.subscribe<geometry_msgs::Quaternion>("/cmd_quat", 1, &ManipulatorController::cmdQuatCallback, this);
        cmd_twist_sub_ = root_nh.subscribe<geometry_msgs::Twist>("/cmd_twist", 1, &ManipulatorController::cmdTwistCallback, this);
        cmd_joint_vel_sub_ = root_nh.subscribe<std_msgs::Float64MultiArray>("/cmd_joint_vel", 1, &ManipulatorController::cmdJointVelCallback, this);
        cmd_manipulator_sub_ = root_nh.subscribe<sp_common::ManipulatorCmd>("/cmd_manipulator", 1, &ManipulatorController::cmdManipulatorCallback, this);
        cmd_cc_sub_ = root_nh.subscribe<sp_common::CustomerControllerCmd>("/cmd_cc", 1, &ManipulatorController::cmdCustomerControllerCallback, this);

        msg_joint_z_cali_sub_ = root_nh.subscribe<std_msgs::Bool>("/cali_msg/joint_z", 1, &ManipulatorController::msgCaliZCallback, this);
        msg_joint_x_cali_sub_ = root_nh.subscribe<std_msgs::Bool>("/cali_msg/joint_x", 1, &ManipulatorController::msgCaliXCallback, this);
        msg_joint_y_cali_sub_ = root_nh.subscribe<std_msgs::Bool>("/cali_msg/joint_y", 1, &ManipulatorController::msgCaliYCallback, this);
        msg_joint_pitch_cali_sub_ = root_nh.subscribe<std_msgs::Bool>("/cali_msg/joint_pitch", 1, &ManipulatorController::msgCaliPitchCallback, this);
        cali_pub_ = root_nh.advertise<std_msgs::Bool>("/calibrated", 10);
        twist_cmd_ = Eigen::Matrix<double, 6, 1>::Zero();

        joint_cmd_ = joint_vel_cmd_= joint_pos_ = joint_pos_cmd_ = Eigen::Matrix<double, 7, 1>::Zero();

        coeff_ = Eigen::Matrix<double, 7, 4>::Zero();

        ROS_INFO("MANIPULATOR: INIT SUCCESS !");
        //initPosition();
        initiated_ = false;
        jacobian = Eigen::Matrix<double, 3, 4>::Zero();
        destination_ = NONE;

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


        geometry_msgs::Quaternion cmd_quat = cmd_rt_buffer_.readFromRT()->cmd_quat_;
        geometry_msgs::Twist cmd_twist = cmd_rt_buffer_.readFromRT()->cmd_twist_;
        std_msgs::Float64MultiArray cmd_joint_vel = cmd_rt_buffer_.readFromRT()->cmd_joint_vel_;
        // TODO: check whether "sp_common::ManipulatorCmd" can be deleted
        sp_common::ManipulatorCmd manipulator_cmd_ = cmd_rt_buffer_.readFromRT()->cmd_manipulator_;
        //cc_cmd_ = cmd_rt_buffer_.readFromRT()->cmd_manipulator_;

        quat_cmd_ = Eigen::Quaterniond(cmd_quat.w, cmd_quat.x, cmd_quat.y, cmd_quat.z);	

        //Update joint_vel_cmd_ message and twist_cmd_ message
        if (!cmd_joint_vel.data.empty())
        {
            for (int i = 0; i < 7; i++)
                joint_vel_cmd_[i] = cmd_joint_vel.data[i];
        }
        else
        {
            // for (int i = 0; i < 7; i++)
            //     joint_pos_cmd_[i] = joint_pos_[i];
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
            ROS_INFO_STREAM("Change mode: " << mode_);
            mode_changed_ = true;
            planed_ = false;
        }

        if (process_ != manipulator_cmd_.control_process)
        {
            process_ = manipulator_cmd_.control_process;
            process_changed_ = true;
        }

        destination_ = manipulator_cmd_.destination;
        //Choose mode
        //AUTO MODE: auto control, be used when camera is vaild.
        //MAUL MODE: maul control by customer controller (twist_cmd)
        //JOINT MODE: joint control by remote_controller (joint_vel_cmd)
        // maulMode(); 
        getPosition();
       

        switch (mode_)
        {
            case CALI:
            {
                caliMode();
                break;
            }
            
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
        last_final_push_ = final_push_;
        final_push_ = manipulator_cmd_.final_push;
        finalPush();
        moveJoint(time, period);
    }

    void ManipulatorController::moveJoint(const ros::Time &time, const ros::Duration &period)
    {
        //ROS_INFO_STREAM("joint_cmd_[0]:" << joint_cmd_[0]);
        // ROS_INFO_STREAM("joint_err_[0]:" << joint_cmd_[0] - ctrl_z_.getPosition());
        if (mode_ != AUTO)
        {
            for (int i = 0; i <  7; i++)
                joint_cmd_[i] += joint_vel_cmd_[i];
        }
        jointPosConstraint();


        ctrl_z_.setCommand(joint_cmd_[0]);
        ctrl_x_.setCommand(joint_cmd_[1]);
        ctrl_y_.setCommand(joint_cmd_[2]);

        ctrl_yaw_.setCommand(joint_cmd_[3]);
        ctrl_roll1_.setCommand(joint_cmd_[4]);

        if (simulate_)
        {
            ctrl_pitch_.setCommand(joint_cmd_[5]);
            ctrl_roll2_.setCommand(joint_cmd_[6]);
        }
        else
        {
            ctrl_diff_.setPitchCommand(joint_cmd_[5]);
            ctrl_diff_.setRollCommand(joint_cmd_[6]);
        }

        ctrl_z_.update(time, period);
        ctrl_x_.update(time, period);
        ctrl_y_.update(time, period);

        ctrl_yaw_.update(time, period);
        ctrl_roll1_.update(time, period); 

        if (simulate_)
        {
            ctrl_pitch_.update(time, period);
            ctrl_roll2_.update(time, period); 
        }
        else
            ctrl_diff_.update(time, period); 

        if (y_has_friction_)
        {
            if ((joint_cmd_[2] - ctrl_y_.joint_.getPosition()) > 0.001)
                ctrl_y_.joint_.setCommand(ctrl_y_.joint_.getCommand() + y_friction_);
            else if ((joint_cmd_[2] - ctrl_y_.joint_.getPosition()) < -0.001)
                ctrl_y_.joint_.setCommand(ctrl_y_.joint_.getCommand() - y_friction_);
        }

    }

    void ManipulatorController::caliMode()
    {
        if (mode_changed_)
        {
            ROS_INFO_STREAM("Enter cali mode");
            mode_changed_ = false;
        }

        if (!z_calibrated_)
            joint_vel_cmd_[0] = 0.001;
        else    
            joint_vel_cmd_[0] = 0.00;
        if (!x_calibrated_)
            joint_vel_cmd_[1] = 0.001;
        else    
            joint_vel_cmd_[1] = 0.00;
         if (!y_calibrated_)
            joint_vel_cmd_[2] = 0.001;
        else    
            joint_vel_cmd_[2] = 0.00;
        if (!pitch_calibrated_)
            joint_vel_cmd_[5] = 0.001;
        else    
            joint_vel_cmd_[5] = 0.00;

        if (z_calibrated_ && x_calibrated_ && y_calibrated_ && pitch_calibrated_ )
        {
            std_msgs::Bool calibrated;
            calibrated.data = true;
            cali_pub_.publish(calibrated);
        }


    }

    void ManipulatorController::autoMode()
    {
        if (mode_changed_)
        {
              //modeChangeProtect();
            ROS_INFO_STREAM("Enter auto mode");
            mode_changed_ = false;
        }


        if (!planed_ && destination_ != NONE)
        {     
            std::string name;
            if (destination_ == HOME)
                name = "home";
            else if (destination_ == GROUND)
                name = "ground";
            else if (destination_ == SLIVER)
                name = "sliver";
            else if (destination_ == GOLD)
                name = "gold";
            else if (destination_ == VISION)
                name = "vision";

            ROS_INFO_STREAM("Destination: "<< name);

            for (int i = 0; i < 7; i++)
            {
                joint_pos_cmd_[i] = joint_destination_[name][i];
                spline_.init(joint_pos_[i], joint_pos_cmd_[i], vel_limit_[i]);
                Eigen::Vector4d coeff;
                spline_.computeCoeff(coeff);
                coeff_(i, 0) = coeff[0];
                coeff_(i, 1) = coeff[1];
                coeff_(i, 2) = coeff[2];
                coeff_(i, 3) = coeff[3];
            }

            begin_time_ = ros::Time::now();
            planed_ = true;
        }

        if (planed_)
        {
            ros::Duration duration;
            now_time_ = ros::Time::now();
            duration = now_time_ - begin_time_;
            double sec = duration.toSec();
            bool reached = true;
            // ROS_INFO_STREAM("yaw_joint");
            // ROS_INFO_STREAM(joint_pos_cmd_[3]);
            // ROS_INFO_STREAM(joint_cmd_[3]);
            // ROS_INFO_STREAM(joint_pos_[3]);
            // ROS_INFO_STREAM("----------------------");


            for (int i = 0; i < 7; i++)
            { 
    
                if (abs(joint_pos_cmd_[i] - joint_cmd_[i]) > 0.002)
                    joint_cmd_[i] = coeff_(i, 0) + coeff_(i, 1)*sec + coeff_(i, 2)*pow(sec, 2) + coeff_(i, 3)*pow(sec, 3);
                else
                    joint_cmd_[i] = joint_pos_cmd_[i];
                if (abs(joint_pos_cmd_[i] - joint_pos_[i]) > 0.02)
                    reached = false;
            }

            if (reached)
            {
                ROS_INFO_STREAM("Destination Reached");
                planed_ = false;
            }

        }

       
        
      
    
        // quat2Euler(); 
        // cartesian_cmd_[0] = 0.0;
        // cartesian_cmd_[1] = 0.4;
        // cartesian_cmd_[2] = 0.4;
        // cartesian_cmd_[3] = 0.3;


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
            //modeChangeProtect();
            ROS_INFO_STREAM("Enter maul mode");
            mode_changed_ = false;
        }
 
        updateJacobian();
        calJointVel();
    }

    void ManipulatorController::jointMode()
    {
        if (mode_changed_)
        {
            ROS_INFO_STREAM("Enter joint mode");
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
        ctrl_x_.update(time, period);
        ctrl_y_.update(time, period);

        ctrl_yaw_.update(time, period);
        ctrl_roll1_.update(time, period); 
        if (simulate_)
        {
            ctrl_pitch_.update(time, period);
            ctrl_roll2_.update(time, period); 
        }
        else
            ctrl_diff_.update(time, period);
 
 

        joint_cmd_[0] = ctrl_z_.getPosition();
        joint_cmd_[1] = ctrl_x_.getPosition();
        joint_cmd_[2] = ctrl_y_.joint_.getPosition();
        joint_cmd_[3] = ctrl_yaw_.joint_.getPosition();
        joint_cmd_[4] = ctrl_roll1_.joint_.getPosition();
        if (simulate_)
        {
            joint_cmd_[5] = ctrl_pitch_.joint_.getPosition();
            joint_cmd_[6] = ctrl_roll2_.joint_.getPosition();
        }
        else
        {
            joint_cmd_[5] = ctrl_diff_.getPitchPosition();
            joint_cmd_[6] = ctrl_diff_.getRollPosition();
        }
   
        getPosition();
    }


    void ManipulatorController::getPosition()
    {   
        joint_pos_[0] = ctrl_z_.getPosition();
        joint_pos_[1] = ctrl_x_.getPosition();
        joint_pos_[2] = ctrl_y_.joint_.getPosition();
        joint_pos_[3] = ctrl_yaw_.joint_.getPosition();
        joint_pos_[4] = ctrl_roll1_.joint_.getPosition();
        if (simulate_)
        {
            joint_pos_[5] = ctrl_pitch_.joint_.getPosition();
            joint_pos_[6] = ctrl_roll2_.joint_.getPosition();
        }
        else
        {
            joint_pos_[5] = ctrl_diff_.getPitchPosition();
            joint_pos_[6] = ctrl_diff_.getRollPosition();
        }
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
        Eigen::JacobiSVD<Eigen::MatrixXd> svd =
        Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
        Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();
        //Change joint_vel_cmd_ by using Jacobian
        Eigen::Vector3d omega = Eigen::Vector3d::Zero();
        Eigen::Vector4d vel = Eigen::Vector4d::Zero();
        omega[0] = twist_cmd_[0];
        omega[1] = twist_cmd_[1];
        omega[2] = twist_cmd_[2];
        vel = pseudo_inverse * omega;

        // if (cc_cmd_.twist_enable)
        // {
            joint_vel_cmd_[3] = vel[0];
            joint_vel_cmd_[4] = vel[1];
            joint_vel_cmd_[5] = vel[2];
            joint_vel_cmd_[6] = vel[3];
        // }
        // else
        // {
        //     joint_vel_cmd_[3] = 0.0;
        //     joint_vel_cmd_[4] = 0.0;
        //     joint_vel_cmd_[5] = 0.0;
        //     joint_vel_cmd_[6] = 0.0;
        // }


   
        // joint_vel_cmd_[6] = -0.6 * twist_cmd_[0];
        // joint_vel_cmd_[5] = 0.6 * twist_cmd_[1];
        // joint_vel_cmd_[3] = 0.6 * twist_cmd_[2];

        // joint_vel_cmd_[0] = twist_cmd_[5];
        // joint_vel_cmd_[1] = twist_cmd_[3];
        // joint_vel_cmd_[2] = twist_cmd_[4];


    }


    void ManipulatorController::updateJacobian()
    {
        jacobian(0, 0) = -cos(joint_pos_[4]) * sin(joint_pos_[5]);
        jacobian(0, 1) = cos(joint_pos_[5]);
        jacobian(0, 2) = 0;
        jacobian(0, 3) = 1;

        jacobian(1, 0) = sin(joint_pos_[4]) * cos(joint_pos_[6]) + cos(joint_pos_[4]) * cos(joint_pos_[5]) * sin(joint_pos_[6]);
        jacobian(1, 1) = sin(joint_pos_[5]) * sin(joint_pos_[6]);
        jacobian(1, 2) = cos(joint_pos_[6]);
        jacobian(1, 3) = 0;

        jacobian(2, 0) = -sin(joint_pos_[4]) * sin(joint_pos_[6]) + cos(joint_pos_[4]) * cos(joint_pos_[5]) * cos(joint_pos_[6]);
        jacobian(2, 1) = sin(joint_pos_[5]) * cos(joint_pos_[6]);
        jacobian(2, 2) = -sin(joint_pos_[6]);
        jacobian(2, 3) = 0;
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
        double vel;
        if (!last_final_push_ && final_push_)
        {
            vel = -0.05;   
            joint_vel_cmd_[1] = -(sin(joint_pos_[3]) * cos(joint_pos_[5]) + cos(joint_pos_[3]) * sin(joint_pos_[4]) * sin(joint_pos_[5])) * vel;
            joint_vel_cmd_[2] = (cos(joint_pos_[3]) * cos(joint_pos_[5]) - sin(joint_pos_[3]) * sin(joint_pos_[4]) * sin(joint_pos_[5])) * vel;
            joint_vel_cmd_[0] = cos(joint_pos_[4]) * sin(joint_pos_[5]) * vel;
        }      
        else if (last_final_push_ && !final_push_)
        {     
            vel = 0.05;
            joint_vel_cmd_[1] = -(sin(joint_pos_[3]) * cos(joint_pos_[5]) + cos(joint_pos_[3]) * sin(joint_pos_[4]) * sin(joint_pos_[5])) * vel;
            joint_vel_cmd_[2] = (cos(joint_pos_[3]) * cos(joint_pos_[5]) - sin(joint_pos_[3]) * sin(joint_pos_[4]) * sin(joint_pos_[5])) * vel;
            joint_vel_cmd_[0] = cos(joint_pos_[4]) * sin(joint_pos_[5]) * vel;
        }
    }

    void ManipulatorController::msgCaliZCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
            z_calibrated_ = true;
    }

    void ManipulatorController::msgCaliXCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
            x_calibrated_ = true;
    }

    void ManipulatorController::msgCaliYCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
            y_calibrated_ = true;
    }

    void ManipulatorController::msgCaliPitchCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
            pitch_calibrated_ = true;
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

    void ManipulatorController::cmdJointVelCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        cmd_struct_.cmd_joint_vel_ = *msg;
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }

    void ManipulatorController::cmdManipulatorCallback(const sp_common::ManipulatorCmd::ConstPtr &msg)
    {
        cmd_struct_.cmd_manipulator_ = *msg;
        cmd_struct_.stamp_ = ros::Time::now();
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }

    void ManipulatorController::cmdCustomerControllerCallback(const sp_common::CustomerControllerCmd::ConstPtr &msg)
    {
        cmd_struct_.cmd_cc_ = *msg;
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }
}

PLUGINLIB_EXPORT_CLASS(manipulator_controller::ManipulatorController, controller_interface::ControllerBase)