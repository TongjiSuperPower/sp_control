#include "manipulator_controller/manipulator_controller.h"
#include <pluginlib/class_list_macros.hpp>
namespace manipulator_controller
{

    bool ManipulatorController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        ROS_INFO("MANIPULATOR: START TO INIT ...");
        ros::NodeHandle nh;

        // Determine whether it is in simulation environment or real environment
        // Simulated engineer uses series pitch and little roll axes, while real engineer uses differential pitch and little roll axes. 
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

        // Get the effort joint interface
        effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();


        if (!ctrl_z_.init(robot_hw, controller_nh, nh_z) || 
            !ctrl_x_.init(robot_hw, controller_nh, nh_x) || !ctrl_y_.init(effort_joint_interface_, nh_y))
            return false;
       
        if (!ctrl_yaw_.init(effort_joint_interface_, nh_yaw) || !ctrl_roll1_.init(effort_joint_interface_, nh_roll1))
            return false;
        //  Determine the use of series axes or differential axes based on simulate property
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

        // Read y axis parameters about friction
        y_has_friction_ = sp_common::getParam(controller_nh, "y/has_friction", false);
        y_friction_ = sp_common::getParam(controller_nh, "y/friction", 0.0);
        XmlRpc::XmlRpcValue xml_rpc_value;

        // Read some parameters
        controller_nh.getParam("vel_limit", xml_rpc_value);
        for (auto it = xml_rpc_value.begin(); it != xml_rpc_value.end(); ++it)
            vel_limit_.push_back(it->second);
       
        controller_nh.getParam("upper_pos_limit", xml_rpc_value);
        for (auto it = xml_rpc_value.begin(); it != xml_rpc_value.end(); ++it)
            upper_pos_limit_.push_back(it->second);

        controller_nh.getParam("lower_pos_limit", xml_rpc_value);
        for (auto it = xml_rpc_value.begin(); it != xml_rpc_value.end(); ++it)
            lower_pos_limit_.push_back(it->second);

        controller_nh.getParam("position_threshold", xml_rpc_value);
        for (auto it = xml_rpc_value.begin(); it != xml_rpc_value.end(); ++it)
            position_threshold_.push_back(it->second);

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

        controller_nh.getParam("process", xml_rpc_value);
        for (auto pro = xml_rpc_value.begin(); pro != xml_rpc_value.end(); ++pro)
        {
            std::string process_name = pro->first;

            if (pro->second.hasMember("process_num"))
            {
                int process_num = pro->second["process_num"];
                joint_process_num_.emplace(std::make_pair(process_name, process_num)); 
            }

            if (pro->second.hasMember("destinations"))
            {
                    std::vector<std::vector<double>> destinations;
                    for (int row = 0; row < pro->second["destinations"].size(); row++)
                    {
                        XmlRpc::XmlRpcValue row_value;
                        row_value = (pro->second["destinations"])[row];
                        std::vector<double> destination;
                        for (auto column = row_value.begin(); column != row_value.end(); ++column)
                            destination.push_back(column->second);
                        destinations.push_back(destination);
                    }
              
                    joint_process_.emplace(std::make_pair(process_name, destinations)); 
            }

            if (pro->second.hasMember("delay_time"))
            {
                double delay_time = pro->second["delay_time"];
                process_delay_time_.emplace(std::make_pair(process_name, delay_time)); 
            }
        }

        

        // Generate subscribers and publishers
        cmd_quat_sub_ = root_nh.subscribe<geometry_msgs::Quaternion>("/cmd_quat", 1, &ManipulatorController::cmdQuatCallback, this);
        cmd_twist_sub_ = root_nh.subscribe<geometry_msgs::Twist>("/cmd_twist", 1, &ManipulatorController::cmdTwistCallback, this);
        cmd_joint_vel_sub_ = root_nh.subscribe<std_msgs::Float64MultiArray>("/cmd_joint_vel", 1, &ManipulatorController::cmdJointVelCallback, this);
        cmd_manipulator_sub_ = root_nh.subscribe<sp_common::ManipulatorCmd>("/cmd_manipulator", 1, &ManipulatorController::cmdManipulatorCallback, this);
        cmd_cc_sub_ = root_nh.subscribe<sp_common::CustomerControllerCmd>("/cmd_cc", 1, &ManipulatorController::cmdCustomerControllerCallback, this);
        //cmd_vision_sub_ = root_nh.subscribe<sp_common::VisionCmd>("/cmd_vision", 1, &ManipulatorController::cmdVisionCallback, this);
        //cmd_vision_sub_ = root_nh.subscribe<geometry_msgs::Pose>("/exchangestation_detect_pose", 1, &ManipulatorController::cmdVisionCallback, this);
        cmd_vision_sub_ = root_nh.subscribe<std_msgs::Float64MultiArray>("/ex_Euler", 1, &ManipulatorController::cmdVisionCallback, this);

        msg_joint_z_cali_sub_ = root_nh.subscribe<std_msgs::Bool>("/cali_msg/joint_z", 1, &ManipulatorController::msgCaliZCallback, this);
        msg_joint_x_cali_sub_ = root_nh.subscribe<std_msgs::Bool>("/cali_msg/joint_x", 1, &ManipulatorController::msgCaliXCallback, this);
        msg_joint_y_cali_sub_ = root_nh.subscribe<std_msgs::Bool>("/cali_msg/joint_y", 1, &ManipulatorController::msgCaliYCallback, this);
        msg_joint_pitch_cali_sub_ = root_nh.subscribe<std_msgs::Bool>("/cali_msg/joint_pitch", 1, &ManipulatorController::msgCaliPitchCallback, this);
        cali_pub_ = root_nh.advertise<std_msgs::Bool>("/calibrated", 1);
        vision_pub_ = root_nh.advertise<std_msgs::Bool>("/begin_identify", 1);
        pos_pub_ = root_nh.advertise<std_msgs::Float64MultiArray>("/joint_pos", 1);
        twist_cmd_ = Eigen::Matrix<double, 6, 1>::Zero();

        // Initialize some values
        joint_cmd_ = joint_vel_cmd_= joint_pos_ = joint_pos_cmd_ = Eigen::Matrix<double, 7, 1>::Zero();
        vision_pose_ = Eigen::Matrix<double, 6, 1>::Zero();
        coeff_ = Eigen::Matrix<double, 7, 4>::Zero();
        initiated_ = false;
        jacobian = Eigen::Matrix<double, 3, 4>::Zero();
        destination_ = NONE;
        vision_execuated_ = true;

        y_lock_pid_.init(ros::NodeHandle(controller_nh, "y/lock_pid"));
        y_lock_pid_.reset();
        ROS_INFO("MANIPULATOR: INIT SUCCESS !");

        return true;
    }

    void ManipulatorController::update(const ros::Time &time, const ros::Duration &period)
    {
        if (!initiated_) // Init process to read the joints' positions
        {
            initPosition(time, period);      
            initiated_ = true;
            ROS_INFO_STREAM("INIT JOINT POSITION");
        }

        // Get subscribed messages
        geometry_msgs::Quaternion cmd_quat = cmd_rt_buffer_.readFromRT()->cmd_quat_;
        geometry_msgs::Twist cmd_twist = cmd_rt_buffer_.readFromRT()->cmd_twist_;
        std_msgs::Float64MultiArray cmd_joint_vel = cmd_rt_buffer_.readFromRT()->cmd_joint_vel_;
        // TODO: check whether "sp_common::ManipulatorCmd" can be deleted
        sp_common::ManipulatorCmd manipulator_cmd_ = cmd_rt_buffer_.readFromRT()->cmd_manipulator_;
        //cc_cmd_ = cmd_rt_buffer_.readFromRT()->cmd_manipulator_;
        y_lock_ = manipulator_cmd_.y_lock;
        //geometry_msgs::Pose vision_pose = cmd_rt_buffer_.readFromRT()->cmd_vision_;
        //vision_pose_ = vision_pose;
        std_msgs::Float64MultiArray vision_pose = cmd_rt_buffer_.readFromRT()->cmd_vision_;
        if (!vision_pose.data.empty())
        {
            vision_pose_[0] = vision_pose.data[0];
            vision_pose_[1] = vision_pose.data[1];
            vision_pose_[2] = vision_pose.data[2];
            vision_pose_[3] = vision_pose.data[3];
            vision_pose_[4] = vision_pose.data[4];
            vision_pose_[5] = vision_pose.data[5];
        }
        if (!y_lock_)
            y_lock_pid_.reset();
        quat_cmd_ = Eigen::Quaterniond(cmd_quat.w, cmd_quat.x, cmd_quat.y, cmd_quat.z);	

        //Update joint_vel_cmd_ message and twist_cmd_ message
        if (!cmd_joint_vel.data.empty())
        {
            for (int i = 0; i < 7; i++)
                joint_vel_cmd_[i] = cmd_joint_vel.data[i];
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
            destination_ = NONE;
            
        }

        if (auto_type_ != manipulator_cmd_.auto_type)
            auto_type_ = manipulator_cmd_.auto_type;

        // if (process_ != manipulator_cmd_.control_process)
        // {
        //     process_ = manipulator_cmd_.control_process;
        //     process_changed_ = true;
        // }


        if (manipulator_cmd_.destination != NONE && !planed_)
            destination_ = manipulator_cmd_.destination;
        if (manipulator_cmd_.process != STOP && !planed_)
            process_ = manipulator_cmd_.process;
        getPosition();
        //Choose mode
        //AUTO MODE: auto control, be used when camera is vaild.
        //MAUL MODE: maul control by customer controller (twist_cmd)
        //JOINT MODE: joint control by remote_controller (joint_vel_cmd)
        // maulMode(); 

        // caliMode();
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

        final_push_ = manipulator_cmd_.final_push;
        finalPush();
        moveJoint(time, period);
    }

    void ManipulatorController::moveJoint(const ros::Time &time, const ros::Duration &period)
    {
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
            ctrl_pitch_.setCommand(joint_cmd_[5] - pitch_offset_);
            ctrl_roll2_.setCommand(joint_cmd_[6]);
        }
        else
        {
            ctrl_diff_.setPitchCommand(joint_cmd_[5] - pitch_offset_);
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
            if (!y_lock_)
            {
                if ((joint_cmd_[2] - ctrl_y_.joint_.getPosition()) > 0.005)
                    ctrl_y_.joint_.setCommand(ctrl_y_.joint_.getCommand() + y_friction_);
                else if ((joint_cmd_[2] - ctrl_y_.joint_.getPosition()) < -0.005)
                    ctrl_y_.joint_.setCommand(ctrl_y_.joint_.getCommand() - y_friction_);
                else
                {
                    double eff = (joint_cmd_[2] - ctrl_y_.joint_.getPosition()) / 0.005;
                    ctrl_y_.joint_.setCommand(ctrl_y_.joint_.getCommand() + eff * y_friction_);
                }
            }
            else
            {
               
                double y_error = joint_cmd_[2] - joint_pos_[2];
                y_lock_pid_.computeCommand(y_error, period);
                double y_cmd = y_lock_pid_.getCurrentCmd();
                //ROS_INFO_STREAM(y_cmd);
                ctrl_y_.joint_.setCommand(y_cmd);
            }
        }
    }

    void ManipulatorController::caliMode()
    {
        if (mode_changed_)
        {
            ROS_INFO_STREAM("Enter cali mode");
            mode_changed_ = false;
        }
        if (!z_calibrated_ && z_calimsg_)
        {
            pitch_offset_ = -1.5708 - joint_pos_[5];
            joint_cmd_[5] += pitch_offset_;
            joint_pos_[5] += pitch_offset_;
            z_calibrated_ = true;
          
        }
        ROS_INFO_STREAM(joint_pos_[5]);

     

        // if (z_calibrated_ && x_calibrated_ && y_calibrated_ && pitch_calibrated_ )
        if (z_calibrated_)
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

        if (auto_type_ == DESTINATION)
            destinationMode();
        else if (auto_type_ == PROCESS)
            processMode();
    }

    void ManipulatorController::destinationMode()
    {
        if (destination_ == NONE)
            return;
        std::string destination_name_;
        if (!planed_)
        {
            if  (destination_ == HOME)
                destination_name_ = "home";
            else if (destination_ == GROUND)
                destination_name_ = "ground";
            else if (destination_ == SLIVER)
                destination_name_ = "sliver";
            else if (destination_ == GOLD)
                destination_name_ = "gold";
            else if (destination_ == EXCHANGE)
                destination_name_ = "exchange";
            else if (destination_ == VISION)
                destination_name_ = "vision"; 

            ROS_INFO_STREAM("Destination: "<< destination_name_);

            if (destination_name_ == "vision")
            {
                // joint_cmd_[6] = vision_pose_[0];
                // joint_cmd_[5] = vision_pose_[1];
                // joint_cmd_[3] = vision_pose_[2];
                ROS_INFO_STREAM("x:" <<vision_pose_[3]);
                ROS_INFO_STREAM("y:" << vision_pose_[5];);
                ROS_INFO_STREAM("z:" <<vision_pose_[4]);
                ROS_INFO_STREAM("roll:" <<vision_pose_[0]);
                ROS_INFO_STREAM("PITCH:" << vision_pose_[1];);
                ROS_INFO_STREAM("YAW:" <<vision_pose_[2]);
            }
            for (int i = 0; i < 7; i++)
            {
                if (destination_name_ != "vision")
                    joint_pos_cmd_[i] = joint_destination_[destination_name_][i];
                else
                {
                    if (i == 0)
                    {
                        double a = -vision_pose_[4];
                        if (a > 1.57)
                            a = 1.57;
                        else if (a < -1.57)
                            a = -1.57;
                        joint_pos_cmd_[i] = a;
                    }
                     if (i == 1)
                    {
                        double a = -vision_pose_[3];
                        if (a > 1.57)
                            a = 1.57;
                        else if (a < -1.57)
                            a = -1.57;
                        joint_pos_cmd_[i] = a;
                    }
                     if (i == 2)
                    {
                        double a = vision_pose_[5];
                        if (a > 1.57)
                            a = 1.57;
                        else if (a < -1.57)
                            a = -1.57;
                        joint_pos_cmd_[i] = a;
                    }
                    if (i == 3)    
                    {
                        double a = -vision_pose_[2];
                        if (a > 1.57)
                            a = 1.57;
                        else if (a < -1.57)
                            a = -1.57;
                        joint_pos_cmd_[i] = a;
                    }
                    else if (i == 5)
                    {
                        double a = vision_pose_[1];
                        if (a > 0.2)
                            a = 0.2;
                        else if (a < -1.57)
                            a = -1.57;
                        joint_pos_cmd_[i] = a;
                    }
                    else if (i == 6)
                    {
                        double a = vision_pose_[0];
                        if (a > 0.87)
                            a = 0.87;
                        else if (a < -0.87)
                            a = -0.87;
                        joint_pos_cmd_[i] = a;
                    }

                }
                generateSplineCoeff(i);
            }
            //ROS_INFO_STREAM(joint_pos_cmd_);

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



            for (int i = 0; i < 7; i++)
            { 
                if (abs(joint_pos_cmd_[i] - joint_cmd_[i]) > 0.002)
                    joint_cmd_[i] = coeff_(i, 0) + coeff_(i, 1)*sec + coeff_(i, 2)*pow(sec, 2) + coeff_(i, 3)*pow(sec, 3);
                else
                    joint_cmd_[i] = joint_pos_cmd_[i];
                if (abs(joint_pos_cmd_[i] - joint_pos_[i]) > position_threshold_[i])
                {
                    ROS_INFO_STREAM(i);
                    ROS_INFO_STREAM(joint_pos_cmd_[i]);
                    ROS_INFO_STREAM(joint_cmd_[i]);
                    ROS_INFO_STREAM(joint_pos_[i]);
                    ROS_INFO_STREAM(position_threshold_[i]);
                    ROS_INFO_STREAM("----------------------");
                    reached = false;
                }
            }

            if (sec > 2)
                reached = true;

            if (reached)
            {
                if (destination_name_ == "exchange")
                    visionInverseKinematic();
                ROS_INFO_STREAM("Destination Reached");
                planed_ = false;
                destination_ = NONE;
            }
        }
    } 


    void ManipulatorController::processMode()
    {
        if (process_ == STOP)
            return;
        if (!planed_)
        {
            if (process_ == TAKE_SLIVER)
                process_name_ = "take_sliver";
            else if (process_ == TAKE_GLOD)
                process_name_ = "take_gold";

            ROS_INFO_STREAM("Process: "<< process_name_;);
            ROS_INFO_STREAM("Process_num_: "<< process_num_);

            ros::Duration duration;
            now_time_ = ros::Time::now();
            duration = now_time_ - reach_time_;
            double sec = duration.toSec();
            if (sec < process_delay_time_[process_name_] && process_num_ > 0)
                return;

            for (int i = 0; i < 7; i++)
            {
                joint_pos_cmd_[i] = joint_process_[process_name_][process_num_][i];
                generateSplineCoeff(i);
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

            for (int i = 0; i < 7; i++)
            { 
                if (abs(joint_pos_cmd_[i] - joint_cmd_[i]) > 0.002)
                    joint_cmd_[i] = coeff_(i, 0) + coeff_(i, 1)*sec + coeff_(i, 2)*pow(sec, 2) + coeff_(i, 3)*pow(sec, 3);
                else
                    joint_cmd_[i] = joint_pos_cmd_[i];
                if (abs(joint_pos_cmd_[i] - joint_pos_[i]) > position_threshold_[i])
                {
                    // ROS_INFO_STREAM(i);
                    // ROS_INFO_STREAM(joint_cmd_[i]);
                    // ROS_INFO_STREAM(joint_pos_[i]);
                    // ROS_INFO_STREAM("----------------------");
                    reached = false;
                }

            }
            if (sec > 2)
                reached = true;

            if (reached)
            {
                planed_ = false;
                reach_time_ = ros::Time::now();
                ROS_INFO_STREAM("num "<<joint_process_num_[process_name_]);  
                process_num_++;   
                if (process_num_ == joint_process_num_[process_name_])
                {
                    ROS_INFO_STREAM("Process compeleted");
                    process_ = STOP;
                    process_num_ = 0; 
                }                        
            }
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
        // ROS_INFO_STREAM(joint_cmd_[2]);
        // ROS_INFO_STREAM(joint_pos_[2]);
        // ROS_INFO_STREAM("----------------------------");
    }

    void ManipulatorController::generateSplineCoeff(int index)
    {
        spline_.init(joint_pos_[index], joint_pos_cmd_[index], vel_limit_[index]);
        Eigen::Vector4d coeff;
        spline_.computeCoeff(coeff);
        coeff_(index, 0) = coeff[0];
        coeff_(index, 1) = coeff[1];
        coeff_(index, 2) = coeff[2];
        coeff_(index, 3) = coeff[3];
    }

    // void ManipulatorController::visionInverseKinematic()
    // {
    //     vision_pose_.orientation;
    // }


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
            joint_pos_[5] = ctrl_diff_.getPitchPosition() + pitch_offset_;
            joint_pos_[6] = ctrl_diff_.getRollPosition();
        }
        std_msgs::Float64MultiArray joint_pos;
        for (int i = 0; i < 7; i++)
            joint_pos.data.push_back(joint_pos_[i]);
        pos_pub_.publish(joint_pos);
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
    }

    void ManipulatorController::finalPush()
    {
        double vel = -0.00025;
        if (final_push_)
        {
            joint_vel_cmd_[1] = (sin(joint_pos_[3]) * cos(joint_pos_[5]) + cos(joint_pos_[3]) * sin(joint_pos_[4]) * sin(joint_pos_[5])) * vel;
            joint_vel_cmd_[2] = -(cos(joint_pos_[3]) * cos(joint_pos_[5]) - sin(joint_pos_[3]) * sin(joint_pos_[4]) * sin(joint_pos_[5])) * vel;
            joint_vel_cmd_[0] = cos(joint_pos_[4]) * sin(joint_pos_[5]) * vel;
        }   
        //ROS_INFO_STREAM(joint_vel_cmd_[1]);
    }

    void ManipulatorController::msgCaliZCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
            z_calimsg_ = true;
    }

    void ManipulatorController::msgCaliXCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
            x_calimsg_ = true;
    }

    void ManipulatorController::msgCaliYCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
            y_calimsg_ = true;
    }

    void ManipulatorController::msgCaliPitchCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
            pitch_calimsg_ = true;
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

     void ManipulatorController::cmdVisionCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        cmd_struct_.cmd_vision_ = *msg;
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }

    // void ManipulatorController::cmdVisionCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    // {
    //     cmd_struct_.cmd_vision_ = *msg;
    //     cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    // }
    
} // namespace manipulator_controller

PLUGINLIB_EXPORT_CLASS(manipulator_controller::ManipulatorController, controller_interface::ControllerBase)