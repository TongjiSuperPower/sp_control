#include "chassis_controller/chassis_base.h"
#include <pluginlib/class_list_macros.hpp>

namespace chassis_controller
{
    bool ChassisBase::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        ROS_INFO("CHASSIS : Initializing Started");

        if (!controller_nh.getParam("publish_rate", publish_rate_) || !controller_nh.getParam("timeout", timeout_) ||
            !controller_nh.getParam("power/vel_coeff", velocity_coeff_) ||
            !controller_nh.getParam("power/effort_coeff", effort_coeff_) ||
            !controller_nh.getParam("power/power_offset", power_offset_))
        {
            ROS_ERROR("Some chassis params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
            std::cout << "Some chassis params doesn't given" << controller_nh.getNamespace().c_str() << std::endl;
            return false;
        }

        wheel_radius_ = sp_common::getParam(controller_nh, "wheel_radius", 0.02);
        wheel_track_ = sp_common::getParam(controller_nh, "wheel_track", 0.410);
        wheel_base_ = sp_common::getParam(controller_nh, "wheel_base", 0.320);
        twist_angular_ = sp_common::getParam(controller_nh, "twist_angular", M_PI / 6);

        if (follow_pid_.init(ros::NodeHandle(controller_nh, "yaw_follow/pid")));

        effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

        // Setup odometry realtime publisher + odom message constant fields
        cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_chassis_vel", 1, &ChassisBase::cmdVelCallback, this);
        cmd_chassis_sub_ = root_nh.subscribe<sp_common::ChassisCmd>("chassis_cmd", 1, &ChassisBase::cmdChassisCallback, this);
        msg_yaw_sub_ = root_nh.subscribe<std_msgs::Float64>("yaw_angle", 1, &ChassisBase::msgYawCallback, this);

        ramp_x_ = new sp_common::RampFilter<double>(5, 0.001);
        ramp_y_ = new sp_common::RampFilter<double>(5, 0.001);
        ramp_z_ = new sp_common::RampFilter<double>(1.8, 0.001);
        setOdomPubFields(root_nh, controller_nh);

        return true;
    }

    void ChassisBase::update(const ros::Time &time, const ros::Duration &period)
    {
        sp_common::ChassisCmd cmd_chassis = cmd_rt_buffer_.readFromRT()->cmd_chassis_;
        geometry_msgs::Twist cmd_vel = cmd_rt_buffer_.readFromRT()->cmd_vel_;
        

        if ((time - cmd_rt_buffer_.readFromRT()->stamp_).toSec() > timeout_)
        {
            cmd_vel.linear.x = 0.;
            cmd_vel.linear.y = 0.;
            cmd_vel.angular.z = 0.;
        }
        

        ramp_x_->setAcc(cmd_chassis.accel.linear.x);
        ramp_y_->setAcc(cmd_chassis.accel.linear.y);
        ramp_z_->setAcc(cmd_chassis.accel.angular.z);
        

        
        if (state_ != cmd_chassis.mode)
        {
            state_ = cmd_chassis.mode;
            state_changed_ = true;
            recovery();
        }

        switch (state_)
        {
            
            case FOLLOW:
            {
                follow(time, period, cmd_vel);
                break;
            }
            case NOFOLLOW:
            {
                nofollow(cmd_vel);
                break;
            }
            case GYRO:
            {
                gyro(cmd_vel);
                break;
            }
        }        

        updateOdom(time, period);
        moveJoint(time, period);
       
    }

    void ChassisBase::follow(const ros::Time &time, const ros::Duration &period, const geometry_msgs::Twist &cmd_vel)
    {
        if (state_changed_)
        {
            state_changed_ = false;
            ROS_INFO_STREAM("[Chassis] Enter FOLLOW");

            recovery();
            follow_pid_.reset();
        }
        double vel_x, vel_y, vel_z;
        vel_x = vel_y = vel_z = 0.0;
        vel_x = cos(yaw_pos_) * cmd_vel.linear.x + sin(yaw_pos_) * cmd_vel.linear.y;
        vel_y = -sin(yaw_pos_) * cmd_vel.linear.x + cos(yaw_pos_) * cmd_vel.linear.y;
        


        double follow_error = angles::shortest_angular_distance(yaw_pos_, 0);
        follow_pid_.computeCommand(follow_error, period);
        vel_z = follow_pid_.getCurrentCmd();
        ramp_x_->input(vel_x);
        ramp_y_->input(vel_y);
        ramp_z_->input(vel_z);
        vel_cmd_.x = ramp_x_->output(); 
        vel_cmd_.y = ramp_y_->output(); 
        vel_cmd_.z = ramp_z_->output(); 
        vel_cmd_.z = vel_z;
    }

    void ChassisBase::nofollow(const geometry_msgs::Twist &cmd_vel)
    {
        if (state_changed_)
        {
            state_changed_ = false;
            ROS_INFO("[Chassis] Enter NOFOLLOW");

            recovery();
        }
        ramp_x_->input(cmd_vel.linear.x);
        ramp_y_->input(cmd_vel.linear.y);
        ramp_z_->input(cmd_vel.angular.z);
        vel_cmd_.x = ramp_x_->output(); 
        vel_cmd_.y = ramp_y_->output(); 
        vel_cmd_.z = ramp_z_->output(); 
        ROS_INFO_STREAM(vel_cmd_);
    
    }

    
    void ChassisBase::gyro(const geometry_msgs::Twist &cmd_vel)
    {
        if (state_changed_)
        {
            state_changed_ = false;
            ROS_INFO("[Chassis] Enter GYRO");

            recovery();
            follow_pid_.reset();
        }
        double vel_x, vel_y, vel_z;
        vel_x = vel_y = vel_z = 0.0;
        vel_x = cos(yaw_pos_) * cmd_vel.linear.x + sin(yaw_pos_) * cmd_vel.linear.y;
        vel_y = -sin(yaw_pos_) * cmd_vel.linear.x + cos(yaw_pos_) * cmd_vel.linear.y;

        vel_z = cmd_vel.angular.z;

        ramp_x_->input(vel_x);
        ramp_x_->input(vel_y);
        ramp_z_->input(vel_z);
        vel_cmd_.x = ramp_x_->output(); 
        vel_cmd_.y = ramp_y_->output(); 
        vel_cmd_.z = ramp_z_->output(); 

    }



    void ChassisBase::recovery()
    {
        ramp_x_->clear(vel_cmd_.x);
        ramp_y_->clear(vel_cmd_.y);
        ramp_z_->clear(vel_cmd_.z);
    }

    void ChassisBase::tfVelToBase(const std::string& from)
    {
        try
        {
            //tf2::doTransform(vel_cmd_, vel_cmd_, robot_state_handle_.lookupTransform("base_link", from, ros::Time(0)));
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
        }
    }


    void ChassisBase::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        cmd_struct_.cmd_vel_ = *msg;
        cmd_struct_.stamp_ = ros::Time::now();
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }

    void ChassisBase::cmdChassisCallback(const sp_common::ChassisCmdConstPtr& cmd)
    {
        cmd_struct_.cmd_chassis_ = *cmd;
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
    }

    void ChassisBase::msgYawCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        yaw_pos_ = msg->data;
    }

    void ChassisBase::setOdomPubFields(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        // Get and check params for covariances
        // XmlRpc::XmlRpcValue pose_cov_list;
        // controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
        // ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        // ROS_ASSERT(pose_cov_list.size() == 6);
        // for (int i = 0; i < pose_cov_list.size(); ++i)
        // ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        XmlRpc::XmlRpcValue twist_cov_list;
        controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
        ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(twist_cov_list.size() == 6);
        for (int i = 0; i < twist_cov_list.size(); ++i)
            ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        // Setup odometry realtime publisher + odom message constant fields
        odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "/odom", 100));
        odom_pub_->msg_.header.frame_id = odom_frame_id_;
        odom_pub_->msg_.child_frame_id = base_frame_id_;
        odom_pub_->msg_.pose.pose.position.z = 0;
        // odom_pub_->msg_.pose.covariance = {
        //     static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
        //     0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
        //     0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
        //     0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
        //     0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
        //     0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5]) };
        odom_pub_->msg_.twist.twist.linear.y = 0;
        odom_pub_->msg_.twist.twist.linear.z = 0;
        odom_pub_->msg_.twist.twist.angular.x = 0;
        odom_pub_->msg_.twist.twist.angular.y = 0;
        odom_pub_->msg_.twist.covariance = {
            static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
            0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
            0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
            0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
            0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
            0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5])};

        odom2base_.header.frame_id = odom_frame_id_;
        odom2base_.child_frame_id = base_frame_id_;
        odom2base_.header.stamp = ros::Time::now();
        odom2base_.transform.translation.z = 0.0;
        odom2base_.transform.rotation.w = 1.0;
        tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(root_nh, "/tf", 100));
        tf_odom_pub_->msg_.transforms.push_back(odom2base_);
    }

    void ChassisBase::updateOdom(const ros::Time &time, const ros::Duration &period)
    {
        geometry_msgs::Twist vel_base = forwardKinematics(); // on base_link frame
        geometry_msgs::Vector3 linear_vel_odom, angular_vel_odom;
        odom2base_.header.stamp = time;
        // integral vel to pos and angle
        tf2::doTransform(vel_base.linear, linear_vel_odom, odom2base_);
        tf2::doTransform(vel_base.angular, angular_vel_odom, odom2base_);
        odom2base_.transform.translation.x += linear_vel_odom.x * period.toSec();
        odom2base_.transform.translation.y += linear_vel_odom.y * period.toSec();
        odom2base_.transform.translation.z += linear_vel_odom.z * period.toSec();
        double length =
            std::sqrt(std::pow(angular_vel_odom.x, 2) + std::pow(angular_vel_odom.y, 2) + std::pow(angular_vel_odom.z, 2));
        if (length > 0.001)
        { // avoid nan quat
            tf2::Quaternion odom2base_quat, trans_quat;
            tf2::fromMsg(odom2base_.transform.rotation, odom2base_quat);
            trans_quat.setRotation(tf2::Vector3(angular_vel_odom.x / length, angular_vel_odom.y / length,
                                                angular_vel_odom.z / length),
                                   length * period.toSec());
            odom2base_quat = trans_quat * odom2base_quat;
            odom2base_quat.normalize();
            odom2base_.transform.rotation = tf2::toMsg(odom2base_quat);
        }

        if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
        {
            if (odom_pub_->trylock())
            {
                odom_pub_->msg_.header.stamp = time;
                odom_pub_->msg_.twist.twist.linear.x = vel_base.linear.x;
                odom_pub_->msg_.twist.twist.linear.y = vel_base.linear.y;
                odom_pub_->msg_.twist.twist.angular.z = vel_base.angular.z;
                odom_pub_->unlockAndPublish();
            }
            if (tf_odom_pub_->trylock())
            {
                geometry_msgs::TransformStamped &odom_frame = tf_odom_pub_->msg_.transforms[0];
                odom_frame.header.stamp = time;
                odom_frame.header.frame_id = odom2base_.header.frame_id;
                odom_frame.child_frame_id = odom2base_.child_frame_id;
                odom_frame.transform.translation = odom2base_.transform.translation;
                odom_frame.transform.rotation = odom2base_.transform.rotation;
                // Dont use the following code, may lead to the TF Disorder !!!
                // tf_odom_pub_->msg_.transforms.push_back(odom2base_);
                tf_odom_pub_->unlockAndPublish();
            }
            last_publish_time_ = time;
        }
    }

} // namespace chassis_controller
