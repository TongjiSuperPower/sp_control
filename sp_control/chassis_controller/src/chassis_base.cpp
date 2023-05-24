#include "chassis_controller/chassis_base.h"
#include <pluginlib/class_list_macros.hpp>

namespace chassis_controller
{
    /*
    template <typename T>
    T getParam(ros::NodeHandle &pnh, const std::string &param_name, const T &default_val)
    {
        T param_val;
        pnh.param<T>(param_name, param_val, default_val);
        return param_val;
    }
*/
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

        effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

        // Setup odometry realtime publisher + odom message constant fields
        cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisBase::cmdVelCallback, this);

        ramp_x_ = new sp_common::RampFilter<double>(2.5, 0.001);
        ramp_y_ = new sp_common::RampFilter<double>(2.5, 0.001);
        ramp_z_ = new sp_common::RampFilter<double>(5, 0.001);
        setOdomPubFields(root_nh, controller_nh);

        return true;
    }

    void ChassisBase::update(const ros::Time &time, const ros::Duration &period)
    {
        geometry_msgs::Twist cmd_vel = cmd_rt_buffer_.readFromRT()->cmd_vel_;

        if ((time - cmd_rt_buffer_.readFromRT()->stamp_).toSec() > timeout_)
        {
            cmd_vel.linear.x = 0.;
            cmd_vel.linear.y = 0.;
            cmd_vel.angular.z = 0.;
        }
        else
        {
        }

        ramp_x_->input(cmd_vel.linear.x);
        ramp_y_->input(cmd_vel.linear.y);
        ramp_z_->input(cmd_vel.angular.z);
        vel_cmd_.x = ramp_x_->output();
        vel_cmd_.y = ramp_y_->output();
        vel_cmd_.z = ramp_z_->output();

        updateOdom(time, period);
        moveJoint(time, period);
    }

    void ChassisBase::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        cmd_struct_.cmd_vel_ = *msg;
        cmd_struct_.stamp_ = ros::Time::now();
        cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
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
