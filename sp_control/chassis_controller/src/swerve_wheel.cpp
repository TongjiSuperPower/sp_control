#include "chassis_controller/swerve_wheel.h"
#include <pluginlib/class_list_macros.hpp>

namespace chassis_controller
{
    bool SwerveWheel::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        ChassisBase::init(robot_hw, root_nh, controller_nh);

        ros::NodeHandle nh_lf_driving = ros::NodeHandle(controller_nh, "left_front_driving");
        ros::NodeHandle nh_rf_driving = ros::NodeHandle(controller_nh, "right_front_driving");
        ros::NodeHandle nh_lb_driving = ros::NodeHandle(controller_nh, "left_back_driving");
        ros::NodeHandle nh_rb_driving = ros::NodeHandle(controller_nh, "right_back_driving");
        if (!ctrl_lf_driving_.init(effort_joint_interface_, nh_lf_driving) || !ctrl_rf_driving_.init(effort_joint_interface_, nh_rf_driving) ||
            !ctrl_lb_driving_.init(effort_joint_interface_, nh_lb_driving) || !ctrl_rb_driving_.init(effort_joint_interface_, nh_rb_driving))
            return false;

        ros::NodeHandle nh_lf_heading = ros::NodeHandle(controller_nh, "left_front_heading");
        ros::NodeHandle nh_rf_heading = ros::NodeHandle(controller_nh, "right_front_heading");
        ros::NodeHandle nh_lb_heading = ros::NodeHandle(controller_nh, "left_back_heading");
        ros::NodeHandle nh_rb_heading = ros::NodeHandle(controller_nh, "right_back_heading");
        if (!ctrl_lf_heading_.init(effort_joint_interface_, nh_lf_heading) || !ctrl_rf_heading_.init(effort_joint_interface_, nh_rf_heading) ||
            !ctrl_lb_heading_.init(effort_joint_interface_, nh_lb_heading) || !ctrl_rb_heading_.init(effort_joint_interface_, nh_rb_heading))
            return false;

        joint_handles_.push_back(ctrl_lf_driving_.joint_);
        joint_handles_.push_back(ctrl_rf_driving_.joint_);
        joint_handles_.push_back(ctrl_lb_driving_.joint_);
        joint_handles_.push_back(ctrl_rb_driving_.joint_);

        joint_handles_.push_back(ctrl_lf_heading_.joint_);
        joint_handles_.push_back(ctrl_rf_heading_.joint_);
        joint_handles_.push_back(ctrl_lb_heading_.joint_);
        joint_handles_.push_back(ctrl_rb_heading_.joint_);


        ROS_INFO("SWERVE CHASSIS : Initializing Completed");
        return true;
    }

    void SwerveWheel::moveJoint(const ros::Time &time, const ros::Duration &period)
    {

        //计算时调用的中间量
        float para_xy, para_x, para_y;
        double half_wheel_track_,half_wheel_base_;

        half_wheel_track_  = wheel_track_/2;
        half_wheel_base_   = wheel_base_/2;

        para_xy = pow(vel_cmd_.x, 2) + pow(vel_cmd_.y, 2) + pow(half_wheel_base_ * vel_cmd_.z, 2) + pow(half_wheel_track_ * vel_cmd_.z, 2);
        para_x = wheel_track_  * vel_cmd_.z * vel_cmd_.x; 
        para_y = wheel_base_ * vel_cmd_.z * vel_cmd_.y;

        // //遥控器无值时直接维持上次的值
        // if (vel_cmd_.x==0 && vel_cmd_.y==0 && vel_cmd_.z==0)
        //         return;

        //注意电机安装方向的不同
        ctrl_rf_driving_.setCommand(sqrt(para_xy + para_x + para_y)/wheel_radius_);
        ctrl_lf_driving_.setCommand(sqrt(para_xy - para_x + para_y)/wheel_radius_);
        ctrl_lb_driving_.setCommand(sqrt(para_xy - para_x - para_y)/wheel_radius_);
        ctrl_rb_driving_.setCommand(sqrt(para_xy + para_x - para_y)/wheel_radius_);

        ctrl_lf_driving_.update(time, period);
        ctrl_rf_driving_.update(time, period);
        ctrl_lb_driving_.update(time, period);
        ctrl_rb_driving_.update(time, period);

        ROS_INFO_STREAM("x:"<<vel_cmd_.x<<" y:"<<vel_cmd_.y<<" z:"<<vel_cmd_.z);
 


        //解算轮角度，atan2(y, x) = atan (y/x)
        float angle[4];
        angle[0]=atan2(vel_cmd_.y + half_wheel_base_ * vel_cmd_.z, vel_cmd_.x + half_wheel_track_* vel_cmd_.z);
        angle[1]=atan2(vel_cmd_.y + half_wheel_base_ * vel_cmd_.z, vel_cmd_.x - half_wheel_track_* vel_cmd_.z);
        angle[2]=atan2(vel_cmd_.y - half_wheel_base_ * vel_cmd_.z, vel_cmd_.x - half_wheel_track_* vel_cmd_.z);
        angle[3]=atan2(vel_cmd_.y - half_wheel_base_ * vel_cmd_.z, vel_cmd_.x + half_wheel_track_* vel_cmd_.z);
        // for(int i=0;i<4;i++)
        // {
        //     if(angle[i]<-1.07)
        //         angle[i]+=3.14;

        // }


        // ctrl_rf_heading_.setCommand(atan2(vel_cmd_.y + half_wheel_base_ * vel_cmd_.z, vel_cmd_.x - half_wheel_track_* vel_cmd_.z));
        // ctrl_lf_heading_.setCommand(atan2(vel_cmd_.y - half_wheel_base_ * vel_cmd_.z, vel_cmd_.x - half_wheel_track_* vel_cmd_.z));
        // ctrl_lb_heading_.setCommand(atan2(vel_cmd_.y - half_wheel_base_ * vel_cmd_.z, vel_cmd_.x + half_wheel_track_* vel_cmd_.z));
        // ctrl_rb_heading_.setCommand(atan2(vel_cmd_.y + half_wheel_base_ * vel_cmd_.z, vel_cmd_.x + half_wheel_track_* vel_cmd_.z));
        
        ctrl_rf_heading_.setCommand(angle[0]);
        ctrl_lf_heading_.setCommand(angle[1]);
        ctrl_lb_heading_.setCommand(angle[2]);
        ctrl_rb_heading_.setCommand(angle[3]);


        ctrl_lf_heading_.update(time, period);
        ctrl_rf_heading_.update(time, period);
        ctrl_lb_heading_.update(time, period);
        ctrl_rb_heading_.update(time, period);
        ROS_INFO_STREAM(" rf_ang:"<<angle[0]<<" lf_ang:"<<angle[1]<<" lb_ang:"<<angle[2]<<" rb_ang:"<<angle[3]);
    }

    geometry_msgs::Twist SwerveWheel::forwardKinematics()
    {
        geometry_msgs::Twist vel_data;

        double rf_velocity = ctrl_rf_driving_.joint_.getVelocity()*wheel_radius_;
        double rf_angle    = ctrl_rf_heading_.joint_.getPosition();

        double lf_velocity = ctrl_lf_driving_.joint_.getVelocity()*wheel_radius_;
        double lf_angle    = ctrl_lf_heading_.joint_.getPosition();

        double lb_velocity = ctrl_lb_driving_.joint_.getVelocity()*wheel_radius_;
        double lb_angle    = ctrl_lb_heading_.joint_.getPosition();

        double rb_velocity = ctrl_rb_driving_.joint_.getVelocity()*wheel_radius_;
        double rb_angle    = ctrl_rb_heading_.joint_.getPosition();

        vel_data.linear.x   = (rf_velocity*cos(rf_angle)+lb_velocity*cos(lb_angle))/2;
        vel_data.linear.y   = (rf_velocity*sin(rf_angle)+lb_velocity*sin(lb_angle))/2;
        vel_data.angular.z  = (rf_velocity*sin(rf_angle)-lb_velocity*sin(lb_angle))/wheel_base_;
        return vel_data;
    }
    PLUGINLIB_EXPORT_CLASS(chassis_controller::SwerveWheel, controller_interface::ControllerBase)
}
