#include "sp_engineer/engineer.h"


namespace sp_operator
{

    bool Engineer::init()
    {
        Operator::init();
        ROS_INFO_STREAM("Engineer");
        controller_nh = ros::NodeHandle("engineer");
        manipulator_cmd_pub_ = nh.advertise<sp_common::ManipulatorCmd>("/manipulator_cmd", 10);
        twist_cmd_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/delta_twist_cmds",10);
        //joint_cmd_pub_ = nh.advertise<control_msgs::JointJog>("/delta_joint_cmds",10);

        x_coeff_ = sp_common::getParam(controller_nh, "chassis/x_coeff", 2.0);
        y_coeff_ = sp_common::getParam(controller_nh, "chassis/y_coeff", 2.0);
        z_mk_coeff_ = sp_common::getParam(controller_nh, "chassis/z_mk_coeff", 70);
        z_rc_coeff_ = sp_common::getParam(controller_nh, "chassis/z_rc_coeff", 1.5);
        x_accel_set_ = sp_common::getParam(controller_nh, "chassis/x_accel_set", 5);
        y_accel_set_ = sp_common::getParam(controller_nh, "chassis/y_accel_set", 5);
        z_accel_set_ = sp_common::getParam(controller_nh, "chassis/z_accel_set", 1.8);

        yaw_coeff_ =  sp_common::getParam(controller_nh, "gimbal/yaw_coeff_", 1.0);
        pitch_mk_coeff_ = sp_common::getParam(controller_nh, "gimbal/pitch_mk_coeff", 1.0);
        pitch_rc_coeff_ = sp_common::getParam(controller_nh, "gimbal/pitch_rc_coeff", 1.0);
        yaw_left_limit_ = sp_common::getParam(controller_nh, "gimbal/yaw_left_limit", -1.57);
        yaw_right_limit_ = sp_common::getParam(controller_nh, "gimbal/yaw_right_limit", 1.57);
        pitch_low_limit_ = sp_common::getParam(controller_nh, "gimbal/pitch_low_limit", -0.52);
        pitch_high_limit_ = sp_common::getParam(controller_nh, "gimbal/pitch_high_limit", 0.52);
        joint_cmd_.joint_names.push_back("joint1");
        joint_cmd_.joint_names.push_back("joint2");
        joint_cmd_.joint_names.push_back("joint3");
        joint_cmd_.joint_names.push_back("joint4");
        joint_cmd_.joint_names.push_back("joint5");
        joint_cmd_.joint_names.push_back("joint6");
        joint_cmd_.joint_names.push_back("joint7");
        for (int i = 0; i < 7; i++)
            joint_cmd_.velocities.push_back(0.0);

        return true;
    }

    void Engineer::run()
    {
        chassis_set();
        cmd_vel_pub_.publish(cmd_vel_); 
        chassis_cmd_pub_.publish(chassis_cmd_);

        gimbal_set();
        cmd_pos_pub_.publish(cmd_pos_); 
        
        manipulator_set();
        manipulator_cmd_pub_.publish(manipulator_cmd_);
        twist_cmd_pub_.publish(twist_cmd_);
        //joint_cmd_pub_.publish(joint_cmd_);

        last_dbus_data_ = dbus_data_;
        ros::spinOnce();
    }


    void Engineer::chassis_set()
    {
        if (dbus_data_.s_l == 1) //chassis_control_mode
        {
            if (dbus_data_.s_r == 1) //Mouse & Keyboard mode
            {
                cmd_vel_.linear.x = x_coeff_ * dbus_data_.ch_r_x;
                cmd_vel_.linear.y = -y_coeff_ * dbus_data_.ch_r_y;
                cmd_vel_.angular.z = -z_rc_coeff_ * dbus_data_.ch_l_x;
            }
            else if (dbus_data_.s_r == 3) //Remote control mode
            {
                if (dbus_data_.key_w)
                    cmd_vel_.linear.x = x_coeff_;
                else if (dbus_data_.key_s)
                    cmd_vel_.linear.x = -x_coeff_;
                else 
                    cmd_vel_.linear.x = 0.0;
                if (dbus_data_.key_a)
                    cmd_vel_.linear.y = y_coeff_;
                else if (dbus_data_.key_d)
                    cmd_vel_.linear.y = -y_coeff_;
                else 
                    cmd_vel_.linear.y = 0.0;
                cmd_vel_.angular.z = -z_mk_coeff_ * dbus_data_.m_x;
            }
            else if (dbus_data_.s_r == 2) //Stop mode
            {
                cmd_vel_.linear.x = 0.0;
                cmd_vel_.linear.y = 0.0;
                cmd_vel_.angular.z = 0.0;
            }
        }
        else
        {
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.angular.z = 0.0;

        }

        if (dbus_data_.key_shift && !last_dbus_data_.key_shift) // Gyro
            chassis_cmd_.mode = GYRO;
        else 
            chassis_cmd_.mode = NOFOLLOW;

        chassis_cmd_.accel.linear.x = x_accel_set_;
        chassis_cmd_.accel.linear.y = y_accel_set_;
        chassis_cmd_.accel.angular.z = z_accel_set_;
        chassis_cmd_.stamp = ros::Time::now();
    }

    void Engineer::gimbal_set()
    {
        if (dbus_data_.s_r == 1) //Mouse & Keyboard mode
        {
            cmd_pos_.y += pitch_rc_coeff_ * dbus_data_.ch_l_y;
        }
        else if (dbus_data_.s_r == 3) //Remote control mode
        {
            if (dbus_data_.key_q && !dbus_data_.key_e)
                cmd_pos_.z += yaw_coeff_;
            else if (dbus_data_.key_e && !dbus_data_.key_q)
                cmd_pos_.z += -yaw_coeff_;
            else if (dbus_data_.key_q && dbus_data_.key_e)
                cmd_pos_.z = 0.0;

            cmd_pos_.y += -pitch_mk_coeff_ * dbus_data_.m_y;
        }

        if (cmd_pos_.z < yaw_left_limit_)
            cmd_pos_.z = yaw_left_limit_;
        else if (cmd_pos_.z > yaw_right_limit_)
            cmd_pos_.z = yaw_right_limit_;

        if (cmd_pos_.y < pitch_low_limit_)
            cmd_pos_.y = pitch_low_limit_;
        else if (cmd_pos_.y > pitch_high_limit_)
            cmd_pos_.y = pitch_high_limit_;
    }
    
    void Engineer::manipulator_set()
    {
        // if (dbus_data_.s_l == 3) //Manipulator control mode
        // {
        //     if (dbus_data_.s_r == 1) //Remote control mode
        //     {
        //         manipulator_cmd_.joint1_pos += 0.005 * dbus_data_.ch_l_x;
        //         manipulator_cmd_.joint2_pos += 0.03 * dbus_data_.ch_r_x;
        //         manipulator_cmd_.joint3_pos += 0.03 * dbus_data_.ch_l_y;
        //         manipulator_cmd_.joint4_pos += 0.01 * dbus_data_.ch_r_y;
        //     }
        //     else if (dbus_data_.s_r == 3)
        //     {
        //         manipulator_cmd_.joint5_pos += 0.03 * dbus_data_.ch_l_y;
        //         manipulator_cmd_.joint6_pos += 0.015 * dbus_data_.ch_l_x;
        //         manipulator_cmd_.joint7_pos += 0.03 * dbus_data_.ch_r_y;
        //     }       
        // }

        // if (manipulator_cmd_.joint1_pos > 0)
        //     manipulator_cmd_.joint1_pos = 0;
        // else if (manipulator_cmd_.joint1_pos < -0.16)
        //     manipulator_cmd_.joint1_pos = -0.16;

        // if (manipulator_cmd_.joint2_pos > 0.55)
        //     manipulator_cmd_.joint2_pos = 0.55;
        // else if (manipulator_cmd_.joint2_pos < 0)
        //     manipulator_cmd_.joint2_pos = 0;

        // if (manipulator_cmd_.joint3_pos > 0.4754)
        //     manipulator_cmd_.joint3_pos = 0.4754;
        // else if (manipulator_cmd_.joint3_pos < 0)
        //     manipulator_cmd_.joint3_pos = 0;

        // if (manipulator_cmd_.joint4_pos > 0.3)
        //     manipulator_cmd_.joint4_pos = 0.3;
        // else if (manipulator_cmd_.joint4_pos < -0.3)
        //     manipulator_cmd_.joint4_pos = -0.3;

        // if (manipulator_cmd_.joint5_pos > 3.14)
        //     manipulator_cmd_.joint5_pos = 3.14;
        // else if (manipulator_cmd_.joint5_pos < 0)
        //     manipulator_cmd_.joint5_pos = 0;

        // if (manipulator_cmd_.joint6_pos > 1.57)
        //     manipulator_cmd_.joint6_pos = 1.57;
        // else if (manipulator_cmd_.joint6_pos < -1.57)
        //     manipulator_cmd_.joint6_pos = -1.57;

        // if (manipulator_cmd_.joint7_pos > 3.14)
        //     manipulator_cmd_.joint7_pos = 3.14;
        // else if (manipulator_cmd_.joint7_pos < -3.14)
        //     manipulator_cmd_.joint7_pos = -3.14;
        twist_cmd_.header.stamp = ros::Time::now();
        joint_cmd_.header.stamp = ros::Time::now();
        if (dbus_data_.s_l == 3) //Manipulator control mode
        {
            if (dbus_data_.s_r == 1) //Remote control mode
            {
                twist_cmd_.twist.linear.z = 0.3 * dbus_data_.ch_l_y;
                twist_cmd_.twist.linear.y = 0.5 * dbus_data_.ch_r_x;
                twist_cmd_.twist.linear.x = 0.2 * dbus_data_.ch_r_y;
                // joint_cmd_.velocities[0] = 0.1 * dbus_data_.ch_l_x;
                // joint_cmd_.velocities[1] = 0.3 * dbus_data_.ch_r_x;
                // joint_cmd_.velocities[2] = 0.3 * dbus_data_.ch_l_y;
                // joint_cmd_.velocities[3] = 0.3 * dbus_data_.ch_r_y;
            }
            else if (dbus_data_.s_r == 3)
            {
                twist_cmd_.twist.angular.x = 0.4 * dbus_data_.ch_l_y;
                twist_cmd_.twist.angular.y = 0.2 * dbus_data_.ch_r_x;
                twist_cmd_.twist.angular.z = 0.3 * dbus_data_.ch_r_y;
            //     joint_cmd_.velocities[4] = 1.5 * dbus_data_.ch_r_y;
            //     joint_cmd_.velocities[5] = 1.5 * dbus_data_.ch_l_y;
            //     joint_cmd_.velocities[6] = -1.5 * dbus_data_.ch_r_y;
            }       
        }
       


    }
    
    



}