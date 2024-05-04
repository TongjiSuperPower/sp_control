#include "sp_operator/infantry.h"


namespace sp_operator
{

    bool Infantry::init()
    {
        Operator::init();
        ROS_INFO_STREAM("Infantry");

        controller_nh = ros::NodeHandle("infantry");

        x_coeff_ = sp_common::getParam(controller_nh, "chassis/x_coeff", 2.5);
        y_coeff_ = sp_common::getParam(controller_nh, "chassis/y_coeff", 2.5);
        gyro_vel_ = sp_common::getParam(controller_nh, "chassis/gyro_vel", 4);
        x_accel_set_ = sp_common::getParam(controller_nh, "chassis/x_accel_set", 7);
        y_accel_set_ = sp_common::getParam(controller_nh, "chassis/y_accel_set", 7);
        z_accel_set_ = sp_common::getParam(controller_nh, "chassis/z_accel_set", 8);

        yaw_mk_coeff_ = sp_common::getParam(controller_nh, "gimbal/yaw_mk_coeff", 0.2);
        yaw_rc_coeff_ = sp_common::getParam(controller_nh, "gimbal/yaw_rc_coeff", 0.005);
        pitch_mk_coeff_ = sp_common::getParam(controller_nh, "gimbal/pitch_mk_coeff", 0.2);
        pitch_rc_coeff_ = sp_common::getParam(controller_nh, "gimbal/pitch_rc_coeff", 0.005);

        shooter_cmd_pub_ = nh.advertise<sp_common::ShooterCmd>("/cmd_shooter", 10);
        cover_cmd_pub_ = nh.advertise<std_msgs::Bool>("/cmd_cover", 1);

        trun_time_ = ros::Time::now();

        return true;
    }

    void Infantry::run()
    {
        chassis_set();
        cmd_chassis_vel_pub_.publish(cmd_chassis_vel_); 
        chassis_cmd_pub_.publish(chassis_cmd_);
        
        gimbal_set();
        cmd_gimbal_vel_pub_.publish(cmd_gimbal_vel_); 
        cover_cmd_pub_.publish(cmd_cover_);

        shooter_set();
        shooter_cmd_pub_.publish(shooter_cmd_);

        last_dbus_data_ = dbus_data_;
        ros::spinOnce();
    }


    void Infantry::chassis_set()
    {
        if (dbus_data_.s_r == 1) //Mouse & Keyboard mode
        {
            cmd_chassis_vel_.linear.x = x_coeff_ * dbus_data_.ch_r_x;
            cmd_chassis_vel_.linear.y = -y_coeff_ * dbus_data_.ch_r_y;       
        }
        else if (dbus_data_.s_r == 3) //Remote control mode
        {
            if (dbus_data_.key_w)
                cmd_chassis_vel_.linear.x = x_coeff_;
            else if (dbus_data_.key_s)
                cmd_chassis_vel_.linear.x = -x_coeff_;
            else 
                cmd_chassis_vel_.linear.x = 0.0;
            if (dbus_data_.key_a)
                cmd_chassis_vel_.linear.y = y_coeff_;
            else if (dbus_data_.key_d)
                cmd_chassis_vel_.linear.y = -y_coeff_;
            else 
                cmd_chassis_vel_.linear.y = 0.0;
            cmd_chassis_vel_.angular.z = gyro_vel_;
        }
        else if (dbus_data_.s_r == 2) //Stop mode
        {
            cmd_chassis_vel_.linear.x = 0.0;
            cmd_chassis_vel_.linear.y = 0.0;
            cmd_chassis_vel_.angular.z = 0.0;
        }

        if (dbus_data_.key_shift) // Gyro
            chassis_cmd_.mode = GYRO;
        else 
            chassis_cmd_.mode = FOLLOW;

        chassis_cmd_.accel.linear.x = x_accel_set_;
        chassis_cmd_.accel.linear.y = y_accel_set_;
        chassis_cmd_.accel.angular.z = z_accel_set_;
        chassis_cmd_.stamp = ros::Time::now();
    }

    void Infantry::gimbal_set()
    {
        if (dbus_data_.s_r == 1) //Remote control mode
        {
            cmd_gimbal_vel_.y = -pitch_rc_coeff_ * dbus_data_.ch_l_y;
            cmd_gimbal_vel_.z = -yaw_rc_coeff_ * dbus_data_.ch_l_x;
        }
        else if (dbus_data_.s_r == 3) //Mouse & Keyboard mode
        {
            cmd_gimbal_vel_.y = pitch_mk_coeff_ * dbus_data_.m_y;
            cmd_gimbal_vel_.z = -yaw_mk_coeff_ * dbus_data_.m_x;



            ros::Duration trun_duration = ros::Time::now() - trun_time_;
            if (trun_duration.toSec() > 1)
                truned_ = false;
            if (dbus_data_.key_x) // Trun back
            {
                cmd_gimbal_vel_.z = -M_PI;
                truned_ = true;
                trun_time_ = ros::Time::now();
            }
            else if (dbus_data_.key_q)
            {
                cmd_gimbal_vel_.z = M_PI / 2;
                truned_ = true;
                trun_time_ = ros::Time::now();
            }
             else if (dbus_data_.key_e)
            {
                cmd_gimbal_vel_.z = -M_PI / 2;
                truned_ = true;
                trun_time_ = ros::Time::now();
            }
        }
    }


    void Infantry::shooter_set()
    {
        if (dbus_data_.s_r == 1) //Mouse & Keyboard mode
        {
            if (dbus_data_.s_l == 1 && last_dbus_data_.s_l != 1)  // Open/Close fric wheels
            {
                if(shooter_cmd_.fric_mode == FRIC_OFF)
                {
                    shooter_cmd_.fric_mode = FRIC_ON;                           
                }
                else
                {
                    shooter_cmd_.fric_mode = FRIC_OFF;
                    shooter_cmd_.shoot_process = SHOOT_STOP;
                }
            }

            if (dbus_data_.s_l == 3 && shooter_cmd_.fric_mode == FRIC_ON)
                shooter_cmd_.shoot_process = SHOOT_READY;

            if (dbus_data_.s_l == 2 && last_dbus_data_.s_l != 2 &&  shooter_cmd_.fric_mode == FRIC_ON) // Begin/End shoot behavior
            {
                if (shooter_cmd_.shoot_process == SHOOT_READY)
                {
                    shooter_cmd_.shoot_process = SHOOT_SINGLE; 
                    shooter_cmd_.shoot_mode = SINGLE_MODE;                            
                }
            }
            else if (dbus_data_.s_l == 2 && shooter_cmd_.fric_mode == FRIC_ON) // Begin/End shoot behavior
            {
                if (shooter_cmd_.shoot_process == SHOOT_SINGLE)
                {
                    sleep(1.5);
                    if (dbus_data_.s_l == 2)
                    {
                        shooter_cmd_.shoot_process = SHOOT_CONTINUOUS;    
                        shooter_cmd_.shoot_mode = CONTINUOUS_MODE;  
                    }                         
                }
            }
            
        }
        else if (dbus_data_.s_r == 3) //Remote control mode
        {
            if (dbus_data_.p_l)
            {
                if (dbus_data_.key_z && !last_dbus_data_.key_z)
                {
                    if (shooter_cmd_.shoot_mode == CONTINUOUS_MODE)
                        shooter_cmd_.shoot_mode = SINGLE_MODE; 
                    else if (shooter_cmd_.shoot_mode == SINGLE_MODE)
                        shooter_cmd_.shoot_mode = CONTINUOUS_MODE;

                }

                if(shooter_cmd_.fric_mode == FRIC_OFF)
                {
                    shooter_cmd_.fric_mode = FRIC_ON;
                    shooter_cmd_.shoot_process = SHOOT_READY;                          
                }

                if (shooter_cmd_.shoot_mode = CONTINUOUS_MODE)
                    shooter_cmd_.shoot_process = SHOOT_CONTINUOUS;
                else if (shooter_cmd_.shoot_mode = SINGLE_MODE)
                    shooter_cmd_.shoot_process = SHOOT_SINGLE;           
   
            }
            else
            {
                shooter_cmd_.shoot_process = SHOOT_READY;  
            }

        }
        else if (dbus_data_.s_r == 2) //Stop mode
        {
            shooter_cmd_.fric_mode = FRIC_OFF;
            shooter_cmd_.shoot_process = SHOOT_STOP;

        }
    }


}