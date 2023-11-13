#include "sp_engineer/infantry.h"


namespace sp_operator
{

    bool Infantry::init()
    {
        Operator::init();
        ROS_INFO_STREAM("Infantry");

        controller_nh = ros::NodeHandle("infantry");

        x_coeff_ = sp_common::getParam(controller_nh, "chassis/x_coeff", 2.0);
        y_coeff_ = sp_common::getParam(controller_nh, "chassis/y_coeff", 2.0);
        z_mk_coeff_ = sp_common::getParam(controller_nh, "chassis/z_mk_coeff", 70);
        z_rc_coeff_ = sp_common::getParam(controller_nh, "chassis/z_rc_coeff", 1.5);
        x_accel_set_ = sp_common::getParam(controller_nh, "chassis/x_accel_set", 5);
        y_accel_set_ = sp_common::getParam(controller_nh, "chassis/y_accel_set", 5);
        z_accel_set_ = sp_common::getParam(controller_nh, "chassis/z_accel_set", 1.8);

        return true;
    }

    void Infantry::run()
    {
        chassis_set();
        cmd_vel_pub_.publish(cmd_vel_); 
        chassis_cmd_pub_.publish(chassis_cmd_);
        last_dbus_data_ = dbus_data_;
        ros::spinOnce();
    }


    void Infantry::chassis_set()
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
                cmd_vel_.linear.y = -y_coeff_;
            else if (dbus_data_.key_d)
                cmd_vel_.linear.y = y_coeff_;
            else 
                cmd_vel_.linear.y = 0.0;
            cmd_vel_.angular.z = z_mk_coeff_ * dbus_data_.m_x;
        }
        else if (dbus_data_.s_r == 2) //Stop mode
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

    void Infantry::shooter_set()
    {
        if (dbus_data_.s_r == 1) //Mouse & Keyboard mode
        {
            if (dbus_data_.s_l == 1 && last_dbus_data_.s_l != 1)
            {
                if(shooter_cmd_.shoot_process == SHOOT_STOP)
                {
                    shooter_cmd_.fric_mode = FRIC_ON;
                    shooter_cmd_.shoot_process = SHOOT_READY;	
                                
                }
                else
                {
                    shooter_cmd_.fric_mode = FRIC_OFF;
                    shooter_cmd_.shoot_process = SHOOT_STOP;
                }
            }
            
        }
        else if (dbus_data_.s_r == 3) //Remote control mode
        {
            if ((dbus_data_.mouse_l && !last_dbus_data_.mouse_l) && frics.fric_state == FRIC_OFF)
            {
                shooter_cmd_.fric_mode = FRIC_ON;
                shooter_cmd_.shoot_process = SHOOT_READY;
                
            }

        }
        else if (dbus_data_.s_r == 2) //Stop mode
        {
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.angular.z = 0.0;
        }
    }


}