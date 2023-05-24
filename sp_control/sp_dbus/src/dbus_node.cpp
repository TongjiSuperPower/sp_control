/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 2019/10/30.
//

#include "sp_dbus/dbus_node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sp_dbus");
  DBusNode dbus_node;
  ros::Rate loop_rate(60);
  while (ros::ok())
  {

    dbus_node.run();
    loop_rate.sleep();
  }
  return 0;
}

DBusNode::DBusNode()
{
  dbus_pub_ = nh_.advertise<sp_common::DbusData>("dbus_data", 1);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  cmd_pos_pub_ = nh_.advertise<geometry_msgs::Vector3>("/cmd_pos", 1);
  gpio_pub_ = nh_.advertise<sp_common::GpioData>("/controllers/gpio_controller/command", 1000);
  gpio_sub_ = nh_.subscribe<sp_common::GpioData>("/controllers/gpio_controller/state", 10, boost::bind(&DBusNode::gpio_callback, this, _1));
  nh_.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
  dbus_.init(serial_port_.data());
  cmd_pos_.x = cmd_pos_.y = cmd_pos_.z = 0.0;
  gripper_signal = sucker_signal = rob_signal = false;
  gpio_data.gpio_name.push_back("left_gripper");
  gpio_data.gpio_name.push_back("right_gripper");
  gpio_data.gpio_name.push_back("sucker");
  gpio_data.gpio_name.push_back("sustaining_rob");
  for (int i = 0; i < 4; i++)
  {
    gpio_data.gpio_state.push_back(false);
    gpio_data.gpio_type.push_back("out");

  }
  
}

void DBusNode::run()
{
  cmd_vel_.linear.x = 0.00;
  cmd_vel_.linear.y = 0.00;
  cmd_vel_.angular.z = 0.00;
  dbus_.read();
  if (dbus_.get_update())
  {
    dbus_.getData(&dbus_cmd_);
    dbus_pub_.publish(dbus_cmd_);
    
    
    if (dbus_cmd_.s_r == 1) // right paddle up,using remote control
    {
      cmd_vel_.linear.x = chassis_x_coeff * dbus_cmd_.ch_r_x;
      cmd_vel_.linear.y = -chassis_y_coeff * dbus_cmd_.ch_r_y;
      cmd_vel_.angular.z = -chassis_z_coeff_rc * dbus_cmd_.ch_l_x;
      cmd_pos_.y += gimbal_y_coeff_rc * dbus_cmd_.ch_l_y;
      if (cmd_pos_.y > 1.57)
        cmd_pos_.y = 1.57;
      else if (cmd_pos_.y < -1.57)
        cmd_pos_.y = -1.57;
    }
    else if (dbus_cmd_.s_r == 3) // right paddle middle, using keyboard control
    {
      if (dbus_cmd_.key_w) // chassis_forward
        cmd_vel_.linear.x = chassis_x_coeff;
      if (dbus_cmd_.key_s) // chassis_back
        cmd_vel_.linear.x = -chassis_x_coeff;
      if (dbus_cmd_.key_a) // chassis_left
        cmd_vel_.linear.y = chassis_y_coeff;
      if (dbus_cmd_.key_d) // chassis_right
        cmd_vel_.linear.y = -chassis_y_coeff;
      if (dbus_cmd_.key_q) // gimbal_counterclockwise
        cmd_pos_.x += gimbal_x_coeff;
      if (dbus_cmd_.key_e) // gimbal_clockwise
        cmd_pos_.x += -gimbal_x_coeff;

      cmd_vel_.angular.z = -chassis_z_coeff_kb * dbus_cmd_.m_x; // chassis_trun
      cmd_pos_.y += -gimbal_y_coeff_kb * dbus_cmd_.m_y;         // gimbal_pitch
      cmd_pos_.z += gimbal_z_coeff * dbus_cmd_.m_z;             // gimbal_height
      if (cmd_pos_.x > 1.57)
        cmd_pos_.x = 1.57;
      else if (cmd_pos_.x < -1.57)
        cmd_pos_.x = -1.57;
      if (cmd_pos_.y > 0.86)
        cmd_pos_.y = 0.86;
      else if (cmd_pos_.y < -0.86)
        cmd_pos_.y = -0.213;
      if (cmd_pos_.z > 0.213)
        cmd_pos_.z = 0.213;
      else if (cmd_pos_.z < 0)
        cmd_pos_.z = 0;
    }

    else if (dbus_cmd_.s_r == 2) // right paddle down, stop chassis
    {
      cmd_vel_.linear.x = 0;
      cmd_vel_.linear.y = 0;
      cmd_vel_.angular.z = 0;
    }
   

    if (dbus_cmd_.s_l == 3)// left paddle middle, move manipulator
    {
       
      
      if (gripper_signal != dbus_cmd_.key_q)
      {
        
        if (dbus_cmd_.key_q == true)
        {
          sleep(0.05);
          dbus_.getData(&dbus_cmd_);
          if (dbus_cmd_.key_q == true)
            gripper_signal = true;
        }
        else
        {
          sleep(0.05);
          dbus_.getData(&dbus_cmd_);
          if (dbus_cmd_.key_q == false)
            gripper_signal = false;
        }                   
      }
      gripper_signal = dbus_cmd_.key_q;

      if (sucker_signal != dbus_cmd_.key_w)
      {
        if (dbus_cmd_.key_w == true)
        {
          sleep(0.05);
          dbus_.getData(&dbus_cmd_);
          if (dbus_cmd_.key_w == true)
            sucker_signal = true;
        }
        else
        {
          sleep(0.05);
          dbus_.getData(&dbus_cmd_);
          if (dbus_cmd_.key_w == false)
            sucker_signal = false;
        }                   
      }
      sucker_signal = dbus_cmd_.key_w;

      if (rob_signal != dbus_cmd_.key_e)
      {
        if (dbus_cmd_.key_e == true)
        {
          sleep(0.05);
          dbus_.getData(&dbus_cmd_);
          if (dbus_cmd_.key_e == true)
            rob_signal = true;
        }
        else
        {
          sleep(0.05);
          dbus_.getData(&dbus_cmd_);
          if (dbus_cmd_.key_e == false)
            rob_signal = false;
        }  
      }
      rob_signal = dbus_cmd_.key_e;

 
    }

    gpio_data.gpio_state[0] = gpio_data.gpio_state[1] = gripper_signal;
    gpio_data.gpio_state[2] = sucker_signal;
    gpio_data.gpio_state[3] = rob_signal;

    
    gpio_pub_.publish(gpio_data);
    cmd_vel_pub_.publish(cmd_vel_);
    cmd_pos_pub_.publish(cmd_pos_);
    ros::spinOnce();
  }
}

void DBusNode::gpio_callback(const sp_common::GpioData::ConstPtr &gpio_data_)
{
  gpio_data = *gpio_data_;
}
