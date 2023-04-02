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

int main(int argc, char** argv)
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
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  nh_.param<std::string>("serial_port", serial_port_, "/dev/usbDbus");

  dbus_.init(serial_port_.data());
}

void DBusNode::run()
{
  cmd_vel_.linear.x = 0.00;
  cmd_vel_.linear.y = 0.00;
  cmd_vel_.angular.z = 0.00;
  dbus_.read();
  dbus_.getData(&dbus_cmd_);
  dbus_pub_.publish(dbus_cmd_);
  cmd_vel_.linear.x = dbus_cmd_.ch_r_x;
  cmd_vel_.linear.y = dbus_cmd_.ch_r_y;
  cmd_vel_.angular.z = dbus_cmd_.ch_l_x;
  cmd_vel_pub_.publish(cmd_vel_);
}
