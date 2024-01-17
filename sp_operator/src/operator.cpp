#include "sp_operator/operator.h"


namespace sp_operator
{

    bool Operator::init()
    {
        cmd_chassis_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_chassis_vel", 10);
        cmd_gimbal_vel_pub_ = nh.advertise<geometry_msgs::Vector3>("/cmd_gimbal_vel", 10);
        chassis_cmd_pub_ = nh.advertise<sp_common::ChassisCmd>("/chassis_cmd", 10);
        dbus_sub_ = nh.subscribe<sp_common::DbusData>("/dbus_data", 10, &Operator::dbus_callback, this);

        return true;
    }


    void Operator::dbus_callback(const sp_common::DbusData::ConstPtr &dbus_data)
    {
        dbus_data_= *dbus_data;
    }

}