#include <ros/ros.h>
#include "sp_common/DbusData.h"
#include "sp_common/ChassisCmd.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sp_common/base_utilities.h>

namespace sp_operator
{
    class Operator
    {
        public:
            Operator() = default;;

            bool init();

            virtual void run() = 0;

        protected:

            void dbus_callback(const sp_common::DbusData::ConstPtr &dbus_data);

            void chassis_set();

            ros::NodeHandle nh;

            ros::Publisher chassis_cmd_pub_;

            ros::Publisher cmd_chassis_vel_pub_;

            ros::Publisher cmd_gimbal_vel_pub_;

            ros::Subscriber dbus_sub_;

            sp_common::DbusData dbus_data_, last_dbus_data_;

            geometry_msgs::Twist cmd_chassis_vel_{}; 

            geometry_msgs::Vector3 cmd_gimbal_vel_{};

            sp_common::ChassisCmd chassis_cmd_;

            enum
            {
                FOLLOW,
                NOFOLLOW,
                GYRO
            };

        
    };
}