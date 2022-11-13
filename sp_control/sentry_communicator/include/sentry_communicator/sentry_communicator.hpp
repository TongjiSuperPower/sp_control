#pragma once
#include "sentry_communicator/socketcan.h"
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <string>

namespace sentry_communicator
{
    struct Command
    {
        geometry_msgs::Twist cmd_vel_;
        ros::Time stamp_;
    };

    struct CanFrameStamp
    {
        can_frame frame;
        ros::Time stamp;
    };

    class CanBus
    {
    public:
        CanBus(const std::string &bus_name, int thread_priority, ros::NodeHandle &root_nh);
        void read(ros::Time time);
        void write();

    private:
        /*
         * @brief This Function will be called when CAN_DATA is captured by the thread.
         */
        void frameCallback(const can_frame &frame);
        void cmdChassisCallback(const geometry_msgs::Twist::ConstPtr &msg);

        const std::string bus_name_;
        can::SocketCAN socket_can_;
        ros::Subscriber cmd_chassis_sub_;

        // Lithesh TODO : if the real-time performance should be concerned ?
        Command cmd_struct_;
        // the int array used to contain data_frame, which has 8 byte
        uint8_t *can_data_;
        can_frame frame_;
        // mutable std::mutex mutex_;
    };
} //  namespace : sentry_communicator