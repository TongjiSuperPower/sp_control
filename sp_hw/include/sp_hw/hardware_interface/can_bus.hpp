#pragma once

#include <string>
#include <mutex>

#include <ros/ros.h>
#include "sp_hw/hardware_interface/socketcan.h"
#include "sp_hw/hardware_interface/data_types.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
namespace sp_hw
{
    struct CanFrameStamp
    {
        can_frame frame;
        ros::Time stamp;
    };

    class CanBus
    {
    public:
        CanBus(const std::string &bus_name, CanDataPtr data_ptr, int thread_priority);
        ~CanBus();

        void read(ros::Time time);
        void write();
        void write(can_frame *can_frame);
        void exitFn();
        const std::string bus_name_;

    private:
        void frameCallback(const can_frame &frame);  
        can::SocketCAN socket_can_;
        CanDataPtr data_ptr_;
        // a simple FIFO to store can_data
        std::vector<CanFrameStamp> read_buffer_;

        can_frame rm_can_frame0_;
        can_frame rm_can_frame1_;
        can_frame mix_can_frame2_;

        mutable std::mutex mutex_;
        Eigen::Matrix3d last_matrix{};
        Eigen::Matrix3d current_matrix{};
        ros::Time last_time{};
        ros::Time current_time{};
        ros::NodeHandle nh;
        ros::Publisher velocity_pub_;
        ros::Publisher imu_pub_;
        geometry_msgs::Twist cmd_velocity{};
        sensor_msgs::Imu cmd_imu_{};
    };
}