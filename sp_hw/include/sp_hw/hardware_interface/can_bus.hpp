#pragma once

#include <string>
#include <mutex>

#include <ros/ros.h>
#include "sp_hw/hardware_interface/socketcan.h"
#include "sp_hw/hardware_interface/data_types.hpp"

#include <std_msgs/Bool.h>

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
        const std::string bus_name_;
        void halt_callback(const std_msgs::Bool::ConstPtr &halt_);

    private:
        void frameCallback(const can_frame &frame);
        can::SocketCAN socket_can_;
        CanDataPtr data_ptr_;
        // a simple FIFO to store can_data
        std::vector<CanFrameStamp> read_buffer_;
        bool get_pose;

        can_frame rm_can_frame0_;
        can_frame rm_can_frame1_;
        can_frame can_frame2_;

        ros::Subscriber halt_sub_; 
      

        mutable std::mutex mutex_;

        bool halt;
    };
}