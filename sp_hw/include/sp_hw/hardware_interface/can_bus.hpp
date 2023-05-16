#pragma once

#include <string>
#include <mutex>

#include <ros/ros.h>
#include "sp_hw/hardware_interface/socketcan.h"
#include "sp_hw/hardware_interface/data_types.hpp"

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

        void read(ros::Time time);
        void write();
        void write(can_frame *can_frame);
        const std::string bus_name_;

    private:
        void frameCallback(const can_frame &frame);
        can::SocketCAN socket_can_;
        CanDataPtr data_ptr_;
        // a simple FIFO to store can_data
        std::vector<CanFrameStamp> read_buffer_;

        can_frame rm_can_frame0_;
        can_frame rm_can_frame1_;
        can_frame can_frame2_;

        mutable std::mutex mutex_;
    };
}