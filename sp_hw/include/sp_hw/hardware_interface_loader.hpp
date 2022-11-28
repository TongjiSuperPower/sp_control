#pragma once

#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include "sp_hw/hardware_interface/hardware_interface.hpp"

namespace sp_hw
{
    using namespace std::chrono;
    using clock = high_resolution_clock;

    class SpRobotHWLoader
    {
    public:
        SpRobotHWLoader(ros::NodeHandle &nh, std::shared_ptr<sp_hw::SpRobotHW> hardware_interface);

        ~SpRobotHWLoader();

        void update();

    private:
        // Startup and shutdown of the internal node inside a roscpp program
        ros::NodeHandle nh_;

        // Settings
        double cycle_time_error_threshold_{};

        // Timing
        std::thread loop_thread_;
        std::atomic_bool loop_running_;
        double loop_hz_{};
        ros::Duration elapsed_time_;
        clock::time_point last_time_;

        std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

        // Abstract Hardware Interface for your robot
        std::shared_ptr<SpRobotHW> hardware_interface_;
    };
} // namespace sp_hw