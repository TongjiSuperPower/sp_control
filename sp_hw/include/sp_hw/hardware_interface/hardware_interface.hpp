#pragma once

#include <unordered_map>

#include <ros/ros.h>
#include <XmlRpcValue.h>

/* ROS Control */
#include <hardware_interface/robot_hw.h>

/* SP HW */
#include "sp_hw/hardware_interface/data_types.hpp"

namespace sp_hw
{
    class SpRobotHW : public hardware_interface::RobotHW
    {
    public:
        SpRobotHW() = default;
        bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

    private:
        // Param Parse
        bool parseActCoeffs(XmlRpc::XmlRpcValue &act_coeffs);
        bool parseActData(XmlRpc::XmlRpcValue &act_data);

        // Actuator
        std::unordered_map<std::string, ActCoeff> type2act_coeffs_;
        std::unordered_map<std::string, std::unordered_map<int, ActData>> bus_id2act_data_;
    };
} // namespace : sp_hw
