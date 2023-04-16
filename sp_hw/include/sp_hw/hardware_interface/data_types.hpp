#pragma once

#include <unordered_map>
#include <sp_common/hardware_interface/gpio_interface.h>

namespace sp_hw
{
    struct ActCoeff
    {
        double act2pos, act2vel, act2effort;
        double pos2act, vel2act, effort2act, max_out;
        double act2pos_offset, act2vel_offset, act2effort_offset, kp2act, kd2act;
        double act2pos2;
    };

    struct ActData
    {
        std::string name;
        std::string type;
        ros::Time stamp;
        // encoder, rotor_rpm, multicycle encoder
        uint16_t q_raw;
        uint16_t q_last;
        int16_t qd_raw;
        int64_t q_circle;
        int64_t seq;

        double pos, vel, effort;
        double cmd_pos, cmd_vel, cmd_effort, exe_effort;
        double offset, offset2;

        bool is_halted = false;
    };

    // only used in can_bus and store the pointers
    struct CanDataPtr
    {
        std::unordered_map<std::string, ActCoeff> *type2act_coeffs_;
        std::unordered_map<int, ActData> *id2act_data_;
        std::unordered_map<int, sp_control::GpioData> *id2gpio_data_;
    };
} // namespace : sp_hw