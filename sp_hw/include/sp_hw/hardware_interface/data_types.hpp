#pragma once

#include <unordered_map>

namespace sp_hw
{
    struct ActCoeff
    {
        double act2pos, act2vel, act2effort, effort2act, max_out;
    };

    struct ActData
    {
        std::string name;
        std::string type;
        ros::Time stamp;
        double pos, vel, effort;
    };
} // namespace : sp_hw