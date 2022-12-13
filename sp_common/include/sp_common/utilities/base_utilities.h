#include <ros/ros.h>

namespace sp_common
{
    template <typename T>
    T getParam(ros::NodeHandle &pnh, const std::string &param_name, const T &default_val);
}