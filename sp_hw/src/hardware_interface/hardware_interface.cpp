#include "sp_hw/hardware_interface/hardware_interface.hpp"

namespace sp_hw
{
    bool SpRobotHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        XmlRpc::XmlRpcValue xml_rpc_value;
        if (!robot_hw_nh.getParam("actuator_coefficient", xml_rpc_value))
            ROS_ERROR("No Actuator Coefficient Specified");
        else if (!parseActCoeffs(xml_rpc_value))
            return false;

        if (!robot_hw_nh.getParam("actuators", xml_rpc_value))
            ROS_ERROR("No Actuator Specified");
        else if (!parseActData(xml_rpc_value))
            return false;
        return true;
    }
} // namespace : sp_hw