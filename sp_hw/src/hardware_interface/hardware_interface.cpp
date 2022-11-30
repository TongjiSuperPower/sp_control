#include "sp_hw/hardware_interface/hardware_interface.hpp"

namespace sp_hw
{
    // TODO : (Lithesh) Maybe we should use ErrorCode and create log file
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

        if (!robot_hw_nh.getParam("bus", xml_rpc_value))
            ROS_WARN("No Bus Specified");
        else if (xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            // TODO : (Lithesh) only check the 1st element is Not Safe enough.
            ROS_ASSERT(xml_rpc_value[0].getType() == XmlRpc::XmlRpcValue::TypeString);
            if (!initCanBus(xml_rpc_value))
                ROS_WARN("Some Bus Communication has not been initialized \n");
        }
        // TEST
        actuator_state_pub_.reset(
            new realtime_tools::RealtimePublisher<sp_common::ActuatorState>(root_nh, "/actuator_states", 100));
        return true;

        // URDF and Transmission
        // Transmission : code reference <transmission_interface/transmission_interface_loader.h>
        if (!loadUrdf(root_nh))
        {
            ROS_ERROR("hardware_interface : Error occurred while loading Urdf model");
            return false;
        }
        if (!setupTransmission(root_nh))
        {
            ROS_ERROR("hardware_interface : Error occurred while loading Transmission in urdf");
            return false;
        }

        actuator_state_pub_.reset(
            new realtime_tools::RealtimePublisher<sp_common::ActuatorState>(root_nh, "/actuator_states", 100));
        return true;
    }

    void SpRobotHW::read(const ros::Time &time, const ros::Duration &period)
    {
        for (auto &bus : can_buses_)
            bus->read(time);
        for (auto &id2act_datas : bus_id2act_data_)
            for (auto &act_data : id2act_datas.second)
            {
                try
                {
                    act_data.second.is_halted = (time - act_data.second.stamp).toSec() > 0.1 || false;
                }
                catch (std::runtime_error &ex)
                {
                }
                if (act_data.second.is_halted)
                {
                    act_data.second.seq = 0;
                    act_data.second.vel = 0;
                    act_data.second.effort = 0;
                }
            }
        /*
    if (is_actuator_specified_)
        act_to_jnt_state_->propagate();
        */
    }

    void SpRobotHW::write(const ros::Time &time, const ros::Duration &period)
    {
        if (is_actuator_specified_)
        {
            jnt_to_act_effort_->propagate();
            for (auto &id2act_data : bus_id2act_data_)
                for (auto &act_data : id2act_data.second)
                    act_data.second.cmd_effort = act_data.second.exe_effort;
        }
        for (auto &bus : can_buses_)
            bus->write();
        publishActuatorState(time);
    }

    void SpRobotHW::setCanBusThreadPriority(const int &thread_priority) { thread_priority_ = thread_priority; }

    /*! @brief  if you want higher efficiency, this function can be ommited in
     *          SpRobotHW::write();
     */
    void SpRobotHW::publishActuatorState(const ros::Time &time)
    {
        if (!is_actuator_specified_)
            return;
        if (actuator_state_pub_->trylock())
        {
            sp_common::ActuatorState actuator_state;
            for (const auto &id2act_datas : bus_id2act_data_)
                for (const auto &act_data : id2act_datas.second)
                {
                    actuator_state.stamp.push_back(act_data.second.stamp);
                    actuator_state.name.push_back(act_data.second.name);
                    actuator_state.type.push_back(act_data.second.type);
                    actuator_state.bus.push_back(id2act_datas.first);
                    actuator_state.id.push_back(act_data.first);

                    actuator_state.position_raw.push_back(act_data.second.q_raw);
                    actuator_state.velocity_raw.push_back(act_data.second.qd_raw);
                    actuator_state.circle.push_back(act_data.second.q_circle);
                    actuator_state.last_position_raw.push_back(act_data.second.q_last);

                    actuator_state.position.push_back(act_data.second.pos);
                    actuator_state.velocity.push_back(act_data.second.vel);
                    actuator_state.effort.push_back(act_data.second.effort);
                    actuator_state.cmd_effort.push_back(act_data.second.cmd_effort);
                    actuator_state.exe_effort.push_back(act_data.second.exe_effort);
                }
            actuator_state_pub_->msg_ = actuator_state;
            actuator_state_pub_->unlockAndPublish();
            last_publish_time_ = time;
        }
    }
} // namespace sp_hw