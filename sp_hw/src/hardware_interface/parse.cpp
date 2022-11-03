#include "sp_hw/hardware_interface/hardware_interface.hpp"
#include "sp_hw/hardware_interface/data_types.hpp"
#include "sp_hw/hardware_interface/param_processor.hpp"
#include <XmlRpcException.h>

namespace sp_hw
{
    /*
     * @brief Plot all actuators in Tree_Form.
     */
    void actuator_tree(const std::unordered_map<std::string, std::unordered_map<int, ActData>> &bus_id2act_data)
    {
        for (auto bus_it = bus_id2act_data.begin(); bus_it != bus_id2act_data.end(); bus_it++)
        {
            std::cout << "|-- " << bus_it->first << std::endl;
            for (auto act_id = bus_it->second.begin(); act_id != bus_it->second.end(); act_id++)
                std::cout << "|   "
                          << "|-- "
                          << "0x" << std::hex << act_id->first << " - " << std::dec
                          << act_id->second.type << " - " << act_id->second.name << std::endl;
        }
    }

    // I think ParseError is more terrible than ParameterMissing
    // ParseError usually means Misunderstanding of what to write in YAML
    // And i hate Misunderstanding.
    bool SpRobotHW::parseActCoeffs(XmlRpc::XmlRpcValue &act_coeffs)
    {
        // ROBUST : Ensure the type of XmlRpcValue is ValueStruct
        ROS_ASSERT(act_coeffs.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        try
        {
            for (auto it = act_coeffs.begin(); it != act_coeffs.end(); it++)
            {
                ActCoeff act_coeff{};

                if (it->second.hasMember("act2pos"))
                    act_coeff.act2pos = xmlRpcGetDouble(it->second, "act2pos");
                else
                    ROS_WARN_STREAM("Actuator Type " << it->first << "has no associated act2pos");

                if (it->second.hasMember("act2vel"))
                    act_coeff.act2vel = xmlRpcGetDouble(it->second, "act2vel");
                else
                    ROS_WARN_STREAM("Actuator Type " << it->first << "has no associated act2vel");

                if (it->second.hasMember("act2effort"))
                    act_coeff.act2effort = xmlRpcGetDouble(it->second, "act2effort");
                else
                    ROS_WARN_STREAM("Actuator Type " << it->first << "has no associated act2effort");

                if (it->second.hasMember("effort2act"))
                    act_coeff.effort2act = xmlRpcGetDouble(it->second, "effort2act");
                else
                    ROS_WARN_STREAM("Actuator Type " << it->first << "has no associated effort2act");

                if (it->second.hasMember("max_out"))
                    act_coeff.max_out = xmlRpcGetDouble(it->second, "max_out");
                else
                    ROS_WARN_STREAM("Actuator Type " << it->first << "has no associated max_out");

                std::string type = it->first;
                if (type2act_coeffs_.find(type) == type2act_coeffs_.end())
                    type2act_coeffs_.emplace(make_pair(type, act_coeff));
                else
                    // Actually this line is redundant, ROS will cover the param.
                    ROS_WARN_STREAM("Repeat actuator coefficient for " << type);
            }
        }
        catch (XmlRpc::XmlRpcException &e)
        {
            // throw TypeError
            ROS_FATAL_STREAM("Exception raised by XmlRpc while Parsing the"
                             << "Configuration: " << e.getMessage() << ".\n"
                             << "Check the Config Yaml.");
            return false;
        }
        return true;
    }

    /*
     * @brief Parse and register actuators.
     */
    bool SpRobotHW::parseActData(XmlRpc::XmlRpcValue &act_data)
    {
        // ROBUST : Ensure the type of XmlRpcValue is ValueStruct
        ROS_ASSERT(act_data.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        try
        {
            for (auto it = act_data.begin(); it != act_data.end(); it++)
            {
                ActCoeff act_coeff{};

                if (!it->second.hasMember("bus"))
                {
                    ROS_ERROR_STREAM("Actuator " << it->first << " has no associated bus.");
                    continue;
                }
                else if (!it->second.hasMember("type"))
                {
                    ROS_ERROR_STREAM("Actuator " << it->first << " has no associated type.");
                    continue;
                }
                else if (!it->second.hasMember("id"))
                {
                    ROS_ERROR_STREAM("Actuator " << it->first << " has no associated ID.");
                    continue;
                }

                std::string bus = it->second["bus"], type = it->second["type"];
                int id = it->second["id"];

                // Constructing the Actuator_Table
                // Bus_ID  --->  Actuator ID  ---> ActData(Struct)
                if (bus_id2act_data_.find(bus) == bus_id2act_data_.end())
                    bus_id2act_data_.emplace(std::make_pair(bus, std::unordered_map<int, ActData>()));

                if (!(bus_id2act_data_[bus].find(id) == bus_id2act_data_[bus].end()))
                {
                    ROS_ERROR_STREAM("Repeat actuator on BUS " << bus << " and ID 0x" << std::hex << id);
                    return false;
                }
                else
                {
                    bus_id2act_data_[bus].emplace(std::make_pair(id, ActData{
                                                                         .name = it->first,
                                                                         .type = type,
                                                                         .stamp = ros::Time::now()}));
                }
                // TODO : use actuator_coefficient to define
                if (type.find("rm") != std::string::npos)
                {
                    hardware_interface::JointStateHandle jnt_state(bus_id2act_data_[bus][id].name,
                                                                   &bus_id2act_data_[bus][id].pos,
                                                                   &bus_id2act_data_[bus][id].vel,
                                                                   &bus_id2act_data_[bus][id].effort);
                    jnt_state_interface_.registerHandle(jnt_state);
                    effort_jnt_interface_.registerHandle(
                        hardware_interface::JointHandle(jnt_state, &bus_id2act_data_[bus][id].exe_effort));
                }
                else
                {
                    ROS_ERROR_STREAM("Actuator " << it->first << "'s type has not been declared.");
                    return false;
                }
            }
        }
        catch (XmlRpc::XmlRpcException &e)
        {
            // throw TypeError
            ROS_FATAL_STREAM("Exception raised by XmlRpc while Parsing the"
                             << "Configuration: " << e.getMessage() << ".\n"
                             << "Check the Config Yaml.");
            return false;
        }
        registerInterface(&jnt_state_interface_);
        registerInterface(&effort_jnt_interface_);
        actuator_tree(bus_id2act_data_);
        return true;
    }
}
