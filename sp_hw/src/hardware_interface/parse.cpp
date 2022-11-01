#include "sp_hw/hardware_interface/hardware_interface.hpp"
#include "sp_hw/hardware_interface/data_types.hpp"
#include "sp_hw/hardware_interface/param_processor.hpp"
#include <XmlRpcException.h>

namespace sp_hw
{
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
                std::cout << it->first << " : "
                          << "0x" << std::hex << id << std::endl;
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
}
