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
        for (auto bus_it = bus_id2act_data.begin(); bus_it != bus_id2act_data.end(); ++bus_it)
        {
            std::cout << "|-- " << bus_it->first << std::endl;
            for (auto act_id = bus_it->second.begin(); act_id != bus_it->second.end(); act_id++)
                std::cout << "|   "
                          << "|-- "
                          << "0x" << std::hex << act_id->first << " - " << std::dec
                          << act_id->second.type << " - " << act_id->second.name << std::endl;
        }
    }

    // Lithesh : I think ParseError is more terrible than ParameterMissing
    // ParseError usually means Misunderstanding of what to write in YAML
    // And i hate Misunderstanding.
    bool SpRobotHW::parseActCoeffs(XmlRpc::XmlRpcValue &act_coeffs)
    {
        // ROBUST : Ensure the type of XmlRpcValue is ValueStruct
        ROS_ASSERT(act_coeffs.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        try
        {
            for (auto it = act_coeffs.begin(); it != act_coeffs.end(); ++it)
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
            for (auto it = act_data.begin(); it != act_data.end(); ++it)
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
                // TODO(DONE) : use actuator_coefficient to define
                if (type2act_coeffs_.find(type) != type2act_coeffs_.end())
                {
                    hardware_interface::ActuatorStateHandle act_state(bus_id2act_data_[bus][id].name,
                                                                      &bus_id2act_data_[bus][id].pos,
                                                                      &bus_id2act_data_[bus][id].vel,
                                                                      &bus_id2act_data_[bus][id].effort);
                    act_state_interface_.registerHandle(act_state);
                    effort_act_interface_.registerHandle(
                        hardware_interface::ActuatorHandle(act_state, &bus_id2act_data_[bus][id].exe_effort));
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
        registerInterface(&act_state_interface_);
        registerInterface(&effort_act_interface_);
        actuator_tree(bus_id2act_data_);
        is_actuator_specified_ = true; // now all the actuators have been parsed.

        return true;
    }

    bool SpRobotHW::initCanBus(XmlRpc::XmlRpcValue &bus_list)
    {
        uint8_t EC = 0;
        for (int i = 0; i < bus_list.size(); ++i)
        {
            std::string bus_name = bus_list[i];
            if (bus_name.find("can") != std::string::npos)
            {
                // can_buses.push_back(new CanBus) may cause memory leak.
                can_buses_.push_back(std::make_unique<CanBus>(bus_name,
                                                              CanDataPtr{.type2act_coeffs_ = &type2act_coeffs_,
                                                                         .id2act_data_ = &bus_id2act_data_[bus_name]},
                                                              thread_priority_));
                ROS_INFO("\033[42;37m [SP_HW] *%s* Initialized ! \033[0m", bus_name.c_str());
            }
            else
            {
                ROS_WARN("\033[41;37m [SP_HW] Unknown Bus : %s\033[0m", bus_name.c_str());
                EC++;
            }
        }
        if (EC == 0)
            return true;
        else
            return false;
    }

    bool SpRobotHW::loadUrdf(ros::NodeHandle &root_nh)
    {
        if (urdf_model_ == nullptr)
            urdf_model_ = std::make_shared<urdf::Model>();
        root_nh.getParam("/robot_description", urdf_string_);
        return !urdf_string_.empty() && urdf_model_->initString(urdf_string_);
    }

    /*
     * @brief   Parse the Transmission block in the URDF.
     *          Create transmission_interface and register handles
     *          via robot_transmissions_.
     */
    bool SpRobotHW::setupTransmission(ros::NodeHandle &root_nh)
    {
        try
        {
            transmission_iface_loader_ =
                std::make_unique<transmission_interface::TransmissionInterfaceLoader>(this, &robot_transmissions_);
        }
        catch (const std::invalid_argument &ex)
        {
            ROS_ERROR_STREAM("Fariled to create transmission interface loader. " << ex.what());
            return false;
        }
        catch (const pluginlib::LibraryLoadException &ex)
        {
            ROS_ERROR_STREAM("Fariled to create transmission interface loader. " << ex.what());
            return false;
        }
        catch (...)
        {
            ROS_ERROR_STREAM("Fariled to create transmission interface loader. ");
            return false;
        }

        // Load all the transmission block in Urdf.
        if (!transmission_iface_loader_->load(urdf_string_))
            return false;

        act_to_jnt_state_ = robot_transmissions_.get<transmission_interface::ActuatorToJointStateInterface>();
        jnt_to_act_effort_ = robot_transmissions_.get<transmission_interface::JointToActuatorEffortInterface>();

        /* Lithesh : create a vector of joint_handle used to control joint data directly.
         * When you use ros-controller, effort_joint_handles is not recommanded to use,
         * as it may cause data conflict.
         */
        // auto eff_jnt_iface = &(loader_data.joint_interfaces.effort_joint_interface)
        auto effort_jnt_iface = this->get<hardware_interface::EffortJointInterface>();
        std::vector<std::string> names = effort_jnt_iface->getNames();
        for (const std::string &name : names)
            effort_joint_handles_.push_back(effort_jnt_iface->getHandle(name));

        return true;
    }
}
