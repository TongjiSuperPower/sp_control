#include "sp_hw/hardware_interface/hardware_interface.hpp"
#include "sp_hw/hardware_interface/data_types.hpp"
#include "sp_hw/hardware_interface/param_processor.hpp"
#include <XmlRpcException.h>

namespace sp_hw
{
    /*
     * @brief Plot all devices in Tree_Form.
     */
    template <typename T>
    void device_tree(const std::unordered_map<std::string, std::unordered_map<int, T>> &bus_id2device_data)
    {
        for (auto bus_it = bus_id2device_data.begin(); bus_it != bus_id2device_data.end(); ++bus_it)
        {
            std::cout << "|-- " << bus_it->first << std::endl;
            for (auto device_id = bus_it->second.begin(); device_id != bus_it->second.end(); device_id++)
                std::cout << "|   "
                          << "|-- "
                          << "0x" << std::hex << device_id->first << " - " << std::dec
                          << device_id->second.type << " - " << device_id->second.name << std::endl;
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

                if (it->first == "DM_J4310")
                {
                    if (it->second.hasMember("pos2act"))
                        act_coeff.pos2act = xmlRpcGetDouble(it->second, "pos2act");
                    else
                        ROS_WARN_STREAM("Actuator Type " << it->first << "has no associated pos2act");

                    if (it->second.hasMember("vel2act"))
                        act_coeff.vel2act = xmlRpcGetDouble(it->second, "vel2act");
                    else
                        ROS_WARN_STREAM("Actuator Type " << it->first << "has no associated vel2act");

                    if (it->second.hasMember("act2pos_offset"))
                        act_coeff.act2pos_offset = xmlRpcGetDouble(it->second, "act2pos_offset");
                    else
                        ROS_WARN_STREAM("Actuator Type " << it->first << "has no associated act2pos_offset");
                    if (it->second.hasMember("act2vel_offset"))
                        act_coeff.act2vel_offset = xmlRpcGetDouble(it->second, "act2vel_offset");
                    else
                        ROS_WARN_STREAM("Actuator Type " << it->first << "has no associated act2vel_offset");
                    if (it->second.hasMember("act2effort_offset"))
                        act_coeff.act2effort_offset = xmlRpcGetDouble(it->second, "act2effort_offset");
                    else
                        ROS_WARN_STREAM("Actuator Type " << it->first << "has no associated act2effort_offset");
                    if (it->second.hasMember("kp2act"))
                        act_coeff.kp2act = xmlRpcGetDouble(it->second, "kp2act");
                    else
                        ROS_WARN_STREAM("Actuator Type " << it->first << "has no associated kp2act");
                    if (it->second.hasMember("kd2act"))
                        act_coeff.kd2act = xmlRpcGetDouble(it->second, "kd2act");
                    else
                        ROS_WARN_STREAM("Actuator Type " << it->first << "has no associated kd2act");
                }

                if (it->first == "MG_8016")
                {
                    if (it->second.hasMember("act2pos2"))
                        act_coeff.act2pos2 = xmlRpcGetDouble(it->second, "act2pos2");
                    else
                        ROS_WARN_STREAM("Actuator Type " << it->first << "has no associated act2pos2");
                }
                if (it->first == "MG_995")
                {
                    if (it->second.hasMember("pos2act"))
                        act_coeff.pos2act = xmlRpcGetDouble(it->second, "pos2act");
                    else
                        ROS_WARN_STREAM("Actuator Type " << it->first << "has no associated pos2act");
                }

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
                                                                         .stamp = ros::Time::now(),
                                                                         .q_raw = 0,
                                                                         .q_last = 0,
                                                                         .qd_raw = 0,
                                                                         .q_circle = 0,
                                                                         .seq = 0,
                                                                         .pos = 0,
                                                                         .vel = 0,
                                                                         .effort = 0,
                                                                         .cmd_pos = 0,
                                                                         .cmd_vel = 0,
                                                                         .cmd_effort = 0,
                                                                         .exe_pos = 0,
                                                                         .exe_effort = 0,
                                                                         .offset = 0,
                                                                         .offset2 = 0}));
                }
                // TODO(DONE) : use actuator_coefficient to define
                if (type2act_coeffs_.find(type) != type2act_coeffs_.end())
                {
                    hardware_interface::ActuatorStateHandle act_state(bus_id2act_data_[bus][id].name,
                                                                      &bus_id2act_data_[bus][id].pos,
                                                                      &bus_id2act_data_[bus][id].vel,
                                                                      &bus_id2act_data_[bus][id].effort);
                    act_state_interface_.registerHandle(act_state);
                    if (type != "MG_995")
                        effort_act_interface_.registerHandle(hardware_interface::ActuatorHandle(act_state, &bus_id2act_data_[bus][id].exe_effort));
                    else
                        position_act_interface_.registerHandle(hardware_interface::ActuatorHandle(act_state, &bus_id2act_data_[bus][id].exe_pos));
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
        registerInterface(&position_act_interface_);
        device_tree<ActData>(bus_id2act_data_);
        is_actuator_specified_ = true; // now all the actuators have been parsed.

        return true;
    }

    bool SpRobotHW::parseGpioData(XmlRpc::XmlRpcValue &gpio_data)
    {
        // ROBUST : Ensure the type of XmlRpcValue is ValueStruct
        ROS_ASSERT(gpio_data.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        try
        {
            for (auto it = gpio_data.begin(); it != gpio_data.end(); ++it)
            {
                if (!it->second.hasMember("bus"))
                {
                    ROS_ERROR_STREAM("Gpio " << it->first << " has no associated bus.");
                    continue;
                }
                else if (!it->second.hasMember("type"))
                {
                    ROS_ERROR_STREAM("Gpio" << it->first << " has no associated type.");
                    continue;
                }

                std::string bus = it->second["bus"];
                sp_control::GpioType type;
                if (it->second["type"] == "out")
                    type = sp_control::OUTPUT;
                else if (it->second["type"] == "in")
                    type = sp_control::INPUT;
                int id = it->second["id"];

                // Constructing the Actuator_Table
                // Bus_ID  --->  Gpio ID  ---> GpioData(Struct)
                if (bus_id2gpio_data_.find(bus) == bus_id2gpio_data_.end())
                    bus_id2gpio_data_.emplace(std::make_pair(bus, std::unordered_map<int, sp_control::GpioData>()));

                if (!(bus_id2gpio_data_[bus].find(id) == bus_id2gpio_data_[bus].end()))
                {
                    ROS_ERROR_STREAM("Repeat actuator on BUS " << bus << " and ID 0x" << std::hex << id);
                    return false;
                }
                else
                {
                    bus_id2gpio_data_[bus].emplace(std::make_pair(id, sp_control::GpioData{
                                                                          .name = it->first,
                                                                          .stamp = ros::Time::now(),
                                                                          .type = sp_control::OUTPUT,
                                                                          .value = false}));
                }
                sp_control::GpioStateHandle gpio_state_handle(bus_id2gpio_data_[bus][id].name, bus_id2gpio_data_[bus][id].type,
                                                              &bus_id2gpio_data_[bus][id].value);
                gpio_state_interface_.registerHandle(gpio_state_handle);
                if (type == sp_control::OUTPUT)
                {
                    sp_control::GpioCommandHandle gpio_command_handle(bus_id2gpio_data_[bus][id].name, bus_id2gpio_data_[bus][id].type,
                                                                      &bus_id2gpio_data_[bus][id].value);
                    gpio_command_interface_.registerHandle(gpio_command_handle);
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
        device_tree<sp_control::GpioData>(bus_id2gpio_data_);
        is_gpio_specified_ = true; // now all the actuators have been parsed.
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
                                                                         .id2act_data_ = &bus_id2act_data_[bus_name],
                                                                         .id2gpio_data_ = &bus_id2gpio_data_[bus_name]},
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
        jnt_to_act_pos_ = robot_transmissions_.get<transmission_interface::JointToActuatorPositionInterface>();

        /* Lithesh : create a vector of joint_handle used to control joint data directly.
         * When you use ros-controller, effort_joint_handles is not recommanded to use,
         * as it may cause data conflict.
         */
        // auto eff_jnt_iface = &(loader_data.joint_interfaces.effort_joint_interface)
        auto effort_jnt_iface = this->get<hardware_interface::EffortJointInterface>();
        auto position_jnt_iface = this->get<hardware_interface::PositionJointInterface>();
        std::vector<std::string> names;

        if (effort_jnt_iface)
        {

            names = effort_jnt_iface->getNames();
            for (const std::string &name : names)
                effort_joint_handles_.push_back(effort_jnt_iface->getHandle(name));
        }
        if (position_jnt_iface)
        {
            names = position_jnt_iface->getNames();
            for (const std::string &name : names)
                position_joint_handles_.push_back(position_jnt_iface->getHandle(name));
        }
        return true;
    }
}
