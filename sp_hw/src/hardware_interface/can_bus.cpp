#include "sp_hw/hardware_interface/can_bus.hpp"

namespace sp_hw
{
    /*!
     * @brief   constrain the number with limitiation.
     * @return  eg : limitAmplitude(-3000,1000) returns -1000
     */
    template <typename T>
    inline T limitAmplitude(T a, T limit)
    {
        limit = fabs(limit);
        int8_t sign = a < 0 ? -1 : 1;
        if (isnan(a))
            return 0.0;
        else if (fabs(a) < limit)
            return a;
        else
            return static_cast<float>(sign * limit);
    }

    /*!
     * @brief   retrun the signed number with smaller amplitude.
     * @return  eg : minAbs(-1,3) returns -1
     */
    template <typename T>
    inline T minAbs(T a, T b)
    {
        return (fabs(a) < fabs(b)) ? a : b;
    }

    CanBus::CanBus(const std::string &bus_name, CanDataPtr data_ptr, int thread_priority = 0)
        : bus_name_(bus_name), data_ptr_(data_ptr)
    {
        while (!socket_can_.open(bus_name_, boost::bind(&CanBus::frameCallback, this, _1), thread_priority) && ros::ok())
            ros::Duration(.5).sleep();

        ROS_INFO("Successfully connected to %s. ", bus_name.c_str());
        rm_can_frame0_.can_id = 0x200;
        rm_can_frame0_.can_dlc = 8;
        rm_can_frame1_.can_id = 0x1FF;
        rm_can_frame1_.can_dlc = 8;
        can_frame2_.can_id = 0x188;
        can_frame2_.can_dlc = 8;
        for (auto &id2act_data : *data_ptr_.id2act_data_)
        {
            if (id2act_data.second.type.find("DM") != std::string::npos)
            {
                can_frame frame{};
                const ActCoeff &act_coeff = data_ptr_.type2act_coeffs_->find(id2act_data.second.type)->second;
                frame.can_id = id2act_data.first;
                frame.can_dlc = 8;
                frame.data[0] = 0xFF;
                frame.data[1] = 0xFF;
                frame.data[2] = 0xFF;
                frame.data[3] = 0xFF;
                frame.data[4] = 0xFF;
                frame.data[5] = 0xFF;
                frame.data[6] = 0xFF;
                frame.data[7] = 0xFC;
                socket_can_.write(&frame);
            }
            else if (id2act_data.second.type.find("MG_8016") != std::string::npos)
            {

                can_frame frame{};
                const ActCoeff &act_coeff = data_ptr_.type2act_coeffs_->find(id2act_data.second.type)->second;
                frame.can_id = id2act_data.first;
                frame.can_dlc = 8;
                frame.data[0] = 0x92;
                frame.data[1] = 0x00;
                frame.data[2] = 0x00;
                frame.data[3] = 0x00;
                frame.data[4] = 0x00;
                frame.data[5] = 0x00;
                frame.data[6] = 0x00;
                frame.data[7] = 0x00;
                socket_can_.write(&frame);
            }
        }
    }

    void CanBus::write()
    {
        bool has_write_frame0 = false, has_write_frame1 = false, has_write_frame2 = false;
        std::fill(std::begin(rm_can_frame0_.data), std::end(rm_can_frame0_.data), 0);
        std::fill(std::begin(rm_can_frame1_.data), std::end(rm_can_frame1_.data), 0);
        std::fill(std::begin(can_frame2_.data), std::end(can_frame2_.data), 0);

        for (auto &id2act_data : *data_ptr_.id2act_data_)
        {

            // RM motors : id ranges from 0x201 to 0x208
            if (id2act_data.second.type.find("rm") != std::string::npos)
            {

                if (id2act_data.second.is_halted)
                    continue;
                const ActCoeff &act_coeff = data_ptr_.type2act_coeffs_->find(id2act_data.second.type)->second;
                int id = id2act_data.first - 0x201;
                // TODO : (Lithesh) is the thread safe here while calcuating the cmd?
                double cmd =
                    limitAmplitude(act_coeff.effort2act * id2act_data.second.exe_effort, act_coeff.max_out);
                // ROS_INFO_STREAM(act_coeff.effort2act * id2act_data.second.exe_effort);
                if (-1 < id && id < 4)
                {
                    rm_can_frame0_.data[2 * id] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
                    rm_can_frame0_.data[2 * id + 1] = static_cast<uint8_t>(cmd);
                    has_write_frame0 = true;
                }
                else if (3 < id && id < 8)
                {
                    rm_can_frame0_.data[2 * (id - 4)] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
                    rm_can_frame0_.data[2 * (id - 4) + 1] = static_cast<uint8_t>(cmd);
                    has_write_frame1 = true;
                }
            }
            else if (id2act_data.second.type.find("DM") != std::string::npos)
            {
                can_frame frame{};
                const ActCoeff &act_coeff = data_ptr_.type2act_coeffs_->find(id2act_data.second.type)->second;
                frame.can_id = id2act_data.first;
                frame.can_dlc = 8;
                uint16_t q_des = (int)(act_coeff.pos2act * (id2act_data.second.cmd_pos - act_coeff.act2pos_offset));
                uint16_t qd_des = (int)(act_coeff.vel2act * (id2act_data.second.cmd_vel - act_coeff.act2vel_offset));
                uint16_t kp = 0;
                uint16_t kd = 0;
                uint16_t tau = (int)(act_coeff.effort2act * (id2act_data.second.exe_effort - act_coeff.act2effort_offset));
                frame.data[0] = q_des >> 8;
                frame.data[1] = q_des & 0xFF;
                frame.data[2] = qd_des >> 8;
                frame.data[3] = ((qd_des & 0xF) << 4) | (kp >> 8);
                frame.data[4] = kp & 0xFF;
                frame.data[5] = kd >> 4;
                frame.data[6] = ((kd & 0xF) << 4) | (tau >> 8);
                frame.data[7] = tau & 0xFF;
                socket_can_.write(&frame);
            }
            else if (id2act_data.second.type.find("MG_8016") != std::string::npos)
            {

                can_frame frame{};
                const ActCoeff &act_coeff = data_ptr_.type2act_coeffs_->find(id2act_data.second.type)->second;
                frame.can_id = id2act_data.first;
                frame.can_dlc = 8;
                double cmd =
                    limitAmplitude(act_coeff.effort2act * id2act_data.second.exe_effort, act_coeff.max_out);
                frame.data[0] = 0xA1;
                frame.data[1] = 0x00;
                frame.data[2] = 0x00;
                frame.data[3] = 0x00;
                frame.data[4] = static_cast<uint8_t>(cmd);
                frame.data[5] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
                frame.data[6] = 0x00;
                frame.data[7] = 0x00;
                socket_can_.write(&frame);
            }
            else if (id2act_data.second.type.find("MG_995") != std::string::npos)
            {
                const ActCoeff &act_coeff = data_ptr_.type2act_coeffs_->find(id2act_data.second.type)->second;
                double cmd =
                    limitAmplitude(act_coeff.pos2act * id2act_data.second.exe_pos , act_coeff.max_out);
                // ROS_INFO_STREAM(cmd);

                int id = id2act_data.first;
                if (id == 0x101)
                {
                    can_frame2_.data[0] = static_cast<int8_t>(cmd);
                    can_frame2_.data[1] = static_cast<int8_t>(static_cast<int16_t>(cmd) >> 8u);
                }
                else if (id == 0x102)
                {
                    can_frame2_.data[2] = static_cast<int8_t>(cmd);
                    can_frame2_.data[3] = static_cast<int8_t>(static_cast<int16_t>(cmd) >> 8u);
                }
                has_write_frame2 = true;
            }
        }

        for (auto &id2gpio_data : *data_ptr_.id2gpio_data_)
        {
            int id = id2gpio_data.first - 0x100;
            bool cmd = id2gpio_data.second.value;

            if (id <= 4)
            {
                if (cmd)
                    can_frame2_.data[id + 3] = 0xFF;
                else
                    can_frame2_.data[id + 3] = 0x00;
            }
            has_write_frame2 = true;
        }

        if (has_write_frame0)
            socket_can_.write(&rm_can_frame0_);
        if (has_write_frame1)
            socket_can_.write(&rm_can_frame1_);
        if (has_write_frame2)
            socket_can_.write(&can_frame2_);
    }
    /*! TODO :  seems that the processing of read_buffer_ will waste a lot of time
     *          because of the thread-switching and mutex_.
     *          Maybe the chassis M3508 doesn't need multiencoder??
     */
    void CanBus::read(ros::Time time)
    {

        std::lock_guard<std::mutex> guard(mutex_);
        for (const CanFrameStamp &can_frame_stamp : read_buffer_)
        {
            can_frame frame = can_frame_stamp.frame;
            // Check if the motor has been registered via ID
            if (data_ptr_.id2act_data_->find(frame.can_id) != data_ptr_.id2act_data_->end())
            {
                ActData &act_data = data_ptr_.id2act_data_->find(frame.can_id)->second;
                const ActCoeff &act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
                if (act_data.type.find("rm") != std::string::npos)
                {
                    act_data.q_raw = (frame.data[0] << 8u) | frame.data[1];
                    act_data.qd_raw = (frame.data[2] << 8u) | frame.data[3];
                    int16_t mapped_current = (frame.data[4] << 8u) | frame.data[5];
                    // ROS_INFO_STREAM(mapped_current);
                    act_data.stamp = can_frame_stamp.stamp;
                    if (act_data.seq < 10)
                    {
                        act_data.offset = act_data.q_raw;
                    }
                    // Basically, motor won't rotate more than 4096 between two time slide.
                    if (act_data.seq != 0)
                    {
                        if (act_data.q_raw - act_data.q_last > 4096)
                            act_data.q_circle--;
                        else if (act_data.q_last - act_data.q_raw > 4096)
                            act_data.q_circle++;
                    }
                    act_data.q_last = act_data.q_raw;
                    act_data.seq++;
                    // convert raw data into standard ActuatorState
                    act_data.pos = act_coeff.act2pos *
                                   static_cast<double>(act_data.q_raw + 8192 * act_data.q_circle - act_data.offset);

                    act_data.vel = act_coeff.act2vel * static_cast<double>(act_data.qd_raw);
                    act_data.effort = act_coeff.act2effort * static_cast<double>(mapped_current);
                    continue; // TODO : ??only process the newest can_frame?? Then why the read_buffer_??S
                }
                else if (act_data.type.find("MG") != std::string::npos)
                {
                    if (frame.data[0] == 0xA1)
                    {
                        if (act_data.seq == 0)
                        {
                            act_data.offset -= act_coeff.act2pos*static_cast<double>((frame.data[7] << 8u) | frame.data[6]);
                        }
                        act_data.q_raw = (frame.data[7] << 8u) | frame.data[6];
                        act_data.qd_raw = (frame.data[5] << 8u) | frame.data[4];

                        int16_t mapped_current = (frame.data[3] << 8u) | frame.data[2];
                        act_data.stamp = can_frame_stamp.stamp;

                        // Basically, motor won't rotate more than 32678 between two time slide.
                        if (act_data.seq > 3)
                        {
                            if (act_data.q_raw - act_data.q_last > 32768)
                                act_data.q_circle--;
                            else if (act_data.q_last - act_data.q_raw > 32768)
                                act_data.q_circle++;
                        }

                        act_data.q_last = act_data.q_raw;
                        act_data.seq++;
                        // convert raw data into  standard ActuatorState
                        act_data.pos = act_coeff.act2pos *
                                           static_cast<double>(act_data.q_raw + 65536 * act_data.q_circle) +
                                       act_data.offset;
                        act_data.vel = act_coeff.act2vel * static_cast<double>(act_data.qd_raw);
                        act_data.effort = act_coeff.act2effort * static_cast<double>(mapped_current);
                        continue;
                    }
                    else if (frame.data[0] == 0x92)
                    {

                        act_data.offset = act_coeff.act2pos2 * static_cast<double>((frame.data[7] << 24u) |
                                                                                   (frame.data[3] << 16u) | (frame.data[2] << 8u) | frame.data[1]);
                        ROS_INFO_STREAM(frame.can_id << " OFFSET" << act_data.offset / 6);
                        continue;
                    }
                }
            }
            else if (frame.can_id == static_cast<unsigned int>(0x000))
            {
                if (data_ptr_.id2act_data_->find(frame.data[0]) != data_ptr_.id2act_data_->end())
                {
                    ActData &act_data = data_ptr_.id2act_data_->find(frame.data[0])->second;
                    const ActCoeff &act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
                    if (act_data.type.find("DM") != std::string::npos)
                    {

                        act_data.q_raw = (frame.data[1] << 8) | frame.data[2];
                        uint16_t qd = (frame.data[3] << 4) | (frame.data[4] >> 4);
                        uint16_t eff = ((frame.data[4] & 0xF) << 8) | frame.data[5];
                        // Multiple cycle
                        // NOTE: The raw data range is -12.5~12.5
                        if (act_data.seq != 0)
                        {
                            double pos_new = act_coeff.act2pos * static_cast<double>(act_data.q_raw) + act_coeff.act2pos_offset +
                                             static_cast<double>(act_data.q_circle) * 25;
                            if (pos_new - act_data.pos > 12.5)
                                act_data.q_circle--;
                            else if (pos_new - act_data.pos < -12.5)
                                act_data.q_circle++;
                        }
                        act_data.stamp = can_frame_stamp.stamp;
                        act_data.seq++;
                        act_data.pos = act_coeff.act2pos * static_cast<double>(act_data.q_raw) + act_coeff.act2pos_offset +
                                       static_cast<double>(act_data.q_circle) * 25;
                        //if (frame.data[0] == 0x01)
                           // ROS_INFO_STREAM(frame.data[0] << "  " << act_data.pos << " " << act_coeff.act2pos_offset << " " << act_data.q_circle);
                        act_data.vel = act_coeff.act2vel * static_cast<double>(qd) + act_coeff.act2vel_offset;
                        act_data.effort = act_coeff.act2effort * static_cast<double>(eff) + act_coeff.act2effort_offset;
                        continue;
                    }
                }
            }
        }
        read_buffer_.clear();
    }
    void CanBus::frameCallback(const can_frame &frame)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        CanFrameStamp can_frame_stamp{.frame = frame, .stamp = ros::Time::now()};
        read_buffer_.push_back(can_frame_stamp);
    }
} // namespace sp_hw