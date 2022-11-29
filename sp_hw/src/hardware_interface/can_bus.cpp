#include "sp_hw/hardware_interface/can_bus.hpp"

namespace sp_hw
{
    /*!
     * @brief   retrun the signed number with smaller amplitude.
     * @return  eg : limitAmplitude(-1,3) returns -1
     */
    template <typename T>
    inline T limitAmplitude(T a, T b)
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
    }

    void CanBus::write()
    {
        bool has_write_frame0 = false, has_write_frame1 = false;
        std::fill(std::begin(rm_can_frame0_.data), std::end(rm_can_frame0_.data), 0);
        std::fill(std::begin(rm_can_frame1_.data), std::end(rm_can_frame1_.data), 0);

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
            /*!
            // Add other type motors here
            else if (id2act_data.second.type.find("kiTech") != std::string::npos)
            { ; }
            */
        }

        if (has_write_frame0)
            socket_can_.write(&rm_can_frame0_);
        if (has_write_frame1)
            socket_can_.write(&rm_can_frame1_);
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
                    act_data.stamp = can_frame_stamp.stamp;

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
                                   static_cast<double>(act_data.q_raw + 8192 * act_data.q_circle);
                    act_data.vel = act_coeff.act2vel * static_cast<double>(act_data.qd_raw);
                    act_data.effort = act_coeff.act2effort * static_cast<double>(mapped_current);
                    continue; // TODO : ??only process the newest can_frame?? Then why the read_buffer_??
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