#include "sp_hw/hardware_interface/can_bus.hpp"
#include "tf/tf.h"



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
     /*!
     * @brief   init the frames[0x200,0x1FF] that will be sent to RM motor, and enable the DM motor.
     */
    bool exit_singal = false;

    void exit_handler(int signum)
    {
        exit_singal = true;
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
        mix_can_frame2_.can_id = 0x188;
        mix_can_frame2_.can_dlc = 8;
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
        velocity_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_velocity", 10);
        quat_pub_ = nh.advertise<geometry_msgs::Quaternion>("cmd_quat", 10);

        signal(SIGINT, exit_handler);
        
        
       
;    }
    /**
     * @brief Sending actuators and gpios commands
     * @param time
     */
    void CanBus::write()
    {
        if (exit_singal)
        {
            exitFn();
        }

        // Fill all the frames to be sent with all 0
        bool has_write_frame0 = false, has_write_frame1 = false, has_write_frame2 = false;
        std::fill(std::begin(rm_can_frame0_.data), std::end(rm_can_frame0_.data), 0);
        std::fill(std::begin(rm_can_frame1_.data), std::end(rm_can_frame1_.data), 0);
        std::fill(std::begin(mix_can_frame2_.data), std::end(mix_can_frame2_.data), 0);

        //Traverse all the mounted motors
        for (auto &id2act_data : *data_ptr_.id2act_data_)
        {
            // RM motors : id ranges from 0x201 to 0x208
            if (id2act_data.second.type.find("rm") != std::string::npos)
            {
                // Block sending data when the motor's receive data is missing. 
                if (id2act_data.second.is_halted)
                    continue;
                // Acquire the motor coefficient
                const ActCoeff &act_coeff = data_ptr_.type2act_coeffs_->find(id2act_data.second.type)->second;
                int id = id2act_data.first - 0x201;
                // Acquire the current value calculated by the upper layer controllers and limit its value for safe.
                double cmd =
                    limitAmplitude(act_coeff.effort2act * id2act_data.second.exe_effort, act_coeff.max_out);
                // Fill the 0x200 frame, which can control 0x201 to 0x204 motor. 
                if (-1 < id && id < 4)
                {
                    rm_can_frame0_.data[2 * id] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
                    rm_can_frame0_.data[2 * id + 1] = static_cast<uint8_t>(cmd);
                    has_write_frame0 = true;
                }
                // Fill the 0x1FF frame, which can control 0x205 to 0x208 motor. 
                else if (3 < id && id < 8)
                {
                    rm_can_frame1_.data[2 * (id - 4)] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
                    rm_can_frame1_.data[2 * (id - 4) + 1] = static_cast<uint8_t>(cmd);
                    has_write_frame1 = true;
                }
            }
            // DM motors : MIT protocol
            else if (id2act_data.second.type.find("DM") != std::string::npos)
            {
                // Block sending data when the motor's receive data is missing.
                if (id2act_data.second.is_halted)
                    continue;
                can_frame frame{};
                // Acquire the motor coefficient
                const ActCoeff &act_coeff = data_ptr_.type2act_coeffs_->find(id2act_data.second.type)->second;
                frame.can_id = id2act_data.first;
                frame.can_dlc = 8;
                // Acquire the pos, vel and effort cmd calculated by the upper layer controllers.
                // Actually only the effort value is used. 
                uint16_t q_des = (int)(act_coeff.pos2act * (id2act_data.second.cmd_pos - act_coeff.act2pos_offset));
                uint16_t qd_des = (int)(act_coeff.vel2act * (id2act_data.second.cmd_vel - act_coeff.act2vel_offset));
                uint16_t kp = 0;
                uint16_t kd = 0;
                uint16_t tau = (int)(act_coeff.effort2act * (id2act_data.second.exe_effort - act_coeff.act2effort_offset));
                // MIT sending protocol.
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
             // MG motors : custom protocol, very strange.
            else if (id2act_data.second.type.find("MG_8016") != std::string::npos)
            {
                // Block sending data when the motor's receive data is missing.
                if (id2act_data.second.is_halted)
                    continue;
                can_frame frame{};
                // Acquire the motor coefficient
                const ActCoeff &act_coeff = data_ptr_.type2act_coeffs_->find(id2act_data.second.type)->second;
                frame.can_id = id2act_data.first;
                frame.can_dlc = 8;
                // Acquire the current value calculated by the upper layer controllers and limit its value for safe.      
                double cmd =
                    limitAmplitude(act_coeff.effort2act * id2act_data.second.exe_effort, act_coeff.max_out);
                // 0xA1 frame: single motor force control. 
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
            // Servo : position control
            else if (id2act_data.second.type.find("MG_995") != std::string::npos)
            {
                // Acquire the servo coefficient
                const ActCoeff &act_coeff = data_ptr_.type2act_coeffs_->find(id2act_data.second.type)->second;
                double cmd =
                    limitAmplitude(act_coeff.pos2act * id2act_data.second.exe_pos, act_coeff.max_out);
                int id = id2act_data.first;
                // There are two servo can be used.
                // Can id from 0x101 to 0x102.
                if (id == 0x101)
                {
                    mix_can_frame2_.data[0] = static_cast<uint8_t>(cmd);
                    mix_can_frame2_.data[1] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
                }
                else if (id == 0x102)
                {
                    mix_can_frame2_.data[2] = static_cast<uint8_t>(cmd);
                    mix_can_frame2_.data[3] = static_cast<uint8_t>(static_cast<int16_t>(cmd) >> 8u);
                }
                has_write_frame2 = true;
            }
        }

        for (auto &id2gpio_data : *data_ptr_.id2gpio_data_)
        {
            int id = id2gpio_data.first - 0x100;
            bool cmd = id2gpio_data.second.value;
            // There are four gpios can be used.
            // Can id from 0x103 to 0x106.
            if (id <= 4)
            {
                if (cmd)
                    mix_can_frame2_.data[id + 3] = 0xFF;
                else
                    mix_can_frame2_.data[id + 3] = 0x00;
            }
            has_write_frame2 = true;
        }

        // 0X200, 0X1FF and 0X188 frame will only be sent when the associated actuators or gpio are mounted
        if (has_write_frame0)
            socket_can_.write(&rm_can_frame0_);
        if (has_write_frame1)
            socket_can_.write(&rm_can_frame1_);
        if (has_write_frame2)
            socket_can_.write(&mix_can_frame2_);
    }
    /**
     * @brief Processing data in the read_buffer_
     * @todo  Traversing the read_buffer_ from beginning to end will cause duplicate refreshes
     * @param time
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
                    // convert raw data into standard ActuatorState.
                    // Multiple cycle.
                    // The original pos is be set 0.
                    act_data.pos = act_coeff.act2pos *
                                   static_cast<double>(act_data.q_raw + 8192 * act_data.q_circle - act_data.offset);

                    act_data.vel = act_coeff.act2vel * static_cast<double>(act_data.qd_raw);
                    act_data.effort = act_coeff.act2effort * static_cast<double>(mapped_current);
                    continue;
                }
                else if (act_data.type.find("MG") != std::string::npos)
                {
                    if (frame.data[0] == 0xA1)
                    {
                        if (act_data.seq == 0)
                        {
                            act_data.offset2 = (frame.data[7] << 8u) | frame.data[6];
                        }
                        act_data.q_raw = ((frame.data[7] << 8u) | frame.data[6]) - act_data.offset2;
                        act_data.qd_raw = (frame.data[5] << 8u) | frame.data[4];

                        int16_t mapped_current = (frame.data[3] << 8u) | frame.data[2];
                        act_data.stamp = can_frame_stamp.stamp;

                        // Basically, motor won't rotate more than 32678 between two time slide.
                        if (act_data.seq != 0)
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
                        ROS_INFO_STREAM(frame.can_id << " OFFSET " << act_data.offset / 6);
                        continue;
                    }
                }
            }
            // DM motor 
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
                        act_data.vel = act_coeff.act2vel * static_cast<double>(qd) + act_coeff.act2vel_offset;
                        act_data.effort = act_coeff.act2effort * static_cast<double>(eff) + act_coeff.act2effort_offset;
                        continue;
                    }
                }
            }
            else if (frame.can_id == static_cast<unsigned int>(0x189))
            {
                last_matrix = current_matrix;
                last_time = current_time;

                
                double w = 0.0001*(int16_t)((frame.data[0] << 8) | frame.data[1]);
                double x = 0.0001*(int16_t)((frame.data[2] << 8) | frame.data[3]);
                double y = 0.0001*(int16_t)((frame.data[4] << 8) | frame.data[5]);
                double z = 0.0001*(int16_t)((frame.data[6] << 8) | frame.data[7]);
                cmd_quat_.w = w;
                cmd_quat_.x = x;
                cmd_quat_.y = y;
                cmd_quat_.z = z;
                Eigen::Quaterniond quat(w, x, y, z);
                quat.normalized(); 
                current_matrix = quat.toRotationMatrix();
                current_time = ros::Time::now();
                ros::Duration duration = current_time - last_time;
                double secs = duration.toSec();
                Eigen::Matrix3d vel_matrix = current_matrix.inverse()*(current_matrix - last_matrix) /secs;
                //ROS_INFO_STREAM("last_matrix: \n" << last_matrix);
                //ROS_INFO_STREAM("current_matrix: \n" << current_matrix);
                // ROS_INFO_STREAM("vel_matrix: \n" << vel_matrix);
                double wx = (vel_matrix(2, 1) -  vel_matrix(1, 2)) / 2;
                double wy = (vel_matrix(0, 2) -  vel_matrix(2, 0)) / 2;
                double wz = (vel_matrix(1, 0) -  vel_matrix(0, 1)) / 2;
                cmd_velocity.angular.x = wx;
                cmd_velocity.angular.y = wy;
                cmd_velocity.angular.z = wz;
                quat_pub_.publish(cmd_quat_);
                velocity_pub_.publish(cmd_velocity);

            }
        }
        read_buffer_.clear();
    }

    void CanBus::exitFn()
    {
        for (auto &id2act_data : *data_ptr_.id2act_data_)
        {

            if (id2act_data.second.type.find("rm") != std::string::npos)
            {
                can_frame frame0{}, frame1{};
                frame0.can_id = 0x200;
                frame1.can_id = 0x1FF;
                frame0.can_dlc = 8;
                frame1.can_dlc = 8;
                for (int i = 0; i < 8; i++)
                {
                    frame0.data[i] = 0x00;
                    frame1.data[i] = 0x00;
                }
                socket_can_.write(&frame0);
                socket_can_.write(&frame1);
            }
            else if (id2act_data.second.type.find("DM") != std::string::npos)
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
                frame.data[7] = 0xFD;
                socket_can_.write(&frame);
            }
            else if (id2act_data.second.type.find("MG_8016") != std::string::npos)
            {

                can_frame frame{};
                const ActCoeff &act_coeff = data_ptr_.type2act_coeffs_->find(id2act_data.second.type)->second;
                frame.can_id = id2act_data.first;
                frame.can_dlc = 8;
                frame.data[0] = 0x80;
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
        ROS_INFO_STREAM("Canbus Exit");
        exit(0);
    }
    /**
     * @brief Callback function called after receive can frame.
     * @param time
     */
    void CanBus::frameCallback(const can_frame &frame)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        CanFrameStamp can_frame_stamp{.frame = frame, .stamp = ros::Time::now()};
        read_buffer_.push_back(can_frame_stamp);
    }
    /**
     * @brief Destructor.
     * @todo seems useless
     * @param time
     */

    CanBus::~CanBus()
    {
        
    }

} // namespace sp_hw
