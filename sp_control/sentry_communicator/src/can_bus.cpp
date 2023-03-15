#include "sentry_communicator/can_bus.hpp"
#define MAX_SPEED 10.f
#define MIN_SPEED -10.f

#define MAX_ACCEL 20.f
#define MIN_ACCEL -20.f
#define ID_ACCEL 0x111

#define MAX_GYRO  10.f
#define MIN_GYRO  -10.f
#define ID_GYRO  0x112

#define MAX_QUAT  1.f
#define MIN_QUAT  -1.f
#define ID_QUAT  0x113

#define MAX_YAW 12.5f
#define MIN_YAW -12.5f
#define ID_YAW  0x114

namespace sentry_communicator
{

    CanBus::CanBus(const std::string &bus_name, int thread_priority, ros::NodeHandle &root_nh)
        : bus_name_(bus_name)
    {
        lowercom_data_pub = root_nh.advertise<nav_msgs::Odometry>("lowercom_data",1000);
        lower_com_data.header.frame_id = "odom";
        lower_com_data.child_frame_id = "base_link";

        while (!socket_can_.open(bus_name, boost::bind(&CanBus::frameCallback, this, _1), thread_priority) && ros::ok())
            ros::Duration(.5).sleep();
        ROS_INFO("[CAN_BUS] : Successfully connected to %s.", bus_name.c_str());
        cmd_chassis_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &CanBus::cmdChassisCallback, this);
        
        frame_.can_id = 0x111;
        frame_.can_dlc = 8;
    }

    void CanBus::write()
    {
        // set all data_byte to 0
        std::fill(std::begin(frame_.data), std::end(frame_.data), 0);
        // Lithesh TODO : add speed_limit
        const geometry_msgs::Twist &command_velocity = realtime_buffer_.readFromNonRT()->cmd_vel_;

        uint16_t vel_x = float2uint(command_velocity.linear.x, MIN_SPEED, MAX_SPEED, 12);
        uint16_t vel_y = float2uint(command_velocity.linear.y, MIN_SPEED, MAX_SPEED, 12);
        uint16_t vel_z = float2uint(command_velocity.angular.z, MIN_SPEED, MAX_SPEED, 12);
        frame_.data[0] = static_cast<uint8_t>(vel_x >> 4u);
        frame_.data[1] = static_cast<uint8_t>((vel_x & 0xF) << 4u | vel_y >> 8u);
        frame_.data[2] = static_cast<uint8_t>(vel_y);
        frame_.data[3] = static_cast<uint8_t>(vel_z >> 4u);
        frame_.data[4] = static_cast<uint8_t>((vel_z & 0xF) << 4u | 0xF);

    	socket_can_.write(&frame_);
    }

    void CanBus::cmdChassisCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        Command cmd_struct_temp{.cmd_vel_ = *msg, .stamp_ = ros::Time::now()};
        realtime_buffer_.writeFromNonRT(cmd_struct_temp);
    }

    void CanBus::frameCallback(const can_frame &frame)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if(frame.can_id == ID_ACCEL)//Acceleration of IMU
        {
            data = (frame.data[0] << 8u) | frame.data[1];
            lower_com_data.twist.twist.linear.x = uint2float(data, MIN_ACCEL, MAX_ACCEL, 16);
            data = (frame.data[2] << 8u) | frame.data[3];
            lower_com_data.twist.twist.linear.y = uint2float(data, MIN_ACCEL, MAX_ACCEL, 16);
            data = (frame.data[4] << 8u) | frame.data[5];
            lower_com_data.twist.twist.linear.z = uint2float(data, MIN_ACCEL, MAX_ACCEL, 16);

            lower_com_data.header.stamp = ros::Time::now();
            lowercom_data_pub.publish(lower_com_data);
            
        }
        if(frame.can_id == ID_GYRO)//Angular velocity of IMU
        {
            data = (frame.data[0] << 8u) | frame.data[1];
            lower_com_data.twist.twist.angular.x = uint2float(data, MIN_GYRO, MAX_GYRO, 16);
            data = (frame.data[2] << 8u) | frame.data[3];
            lower_com_data.twist.twist.angular.y = uint2float(data, MIN_GYRO, MAX_GYRO, 16);
            data = (frame.data[4] << 8u) | frame.data[5];
            lower_com_data.twist.twist.angular.z = uint2float(data, MIN_GYRO, MAX_GYRO, 16);

            lower_com_data.header.stamp = ros::Time::now();
            lowercom_data_pub.publish(lower_com_data);

        }

        if(frame.can_id == ID_QUAT)//Quat of IMU
        {
            data = (frame.data[0] << 8u) | frame.data[1];
            lower_com_data.pose.pose.orientation.x = uint2float(data, MIN_QUAT, MAX_QUAT, 16);
            data = (frame.data[2] << 8u) | frame.data[3];
            lower_com_data.pose.pose.orientation.y = uint2float(data, MIN_QUAT, MAX_QUAT, 16);
            data = (frame.data[4] << 8u) | frame.data[5];
            lower_com_data.pose.pose.orientation.z = uint2float(data, MIN_QUAT, MAX_QUAT, 16);
            data = (frame.data[6] << 8u) | frame.data[7];
            lower_com_data.pose.pose.orientation.w = uint2float(data, MIN_QUAT, MAX_QUAT, 16);

            lower_com_data.header.stamp = ros::Time::now();
            lowercom_data_pub.publish(lower_com_data);

        }

        if(frame.can_id == ID_YAW)
        {
            data = (frame.data[0] << 8u) | frame.data[1];
            yaw = uint2float(data, MIN_YAW, MAX_YAW, 16);
            // ROS_INFO("YAW %.2lf",yaw);
            tf_yaw2chassis.sendTransform(
                        tf::StampedTransform(
                                tf::Transform(tf::createQuaternionFromRPY(0.0, 0.0, yaw), tf::Vector3(0, 0, 0)),
                                ros::Time::now(),"base_link","chassis_link"));
            
        }
    }

} // namespace : sentry_communicator
