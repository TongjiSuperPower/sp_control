#include "sentry_communicator/sentry_communicator.hpp"

namespace sentry_communicator
{
    CanBus::CanBus(const std::string &bus_name, int thread_priority, ros::NodeHandle &root_nh)
        : bus_name_(bus_name)
    {
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
        const geometry_msgs::Twist &command_velocity = cmd_struct_.cmd_vel_;
        frame_.data[0] = static_cast<uint8_t>(static_cast<int16_t>(command_velocity.linear.x) >> 8u);
        frame_.data[1] = static_cast<uint8_t>(static_cast<int16_t>(command_velocity.linear.x));
        frame_.data[2] = static_cast<uint8_t>(static_cast<int16_t>(command_velocity.linear.y) >> 8u);
        frame_.data[3] = static_cast<uint8_t>(static_cast<int16_t>(command_velocity.linear.y));
        frame_.data[4] = static_cast<uint8_t>(static_cast<int16_t>(command_velocity.angular.z) >> 8u);
        frame_.data[5] = static_cast<uint8_t>(static_cast<int16_t>(command_velocity.angular.z));

        socket_can_.write(&frame_);
    }

    void CanBus::cmdChassisCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        cmd_struct_.cmd_vel_ = *msg;
        cmd_struct_.stamp_ = ros::Time::now();
        /* Lithesh : I haven't learn the realtime_tools, so the code may cause realtime problem
         Sorry :)
        */
    }

    void CanBus::frameCallback(const can_frame &frame)
    {
        ;
    }

} // namespace : sentry_communicator

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sentry_communicator");
    ros::NodeHandle nh;

    sentry_communicator::CanBus can_bus("can0", 0, nh);
    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok())
    {
        can_bus.write();
        ros::spinOnce();
    }
    return 0;
}
