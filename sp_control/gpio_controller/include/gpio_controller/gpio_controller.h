//
// Created by myx on 2022/4/29.
//

#pragma once

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <sp_common/hardware_interface/gpio_interface.h>
#include <sp_common/GpioData.h>

namespace gpio_controller
{
  class GpioController : public controller_interface::MultiInterfaceController<sp_control::GpioStateInterface,
                                                                           sp_control::GpioCommandInterface>
  {
  public:
    GpioController() = default;

    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

  private:
    void setGpioCmd(const sp_common::GpioDataConstPtr &msg);

    sp_control::GpioStateHandle state_handle_;
    sp_control::GpioCommandHandle command_handle_;

    ros::Subscriber gpio_command_sub_;
    typedef std::shared_ptr<realtime_tools::RealtimePublisher<sp_common::GpioData>> RtpublisherPtr;
    RtpublisherPtr gpio_state_pub_;
    
  };
} // namespace gpio_controller
