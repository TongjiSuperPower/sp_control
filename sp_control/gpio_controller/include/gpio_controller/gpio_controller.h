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
  class GpioController : public controller_interface::Controller<sp_control::GpioCommandInterface>
  {
  public:
    GpioController() = default;

    bool init(sp_control::GpioCommandInterface *robot, ros::NodeHandle &nh);

    void update(const ros::Time &time, const ros::Duration &period) override;

    void setCommand(bool value);

    bool getState();


  private:
    void setCommandCB(const sp_common::GpioDataConstPtr &msg);

    sp_control::GpioCommandHandle command_handle_;

    ros::Subscriber gpio_command_sub_;
    typedef std::shared_ptr<realtime_tools::RealtimePublisher<sp_common::GpioData>> RtpublisherPtr;
    RtpublisherPtr gpio_state_pub_;
    
  };
} // namespace gpio_controller
