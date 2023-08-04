//
// Created by CherryBlossomNight on 2023/8/4.
//

#include "gpio_controller/gpio_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace gpio_controller
{
  bool GpioController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh)
  {
    std::string gpioName;
    if (!nh.getParam("gpio", gpioName))
    {
      ROS_ERROR("No gpio given (namespace: %s)", nh.getNamespace().c_str());
      return false;
    }


    // realtime publisher
    //
    state_handle_ = robot_hw->get<sp_control::GpioStateInterface>()->getHandle(gpioName);
    gpio_state_pub_.reset(new realtime_tools::RealtimePublisher<sp_common::GpioData>(nh, "state", 100));
    gpio_state_pub_->msg_.gpio_name = state_handle_.getName();
    gpio_state_pub_->msg_.gpio_state = state_handle_.getValue();
    if (state_handle_.getType() == sp_control::OUTPUT)
    {
      gpio_state_pub_->msg_.gpio_type = "out";
    }
    else
    {
      gpio_state_pub_->msg_.gpio_type = "in";
    }
    ROS_INFO("Got state_gpio %s", gpioName.c_str());
    if (state_handle_.getType() == sp_control::OUTPUT)
    {
      command_handle_ = robot_hw->get<sp_control::GpioCommandInterface>()->getHandle(gpioName);
      ROS_INFO("Got command_gpio %s", gpioName.c_str());
    }
   
    gpio_command_sub_ = nh.subscribe<sp_common::GpioData>("command", 1, &GpioController::setGpioCmd, this);

    
    return true;
  }

  void GpioController::update(const ros::Time &time, const ros::Duration &period)
  {
    
    if (gpio_state_pub_->trylock())
    {
      gpio_state_pub_->msg_.gpio_state = state_handle_.getValue();   
      gpio_state_pub_->msg_.header.stamp = time;
      gpio_state_pub_->unlockAndPublish();
    }
      //ROS_WARN_STREAM("GPIO_PUBLISHER");
  }

  void GpioController::setGpioCmd(const sp_common::GpioDataConstPtr &msg)
  {
    if (msg->gpio_name == command_handle_.getName())
    {
      command_handle_.setCommand(msg->gpio_state);
    }  
    return;
  }

} // namespace gpio_controller

PLUGINLIB_EXPORT_CLASS(gpio_controller::GpioController, controller_interface::ControllerBase)