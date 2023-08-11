//
// Created by CherryBlossomNight on 2023/8/4.
//

#include "gpio_controller/gpio_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace gpio_controller
{
  bool GpioController::init(sp_control::GpioCommandInterface *robot, ros::NodeHandle &nh)
  {
    std::string gpioName;
    if (!nh.getParam("gpio", gpioName))
    {
     
      ROS_ERROR("No gpio given (namespace: %s)", nh.getNamespace().c_str());
      return false;
    }
   
    
    
    command_handle_ = robot->getHandle(gpioName);
 
    gpio_state_pub_.reset(new realtime_tools::RealtimePublisher<sp_common::GpioData>(nh, "gpio_state", 100));
    gpio_state_pub_->msg_.gpio_name = command_handle_.getName();
    gpio_state_pub_->msg_.gpio_state = command_handle_.getValue();
    
    if (command_handle_.getType() == sp_control::OUTPUT)
    {
      gpio_state_pub_->msg_.gpio_type = "out";
      gpio_command_sub_ = nh.subscribe<sp_common::GpioData>("gpio_command", 1, &GpioController::setCommandCB, this);
    }
    else
    {
      gpio_state_pub_->msg_.gpio_type = "in";
    }
    ROS_INFO("Got gpio %s", gpioName.c_str());

    
    return true;
  }

  void GpioController::update(const ros::Time &time, const ros::Duration &period)
  {
    
    if (gpio_state_pub_->trylock())
    {
      gpio_state_pub_->msg_.gpio_state = command_handle_.getValue();  
      gpio_state_pub_->msg_.header.stamp = time;
      gpio_state_pub_->unlockAndPublish();
    }
  }

  void GpioController::setCommand(bool cmd)
  {
    command_handle_.setCommand(cmd);
  }

  void GpioController::setCommandCB(const sp_common::GpioDataConstPtr &msg)
  {
    if (msg->gpio_name == command_handle_.getName())
    {
      setCommand(msg->gpio_state);
    }  
    return;
  }

  bool GpioController::getState()
  {
    return command_handle_.getValue();
  }

} // namespace gpio_controller

PLUGINLIB_EXPORT_CLASS(gpio_controller::GpioController, controller_interface::ControllerBase)