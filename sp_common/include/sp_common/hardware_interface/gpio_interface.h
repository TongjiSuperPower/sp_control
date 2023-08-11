#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace sp_control
{
enum GpioType
{
  INPUT,
  OUTPUT
};

struct GpioData
{
  std::string name;
  ros::Time stamp;
  GpioType type;
  bool value;
  bool cmd;
};

class GpioStateHandle
{
public:
  GpioStateHandle() = default;
  GpioStateHandle(std::string name, GpioType type, bool* value) : name_(std::move(name)), type_(type), value_(value)
  {
    if (!value)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. value pointer is null.");
  }
  std::string getName() const
  {
    return name_;
  }
  GpioType getType() const
  {
    return type_;
  }
  bool getValue() const
  {
    return *value_;
  }
  void setValue(bool value)
  {
    *value_ = value;
  }

private:
  std::string name_;
  GpioType type_;
  bool* value_ = { nullptr };
  
};

class GpioCommandHandle: public GpioStateHandle
{
public:
  GpioCommandHandle() = default;

  GpioCommandHandle(const GpioStateHandle& gs, bool* cmd) : GpioStateHandle(gs), cmd_(cmd)
  {
    if (!cmd)
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + gs.getName() +
                                                           "'. command pointer is null.");

  }

  void setCommand(bool cmd)
  {
    *cmd_ = cmd;
    setValue(cmd);
  }

  bool getCommand() const
  {  
    return *cmd_;
  }

private:
  bool* cmd_ = { nullptr };
};

class GpioStateInterface
  : public hardware_interface::HardwareResourceManager<GpioStateHandle, hardware_interface::DontClaimResources>
{
};

class GpioCommandInterface
  : public hardware_interface::HardwareResourceManager<GpioCommandHandle, hardware_interface::ClaimResources>
{
};

}  // namespace sp_control
