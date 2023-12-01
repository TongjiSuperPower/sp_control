#include "test_controller.h"
#include <pluginlib/class_list_macros.hpp>
// #include <sp_common/base_utilities.h>

namespace test_controller
{
  bool TestController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {

    // Get joint name from parameter server
    std::string joint_name;
    if (!root_nh.getParam("joint1", joint_name))
    {
      ROS_ERROR("No joint given (namespace: %s)", root_nh.getNamespace().c_str());
      return false;
    }
    ros::NodeHandle nh_test = ros::NodeHandle(controller_nh, "test_joint");
    // ros::NodeHandle nh_right = ros::NodeHandle(controller_nh, "right_joint");
    sub_ = nh_test.subscribe<std_msgs::Float64>("cmd", 1, &TestController::setCommandCB, this);

    effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (!ctrl_test.init(effort_joint_interface_, nh_test))
        return false;
    ROS_INFO("Initializing Completed");
    // Get joint handle from hardware interface
    // joint_ = robot_hw->getHandle(joint1);

    
    return true;
  }

  void TestController::update(const ros::Time &time, const ros::Duration &period)
  { 
    ROS_INFO_STREAM("LEFT_POS: " << ctrl_test.joint_.getPosition());

    // ROS_INFO_STREAM(ctrl_test.joint_);
    ctrl_test.setCommand(cmd_);
    ctrl_test.update(time, period);
  }

  double TestController::getPosition()
  {
    return pos_;
  }

  void TestController::setCommand(double cmd)
  {
    ROS_INFO_STREAM(cmd);
    cmd_ = cmd;
  }

  void TestController::setCommandCB(const std_msgs::Float64::ConstPtr &msg)
  {
    setCommand(msg->data);
  }

} // namespace syn_controller

PLUGINLIB_EXPORT_CLASS(test_controller::TestController, controller_interface::ControllerBase)//?