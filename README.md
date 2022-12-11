# Engineer_on_ROS
version 0.2.0

| 时间       | 包名                            | 修改                                           |
| ---------- | ------------------------------- | ---------------------------------------------- |
| 2022/4/12  | chassis_controller              | 实现基类chassis_controller                     |
|            | moveit_config                   | 修复了arm_controller和chassis的耦合问题        |
| 2022/4/15  | chassis_controller              | 增加了里程计，并发布至/tf和/odom               |
|            | moveit_config                   | 使用panda测试了armplanNode的自动规划           |
| 2022/4/19  | sp_common                       | 新增加了RampFilter                             |
|            | chassis_controller              | 修改cmd_vel的处理逻辑                          |
|            | sp_description                  | 工程可在rviz中完整显示                         |
| 2022/4/20  | sp_description                  | 修改了机械臂工程部分细节                       |
|            | manipulator_moveit_config       | add moveit_config                              |
| 2022/4/22  | ira_laser_tools                 | 用于激光雷达合成，从其他仓库搬过来的           |
|            | sp_description && moveit_config | 清理实验型工程冗余代码                         |
| 2022/4/23  | gimbal_controller               | 实现工程桅杆的三个自由度                       |
| 2022/4/24  | sp_description                  | 修复多处导致gazebo崩溃的模型错误               |
|            | arm_control                     | 该包用于实现机械臂规划，已完成关节空间函数编写 |
| 2022/11/03 | sp_hw                           | Parse.cpp                                      |
| 2022/11/14 | sentry_communicator             | 实现                                           |
| 2022/12/11 | moveit_config                   | 更好的机械臂                                   |



提醒：

1.  为了保证更高效的机械臂解算，需要安装解算插件包。

   ```c++
   sudo apt-get install ros-noetic-trac-ik-kinematics-plugin
   ```

   


TODO:

对gimbal_controller的msg进行修改

修改ira_laser_tools包，使之支持realtime_tools

实现nav功能包的导航功能



Reference:
1. git@github.com:iralabdisco/ira_laser_tools.git
2. git@github.com:rm-controls/rm_controllers.git
3. git@github.com:ros-controls/ros2_controllers.git
