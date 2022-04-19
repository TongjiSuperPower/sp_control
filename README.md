# Engineer_on_ROS
version 0.0.4

| 时间      | 包名               | 修改                                    |
| --------- | ------------------ | --------------------------------------- |
| 2022/4/12 | chassis_controller | 实现基类chassis_controller              |
|           | moveit_config      | 修复了arm_controller和chassis的耦合问题 |
| 2022/4/15 | chassis_controller | 增加了里程计，并发布至/tf和/odom        |
|           | moveit_config      | 使用panda测试了armplanNode的自动规划    |
| 2022/4/19 | sp_common          | 新增加了RampFilter                      |
|           | chassis_controller | 修改cmd_vel的处理逻辑                   |

TODO:

完成车体模型urdf迁移

完成机械臂新模型moveit包生成

完成规划路径C++迁移

增加激光雷达并实现nav功能包的导航功能
