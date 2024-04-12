/**********res.h声明全局变量************/
#pragma once
#include<ros/ros.h>
// #include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>

 
const int publish_rate = 15; 
 
extern char g_szBuffer[]; // 环形缓冲区
extern geometry_msgs::PoseStamped lastPose,currentPose;
// extern geometry_msgs::TwistStamped vel;
extern geometry_msgs::Twist vel;
extern ros::Publisher* pub_vel;
float linearx = 0.005;
float lineary = -0.03;
float linearz = 0.005;
float angularx = 0.015;
float angulary = 0.02;
float angularz = 0.01;
