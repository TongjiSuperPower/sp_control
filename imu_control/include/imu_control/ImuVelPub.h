#pragma once
#include<ros/ros.h>
// #include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>

float MAX_DIFFERENCE = 40;

extern geometry_msgs::Twist vel;
extern ros::Publisher* pub_vel;
float linearx = 0.005;
float lineary = -0.03;
float linearz = 0.005;
float angularx = -1 * 0.00005;
float angulary = -1 * 0.00005;
float angularz = -1 * 0.00005;