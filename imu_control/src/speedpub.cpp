#include <eigen3/Eigen/Dense>
// #include <opencv4/opencv2/opencv.hpp>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<ros/ros.h>
// #include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include "speedpub.h"
#include "Kalman.h"
#include<nav_msgs/Odometry.h>

geometry_msgs::PoseStamped lastPose,currentPose;
geometry_msgs::Twist vel;
geometry_msgs::TwistStamped emptyvel;
geometry_msgs::PoseStamped emptyPose;
double dx,dy,dz,lyaw,lpitch,lroll,cyaw,cpitch,croll;
ros::Publisher* pub_vel;
ros::Time last_time;
ros::Time current_time;


using namespace Eigen;
using namespace std;
Kalman kf;
int ct = 0;

class MovingAverageFilter {
private:
    std::vector<double> buffer;
    size_t windowSize;
    double sum;

public:
    MovingAverageFilter(size_t size) : windowSize(size), sum(0.0) {
        buffer.reserve(windowSize);
    }

    double update(double newValue) {
        if (buffer.size() == windowSize) {
            sum -= buffer.front();
            buffer.erase(buffer.begin());
        }
        buffer.push_back(newValue);
        sum += newValue;

        return sum / buffer.size();
    }
};

MovingAverageFilter filterlz(10);
MovingAverageFilter filterly(10);
MovingAverageFilter filterlx(10);
MovingAverageFilter filteraz(10);
MovingAverageFilter filteray(10);
MovingAverageFilter filterax(10);


void pose_callback(const nav_msgs::Odometry::ConstPtr& msg_p)
{
    if(lastPose.pose == emptyPose.pose){
        lastPose.pose = msg_p->pose.pose;
        last_time = ros::Time::now();
    }
    else{
        if(currentPose.pose != emptyPose.pose)
        {
        lastPose.pose = currentPose.pose;
        last_time = current_time;
        }
        currentPose.pose =  msg_p->pose.pose;
        double lw = lastPose.pose.orientation.w ;
        double lx = lastPose.pose.orientation.x ;
        double ly = lastPose.pose.orientation.y ;
        double lz = lastPose.pose.orientation.z ;
        double cw = currentPose.pose.orientation.w ;
        double cx = currentPose.pose.orientation.x ;
        double cy = currentPose.pose.orientation.y ;
        double cz = currentPose.pose.orientation.z ;
        Eigen::Quaterniond quatl(lw, lx, ly, lz),quatc(cw, cx, cy, cz);
        Eigen::Matrix3d current_matrix = quatc.toRotationMatrix();
        Eigen::Matrix3d last_matrix = quatl.toRotationMatrix();
        current_time = ros::Time::now();
        ros::Duration duration = current_time - last_time;
        double secs = duration.toSec();
        ROS_INFO_STREAM(secs);


        Eigen::Matrix3d vel_matrix = current_matrix.inverse()*(current_matrix - last_matrix) /secs;
        double wx = (vel_matrix(2, 1) -  vel_matrix(1, 2)) / 2;
        double wy = (vel_matrix(0, 2) -  vel_matrix(2, 0)) / 2;
        double wz = (vel_matrix(1, 0) -  vel_matrix(0, 1)) / 2;
        vel.angular.x = wx;
        vel.angular.y = wy;
        vel.angular.z = wz;

        dx = currentPose.pose.position.x - lastPose.pose.position.x;
        dy = currentPose.pose.position.y - lastPose.pose.position.y;
        dz = currentPose.pose.position.z - lastPose.pose.position.z;
        vel.linear.y = -dx/secs;
        vel.linear.x = -dy/secs;
        vel.linear.z = -dz/secs;
        last_time = current_time;

        MatrixXf velmat = MatrixXf::Zero(6, 1);
        velmat <<vel.linear.x,
                vel.linear.y,
                vel.linear.z,
                vel.angular.x,
                vel.angular.y,
                vel.angular.z;
        // bool isApprox = velmat.isApprox(MatrixXf::Zero(6, 1),0.05);
        //isApprox() 函数会逐元素比较两个矩阵，如果它们的元素之间的差异在给定的误差范围内，则返回 false，否则返回 true

        if(kf.stateOpt.isApprox(MatrixXf::Zero(12, 1)))
            {
               kf.stateOpt << vel.linear.x,
                       vel.linear.y,
                       vel.linear.z,
                       vel.angular.x,
                       vel.angular.y,
                       vel.angular.z,
                       0,0,0,0,0,0;
            }
            else
            {
             kf.predict();
             MatrixXf measurement(6,1); 
             measurement << vel.linear.x,
                       vel.linear.y,
                       vel.linear.z,
                       vel.angular.x,
                       vel.angular.y,
                       vel.angular.z;
            kf.correct(measurement);

            vel.linear.x = std::abs(kf.stateOpt(0)) >= 0.07 ? kf.stateOpt(0) : 0;
            vel.linear.z = std::abs(kf.stateOpt(1)) >= 0.07 ? kf.stateOpt(1) : 0;
            vel.linear.y = std::abs(kf.stateOpt(2)) >= 0.07 ? kf.stateOpt(2) : 0;
            vel.angular.y = std::abs(kf.stateOpt(3)) >= 0.20 ? -kf.stateOpt(3) : 0;
            vel.angular.z = std::abs(kf.stateOpt(4)) >= 0.20 ? kf.stateOpt(4) : 0;
            vel.angular.x = std::abs(kf.stateOpt(5)) >= 0.20 ? kf.stateOpt(5) : 0;

            if (msg_p->twist.twist!= emptyvel.twist)
            {
                vel = msg_p->twist.twist;
                vel.linear.z = std::abs(msg_p->twist.twist.linear.z) >= 0.08 ? msg_p->twist.twist.linear.z : 0;
                vel.linear.x = std::abs(msg_p->twist.twist.linear.x) >= 0.08 ? msg_p->twist.twist.linear.x : 0;
                vel.linear.y = std::abs(msg_p->twist.twist.linear.y) >= 0.08 ? msg_p->twist.twist.linear.y : 0;
                vel.angular.x = std::abs(msg_p->twist.twist.angular.x) >= 0.10 ? msg_p->twist.twist.angular.x : 0;
                vel.angular.y = std::abs(msg_p->twist.twist.angular.y) >= 0.10 ? msg_p->twist.twist.angular.y : 0;
                vel.angular.z = std::abs(msg_p->twist.twist.angular.z) >= 0.10 ? msg_p->twist.twist.angular.z : 0;
            }

            vel.linear.z = vel.linear.z*linearz;
            vel.linear.x = vel.linear.x*linearx;
            vel.linear.y = vel.linear.y*lineary;
            vel.angular.x =  vel.angular.x*angularx;
            vel.angular.y = - vel.angular.y*angulary;
            vel.angular.z = - vel.angular.z*angularz;

            
            vel.linear.z = filterlz.update(vel.linear.z);
            vel.linear.y = filterly.update(vel.linear.y);
            vel.linear.x = filterlx.update(vel.linear.x);
            vel.angular.x = filterax.update(vel.angular.x);
            vel.angular.y = filteray.update(vel.angular.y);
            vel.angular.z = filteraz.update(vel.angular.z);


            ROS_INFO_STREAM(vel);
            pub_vel->publish(vel);
            }
        
        
     }
}

int main(int argc, char  *argv[])
{   
	
    //KF 初始化
	kf.init(12,6);
	//A
	double t = 1/15,q = 4,q0 = 1.1 ,q1 = 0.85, q2 = 98;
    kf.transMat << 1, 0, 0, 0, 0, 0, t, 0, 0, 0, 0, 0,
				   0, 1, 0, 0, 0, 0, 0, t, 0, 0, 0, 0,
				   0, 0, 1, 0, 0, 0, 0, 0, t, 0, 0, 0,
				   0, 0, 0, 1, 0, 0, 0, 0, 0, t, 0, 0,
		           0, 0, 0, 0, 1, 0, 0, 0, 0, 0, t, 0,
		           0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, t,
		           0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
		           0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
		           0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
		           0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
		           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
		           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
 
 
	//H
	kf.measureMat << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                     0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                     0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                     0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
					 
 
	//Q  
	kf.processNoiseCov << q0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					      0, q0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		                  0, 0, q0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		                  0, 0, 0, q0, 0, 0, 0, 0, 0, 0, 0, 0,
		                  0, 0, 0, 0, q0, 0, 0, 0, 0, 0, 0, 0,
		                  0, 0, 0, 0, 0, q0, 0, 0, 0, 0, 0, 0,
		                  0, 0, 0, 0, 0, 0, q, 0, 0, 0, 0, 0,
		                  0, 0, 0, 0, 0, 0, 0, q, 0, 0, 0, 0,
		                  0, 0, 0, 0, 0, 0, 0, 0, q, 0, 0, 0,
		                  0, 0, 0, 0, 0, 0, 0, 0, 0, q, 0, 0,
		                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, q, 0,
		                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, q;
    
    // kf.processNoiseCov = 1.2*MatrixXf::Identity(12, 12);
 
	kf.processNoiseCov_Slow <<  q1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, q1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, q1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, q1, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, q1, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, q1, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, q1, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, q1, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, q1, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, q1, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, q1, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, q1;
 
	kf.processNoiseCov_Fast <<  q2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, q2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, q2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, q2, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, q2, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, q2, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, q2, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, q2, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, q2, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, q2, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, q2, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, q2;
 
	//R  
	double r = 1.1; //100 ~ 500
	double r_fast = 255;
	double r_slow = 100;
	// kf.measureNosiseCov <<r, 0, 0, 0, 0, 0,
	// 				         0, r, 0, 0, 0, 0,
    //                       0, 0, r, 0, 0, 0,
    //                       0, 0, 0, r, 0, 0,
    //                       0, 0, 0, 0, r, 0,
    //                       0, 0, 0, 0, 0, r;
    kf.measureNosiseCov = r*MatrixXf::Identity(6, 6);
    kf.measureNosiseCov_Slow = r_slow*MatrixXf::Identity(6, 6);
    kf.measureNosiseCov_Fast = r_fast*MatrixXf::Identity(6, 6);
	//P
	// kf.errorCovOpt << 1, 0, 0, 0, 0, 0,
	// 			   	  0, 1, 0, 0, 0, 0,
	// 				  0, 0, 1, 0, 0, 0,
	// 	              0, 0, 0, 1, 0, 0,
	// 	              0, 0, 0, 0, 1, 0,
	// 	              0, 0, 0, 0, 0, 1;
    kf.errorCovOpt = MatrixXf::Identity(12,12);
	//
	kf.eps = 0.01;
	kf.rho = 1;
    
    
    //设置编码
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"speedpub");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;
    // ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("cmd_twist",10);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_twist",10);
    pub_vel = &(pub);
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/orb_slam3/pose",10,&pose_callback);



    //节点不死
    while (ros::ok())
    {
        //使用 stringstream 拼接字符串与编号


        //暂无应用
        ros::spinOnce();
    }


    return 0;
}