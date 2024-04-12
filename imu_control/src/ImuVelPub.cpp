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
#include "ImuVelPub.h"
#include "Kalman.h"
#include<geometry_msgs/Twist.h>

geometry_msgs::Twist vel;
geometry_msgs::Twist previous_velocity;
geometry_msgs::Twist empty_vel;

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

// 存储上一帧的速度值

void vel_callback(const geometry_msgs::Twist::ConstPtr& msg_p)
{
    // if(previous_velocity == empty_vel){
    //     last_time = ros::Time::now();
    // }
        vel = *msg_p ;
            // 计算当前帧与上一帧的差异值
        double diff_linear_x = fabs(vel.linear.x - previous_velocity.linear.x);
        double diff_linear_y = fabs(vel.linear.y - previous_velocity.linear.y);
        double diff_linear_z = fabs(vel.linear.z - previous_velocity.linear.z);
        double diff_angular_x = fabs(vel.angular.x - previous_velocity.angular.x);
        double diff_angular_y = fabs(vel.angular.y - previous_velocity.angular.y);
        double diff_angular_z = fabs(vel.angular.z - previous_velocity.angular.z);

        if (diff_linear_x > 0.6)
            // if (std::abs(twist_cmd_[i] - last_twist_cmd_[i]) > VChangeConstraint[i])
            {
                vel.linear.x = previous_velocity.linear.x + (vel.linear.x - previous_velocity.linear.x)/diff_linear_x * MAX_DIFFERENCE;
            }
        if (diff_linear_y > 0.6)
            // if (std::abs(twist_cmd_[i] - last_twist_cmd_[i]) > VChangeConstraint[i])
            {
                vel.linear.y = previous_velocity.linear.y + (vel.linear.y - previous_velocity.linear.y)/diff_linear_y * MAX_DIFFERENCE;
            }
        if (diff_linear_z > 0.6)
            // if (std::abs(twist_cmd_[i] - last_twist_cmd_[i]) > VChangeConstraint[i])
            {
                vel.linear.z = previous_velocity.linear.z + (vel.linear.z - previous_velocity.linear.z)/diff_linear_z * MAX_DIFFERENCE;
            }
        if (diff_angular_x > 0.6)
            // if (std::abs(twist_cmd_[i] - last_twist_cmd_[i]) > VChangeConstraint[i])
            {
                vel.angular.x = previous_velocity.angular.x + (vel.angular.x - previous_velocity.angular.x)/diff_angular_x * MAX_DIFFERENCE;
            }
        if (diff_angular_y > 0.6)
            // if (std::abs(twist_cmd_[i] - last_twist_cmd_[i]) > VChangeConstraint[i])
            {
                vel.angular.y = previous_velocity.angular.y + (vel.angular.y - previous_velocity.angular.y)/diff_angular_y * MAX_DIFFERENCE;
            }
        if (diff_angular_z > 0.6)
            // if (std::abs(twist_cmd_[i] - last_twist_cmd_[i]) > VChangeConstraint[i])
            {
                vel.angular.z = previous_velocity.angular.z + (vel.angular.z - previous_velocity.angular.z)/diff_angular_z * MAX_DIFFERENCE;
            }



        // 存储当前帧的速度值作为上一帧的速度值
        previous_velocity = vel;
        
       
        // // bool isApprox = velmat.isApprox(MatrixXf::Zero(6, 1),0.05);
        // //isApprox() 函数会逐元素比较两个矩阵，如果它们的元素之间的差异在给定的误差范围内，则返回 false，否则返回 true

        // if(kf.stateOpt.isApprox(MatrixXf::Zero(12, 1)))
        //     {
        //        kf.stateOpt << vel.linear.x,
        //                vel.linear.y,
        //                vel.linear.z,
        //                vel.angular.x,
        //                vel.angular.y,
        //                vel.angular.z,
        //                0,0,0,0,0,0;
        //     }
        //     else
        //     {
        //      kf.predict();
        //      MatrixXf measurement(6,1); 
        //      measurement << vel.linear.x,
        //                vel.linear.y,
        //                vel.linear.z,
        //                vel.angular.x,
        //                vel.angular.y,
        //                vel.angular.z;
        //     kf.correct(measurement);
            

            // vel.linear.x = std::abs(kf.stateOpt(0)) >= 0.07 ? kf.stateOpt(0) : 0;
            // vel.linear.z = std::abs(kf.stateOpt(1)) >= 0.07 ? kf.stateOpt(1) : 0;
            // vel.linear.y = std::abs(kf.stateOpt(2)) >= 0.07 ? kf.stateOpt(2) : 0;
            // vel.angular.y = std::abs(kf.stateOpt(3)) >= 0.20 ? -kf.stateOpt(3) : 0;
            // vel.angular.z = std::abs(kf.stateOpt(4)) >= 0.20 ? kf.stateOpt(4) : 0;
            // vel.angular.x = std::abs(kf.stateOpt(5)) >= 0.20 ? kf.stateOpt(5) : 0;


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
            // }
        
        
     
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

    ros::init(argc,argv,"ImuVelPub");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;
    // ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("cmd_twist",10);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_twist",10);
    pub_vel = &(pub);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("cmd_velocity",10,&vel_callback);



    //节点不死
    while (ros::ok())
    {
        //使用 stringstream 拼接字符串与编号


        //暂无应用
        ros::spinOnce();
    }


    return 0;
}