#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <iostream>
#include <math.h>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ir_odom/Odometry.h>
#include <ir_odom/Euler.h>
#include <ir_odom/MPU.h>
#include <geometry_msgs/Pose2D.h>
#include <ir_odom/VelocityM.h>
//parameters for irobot from specs 
#define WHEEL_DIAMETER 72 //mm
#define RESOLUTION 508.8 // counts/rev
#define DT_RADIUS 117.5 //mm
#define WHEEL_BASE 235.0 //mm
#define PI 3.141592653589793238463

namespace estimate{ 
	void getTime();
	
	class Odometry{

		public: 
			Odometry(ros::NodeHandle nh);
			//void get_pose(const ir_odom::Euler::ConstPtr& angle, const ir_odom::Magnet::ConstPtr& field);
			void countRead(const std_msgs::Int32MultiArray::ConstPtr &count);
			void getState();
			void mpuRead(const ir_odom::MPU::ConstPtr &data);
			void publishOdom();
			void getVelocity(const ros::TimerEvent&);
		private: 
			ros::Subscriber imu_sub, count_sub;
			ros::Publisher pose_pub, velocity_m_pub;
			ros::NodeHandle nh_;
			ros::Timer velocity_timer;
	};

}
#endif
