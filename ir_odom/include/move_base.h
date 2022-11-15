#ifndef move_base_h
#define move_base_h

#include <iostream>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <stdlib.h>
#include <stdint.h>
#include <boost/bind.hpp>
#include "std_msgs/Header.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Pose2D.h"
#include "math.h"

#define PI 3.141592653589793238463
#define WHEEL_DIAMETER 72 //mm
#define RESOLUTION 508.8 // counts/rev
#define DT_RADIUS 117.5 //mm
#define WHEEL_BASE 235.0 //mm

class move_base{

    public: 
        move_base();
        void pose2DSub(const geometry_msgs::Pose2D::ConstPtr& pose);
        float pub_vel();
		void goToGoal(float x_goal, float y_goal);

    private:
        ros::NodeHandle nh_;
        ros::Publisher cmd_vel_pub;
        ros::Subscriber count_sub;
        ros::Subscriber pose_sub;
        ros::Subscriber imu_sub;

};

#endif

