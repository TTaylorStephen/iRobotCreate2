#ifndef move_base_h
#define move_base_h


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

#define PI 3.141592653589793238463
#define WHEEL_DIAMETER 72 //mm
#define RESOLUTION 508.8 // counts/rev
#define DT_RADIUS 117.5 //mm

class move_base{

    public: 
        move_base();
        void encoder_vel(const std_msgs::Float32MultiArray::ConstPtr& odom_array);
        void query_read(const std_msgs::Int32MultiArray::ConstPtr& query_array);
        void imu_read(const geometry_msgs::Accel::ConstPtr& _accel);
        double pub_vel(double vl_cmd, double vr_cmd);
        void vel_plan(const ros::TimerEvent&);
        void count_read(const std_msgs::Int32MultiArray::ConstPtr& count);
        void imu_read(const std_msgs::Float32MultiArray::ConstPtr& state);

    private:
        ros::NodeHandle nh_;
        ros::Publisher cmd_vel_pub;
        ros::Subscriber count_sub;
        ros::Subscriber query_sub;
        ros::Subscriber imu_sub;

};

#endif

