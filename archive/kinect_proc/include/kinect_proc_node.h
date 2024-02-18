#ifndef kinect_proc_node_h
#define kinect_proc_node_h

// Include the ROS library
#include "ros/ros.h"
#include "std_msgs/Float32.h"

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Include Pointcloud libraries
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class kinect{
    public: 
        kinect();
        void get_depth(const sensor_msgs::Image::ConstPtr& pc);
    
    private:

        ros::NodeHandle nh;
        ros::Subscriber raw_points;
        ros::Publisher d_pub;

};

 #endif