#include "kinect_proc_node.h"


kinect::kinect(){
    raw_points=nh.subscribe<sensor_msgs::Image>("/camera/depth_registered/image_raw", 30, &kinect::get_depth, this);
    d_pub=nh.advertise<std_msgs::Float32>("/kinect_depth", 30);

}

pcl::PointCloud <pcl::PointXYZRGB> raw_cloud;
cv::Mat PC_display;
cv_bridge::CvImagePtr cv_ptr;

void kinect::get_depth(const sensor_msgs::Image::ConstPtr& pc){

    //display w/ OpenCV
    try{
        cv_ptr = cv_bridge::toCvCopy(pc, sensor_msgs::image_encodings::TYPE_16UC1); //converts to 16 bit greyscale image
    }
    catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }
    //printf("width=%d, height=%d \n", points->width, points->height);
    cv::namedWindow("kinect_view", CV_WINDOW_NORMAL);
    cv::resizeWindow("kinect_view", 740,500);
    cv::imshow("kinect_view", cv_ptr->image); 
    cv::waitKey(3);

    //get depth of a point
    float depth = cv_ptr->image.at<short int>(cv::Point(370,250));
    ROS_INFO("depth at center of screen in meters = %3f", depth/1000);

    //publish to ROS
    std_msgs::Float32 dist;
    dist.data=depth/1000;
    d_pub.publish(dist);

}

int main(int argc, char **argv){   
     
    ros::init(argc, argv, "depth_calcs");
    kinect test;
    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
}