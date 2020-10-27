#ifndef ir_rw_h
#define ir_rw_h

#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/UInt8.h"

#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*define sensors*/
#define BUMP_DROP 7
#define L_CLIFF 9
#define FL_CLIFF 10
#define FR_CLIFF 11
#define R_CLIFF 12
#define DISTANCE 19
#define L_ENCODER 43
#define R_ENCODER 44
#define L_LIGHT 46
#define FL_LIGHT 47
#define CL_LIGHT 48
#define CR_LIGHT 49
#define FR_LIGHT 50
#define R_LIGHT 51

#define PI 3.141592653589793238463
#define WHEEL_DIAMETER 72 //mm
#define RESOLUTION 508.8 // counts/rev
#define BASE_LENGTH 235.0 //mm


class ir_rw{

    public: 
        ir_rw();
        int byte_read(uint8_t byte, ros::Publisher sensor_state);
       // int read_packets();
        int data_query();
        void drive_robot(const std_msgs::Float32MultiArray::ConstPtr& wheel_vel);
        void get_count(const ros::TimerEvent&); 
        void get_packets(const ros::TimerEvent&);

    private:
        ros::NodeHandle nh;

		ros::Publisher bump_drop_state; 
		ros::Publisher l_cliff_state;
        ros::Publisher r_cliff_state;
		ros::Publisher fl_cliff_state; 
		ros::Publisher fr_cliff_state;
		ros::Publisher encoder_state;

        ros::Publisher l_light_state;
        ros::Publisher fl_light_state;
		ros::Publisher cl_light_state;
		ros::Publisher cr_light_state;
		ros::Publisher fr_light_state;
		ros::Publisher r_light_state;

        ros::Publisher get_states;

        ros::Subscriber wheel_vel_sub;
        ros::Timer count_timer;
        ros::Timer packet_timer;

};

#endif 
