#ifndef CREATE_COMM_H
#define CREATE_COMM_H

#include "create_headers.h"
#include "create_config.h"

/*define sensors*/
#define BUMP_DROP 7 //1
#define L_CLIFF 9   //1
#define FL_CLIFF 10 //1
#define FR_CLIFF 11 //1
#define R_CLIFF 12  //1
#define DISTANCE 19
#define L_ENCODER 43 //2
#define R_ENCODER 44 //2
#define L_LIGHT 46   //2
#define FL_LIGHT 47  //2  
#define CL_LIGHT 48  //2
#define CR_LIGHT 49  //2
#define FR_LIGHT 50  //2  
#define R_LIGHT 51   //2
#define CHARGE 25    //2

#define PI 3.141592653589793238463
#define WHEEL_DIAMETER 72 //mm
#define RESOLUTION 508.8 // counts/rev
#define BASE_LENGTH 235.0 //mm

class CreateConfig;
class CreateComm{

    public: 
    
        CreateConfig* create_ptr;
        CreateComm();
        void init(CreateConfig *config);
        
        //for reading from create
        int byteRead(uint8_t byte, ros::Publisher sensor_state);
        int dataStream();
        void getCommandVel(const std_msgs::Int32MultiArray::ConstPtr& wheel_vel);
        void getPackets(const ros::TimerEvent&);
        //void getMeasuredVel(const ir_odom::VelocityM::ConstPtr& v_meas);
        
        //for writing to create
        void driveRobot();
        void writeBytes(const std_msgs::Int32MultiArray::ConstPtr& byte);
        int direct_drive(float right_v, float left_v);
    
    
    private:
        ros::NodeHandle nh;

		ros::Publisher bump_drop_state; 
		ros::Publisher l_cliff_state;
        ros::Publisher r_cliff_state;
		ros::Publisher fl_cliff_state; 
		ros::Publisher fr_cliff_state;
        ros::Publisher l_light_state;
        ros::Publisher fl_light_state;
		ros::Publisher cl_light_state;
		ros::Publisher cr_light_state;
		ros::Publisher fr_light_state;
		ros::Publisher r_light_state;
		ros::Publisher voltage_level;
		ros::Publisher get_states, encoder_state;
        
        ros::Publisher motor_control;
        ros::Subscriber wheel_vel_sub, vel_m_sub, byte_sub;
        ros::Timer packet_timer;

};

#endif 
