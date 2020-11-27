#ifndef ir_master_node_h
#define ir_master_node_h

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h> 
#include <iostream>
#include <string.h>
#include "std_msgs/Float32MultiArray.h"
#include "ros/ros.h"
#include <sys/ioctl.h>

#define BAUD_RATE B115200
int fd; 

//class for setting up serial communication and sending pre-defined create byte commands
class ir_setup{

    public: 

        int init(const char* device);
        int start_io();
        int stop_io();
        int off();
        int reset();
        int led_set(uint8_t bit, uint8_t color, uint8_t intensity);
        int mode_set(int mode); //1=safe, 2=full, 3=passive 
        int direct_drive(float right_v, float left_v);
        ir_setup();
        ~ir_setup();
};

#endif 
