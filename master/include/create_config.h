#ifndef CREATE_CONFIG_H
#define CREATE_CONFIG_H

#include "create_headers.h"
#include "create_comm.h"


#define BAUD_RATE B115200

class CreateComm;
//class for setting up serial communication and sending pre-defined create byte commands
class CreateConfig{

    public: 
        CreateComm* c_ptr;
        int fd; 
        int configure(const char* device);
        void init(CreateComm* comm);
        int start_io();
        int stop_io();
        int off();
        int reset();
        int led_set(uint8_t bit, uint8_t color, uint8_t intensity);
        int mode_set(int mode); //1=safe, 2=full, 3=passive 
        CreateConfig();
        ~CreateConfig();
};

#endif 
