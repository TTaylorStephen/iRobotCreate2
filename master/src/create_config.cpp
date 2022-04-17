#include "create_config.h"

//define device to be opened
const char *create = "/dev/create";
int count = 0;

CreateConfig::CreateConfig(){
    configure(create);
}

CreateConfig::~CreateConfig(){
    usleep(1000000);
    c_ptr->direct_drive(0,0); //stop motion
    usleep(1000000);
    stop_io(); //stop communication
    usleep(1000000);
    off(); //turn off
	//reset();
    close(fd); // close port
}

void CreateConfig::init(CreateComm* comm){
	c_ptr=comm;
	start_io();
    usleep(1000000);
    mode_set(2);
    usleep(1000000);
    led_set(0,100,200);//2^0=dirt detect, 2^1=spot, 2^2=dock, 2^3=check robot
    usleep(1000000);
}

int CreateConfig::configure(const char *device){

    //open device in read/write mode, unable to control terminal, and set to return without delay
	if(0 == fd){
		fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
		if(fd<0){
			perror("failed to open connection with create\n");
				return -1;
		}
		printf("port opened with status %d\n",fd);
		fcntl(fd,F_SETFL,0); //sets file flags on fd (O_NDELAY)
		tcflush(fd,TCIOFLUSH);//flush buffer
    }

    //define structure for port settings apply attributes defined in open command
    struct termios settings;

    if(tcgetattr(fd, &settings)<0){
        perror("error getting port settings\n");
        return -1;
    }
    	// or forces values to 1; and forces all values to 0 (off);
	settings.c_cflag &= ~(PARENB);//no parity
	settings.c_cflag &= ~(CSTOPB);
	settings.c_cflag &= ~CSIZE; //mask the character bits
	settings.c_cflag |= (CS8); //8 bit character size
    cfmakeraw(&settings); //sets remaining flags for raw mode, input available character by character

    //set baud rate of 115200
    if(cfsetispeed(&settings,BAUD_RATE)!=0){
        perror("error setting input baud rate\n");
        return -1;
    }
    if(cfsetospeed(&settings,BAUD_RATE)!=0){
        perror("error setting output baud rate\n");
        return -1;
    }

    //final definition of port settings
    if(tcsetattr(fd, TCSANOW, &settings)!=0){
        perror("error finalizing port settings\n");
        return -1;
    }

    return fd;
}

//open IO for communication; also sets to passive mode
int CreateConfig::start_io(){
    uint8_t command[1]={128};
    if(write(fd, command, sizeof(command))<0){
        perror("could not start open interface\n");
        return -1;
    }
    return 0;
}

//closes IO communication
int CreateConfig::stop_io(){
	uint8_t command[2]={128, 173};
	if(write(fd, command, sizeof(command))<0){
		perror("robot failed to close I/O ");
		return -1;
	}
    printf("\nClosing I/O \n");
	return 0;
}

//power off robot
int CreateConfig::off(){
	uint8_t command[2]={128,133};
	if(write(fd, command, sizeof(command))<0){
		perror("robot failed to power off ");
		return -1;
	}
	printf("powering robot off...\n");
	return 0;
}

//reset robot as if battery was taken out
int CreateConfig::reset(){
	uint8_t command[2]={128,7};
	if(write(fd, command, sizeof(command))<0){
		perror("failed to reset robot...");
		return -1;
	}
	return 0;
}

//set operating mode to either safe, full, or passive
int CreateConfig::mode_set(int mode){

	 //safe mode
	if(mode==1){
		uint8_t command[1]={131};
		if (write(fd,command, sizeof(command))<0){
			perror("mode set failed");
			return -1;
		}
		printf("robot mode set to safe \n");
	}

	//full mode
	else if(mode==2){
		uint8_t command[1]={132};
		if (write(fd,command, sizeof(command))<0){
	 		perror("mode set failed");
	 		return -1;
	 	}
		printf("robot mode set to full \n");
	}

	//passive mode
	else if(mode==3){
		uint8_t command[1]={128};//passive
		if (write(fd,command, sizeof(command))<0){
	 		perror("mode set failed");
	 		return -1;
	 	}
		printf("robot mode set to passive \n");
	}

	return 0;
}

//set LEDs to desired color and intensity
int CreateConfig::led_set(uint8_t bit, uint8_t color, uint8_t intensity){
    uint8_t command[4]={139, bit, color, intensity};
    if(write(fd, command, sizeof(command))<0){
        perror("failed to set LED state");
        return -1;
    }
    printf("setting leds to blah blah\n");
    return 0;
}


