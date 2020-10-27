#include "ir_setup.h"
#include "ir_rw.h"

//define device to be opened
const char *create = "/dev/i_robot";

ir_setup::ir_setup(){
    init(create);
    start_io();
    usleep(1000000);
    mode_set(2);
    usleep(1000000);
    led_set(0,100,200);//2^0=dirt detect, 2^1=spot, 2^2=dock, 2^3=check robot
    usleep(1000000);

}

ir_setup::~ir_setup(){
    usleep(1000000);
    direct_drive(0,0); //stop motion
    usleep(1000000);
    stop_io(); //stop communication
    usleep(1000000);
    off(); //turn off
	//reset();
    close(fd); // close port
}

ir_rw::ir_rw(){

	//indidual sensors
	bump_drop_state=nh.advertise<std_msgs::Int32>("/bump_state",30);
	l_cliff_state=nh.advertise<std_msgs::Int32>("/l_cliff_state",30);
	r_cliff_state=nh.advertise<std_msgs::Int32>("/r_cliff_state",30);
	fl_cliff_state=nh.advertise<std_msgs::Int32>("/fl_cliff_state",30);
	fr_cliff_state=nh.advertise<std_msgs::Int32>("/fr_cliff_state",30);
	l_light_state=nh.advertise<std_msgs::Int32>("/l_light_state",30);
	fl_light_state=nh.advertise<std_msgs::Int32>("/fl_light_state",30);
	cl_light_state=nh.advertise<std_msgs::Int32>("/cl_light_state", 30);
	cr_light_state=nh.advertise<std_msgs::Int32>("/cr_light_state", 30);
	fr_light_state=nh.advertise<std_msgs::Int32>("/fr_light_state", 30);
	r_light_state=nh.advertise<std_msgs::Int32>("/r_light_state", 30);

	//arrays of sensor data
	get_states=nh.advertise<std_msgs::Int32MultiArray>("/query_list",30);
	encoder_state=nh.advertise<std_msgs::Int32MultiArray>("/total_count", 30);

	//subscribes to wheel count and drives robot
	wheel_vel_sub=nh.subscribe("/wheel_vel", 30, &ir_rw::drive_robot, this);

	//timer to request sensor packets every 0.05 seconds
	packet_timer=nh.createTimer(ros::Duration(0.05),&ir_rw::get_packets,this);

}

int ir_setup::init(const char *device){

    //open device in read/write mode, unable to control terminal, and set to return without delay
	if(0 == fd){
		fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
		if(fd<0){
			perror("failed to open connection with i_robot\n");
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
	settings.c_cflag &= ~(PARENB | CSTOPB);//no parity
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
int ir_setup::start_io(){
    uint8_t command[1]={128};
    if(write(fd, command, sizeof(command))<0){
        perror("could not start open interface\n");
        return -1;
    }
    return 0;
}

//closes IO communication
int ir_setup::stop_io(){
	uint8_t command[2]={128, 173};
	if(write(fd, command, sizeof(command))<0){
		perror("robot failed to close I/O ");
		return -1;
	}
    printf("\nClosing I/O \n");
	return 0;
}

//power off robot
int ir_setup::off(){
	uint8_t command[2]={128,133};
	if(write(fd, command, sizeof(command))<0){
		perror("robot failed to power off ");
		return -1;
	}
	printf("powering robot off...\n");
	return 0;
}

//reset robot as if battery was taken out
int ir_setup::reset(){
	uint8_t command[2]={128,7};
	if(write(fd, command, sizeof(command))<0){
		perror("failed to reset robot...");
		return -1;
	}
	return 0;
}

//set operating mode to either safe, full, or passive
int ir_setup::mode_set(int mode){

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
int ir_setup::led_set(uint8_t bit, uint8_t color, uint8_t intensity){
    uint8_t command[4]={139, bit, color, intensity};
    if(write(fd, command, sizeof(command))<0){
        perror("failed to set LED state");
        return -1;
    }
    printf("setting leds to blah blah\n");
    return 0;
}

int ir_setup::direct_drive(float right_v, float left_v){

	short int mm_r=right_v*1000, mm_l=left_v*1000,i=0;
	int8_t r_byte[2], l_byte[2];

	//mask high bytes shift 8 bites to the right
	r_byte[1]=(int8_t)(mm_r & 0xFF); //low byte
	r_byte[0]=(int8_t)(mm_r >> 8 & 0xFF); //shift by 8 to the right for high byte and mask again
	//printf("%i\n",r_byte[0]); printf("%i\n",r_byte[1]);

	l_byte[1]=(int8_t)(mm_l & 0xFF);
	l_byte[0]=(int8_t)(mm_l >> 8 & 0xFF);
	//printf("%i\n",l_byte[0]); printf("%i\n",l_byte[1]);

	int8_t command[5]={(int8_t)145,r_byte[0],r_byte[1],l_byte[0],l_byte[1]};
	if(write(fd,command,sizeof(command))<0){
		perror("failed to drive robot");
		return -1;
	}

	return 0;
}

double t1, t2, tt, dt;
int bump_drop, l_cliff, fl_cliff, r_cliff, fr_cliff, l_light, fl_light, cl_light, cr_light, fr_light, r_light;
int l_count, l_count1, r_count, r_count1,delt_l, delt_l1, delt_r, delt_r1, total_count_l, total_count_r, total_count_l1, total_count_r1, dist_l, dist_r;

//Request a list of sensor packets from robot
int ir_rw::data_query(){

	t1=t2;
	
	//flush stale values from buffer
	tcflush(fd,TCIOFLUSH);
	//send bytes to request left and right encoder packets
	uint8_t command[15]={149,13,L_ENCODER,R_ENCODER,BUMP_DROP,L_CLIFF,FL_CLIFF,R_CLIFF,FR_CLIFF,L_LIGHT,FL_LIGHT,CL_LIGHT,CR_LIGHT,FR_LIGHT,R_LIGHT};
	if(write(fd, command, sizeof(command))<0){
		perror("failed to retrieve encoder count packets\n");
	}
	uint8_t response[21];//buffer
	if(read(fd,response,sizeof(response))!=21){
		//perror("failed to write data\n");
		return -1;
	}
	
	//typecast left and right encoder packets into int for handling (4 bytes total, high byte first on each pair)
	l_count1=l_count;
	l_count={(int)(response[0]<<8) | (int)(response[1] & 0xFF)}; //left
	//calculate total count from zero
	delt_l1=delt_l;
	if(l_count1>0){
		delt_l=l_count-l_count1;
	}
	//keep counting when encoders reset
	if(l_count1>l_count){
		delt_l=delt_l1;
	}	
	total_count_l+=delt_l;
	dist_l+=delt_l;

	r_count1=r_count;
	r_count={(int)(response[2]<<8) | (int)(response[3] & 0xFF)}; //right
	//calculate total count from zero
	delt_r1=delt_r;
	if(r_count1>0){
		delt_r=r_count-r_count1;
	}
	//keep counting when encoders reset
	if(r_count1>r_count){
		delt_r=delt_r1;
	}
	//two counts: one for cummulative and one to reset everytime the wheel stops
	total_count_r+=delt_r;
	dist_r+=delt_r;

	for(int i; i<10; i++){
		if (l_count==l_count1){
			total_count_l1=0, total_count_l=0; 
		}
		if(r_count==r_count1){
			total_count_r1=0; total_count_r=0;
		}
	}

	ROS_INFO("total_encoder left encoder count= %d", total_count_l);
	ROS_INFO("total_encoder right encoder count= %d", total_count_r);
	ROS_INFO("delta left encoder= %d", delt_l);
	ROS_INFO("delta right encoder= %d", delt_r);
	printf("right encoder count=%d\n, left encoder count=%d\n", r_count,  l_count);
	
	//store in vector for ros
	std::vector<int> wheel_info(4);
	wheel_info[0]=dist_l;
	wheel_info[1]=dist_r;
	wheel_info[2]=total_count_l;
	wheel_info[3]=total_count_r;

	//ros array for publishing
	std_msgs::Int32MultiArray total_count;
	total_count.data=wheel_info;

	//publish count
	encoder_state.publish(total_count);

	//define variable for each sensor packet
	bump_drop=response[4];
	l_cliff=response[5];
	fl_cliff=response[6];
	r_cliff=response[7];
	fr_cliff=response[8];
	l_light={(int)(response[9]<<8) | (int)(response[10] & 0xFF)};//light sensors 2 bytes, high byte first
	fl_light={(int)(response[11]<<8) | (int)(response[12] & 0xFF)};
	cl_light={(int)(response[13]<<8) | (int)(response[14] & 0xFF)};
	cr_light={(int)(response[15]<<8) | (int)(response[16] & 0xFF)};
	fr_light={(int)(response[17]<<8) | (int)(response[18] & 0xFF)};
	r_light={(int)(response[19]<<8) | (int)(response[20] & 0xFF)};

	//store sensor packets ros format
	std_msgs::Int32 bump_sensor, l_cliff_sensor, fl_cliff_sensor, r_cliff_sensor, fr_cliff_sensor, l_light_sensor, fl_light_sensor, cl_light_sensor, cr_light_sensor, fr_light_sensor, r_light_sensor;
	bump_sensor.data=bump_drop;
	l_cliff_sensor.data=l_cliff;
	fl_cliff_sensor.data=fl_cliff;
	r_cliff_sensor.data=r_cliff;
	fr_cliff_sensor.data=fr_cliff;
	l_light_sensor.data=l_light;
	fl_light_sensor.data=fl_light;
	cl_light_sensor.data=cl_light;
	cr_light_sensor.data=cr_light;
	fr_light_sensor.data=fr_light;
	r_light_sensor.data=r_light;

	//publish sensor packets individually
	bump_drop_state.publish(bump_sensor);
	l_cliff_state.publish(l_cliff_sensor);
	fl_cliff_state.publish(fl_cliff_sensor);
	r_cliff_state.publish(r_cliff_sensor);
	fr_cliff_state.publish(fr_cliff_sensor);
	l_light_state.publish(l_light_sensor);
	fl_light_state.publish(fl_light_sensor);
	cl_light_state.publish(cl_light_sensor);
	cr_light_state.publish(cr_light_sensor);
	fr_light_state.publish(fr_light_sensor);
	r_light_state.publish(r_light_sensor);

	//publish sensor packets as array
	std::vector<int> query_list(11);
	query_list[0]=bump_drop;
	query_list[1]=l_cliff;
	query_list[2]=fl_cliff;
	query_list[3]=r_cliff;
	query_list[4]=fr_cliff;
	query_list[5]=r_light;
	query_list[6]=fr_light;
	query_list[7]=cr_light;
	query_list[8]=cl_light;
	query_list[9]=fl_light;
	query_list[10]=l_light;

	std_msgs::Int32MultiArray list;
	list.data=query_list;
	get_states.publish(list);

	t2 = ros::Time::now().toSec();
	if(t1>0){
		dt = t2-t1;
	}
	tt+=dt;

	//flush input and output buffer to ensure everything has been read/written
	tcflush(fd, TCIOFLUSH);

}

//object for initializing robot with pre-defined commands
ir_setup command;
//call back to send left and right wheel velocities to robot from move_base_node
void ir_rw::drive_robot(const std_msgs::Float32MultiArray::ConstPtr& wheel_vel){
	float vel_l=wheel_vel->data[0];
	float vel_r=wheel_vel->data[1];
	//ROS_INFO("subscribed left wheel velocity=%f\n",vel_l);
	command.direct_drive(vel_l,vel_r);
}

//request list of packets every ros::Duration(N)
void ir_rw::get_packets(const ros::TimerEvent&){
	data_query();
}

int main(int argc, char **argv){

	ros::init(argc,argv,"serial_query");
	ir_rw read_robot; //object for reading data packets

	//call subscribers equivalent of ros::spin()
	while(ros::ok()){
		ros::spinOnce();
	}
    return 0;
}

