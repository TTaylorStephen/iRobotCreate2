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
	bump_drop_state=nh.advertise<std_msgs::Int32>("/bump",30);
	l_cliff_state=nh.advertise<std_msgs::Int32>("/cliff/left",30);
	r_cliff_state=nh.advertise<std_msgs::Int32>("/cliff/right",30);
	fl_cliff_state=nh.advertise<std_msgs::Int32>("/cliff/front_left",30);
	fr_cliff_state=nh.advertise<std_msgs::Int32>("/cliff/front_right",30);
	l_light_state=nh.advertise<std_msgs::Int32>("/light/left",30);
	fl_light_state=nh.advertise<std_msgs::Int32>("/light/front_left",30);
	cl_light_state=nh.advertise<std_msgs::Int32>("/light/center_left", 30);
	cr_light_state=nh.advertise<std_msgs::Int32>("/light/center_right", 30);
	fr_light_state=nh.advertise<std_msgs::Int32>("/light/front_right", 30);
	r_light_state=nh.advertise<std_msgs::Int32>("/light/right", 30);
	voltage_level=nh.advertise<std_msgs::Int32>("/power_status",30);
	

	//arrays of sensor data
	get_states=nh.advertise<std_msgs::Int32MultiArray>("/query_list",30);
	encoder_state=nh.advertise<std_msgs::Int32MultiArray>("/total_count", 30);
	
	//subscribes to wheel count and drives robot
	wheel_vel_sub=nh.subscribe<std_msgs::Float32MultiArray>("/wheel_vel", 30, &ir_rw::drive_robot, this);

	//timer to request sensor packets every 0.05 seconds
	//packet_timer=nh.createTimer(ros::Duration(0.01),&ir_rw::get_packets,this);

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

	short int mm_r=right_v*1000, mm_l=left_v*1000;
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
int bump_drop, l_cliff, fl_cliff, r_cliff, fr_cliff, l_light, fl_light, cl_light, cr_light, fr_light, r_light, voltage;
int l_count, l_count1, r_count, r_count1,delt_l, delt_l1, delt_r, delt_r1, total_count_l, total_count_r, total_count_l1, total_count_r1, dist_l, dist_r;

//Request a list of sensor packets from robot
int ir_rw::data_stream(){

	//flush stale values from buffer
	tcflush(fd,TCIOFLUSH);
	
	//send bytes to request left and right encoder packets
	uint8_t command[16]={148,14, L_ENCODER,R_ENCODER,BUMP_DROP,L_CLIFF,FL_CLIFF,FR_CLIFF, R_CLIFF, L_LIGHT,FL_LIGHT,CL_LIGHT,CR_LIGHT,FR_LIGHT,R_LIGHT,CHARGE};
	if(write(fd, command, sizeof(command))<0){
		perror("failed to retrieve encoder count packets\n");
	}
	//read response
	uint8_t response[40];//buffer
	if(read(fd,response,sizeof(response))<0){
		perror("failed to read data\n");
		printf("size of response=%lu\n", read(fd,response,sizeof(response)));
		return -1;
	}
	
	//perform checksum of data and read value
	int sum=0, check_sum=0;
	if(response[0]==19){
		for(int i=0; i<sizeof(response); ++i){
			sum+=response[i];
			if(i==39){
				check_sum=sum;
			}
			//view packet as its being received
			ROS_INFO("byte %d=%d", i, response[i]);
		}
		ROS_INFO("masked checksum=%d\n", check_sum & 0xFF);
		if((check_sum & 0xFF)==0){ //1280 & 0xFF=0
		
			/**********left wheel**********/
			//save previous iteration
			l_count1=l_count;
			delt_l1=delt_l;
			//typecast left encoder packets to int for handling (4 bytes total, high byte first on each pair)
			l_count={(int)(response[3]<<8) | (int)(response[4] & 0xFF)}; //left
			//account for rollover
			if(l_count-l_count1<-100){
					l_count1=0;
			}
			//calculate total count from zero
			if(l_count1!=0){
				delt_l=l_count-l_count1;
			}
			//save two counts: one for cummulative(total) and one to reset everytime the wheel stops(dist)
			total_count_l+=delt_l;
			dist_l+=delt_l;
			
			/**********right wheel**********/
			//save previous iteration
			r_count1=r_count;
			delt_r1=delt_r;
			
			//typecast right encoder packets to int for handling (4 bytes total, high byte first on each pair)	
			r_count={(int)(response[6]<<8) | (int)(response[7] & 0xFF)}; //right
			//account for rollover
			if(r_count-r_count1<-100){
				r_count1=0;
			}
			//calculate total count from zero
			if(r_count1!=0){
				delt_r=r_count-r_count1;
			}
			//save two counts: one for cummulative and one to reset everytime the wheel stops
			total_count_r+=delt_r;
			dist_r+=delt_r;	
			
			for(int i=0; i<10; i++){
				if (delt_l==0){
					dist_l=0; 
				}
				if(delt_r==0){
					dist_r=0;
				}
			}
			
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
			bump_drop=response[9];
			l_cliff=response[11];
			fl_cliff=response[13];
			r_cliff=response[15];
			fr_cliff=response[17];
			l_light={(int)(response[19]<<8) | (int)(response[20] & 0xFF)};//light sensors 2 bytes, high byte first
			fl_light={(int)(response[22]<<8) | (int)(response[23] & 0xFF)};
			cl_light={(int)(response[25]<<8) | (int)(response[26] & 0xFF)};
			cr_light={(int)(response[28]<<8) | (int)(response[29] & 0xFF)};
			fr_light={(int)(response[31]<<8) | (int)(response[32] & 0xFF)};
			r_light={(int)(response[34]<<8) | (int)(response[35] & 0xFF)};
			voltage={(int)(response[37]<<8) | (int)(response[38] & 0xFF)};	
		

			//store sensor packets ros format
			std_msgs::Int32 bump_sensor, l_cliff_sensor, fl_cliff_sensor, r_cliff_sensor, fr_cliff_sensor, l_light_sensor, fl_light_sensor, cl_light_sensor, cr_light_sensor, fr_light_sensor, r_light_sensor, volts;
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
			volts.data=voltage;

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
			voltage_level.publish(volts);

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
			query_list[9]=fl_light;
			query_list[10]=l_light;

			std_msgs::Int32MultiArray list;
			list.data=query_list;
			get_states.publish(list);
			
			ROS_INFO("total_encoder left encoder count= %d", total_count_l);
			ROS_INFO("total_encoder right encoder count= %d", total_count_r);
			ROS_INFO("delta left encoder= %d", delt_l);
			ROS_INFO("delta right encoder= %d", delt_r);
			ROS_INFO("left increment=%d, total left=%d\n", dist_l, total_count_l);
			ROS_INFO("total left count=%d, total right count=%d\n", l_count, r_count);	
		}
		ROS_INFO("checksum=%d\n", check_sum);	
	}	
	
	//flush input and output buffer to ensure everything has been read/written
	tcflush(fd, TCIOFLUSH);
	return sum;
}

//object for initializing robot with pre-defined commands
ir_setup send_command;

//call back to send left and right wheel velocities to robot from move_base_node
float vel_l, vi_l, vel_r, vi_r;
void ir_rw::drive_robot(const std_msgs::Float32MultiArray::ConstPtr& wheel_vel){
	
	//save previous iteration
	vi_l=vel_l;
	vi_r=vel_r;
	
	//save published velocity
	vel_l=wheel_vel->data[0];
	vel_r=wheel_vel->data[1];
	//ROS_INFO("subscribed left wheel velocity=%f\n",vel_l);
	
	//send velocity once
	if(vi_l != vel_l | vi_r != vel_r){
		send_command.direct_drive(vel_l,vel_r);
		printf("velocity l:%3f, velocity r:%3f\n", vel_l, vel_r);
	}
}

//request list of packets every ros::Duration(N)
void ir_rw::get_packets(){
	data_stream();
	usleep(50000);
}

int main(int argc, char **argv){


	ros::init(argc,argv,"serial_query");
	ir_rw read_robot; //object for reading data packets

	//call subscribers equivalent of ros::spin()
	while(ros::ok()){
		read_robot.get_packets();
		ros::spinOnce();
	}
    return 0;
}

