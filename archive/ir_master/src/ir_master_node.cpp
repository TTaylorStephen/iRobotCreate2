#include "ir_setup.h"
#include "ir_rw.h"

//define device to be opened
const char *create = "/dev/i_robot";
int count = 0;

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
	bump_drop_state=nh.advertise<std_msgs::Int32>("/bump", 10);
	l_cliff_state=nh.advertise<std_msgs::Int32>("/cliff/left", 10);
	r_cliff_state=nh.advertise<std_msgs::Int32>("/cliff/right", 10);
	fl_cliff_state=nh.advertise<std_msgs::Int32>("/cliff/front_left", 10);
	fr_cliff_state=nh.advertise<std_msgs::Int32>("/cliff/front_right", 10);
	l_light_state=nh.advertise<std_msgs::Int32>("/light/left", 10);
	fl_light_state=nh.advertise<std_msgs::Int32>("/light/front_left", 10);
	cl_light_state=nh.advertise<std_msgs::Int32>("/light/center_left", 10);
	cr_light_state=nh.advertise<std_msgs::Int32>("/light/center_right", 10);
	fr_light_state=nh.advertise<std_msgs::Int32>("/light/front_right", 10);
	r_light_state=nh.advertise<std_msgs::Int32>("/light/right", 10);
	voltage_level=nh.advertise<std_msgs::Int32>("/power_status",10);
	
	//arrays of sensor data
	get_states=nh.advertise<std_msgs::Int32MultiArray>("/query_list",10);
	encoder_state=nh.advertise<std_msgs::Float32MultiArray>("/total_count", 10);
	
	//subscribes to wheel count and drives robot
	wheel_vel_sub=nh.subscribe<std_msgs::Int32MultiArray>("/wheel_velocity/commanded", 30, &ir_rw::getCommandVel, this);
	//vel_m_sub = nh.subscribe<ir_odom::VelocityM>("wheel_velocity/measured", 30, &ir_rw::getMeasuredVel, this);
	byte_sub=nh.subscribe<std_msgs::Int32MultiArray>("/bytes", 10, &ir_rw::writeBytes, this);

	//timer to request sensor packets every 0.02s or 50 times a second, daa is avaiable from stream every 15ms ~66x/sec
	packet_timer=nh.createTimer(ros::Duration(0.017),&ir_rw::getPackets,this);

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

int ir_setup::direct_drive(float left_v, float right_v){	//drive robot using given cmd velocity

	short int mm_l=left_v, mm_r=right_v;
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

/*.........................request sensor packets from robot and publish to ros...........................*/

int bump_drop, l_cliff, fl_cliff, r_cliff, fr_cliff, l_light, fl_light, cl_light, cr_light, fr_light, r_light, voltage;
int l_count=0,  r_count=0;
double now=0, last_update=0, dt=0;
bool start=true;

int ir_rw::data_stream(){ //Request a list of sensor packets from robot
	
	//intialize packet stream on first iteration
	if(start==true){
		tcflush(fd,TCIOFLUSH);
		//send bytes to request left and right encoder packets
		uint8_t command[16]={148,14, L_ENCODER,R_ENCODER,BUMP_DROP,L_CLIFF,FL_CLIFF,FR_CLIFF, R_CLIFF, L_LIGHT,FL_LIGHT,CL_LIGHT,CR_LIGHT,FR_LIGHT,R_LIGHT,CHARGE};
		if(write(fd, command, sizeof(command))<0){
			perror("failed to retrieve packets\n");
		}
		start=false;
	}
	
	//read response (irobot update rate @ ~0.015s)
	uint8_t response[40];//buffer
	if(read(fd,response,sizeof(response))<0){
		perror("failed to read data\n");
		printf("size of response=%lu\n", read(fd,response,sizeof(response)));
		return -1;
	}
	
	//perform checksum of data and read value if passes
	int sum=0, check_sum=0;
	if(response[0]==19){
		for(int i=0; i<sizeof(response); ++i){
			sum+=response[i];
			if(i==39){
				check_sum=sum;
			}
		}
		//ROS_INFO("masked checksum=%d\n", check_sum & 0xFF);	
		//if lower byte of checksum=0 read data
		if((check_sum & 0xFF)==0){    
		
			now = ros::Time::now().toSec();
			dt=(now-last_update);
			last_update=now;
			//ROS_INFO("dt=%3f", dt);    
			              
			//typecast left encoder packets to int for handling (4 bytes total, high byte first on each pair)
			l_count={(int)(response[3]<<8) | (int)(response[4] & 0xFF)}; //left
			//typecast right encoder packets to int for handling (4 bytes total, high byte first on each pair)	
			r_count={(int)(response[6]<<8) | (int)(response[7] & 0xFF)}; //right	
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
		    
		    //store in vector for ros
			std::vector<float> wheel_info(3);
			wheel_info[0]=l_count;
			wheel_info[1]=r_count;
			wheel_info[2]=dt;
			
			//ros array for publishing
			std_msgs::Float32MultiArray total_count;
			total_count.data=wheel_info;

			//publish count
			encoder_state.publish(total_count);
			
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
		}	
	}	
	tcflush(fd,TCIOFLUSH);
	return 0;

}

void ir_rw::getPackets(const ros::TimerEvent&){ 	//request initial stream and read @ intervals set by timer
	data_stream();		
}


/*............................read cmd vel from publisher and send to robot.............................*/ 

int vleft=0, vright=0, vleft1=0, vright1=0; 
void ir_rw::getCommandVel(const std_msgs::Int32MultiArray::ConstPtr& vc){	//read data when available
	vleft=vc->data[0];
	vright=vc->data[1];	
}

ir_setup send_command;	//initialize object for sending commands
void ir_rw::driveRobot(){ 	//send velocity if new velocity has arrived, else keep using previous 
	if(vleft1!=vleft){
		send_command.direct_drive(vleft,vright);
		vleft1=vleft, vright1=vright;
	}
}

/*.....................subcribe to byte stream and write any commands that are send......................*/

void ir_rw::writeBytes(const std_msgs::Int32MultiArray::ConstPtr& byte){

	int array_size=byte->data.size();
	uint8_t command[array_size];
	for(int i=0; i<array_size;i++){
		command[i]=byte->data[i];
		ROS_INFO("bytes to be written=%d\n",command[i]);
	}
	if(write(fd, command, sizeof(command))<0){
		perror("failed to retrieve encoder count packets\n");
	}
}



int main(int argc, char **argv){

	ros::init(argc,argv,"serial_query");
	ir_rw create; //object for reading data packets

//	ros::Rate lr(10);
	while(ros::ok()){
		create.driveRobot();
//		lr.sleep();
		ros::spinOnce();
	
	}
    return 0;
   	
}

