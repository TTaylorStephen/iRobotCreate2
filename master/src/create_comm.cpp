#include "create_comm.h"

CreateComm::CreateComm(){
}


void CreateComm::init(CreateConfig* config){
	create_ptr=config;
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
	///	wheel_vel_sub=nh.subscribe<std_msgs::Int32MultiArray>("/wheel_velocity/commanded", 30, &CreateComm::getCommandVel, this);
	//vel_m_sub = nh.subscribe<ir_odom::VelocityM>("wheel_velocity/measured", 30, &ir_rw::getMeasuredVel, this);
	//	byte_sub=nh.subscribe<std_msgs::Int32MultiArray>("/bytes", 10, &CreateComm::writeBytes, this);
	//timer to request sensor packets every 0.02s or 50 times a second, daa is avaiable from stream every 15ms ~66x/sec
	packet_timer=nh.createTimer(ros::Duration(0.017),&CreateComm::getPackets,this);
}


/*.........................request sensor packets from robot and publish to ros...........................*/
double now=0, last_update=0, dt=0;
bool start=true;

int CreateComm::dataStream(){ //Request a list of sensor packets from robot
	
	int bump_drop, l_cliff, fl_cliff, r_cliff, fr_cliff, l_light, fl_light, cl_light, cr_light, fr_light, r_light, voltage;
	int l_count=0,  r_count=0;
	
	//intialize packet stream on first iteration
	if(start==true){
		tcflush(create_ptr->fd,TCIOFLUSH);
		//send bytes to request left and right encoder packets
		uint8_t command[16]={148,14, L_ENCODER,R_ENCODER,BUMP_DROP,L_CLIFF,FL_CLIFF,FR_CLIFF, R_CLIFF, L_LIGHT,FL_LIGHT,CL_LIGHT,CR_LIGHT,FR_LIGHT,R_LIGHT,CHARGE};
		if(write(create_ptr->fd, command, sizeof(command))<0){
			perror("failed to retrieve packets\n");
		}
		start=false;
	}
	
	//read response (irobot update rate @ ~0.015s)
	uint8_t response[40];//buffer
	if(read(create_ptr->fd,response,sizeof(response))<0){
		perror("failed to read data\n");
		printf("size of response=%lu\n", read(create_ptr->fd,response,sizeof(response)));
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
	tcflush(create_ptr->fd,TCIOFLUSH);
	return 0;
}

void CreateComm::getPackets(const ros::TimerEvent&){ 	//request initial stream and read @ intervals set by timer
	dataStream();		
}

/*==================================Writing Functions==================================*/

//subcribe to byte stream and write any commands that are send
void CreateComm::writeBytes(const std_msgs::Int32MultiArray::ConstPtr& byte){

	int array_size=byte->data.size();
	uint8_t command[array_size];
	for(int i=0; i<array_size;i++){
		command[i]=byte->data[i];
		ROS_INFO("bytes to be written=%d\n",command[i]);
	}
	if(write(create_ptr->fd, command, sizeof(command))<0){
		perror("failed to retrieve encoder count packets\n");
	}
}


int CreateComm::direct_drive(float left_v, float right_v){	//drive robot using given cmd velocity

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
	if(write(create_ptr->fd,command,sizeof(command))<0){
		perror("failed to drive robot");
		return -1;
	}

	return 0;
}

/*............................read cmd vel from publisher and send to robot.............................*/ 
int vleft=0, vright=0, vleft1=0, vright1=0; 
void CreateComm::getCommandVel(const std_msgs::Int32MultiArray::ConstPtr& vc){	//read data when available
	vleft=vc->data[0];
	vright=vc->data[1];	
}

void CreateComm::driveRobot(){ 	//send velocity if new velocity has arrived, else keep using previous 
	if(vleft1!=vleft){
		direct_drive(vleft,vright);
		vleft1=vleft, vright1=vright;
	}
}