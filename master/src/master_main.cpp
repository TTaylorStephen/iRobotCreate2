#include "create_config.h"

int main(int argc, char **argv){

	const char* create_id="/dev/create";
	ros::init(argc,argv,"serial_query");
	CreateConfig config(create_id); 
	CreateComm comm;

	config.init(&comm);
	comm.init(&config);

//	ros::Rate lr(10);
	while(ros::ok()){
		//create.driveRobot();
//		lr.sleep();
		ros::spinOnce();
	
	}
    return 0;
   	
}

