#include "move_base.h"


int bump_drop, l_cliff, fl_cliff, r_cliff, fr_cliff, l_light, fl_light, cl_light, cr_light, fr_light, r_light;
double t1, t2;
float dt=0.1; //sampling time
float r_vel=0.3, l_vel=0.3, heading;
float xi=0, yi=0, gi_heading=0;
float total_heading_error=0, delta_error=0;
float v_max=0.2, w_max=30*(PI/180), wheel_offset=0.1175;
float v_left=0, v_right=0;
float h_delt1=0;
move_base::move_base(){
    
    //initialize publisher for wheel velocity
    cmd_vel_pub=nh_.advertise<std_msgs::Float32MultiArray>("/wheel_velocity/commanded",30);
    pose_sub=nh_.subscribe<geometry_msgs::Pose2D>("/2Dpose", 30, &move_base::pose2DSub, this);
//  imu_sub=nh_.subscribe<std_msgs::Float32MultiArray>("/imu_data", 30, &move_base::imu_read, this);

} 

//subscribe to and define all sensor values for use elsewhere
void move_base::pose2DSub(const geometry_msgs::Pose2D::ConstPtr& pose){
	xi=pose->x, yi=pose->y, gi_heading=pose->theta;
	ROS_INFO("x=%3f, y=%3f, yaw=%3f\n", xi, yi, gi_heading);
}

void move_base::goToGoal(float x_goal, float y_goal){
	static float kv_P=0.8;    // constant for proportional velocity control, ramp down when within 0.2 of goal
	static float kw_P=4.25;   //k for proportional control on angular velocity
	static float kw_I=0.1; 	  //k for integral control on angular velocity 
	static float kw_D=0.03;   //k for derivative control on angular velocity
	float wI_limit=20*PI/180; //rad/sec
	
	
	float dist_delt=sqrt(pow(x_goal-xi,2)+pow(y_goal-yi,2)); //desired distance to be travelled
	float h_goal=atan2((y_goal-yi),(x_goal-xi));	//desired heading to face pt B from pt A
	
	/*get desired velocity*/
	float v_desired=kv_P*dist_delt; //keep velocity within max/min range, as distance is reached velocity slows	
	if(v_desired>v_max){	//when 0.8 of distance left is less than 0.3, speed reduces
		v_desired=v_max;
	}
	else if(v_desired<-v_max){
		v_desired=-v_max;
	}
	

	/*if(h_goal<0){
		h_goal=remainder(h_goal+2*PI, 2*PI);
	}*/
	
	//estimate change in heading 
	float h_delt=remainder(h_goal-gi_heading, 2*PI); 
	
	//wrap angle of rotation to PI
	if(h_delt>PI){					
		h_delt-=(2*PI);
	}
	else if(h_delt<-PI){
		h_delt+=(2*PI);
	}
	
	/* get desired angular velocity*/
	//proportional control on heading for angular velocity - current heading remaining
	float wP_desired=kw_P*h_delt; 	
	
	//integral control on total heading error
	total_heading_error+=h_delt;				//sum total heading traveled
	float wI_desired=kw_I*total_heading_error; 

	if(wI_desired>wI_limit){	//keeps integral term within prescribed limit
		wI_desired=wI_limit;	
	}
	else if(wI_desired<-wI_limit){
		wI_desired=-wI_limit;
	}
	
	
	//derivative control on change of heading error
	delta_error=h_delt-h_delt1;
	float wD_desired=kw_D*delta_error;
	h_delt1=h_delt;
	
	
	//combine for PI control
	float w_desired=wP_desired+wI_desired;				
	
	if(w_desired>w_max){				//keep angular rate w/in range 
		w_desired=w_max;
	}
	else if(w_desired<-w_max){
		w_desired=-w_max;
	}
	
	ROS_INFO("change in distance neede=%3f\n",dist_delt);
	ROS_INFO("current heading error=%3f\n", h_delt);
	
	if(dist_delt>0.05){
		if(abs(h_delt)<PI/2){ 
			v_left=v_desired*cos(h_delt)+w_desired*wheel_offset;
			v_right=v_desired*cos(h_delt)-w_desired*wheel_offset;
		}
		else{
			v_left=w_desired*wheel_offset;
			v_right=-w_desired*wheel_offset;
		}
	}
	else{
		h_delt=0, v_left=0, v_right=0;
	}
}

std::vector<float> cmd_vel(2);
float move_base::pub_vel(){
    
    goToGoal(0.5,0.5);
    
    ROS_INFO("left velocity=%3f\n", v_left);
    
    cmd_vel[0]=v_left*1000;
    cmd_vel[1]=v_right*1000;
    std_msgs::Float32MultiArray LRvel;
    LRvel.data=cmd_vel;
	cmd_vel_pub.publish(LRvel);
 
}

int main(int argc, char **argv){
	
    ros::init(argc, argv, "move_base_node");  
    move_base plan;
    
    while(ros::ok()){
    	plan.pub_vel();
    	ros::spinOnce();
    }
    return 0;
}

