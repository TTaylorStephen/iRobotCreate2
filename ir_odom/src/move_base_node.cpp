#include "move_base.h"


int bump_drop, l_cliff, fl_cliff, r_cliff, fr_cliff, l_light, fl_light, cl_light, cr_light, fr_light, r_light;
double t1, t2;
float dt=0.1; //sampling time
float r_vel=0.3, l_vel=0.3, heading;

move_base::move_base(){
    
    //initialize publisher for wheel velocity
    cmd_vel_pub=nh_.advertise<std_msgs::Float32MultiArray>("/wheel_vel",30);
    //initialize subscribers for all sensor data and distance info
    count_sub=nh_.subscribe<std_msgs::Int32MultiArray>("/total_count",30, &move_base::count_read, this);
    query_sub=nh_.subscribe<std_msgs::Int32MultiArray>("/query_list", 30, &move_base::query_read, this);
//    imu_sub=nh_.subscribe<std_msgs::Float32MultiArray>("/imu_data", 30, &move_base::imu_read, this);

} 

//subscribe to and define all sensor values for use elsewhere
void move_base::query_read(const std_msgs::Int32MultiArray::ConstPtr& query_array){
    bump_drop=query_array->data[0]; 
    l_cliff=query_array->data[1]; 
    fl_cliff=query_array->data[2]; 
    r_cliff=query_array->data[3]; 
    fr_cliff=query_array->data[4];
    l_light=query_array->data[5]; 
    fl_light=query_array->data[6]; 
    cl_light=query_array->data[7];
    cr_light=query_array->data[8]; 
    fr_light=query_array->data[9]; 
    r_light=query_array->data[10];
}

float l_tot_dist_m, l_tot_dist_mm, r_tot_dist_m, r_tot_dist_mm, l_dist_m, l_dist_mm, r_dist_m, r_dist_mm, avg_dist, avg_total_dist;
int l_count, l_count1, r_count, r_count1, ll_count1, ll_count, rl_count1, rl_count;
float w_el, w_er, avg_v_el, v_el, v_er, avg_v_er, avg_ve, avg_we;
int t_i, t_p;

 void move_base::count_read(const std_msgs::Int32MultiArray::ConstPtr& count){ 
		
	//save previous iteration
	l_count1=l_count;
	r_count1=r_count;
	//total distance
	l_count=count->data[0];
	r_count=count->data[1];
	l_tot_dist_mm=(l_count*PI*WHEEL_DIAMETER)/508.8; //508.8 counts/rev = encoder resolution
	r_tot_dist_mm=(r_count*PI*WHEEL_DIAMETER)/508.8;
	l_tot_dist_m=l_dist_mm/1000;
	r_tot_dist_m=r_dist_mm/1000;
	avg_total_dist=(l_tot_dist_m+r_tot_dist_m)/2;
	ROS_INFO("left distance traveled in meters is: %f",l_tot_dist_m);
	ROS_INFO("right distance traveled in meters is: %f",r_tot_dist_m);
	
	//save previous iteration
	ll_count1=ll_count;
	rl_count1=rl_count;
	//distance since last wheel movement
	ll_count=count->data[2];
	rl_count=count->data[3];
	l_dist_mm=(ll_count*PI*WHEEL_DIAMETER)/508.8; //508.8 counts/rev = encoder resolution
	r_dist_mm=(rl_count*PI*WHEEL_DIAMETER)/508.8;
	l_dist_m=l_dist_mm/1000;
	r_dist_m=r_dist_mm/1000;
	avg_dist=(r_dist_m+l_dist_m)/2;
	// ROS_INFO("dr is: %f",l_dist_mm);
	// ROS_INFO("dl is: %f",r_dist_mm);
	
	// source: cs.cmu.edu/~16311/s07/labs/NXTLabs/Lab%203.html
	//left and right angular velocities 
	w_el=(ll_count-ll_count1)*PI/(dt*180);  
	w_er=(rl_count-rl_count1)*PI/(dt*180);
	avg_we=(w_el-w_er)/WHEEL_BASE;
	
	//left and right linear velocities
	v_el=w_el*(WHEEL_DIAMETER/2)/1000; 
	v_er=w_er*(WHEEL_DIAMETER/2)/1000; 
	avg_ve=(v_el+v_er)/2;
	
	//averages and subtract offset for constant error observed
	avg_v_el=v_el;
	avg_v_er=v_er;
	
	//heading
	heading=avg_we*dt;
	ROS_INFO("heading= %3f rad/sec",heading);
	
 	ROS_INFO("left_wheel velocity=%3f", v_el);
	ROS_INFO("average left wheel velocity=%3f\n", avg_v_el);
 	//ROS_INFO("normalized left wheel velocity=%3f\n", norm_v_el);
 	
	ROS_INFO("right wheel velocity=%3f", v_er);
	ROS_INFO("average right wheel velocity=%3f", avg_v_er);
	//ROS_INFO("normalized right wheel velocity=%3f\n", norm_v_er);	
 }
 
std::vector<float> cmd_vel(2);
float move_base::pub_vel(float vl_cmd,float vr_cmd){
    
    cmd_vel[0]=vl_cmd;
    cmd_vel[1]=vr_cmd;
    std_msgs::Float32MultiArray LRvel;
    LRvel.data=cmd_vel;
	cmd_vel_pub.publish(LRvel);
    //state_calcs();
}
 


void move_base::nav_plan(){
	if(abs(avg_dist)>10){
		pub_vel(0,0);
	}	
	else{
		//printf("average distance traveled=%3f\n",avg_dist);
		pub_vel(r_vel,l_vel);	
	}	
}

int main(int argc, char **argv){
	
    ros::init(argc, argv, "move_the_base");  
    move_base plan;
    
    while(ros::ok()){
    	plan.nav_plan();
    	ros::spinOnce();
    }
    return 0;
}

