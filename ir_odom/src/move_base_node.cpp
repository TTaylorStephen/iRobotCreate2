#include "move_base.h"


int bump_drop, l_cliff, fl_cliff, r_cliff, fr_cliff, l_light, fl_light, cl_light, cr_light, fr_light, r_light;
double t1, t2, dt; 

std::vector<float> cmd_vel(2);


move_base::move_base(){
    //initialize publisher for wheel velocity
    cmd_vel_pub=nh_.advertise<std_msgs::Float32MultiArray>("/wheel_vel",30);
    //initialize subscribers for all sensor data and distance info
    count_sub=nh_.subscribe("/total_count",30, &move_base::count_read, this);
    query_sub=nh_.subscribe<std_msgs::Int32MultiArray>("/query_list", 30, &move_base::query_read, this);
  //  imu_sub=nh_.subscribe<geometry_msgs::Accel>("/accel_data", 30, &move_base::imu_read, this);
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

/*void move_base::imu_read(const geometry_msgs::Accel::ConstPtr& _accel){
	t1=t2; xl_vel_i=xl_vel;
	
	//read acceleration
	xl_accel=_accel->linear.x;
	yl_accel=_accel->linear.y;
	zl_accel=_accel->linear.z;
	za_vel=_accel->angular.z;
	heading=za_vel*dt;

	//integrate for velocity
	//xl_vel=xl_vel_i+xl_accel*dt;
	//yl_vel=yl_accel*dt;
	//zl_vel=zl_accel*dt;
	//velocity=sqrt((xl_vel*xl_vel)+(yl_vel*yl_vel));

	//integrate again for position
	//x=xl_vel*dt;
	//y=yl_vel*dt;
	//z=zl_vel*dt;
	
	t2=ros::Time::now().toSec();
	dt=t2-t1;
	//ROS_INFO("x accel: %.3f m/s, y accel: %.3f m/s, z accel: %.3f m/s\n", xl_accel, yl_accel, zl_accel);
	//ROS_INFO("x vel: %.3f m/s, y vel: %.3f m/s, z vel: %.3f m/s\n", xl_vel, yl_vel, zl_vel);
	//ROS_INFO("x pos: %.3f m/s, y pos: %.3f m/s, z pos: %.3f m/s, dt=%3f \n", x, y, z, dt);
	//ROS_INFO("angular velocity is: %3f, heading is: %3f\n", za_vel, heading);
	//ROS_INFO("linear velocity is: %3f", velocity);
}*/
float l_tot_dist_m, l_tot_dist_mm, r_tot_dist_m, r_tot_dist_mm, l_dist_m, l_dist_mm, r_dist_m, r_dist_mm, total_dist;
int l_count, l_count1, r_count, r_count1, ll_count, rl_count, i;
 void move_base::count_read(const std_msgs::Int32MultiArray::ConstPtr& count){ 
     
    //total distance
    l_count=count->data[0];
    r_count=count->data[1];
    l_tot_dist_mm=(l_count*PI*WHEEL_DIAMETER)/508.8; //508.8 counts/rev = encoder resolution
    r_tot_dist_mm=(r_count*PI*WHEEL_DIAMETER)/508.8;
    l_tot_dist_m=l_dist_mm/1000;
    r_tot_dist_m=r_dist_mm/1000;
    ROS_INFO("left distance traveled in meters is: %f",l_tot_dist_mm);
    ROS_INFO("right distance traveled in meters is: %f",r_tot_dist_mm);

    //distance since last wheel movement
    ll_count=count->data[2];
    rl_count=count->data[3];
    l_dist_mm=(ll_count*PI*WHEEL_DIAMETER)/508.8; //508.8 counts/rev = encoder resolution
    r_dist_mm=(rl_count*PI*WHEEL_DIAMETER)/508.8;
    l_dist_m=l_dist_mm/1000;
    r_dist_m=r_dist_mm/1000;
    ROS_INFO("dr is: %f",l_dist_mm);
    ROS_INFO("dl is: %f",r_dist_mm);

    //testing distance readings//
        pub_vel(0.1,0.1);
        if(i>50){
            pub_vel(0,0);
        }
        if(i>100){
            pub_vel(0.2,0.2);
        }
        i++;

 }

double move_base::pub_vel(double vl_cmd,double vr_cmd){
    cmd_vel[0]=vl_cmd;
    cmd_vel[1]=vr_cmd;
    std_msgs::Float32MultiArray LRvel;
    LRvel.data=cmd_vel;
    cmd_vel_pub.publish(LRvel);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "move_the_base");       
    move_base plan;
    while(ros::ok()){
    	ros::spinOnce();
    } 
    return 0;
}

