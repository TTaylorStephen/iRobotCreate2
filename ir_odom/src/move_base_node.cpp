#include "move_base.h"

int bump_drop, l_cliff, fl_cliff, r_cliff, fr_cliff, l_light, fl_light, cl_light, cr_light, fr_light, r_light;
double t1, t2;
std::vector<float> cmd_vel(2);

move_base::move_base(){
    //initialize publisher for wheel velocity
    cmd_vel_pub=nh_.advertise<std_msgs::Float32MultiArray>("/wheel_vel",30);
    //initialize subscribers for all sensor data and distance info
    count_sub=nh_.subscribe("/total_count",30, &move_base::count_read, this);
    
    query_sub=nh_.subscribe<std_msgs::Int32MultiArray>("/query_list", 30, &move_base::query_read, this);
    imu_sub=nh_.subscribe<std_msgs::Float32MultiArray>("/imu_data", 30, &move_base::imu_read, this);
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
   // ROS_INFO("left distance traveled in meters is: %f",l_tot_dist_mm);
   // ROS_INFO("right distance traveled in meters is: %f",r_tot_dist_mm);

    //distance since last wheel movement
    ll_count=count->data[2];
    rl_count=count->data[3];
    l_dist_mm=(ll_count*PI*WHEEL_DIAMETER)/508.8; //508.8 counts/rev = encoder resolution
    r_dist_mm=(rl_count*PI*WHEEL_DIAMETER)/508.8;
    l_dist_m=l_dist_mm/1000;
    r_dist_m=r_dist_mm/1000;
    // ROS_INFO("dr is: %f",l_dist_mm);
    // ROS_INFO("dl is: %f",r_dist_mm);

 }

float tc,tf,ax,ay,az,wx,wy,wz,mx,my,mz;
void move_base::imu_read(const std_msgs::Float32MultiArray::ConstPtr& state){
    tc=state->data[0];
    tf=state->data[1];
    ax=state->data[2];
    ay=state->data[3];
    az=state->data[4];
    wx=state->data[5];
    wy=state->data[6];
    wz=state->data[7];
    mx=state->data[8];
    my=state->data[9];
    mz=state->data[10];

    pub_vel(0.2,0.2);
    //ROS_INFO("tempC=%3f, tempF=%3f\n\n, ax=%3f, ay=%3f, az=%3f\n\n, wx=%3f, wy=%3f, wz=%3f\n\n, mx=%3f, my=%3f, mz=%3f\n\n", tc,tf,ax,ay,az,wx,wy,wz,mx,my,mz);
}

float vdx, vdy, vdz, dtheta, dt=0.05, delt_heading;
void move_base::state_calcs(){
    vdx=(ax*dt);
    vdy=(ay*dt);
    vdx=(ay*dt);
    delt_heading=wz*dt;
    ROS_INFO("change in x velocity=%3f\n, change in y velocity=%3f\n, change in z velocity=%3f\n, heading=%3f", vdx, vdy, vdz, delt_heading);
}

double move_base::pub_vel(double vl_cmd,double vr_cmd){
    cmd_vel[0]=vl_cmd;
    cmd_vel[1]=vr_cmd;
    std_msgs::Float32MultiArray LRvel;
    LRvel.data=cmd_vel;
    cmd_vel_pub.publish(LRvel);
    state_calcs();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "move_the_base");       
    move_base plan;
    while(ros::ok()){
    	ros::spinOnce();
    } 
    return 0;
}

