#include "odometry.h"

double dt=0, now=0, last_update=0;
float v_left=0, v_right=0, d_right=0, d_left=0;
float g_heading=0, x_pos=0, y_pos=0;

//variables for resetting count and getting distance
int l_count=0, r_count=0, l_count1=0, r_count1=0;
float delt_l=0, delt_r=0, l_total_count=0, r_total_count=0, l1_total_count=0, r1_total_count=0, dist_l=0, dist_r=0;
float l_tot_dist_m=0, l_tot_dist_mm=0, r_tot_dist_m=0, r_tot_dist_mm=0, l_dist_m=0, l_dist_mm=0, r_dist_m=0, r_dist_mm=0, avg_dist=0, avg_total_dist=0;
float l_dist_mm1=0, r_dist_mm1=0;


estimate::Odometry::Odometry(ros::NodeHandle nh)
	:nh_(nh)
{
	count_sub=nh_.subscribe<std_msgs::Int32MultiArray>("/total_count",10, &estimate::Odometry::countRead, this);
	imu_sub=nh_.subscribe<ir_odom::MPU>("/imu_data", 30, &estimate::Odometry::mpuRead, this);
	pose_pub=nh_.advertise<geometry_msgs::Pose2D>("/2Dpose", 30);
	velocity_m_pub=nh_.advertise<ir_odom::VelocityM>("/wheel_velocity/measured", 30);
	velocity_timer=nh_.createTimer(ros::Duration(0.017),&estimate::Odometry::getVelocity,this);
}


/*.............................read in necessary data and process for state info.....................................*/

void estimate::Odometry::mpuRead(const ir_odom::MPU::ConstPtr &data){
	g_heading=data->yaw;
}

void estimate::Odometry::countRead(const std_msgs::Int32MultiArray::ConstPtr &count){ 
	
	l_count=count->data[0];		//total distance
	r_count=count->data[1];
	//ROS_INFO("left count:%d\n", l_count);
	//dt=count->data[2]; 			//change of time between samples 
}

void::estimate::Odometry::getState(){

	if(l_count!=l_count1){
		
		getTime();	
		/*account for rollover @ 65535*/
		//ROS_INFO("lcount: %d, lcount1: %d\n", l_count, l_count1);
		if(l_count-l_count1<-100){ 	  
			l_count1=0;
		}
		if(l_count-l_count1>100){
			l_count1=0;
		}		
		if(r_count-r_count1<-100){
			r_count1=0;
		}
		if(r_count-r_count1>100){
			r_count1=0;
		}
		
		for(int i=0; i<10; i++){	 //reset variables if wheels stop	
			if (delt_l==0){
				dist_l=0; 
			}
			if(delt_r==0){
				dist_r=0;
			}
		}	
		
		/*reset count and sum for total on each wheel*/
		if(l_count1!=0){    		 
			delt_l=l_count-l_count1; //change in count
		}
		l_total_count+=delt_l;	//total from start of motion    
		dist_l+=delt_l;			//since last wheel movement
		if(r_count1!=0){		//same for right wheel  	 
			delt_r=r_count-r_count1;
		}
		r_total_count+=delt_r;      
		dist_r+=delt_r;	
		//ROS_INFO("Change in left count: %3f\n", delt_l);
		/*total distance*/ 
		l_tot_dist_mm=(l_total_count*PI*WHEEL_DIAMETER)/508.8; //508.8 counts/rev = encoder resolution
		r_tot_dist_mm=(r_total_count*PI*WHEEL_DIAMETER)/508.8;
		l_tot_dist_m=l_tot_dist_mm/1000;
		r_tot_dist_m=r_tot_dist_mm/1000;
		avg_total_dist=(l_tot_dist_m+r_tot_dist_m)/2;
		
		/*distance since last week movement*/
		l_dist_mm=(dist_l*PI*WHEEL_DIAMETER)/508.8; 
		r_dist_mm=(dist_r*PI*WHEEL_DIAMETER)/508.8;		
		l_dist_m=l_dist_mm/1000;
		r_dist_m=r_dist_mm/1000;
		avg_dist=(r_dist_m+l_dist_m)/2; 
		//ROS_INFO("average distance in meters=%3f", avg_dist);
		//ROS_INFO("Heading = %3f\n", g_heading);
		
		/*change of distance in wheels (dx)*/
		d_left=l_dist_mm-l_dist_mm1;	
		d_right=r_dist_mm-r_dist_mm1;
		ROS_INFO("change in left distance: %3f\n", d_left);
		/* calculate position and velocity*/
	 	x_pos=avg_dist*cos(g_heading); 
	 	y_pos=avg_dist*sin(g_heading);
		v_left=d_left/dt;				
		v_right=d_right/dt; 
		//ROS_INFO("Estimated left wheel velocity is: %3f m/s\n", v_right);
		//ROS_INFO("Change in time between cycles is: %3f sec\n", dt);
		l_count1=l_count;	//save previous calculations
		r_count1=r_count;	
		l1_total_count=l_total_count;
		l_dist_mm1=l_dist_mm, r_dist_mm1=r_dist_mm;	
	}
		
}

void estimate::Odometry::publishOdom(){
	
	ir_odom::VelocityM vel;
	//publish measured velocity for controls
	ROS_INFO("right velocity: %3f\n", v_left);
	vel.leftVm  = v_left;
	vel.rightVm = v_right;
	vel.dt = dt;
	velocity_m_pub.publish(vel);

	geometry_msgs::Pose2D loco;
	//publish 2d pose for now
	loco.x=x_pos;
	loco.y=y_pos;
	loco.theta=g_heading; 	
	pose_pub.publish(loco);

//	ROS_INFO("x = %3f, y = %3f\n",x_pos, y_pos); 	
	 	
 }

void estimate::Odometry::getVelocity(const ros::TimerEvent&){
	publishOdom();
}
/*..............................miscellaneous functions to help processing data ...................................*/


//get delta t
void estimate::getTime(){
	now = ros::Time::now().toSec();	
	if(last_update>0){
		dt = (now - last_update); // set integration time by time elapsed since last filter update
	}	
	last_update = now;	
}


/*..................................................run node.......................................................*/

 
 int main(int argc, char* argv[]){
 	ros::init(argc, argv, "odometry");
 	
 	ros::NodeHandle nh;
 	estimate::Odometry ir(nh); 
 	
 	while(ros::ok()){
 		ir.getState();
 		ros::spinOnce();
 	}
 	
 	return 0;
 }
