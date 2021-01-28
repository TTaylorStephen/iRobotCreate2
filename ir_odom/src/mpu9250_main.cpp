#include "mpu9250.h"

uint8_t mpu_id, ak_id; // variable to hold register addresses 
uint8_t  a_scale = AFS_2G, g_scale = GFS_250DPS, m_scale=MFS_16BITS; // define scales for return values
float a_res, g_res, m_res;  //variables to store resolution
uint8_t a_rate = 0x04; 		//sets to 200Hz 

float mag_calibration[3] = {0, 0, 0}; //vector for storing x,y,z magnetometer sensitivity adjustment values
uint8_t m_rate=0x06; //0010(0x02)=8hz, 0110(0x06)=100hz sample rate

//used for accel/gyro (mpu) calibration - output biases and scale recorded from calibration functions 
float a_bias[3] = {-0.0221812, 0.015564, -0.030273}, g_bias[3] = {-1.6504434, 0.847328, 0.244275};
float m_bias[3] = {557.109741, 212.527557, -340.863892}, mag_scale[3] = {1.054201, 0.960494, 0.989822};
const int room_offset = 0;

int16_t mag_data[3] = {0};			//for reading raw ak8963s
std::vector<int16_t> mpu_data(7);	//for reading raw mpu values
std::vector<float> accel_data(6);   //for publishing acceleration values

std_msgs::Float32MultiArray list;
geometry_msgs::Accel acc;
ir_odom::Euler ang;
ir_odom::Magnet mag;

//varaibles for filtering w/ madgewick
double del_t=0, now=0, last_update=0;

//variables for getting angles and filtering w/ low pass
float theta_a=0, theta_af=0, theta_g=0, 		
 	  phi_a=0, phi_af=0, phi_g=0,			  
 	  psi_g=0, psi_m=0;

float theta=0, phi=0, psi=0;  					//final euler angles
float cf_alpha=0.97, lp_alpha=0.4, m_alpha=0.2; //alpha coefs for low pass

int count=1;
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0, mx=0, my=0, mz=0, tc=0, tf=0; //holds original values
float ax_f=0, ay_f=0, az_f=0, mx_f=0, my_f=0, mz_f=0;	//holds filtered values

std::vector<float> avg_accel(3), total_accel(3), avg_gyro(3), total_gyro(3), avg_mag(3), total_mag(3);//for averaging
std::vector<float> comp_mag(3);

/*..........................miscellaneous functions to help processing data ...................................*/

//get delta t
float getTime(){
	now = ros::Time::now().toSec();	
	if(last_update>0){
		del_t = (now - last_update); // set integration time by time elapsed since last filter update
	}	
	last_update = now;	
	//MadgwickQuaternionUpdate(-ax, ay, az, gx*PI/180, -gy*PI/180, -gz*PI/180, -mx, my, mz, del_t, q);		
	return del_t;
}



void getAverages(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, const int ss){
		//reset if larger than sample size
		if(count>(ss)){
			count=1;
			total_accel[0]=avg_accel[0], total_accel[1]=avg_accel[1], total_accel[2]=avg_accel[2];
			total_gyro[0]=avg_gyro[0], total_gyro[1]=avg_gyro[1], total_gyro[2]=avg_gyro[2];
			total_mag[0]=avg_mag[0], total_mag[1]=avg_mag[1], total_mag[2]=avg_mag[2];

		}	
		avg_accel[0]=total_accel[0]/count, avg_accel[1]=total_accel[1]/count, avg_accel[2]=total_accel[2]/count;
		avg_gyro[0]=total_gyro[0]/count, avg_gyro[1]=total_gyro[1]/count, avg_gyro[2]=total_gyro[2]/count;
		avg_mag[0]=total_mag[0]/count, avg_mag[1]=total_mag[1]/count, avg_mag[2]=total_mag[2]/count;
		
		total_accel[0]+=ax, total_accel[1]+=ay, total_accel[2]+=az;		
		total_gyro[0]+=gx, total_gyro[1]+=gy, total_gyro[2]+=gz;
		total_mag[0]+=mx, total_mag[1]+=my, total_mag[2]+=mz;
}	

/*..........................................setup communication.................................................*/

adc::mpu9250::mpu9250(){
	fd = 0;	   //initialize variable to store imu address for reading/writing
	imuInit(); //begin i2c communication
	
	//initialize publishers
	accel_pub=nh.advertise<geometry_msgs::Accel>("/accel",30);
	ang_pub=nh.advertise<ir_odom::Euler>("/angles", 30);
	mag_pub=nh.advertise<ir_odom::Magnet>("/mag_fields", 30); 
	
	if(idMPU9250()==0x71){
		reset();
		a_res = getAres(a_scale), g_res = getGres(g_scale), m_res = getMres(m_scale); //get resolutions	
		//calibrateMPU(g_bias, a_bias);		//calibrate accel and gyro
		delay(1000);
		
		mpuInit(g_scale, a_scale, a_rate); //initialize MPU9250 for accel and gyro data		
		if(idAK8963()==0x48){
			initAKslave(m_scale, m_rate, mag_calibration);
			//calibrateMag(m_bias, mag_scale); //uncomment if needing to recalibrate and collect necessary values
		}
	}	
}

/*.......................................main conversions and processing........................................*/

void adc::mpu9250::convert(){
	
	//read temp, gyro, and accelerometer data
	mpu9250ReadAll(mpu_data); 		
	
	tc = ((float)mpu_data[3]-room_offset)/333.87 + 21;  //convert to temp deg celsius
	tf = (tc*1.8)+32.0; 								//convert to fareheight 
	
	//get change in time since last loops	
	getTime();
	ROS_INFO("dt=%3f", del_t);
	
	ax = ((float)mpu_data[0]*a_res)-a_bias[0]; 		   //convert acceleration readings to Gs    			       
	ay = ((float)mpu_data[1]*a_res); 
	az = ((float)mpu_data[2]*a_res)-a_bias[2];
	gx = ((float)mpu_data[4]*g_res)-g_bias[0];		   //convert gyroscope readings to deg/sec
	gy = ((float)mpu_data[5]*g_res)-g_bias[1];
	gz = ((float)mpu_data[6]*g_res)-g_bias[2]; 
	
	//read data from magnetometer returns in milliGauss
	magRead(mag_data); 
	mx  = ((float)mag_data[0]*m_res*mag_calibration[0]-m_bias[0]);
	my  = ((float)mag_data[1]*m_res*mag_calibration[1]-m_bias[1]);
	mz = ((float)mag_data[2]*m_res*mag_calibration[2]-m_bias[2]);
	mx *= mag_scale[0];
	my *= mag_scale[1]; 
	mz *= mag_scale[2];

	//get update of averages at sample size set below		
	getAverages(ax, ay, az, gx, gy, gz, mx, my, mz, 10);
	
	//low pass filter on accel and mag data
	mx_f=(1-m_alpha)*mx_f+m_alpha*avg_mag[0];
	my_f=(1-m_alpha)*my_f+m_alpha*avg_mag[1];
	mz_f=(1-m_alpha)*mz_f+m_alpha*avg_mag[2];
	ax_f = (1-lp_alpha)*ax_f +lp_alpha*avg_accel[0];	
	ay_f = (1-lp_alpha)*ay_f +lp_alpha*avg_accel[1];
	
	phi_a = atan2f(ay_f, sqrtf(pow(ax_f,2)+pow(avg_accel[2],2))); //pitch and roll from accelorometer values
	theta_a = atan2f(ax_f, sqrtf(pow(ay_f,2)+pow(avg_accel[2],2)));
	phi_g += gx*del_t; 	    //pitch roll and yaw from gyro and convert to radians
	theta_g -= gy*del_t;
	psi_g -= gz*del_t; 		//yaw from gyro drifts ~0.08 deg/sec
	
	//complimentary filter for angle smoothing (only good for pitch angle)
	phi = (1-cf_alpha)*(phi-phi_g) + (cf_alpha*phi_a); 
	theta = (1-cf_alpha)*(theta-theta_g) + (cf_alpha*theta_a);

	//compensate mag readings for tilt in x/y directions using angles calculated from gyro
	comp_mag[0] = avg_mag[0]*cos(phi) + avg_mag[1]*sin(theta)*sin(phi) - avg_mag[2]*cos(theta)*sin(phi);
	comp_mag[1] = avg_mag[1]*cos(phi) - avg_mag[2]*sin(phi);
	psi_m = atan2f(-comp_mag[1], comp_mag[0]);    //yaw from magnetometer
	
	//publish to ros for plotting with rqt_plot
	ang.aPitch = phi_a*180/PI;
	ang.gPitch = phi_g;
	ang.Pitch = phi;
	ang.aRoll = theta_a*180/PI;
	ang.gRoll = theta_g; 
	ang.Roll = theta;	
	ang.gYaw = psi_g;
	ang.mYaw = (psi_m*180/PI)-9.22;
	ang_pub.publish(ang);
	
	//store values from accelerometer m/s/s = linear rate of accereation 
	acc.linear.x = ax_f;
	acc.linear.y = ay_f;
	acc.linear.z = az;	
	//store filtered values from gyroscope deg/sec = ang rate velocity
	acc.angular.x = gx;
	acc.angular.y = gy;
	acc.angular.z = gz;
	accel_pub.publish(acc);
	//store filtered values from magnetometer in mG 
	mag.magX = mx_f;
	mag.magY = my_f;
	mag.magZ = mz_f; 
	mag_pub.publish(mag);
	
	count++;
}


int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "imu_accel_data"); 
	adc::mpu9250 imu1; //create imu object
	//ros::Rate lr(100);
	while(ros::ok()){
			imu1.convert();
		//		lr.sleep();
			ros::spinOnce();
	}
	return 0;
}


