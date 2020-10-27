#include "imu__init.h"

uint8_t sample_rate= 0x04; //sets to 200Hz TODO: why?

int fd;
float a_scale, g_scale;
float a_res=2; //+-g
float g_res=250; //deg/sec
float dt = 0.05; //200Hz sample rate or 0.05s/sample)

mpu9255::mpu9255(){
	
	//begin i2c communication
	imu_init();
	g_scale = g_res/32768 , a_scale = a_res/32768;
	//initialize both boards on and set AK8963 as slave to MPU9250
	mpu_init(g_scale, a_scale);
	init_ak();
	
	calibrate_mpu();
	init_ak_slave();
	
	//publisher for 
	accel_data=nh.advertise<geometry_msgs::Accel>("/accel_data",10);
}


int mpu9255::imu_init(){
/*initialize I2C pin*/ 
	if(0==fd){
	  	fd = open("/dev/i2c-1",O_RDWR);
	  	printf("i2c bus initialized at register of 0x%X\n", who_am_i());
  	}
	if(fd < 0){
		perror("issue initializing i2c comm\n"); //check for error
		return -1;
	}

	tcflush(fd,TCIOFLUSH);//flush buffer
	
	usleep(100000);
	return fd;
}

uint8_t mpu9255::write_byte(uint8_t dev_add, uint8_t reg_add, uint8_t data){
	if(ioctl(fd,I2C_SLAVE,dev_add)<0){
		perror("failed to add icm20948\n");
		return -1;
	}
	uint8_t data_write[2];
	data_write[0]=reg_add;
	data_write[1]=data;
	if(write(fd,data_write,2)!=2){
		perror("failed to write to device\n");
		return -1;
	}
}
			
uint8_t mpu9255::read_byte(uint8_t dev_add, uint8_t reg_add){
/*specify device address to communicate with*/
	if(ioctl(fd,I2C_SLAVE,dev_add)<0){
		perror("failed to add icm20948\n");
		return -1;
	}	
	uint8_t write_data[1];
	write_data[0]=reg_add;
	if(write(fd,write_data,1)!=1){
		perror("failed to request data\n");
	}
	uint8_t response[1];
	if(read(fd,response,1)!=1){
		perror("failed to read data\n");
		return -1;
	}
	tcflush(fd, TCIOFLUSH);
	return response[0];
}

uint8_t mpu9255::read_bytes(uint8_t dev_add, uint8_t reg_add, uint8_t num_bytes, uint8_t * buf){
/*specify device address to communicate with*/
	if(ioctl(fd,I2C_SLAVE,dev_add)<0){
		perror("failed to add icm20948\n");
		return -1;
	}
		
	uint8_t write_data[1];
	write_data[0]=reg_add;
	if(write(fd,write_data,1)!=1){
		perror("write multiple bytes failed:\n");
		return -1;
	}

	uint8_t response[num_bytes];
	if(read(fd,response,num_bytes)<0){
		perror("error reasing multiple bytes:");
		return -1;
	}
	
	for(int i=0; i<num_bytes; i++){
		//printf("mulptiple bytes read = %X\n\n", response[i]);	
		buf[i]=response[i];
	}
	

	//tcflush(fd, TCIOFLUSH);
	return 0;
}

void mpu9255::calibrate_mpu(){

	uint8_t raw_data[12]; //will hold x,y,z accel and gyro data samples
	int16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]={0,0,0}, accel_bias[3]={0,0,0};
	float final_data[3]={0,0,0};
	
	//reset device
	write_byte(IMU_ADD, PWR_MGMT_1, 0x80);
	usleep(100000);
	
	write_byte(IMU_ADD, PWR_MGMT_1, 0x01);
	write_byte(IMU_ADD, PWR_MGMT_2, 0x00);
	usleep(200000);
	
	//configure device for bias calibration
	write_byte(IMU_ADD, INT_ENABLE, 0x00); //disable interrupts
	write_byte(IMU_ADD, FIFO_EN, 0x00); //disable fifo (first in first out reading)
	write_byte(IMU_ADD, PWR_MGMT_1, 0x00); // turn on internal clock source
	write_byte(IMU_ADD, I2C_MST_CTRL, 0x00); //disable i2c master
	write_byte(IMU_ADD, USER_CTRL, 0x00);	//dsiable fifo and i2c master modes
	write_byte(IMU_ADD, USER_CTRL, 0x0C); //reset fifo and dmp
	usleep(15000);
	
	//configure MPU6050 gyo and accel calibration
	write_byte(IMU_ADD, CONFIG, 0x01); //set low-pass filter to 188 hz
	write_byte(IMU_ADD, SMPLRT_DIV, 0x00);
	write_byte(IMU_ADD, GYRO_CONFIG, 0x00); //set gyro full scale @ 250 degrees/sec max sensitivity
	write_byte(IMU_ADD, ACCEL_CONFIG, 0x00);//set accelerometer full-scale to 2 g, maximum sensitivity
	
	uint16_t gyro_sensitivity = 131;     //from specs LSB/deg/sec
	uint16_t accel_sensitivity = 16384; //from specs LSB/g
	
	write_byte(IMU_ADD, USER_CTRL, 0x40); //enable fifo, 0x40=01000000 
	write_byte(IMU_ADD, FIFO_EN, 0x78); //enable gyro and accelerometer sensors for fifo, 0x78=01111000
	usleep(40000); //accumulate 40 samples in 40 millisecond = 480samples/12bytes
	
	//configure FIFO to capture accel and gyro data for bias calibration
	write_byte(IMU_ADD, FIFO_EN, 0x00); //disable gyro/accel sensors
	read_bytes(IMU_ADD, FIFO_COUNTH, 2, &raw_data[0]); //read from the fifo buffer	
	fifo_count= ((uint16_t)raw_data[0]<<8) | raw_data[1]; 
	packet_count = fifo_count/12; //12 full sets of gyro/accel data 6 bytes each so divide by 12 to get amount of fifo sent

	for(ii=0; ii<packet_count; ii++){
	
		int16_t accel_temp[3], gyro_temp[3]; //store temp values
		read_bytes(IMU_ADD, FIFO_R_W, 12, &raw_data[0]); //read data setup for averaging	
		accel_temp[0]=(int16_t)(((int16_t)raw_data[0]) | raw_data[1]); //form a 16bit integer for each sample in FIFO
		accel_temp[1]=(int16_t)(((int16_t)raw_data[2]) | raw_data[3]); 
		accel_temp[2]=(int16_t)(((int16_t)raw_data[4]) | raw_data[5]);
		gyro_temp[0]=(int16_t)(((int16_t)raw_data[6]) | raw_data[7]);
		gyro_temp[1]=(int16_t)(((int16_t)raw_data[8]) | raw_data[9]);
		gyro_temp[2]=(int16_t)(((int16_t)raw_data[10]) | raw_data[11]);
		
		//sum individual signed 16bit biases to get int value 
		accel_bias[0] += (int32_t)accel_temp[0]; 
		accel_bias[1] += (int32_t)accel_temp[1];
		accel_bias[2] += (int32_t)accel_temp[2];
		gyro_bias[0] += (int32_t)gyro_temp[0];
		gyro_bias[1] += (int32_t)gyro_temp[1];
		gyro_bias[2] += (int32_t)gyro_temp[2];
	}
		//normalize sums to get average count biases
		accel_bias[0]/=(int32_t)packet_count;
		accel_bias[1]/=(int32_t)packet_count;
		accel_bias[2]/=(int32_t)packet_count;
		gyro_bias[0]/=(int32_t)packet_count;
		gyro_bias[1]/=(int32_t)packet_count;
		gyro_bias[2]/=(int32_t)packet_count;
	
	//remove gravity from the z-accelerometer bias calc (L means long)
	if(accel_bias[2]> 0L){
		accel_bias[2] -= (int32_t)accel_sensitivity;
	}
	else{
		accel_bias[2]+= (int32_t)accel_sensitivity; 
	}	
	
	//construct gyro biases for push to hardware gyro bias registers which are reset upon device startup
	raw_data[0]=(-gyro_bias[0]/4 >> 8) & 0xFF; //divide 131 by 4 to get 32.9 LSB/deg/s to conform to expected bias input, more info in X_OFFS_USR
	raw_data[1]=(-gyro_bias[0]/4) & 0xFF;		//biases are additive, so change sign on calculated gryo biases
	raw_data[2]=(-gyro_bias[1]/4 >> 8) & 0xFF; //set up high byte
	raw_data[3]=(-gyro_bias[1]/4) & 0xFF;	    
	raw_data[4]=(-gyro_bias[2]/4 >> 8) & 0xFF; //set up low byte 
	raw_data[5]=(-gyro_bias[2]/4) & 0xFF;
	
	//push gyro biases to the hardware registers
	write_byte(IMU_ADD, XG_OFFSET_H, raw_data[0]); 
	write_byte(IMU_ADD, XG_OFFSET_L, raw_data[1]);
	write_byte(IMU_ADD, YG_OFFSET_H, raw_data[2]);
	write_byte(IMU_ADD, YG_OFFSET_L, raw_data[3]);
	write_byte(IMU_ADD, ZG_OFFSET_H, raw_data[4]);
	write_byte(IMU_ADD, ZG_OFFSET_L, raw_data[5]);
	
	//save biases for display or use in program
	float g_bias_buf[3];
	g_bias_buf[0] = (float)gyro_bias[0]/(float)gyro_sensitivity;
	g_bias_buf[1] = (float)gyro_bias[1]/(float)gyro_sensitivity;
	g_bias_buf[2] = (float)gyro_bias[2]/(float)gyro_sensitivity;
	
	//construct the accelerometer biases for push to the hardware accelerometer bias registers
	int32_t accel_bias_reg[3];
	read_bytes(IMU_ADD, XA_OFFSET_H, 2, &raw_data[0]); //read factory trim values
	accel_bias_reg[0] = (int32_t)(((int16_t)raw_data[0]<<8) | raw_data[1]); //read bytes into an int
	read_bytes(IMU_ADD, YA_OFFSET_H, 2, &raw_data[0]);
	accel_bias_reg[1] = (int32_t)(((int16_t)raw_data[0]<<8) | raw_data[1]);
	read_bytes(IMU_ADD, ZA_OFFSET_H, 2, &raw_data[0]);
	accel_bias_reg[2] = (int32_t)(((int16_t)raw_data[0]<<8) | raw_data[1]);
	
	uint32_t mask = 1uL; //unsigned long assigned for mask of temp comp, bit 0 of lower byte of accel bias registers
	uint8_t mask_bit[3];
	
	for(ii<0; ii<3; ii++){
		if(accel_bias_reg[ii] & mask){
			mask_bit[ii] = 0x01; //if temperature comp bit is set, record that fact in mask_bit
		}
	}
	
	//construct total accelerometer bias, including calculated average accel bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); //subtract calculated avg bias scaled to 2048 LSB/g (16g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);
	
	raw_data[0]=(accel_bias_reg[0] >> 8) & 0xFF;
	raw_data[1]=(accel_bias_reg[0]) & 0xFF;
	raw_data[1]=raw_data[1] | mask_bit[0]; //preserve temp compensation bit when writing back to acc registers
	raw_data[2]=(accel_bias_reg[1] >> 8) & 0xFF;
	raw_data[3]=(accel_bias_reg[1]) & 0xFF;
	raw_data[3]=raw_data[3] | mask_bit[1];
	raw_data[4]=(accel_bias_reg[0] >> 8) & 0xFF;
	raw_data[5]=(accel_bias_reg[0]) & 0xFF;
	raw_data[5]=raw_data[5] | mask_bit[2];
	
	//some writing is here in the original code and noted to not be working so offsets never get pushed to the register
	
	//output scaled accelerometer values
	final_data[0] = (float)accel_bias[0]/(float)accel_sensitivity;
	final_data[1] = (float)accel_bias[1]/(float)accel_sensitivity;
	final_data[2] = (float)accel_bias[2]/(float)accel_sensitivity;
}

	

//get accelorometer and gyroscope address
uint8_t address; 
uint8_t mpu9255::who_am_i(){
	address=read_byte(IMU_ADD, 0x75);
	//printf("MPU9250 address = %X , ", address);
	return address;
}

int16_t temp_raw; 
float temp_c, temp_f;
float mpu9255::temp_read(){
	uint8_t raw_data[2];
	read_bytes(IMU_ADD,TEMP_OUT_H,2,&raw_data[0]);
	temp_raw=((int16_t)raw_data[0] << 8) | raw_data[1];
	temp_c=temp_raw/333.87 + 21;
	temp_f=temp_c*1.8+32.0;
	printf("temp in C = %3f, temp in F = %3f\n", temp_c, temp_f);
}

int mpu9255::mpu_init(uint8_t g_scale, uint8_t a_scale){
		
	//general configuration for board	
	write_byte(IMU_ADD, PWR_MGMT_1, 0x00); //enable all sensors
	usleep(100000);
	write_byte(IMU_ADD, PWR_MGMT_1, 0x01); //2nd bit sets PLL (best clock source) 
	usleep(200000);
	write_byte(IMU_ADD, CONFIG, 0x03); //dlpf_cfg = bits 2:0 = 011(3) = 0x03
	write_byte(IMU_ADD, SMPLRT_DIV, 0x04); //set sample rate to 200 Hz 
	
	//configure gyro
	 //get desired resolution
	uint8_t c = read_byte(IMU_ADD, GYRO_CONFIG); //get current configuration
	c = c & ~0x02; //clears Fchoice bits 00000011=0x02
	c = c & ~0x18; //clears full scale settings 00011000=0x18
	c = c | g_scale << 3; //set full scale range by shifting 3 bits left
	write_byte(IMU_ADD, GYRO_CONFIG, c);
	
	//configure accelerometer fs range
	//get desired resolution
	c = read_byte(IMU_ADD, ACCEL_CONFIG); //get current accelerometer configuration
	c = c & ~0x18; //clear FS accel settings 
	c = c | a_scale << 3; //bitwise or to set new FS range
	write_byte(IMU_ADD, ACCEL_CONFIG,c);
	
	//set accelerometer sample rate config
	c = read_byte(IMU_ADD, ACCEL_CONFIG2);
	c = c & ~0x0F; //clear lower 4 bits for accel_fchoiceb and a_dlpfg 
	c = c & 0x03; //set a_dlpfg to 3 for 41hz bandwidth and 1000kHz sample rate
	write_byte(IMU_ADD, ACCEL_CONFIG2, c);
	
	//set pins to high(00000010=0x10) and enable raw sensor data ready (bit 0 00000001=0x01) interrupt
	write_byte(IMU_ADD, INT_PIN_CFG, 0x10);
	write_byte(IMU_ADD, INT_ENABLE, 0x01);	
	usleep(100000);
	
	write_byte(IMU_ADD, USER_CTRL, 0x20); //enable I2C master mode
	write_byte(IMU_ADD, I2C_MST_CTRL, 0x1D); //bit 4 controls i2c master's transition from one slave to next if set to 1 there is a pause b/w reads
											 //if set to 0 there is a reset inbetween reads, dec 13=1101 for bits [3:0] 
	write_byte(IMU_ADD, I2C_MST_DELAY_CTRL, 0x81); //use blocking data retrieval and enable delay for mag sample rate mismatch
	write_byte(IMU_ADD, I2C_SLV4_CTRL, 0x01); //delay mag data retriveal to once every other accel/gyro data sample
	
}	

//returns value in G/byte (+-2^16=+-32768), multiply by read byte for G/sec and multiple again by gravity to get m/s
/*float mpu9255::set_a_res(uint8_t a_scale){
	switch(a_scale){
		case AFS_2G:
			a_res=2.0/32768.0;
			return a_res;
			break;
		case AFS_4G:
			a_res=4.0/32768.0;
			return a_res;
			break;
		case AFS_8G:
			a_res=8.0/32768.0;
			return a_res;
			break;
		case AFS_16G:
			a_res=16.0/32768.0;
			return a_res;
			break;
	}
}*/

geometry_msgs::Accel _accel;
float vx, vy, vz;
uint8_t x_hl[2], y_hl[2], z_hl[2];
int16_t x_accel_raw, y_accel_raw, z_accel_raw;
float xi_accel, yi_accel, zi_accel, x_vel, y_vel;
float x_accel, y_accel, z_accel,x_accel1, y_accel1, z_accel1;

int mpu9255::accel_read(){
	
	
		tcflush(fd, TCIOFLUSH);

		//read x acceleration
		x_hl[0]=read_byte(IMU_ADD, ACCEL_XOUT_L); //lsb is low byte; read first.
		x_hl[1]=read_byte(IMU_ADD, ACCEL_XOUT_H);
		x_accel_raw=(x_hl[1]<<8)|x_hl[0];
		xi_accel=x_accel_raw*GRAVITY/16384;
		//printf("x-acceleration = %3f\n", xi_accel);
		
		//read y acceleration
		y_hl[0]=read_byte(IMU_ADD, ACCEL_YOUT_L);
		y_hl[1]=read_byte(IMU_ADD, ACCEL_YOUT_H);
		y_accel_raw=(y_hl[1]<<8)|y_hl[0];
		yi_accel=y_accel_raw*GRAVITY/16384;

		//read z acceleration
	 	z_hl[0]=read_byte(IMU_ADD, ACCEL_ZOUT_L);
		z_hl[1]=read_byte(IMU_ADD, ACCEL_ZOUT_H);
		z_accel_raw=(z_hl[1]<<8)|z_hl[0];
		zi_accel=z_accel_raw*GRAVITY/16384;

		tcflush(fd, TCIOFLUSH);
		//printf("-------------------------------------------------------------------------\n");
		ROS_INFO("x accel: %.3f m/s, y accel: %.3f m/s, z accel: %.3f m/s", xi_accel, yi_accel, zi_accel);
	
		//publish linear accels to ros	
		_accel.linear.x=xi_accel;
		_accel.linear.y=yi_accel;
		_accel.linear.z=zi_accel;
		accel_data.publish(_accel);
		
		//calculate velocity from sample rate of 200hz (0.05s)
		vx=xi_accel*dt;
		vy=yi_accel*dt;
		vz=zi_accel*dt;
		ROS_INFO("x vel = %3f, y vel = %3f, z vel = %3f",vx, vy, vz); 

	return 0;
}

//returns value in DPS/byte, multiply by read byte for degrees/sec
/*float mpu9255::set_g_res(uint8_t g_scale){
	switch(g_scale){
		case GFS_250DPS:
			g_res=250.0/32768;
			return g_res;
			break;
		case GFS_500DPS:
			g_res=500.0/32768;
			return g_res;
			break;
		case GFS_1000DPS:
			g_res=1000.0/32768;
			return g_res;
			break;
		case GFS_2000DPS:
			g_res=2000.0/32768;
			return g_res;
		break;
	}
}*/

uint8_t gyro_raw[6];
int16_t gyro[3];
float x_ang, y_ang, z_ang, wx, wy, wz;
float xi_ang, yi_ang, zi_ang, yaw;
int mpu9255::gyro_read(){
	
	//tcflush(fd, TCIOFLUSH);
	int gyro_sensitivity=131; //131 LSB/(deg/s)
	//read x acceleration
	read_bytes(IMU_ADD, GYRO_XOUT_H, 6, &gyro_raw[0]);
	gyro[0]=((int16_t)gyro_raw[0] << 8) | gyro_raw[1];
	gyro[1]=((int16_t)gyro_raw[2] << 8) | gyro_raw[3];
	gyro[2]=((int16_t)gyro_raw[4] << 8) | gyro_raw[5];
	tcflush(fd, TCIOFLUSH);
	
	wx=(float)gyro[0]/gyro_sensitivity;
	wy=(float)gyro[1]/gyro_sensitivity;
	wz=(float)gyro[2]/gyro_sensitivity;
	
	ROS_INFO("x ang rate: %.5f deg/s, y ang rate: %.5f deg/s, z ang rate: %.5f deg/s", wx, wy, wz);
	//printf("-------------------------------------------------------------------------\n");
	
	//publish angular accels to ros
	_accel.angular.x=wx;
	_accel.angular.y=wy;
	_accel.angular.z=wz;
	
	//yaw calc
	yaw=wz*dt;
	ROS_INFO("change in heading is %3f degrees", yaw);
	
	accel_data.publish(_accel);
	
	return 0;
}

uint8_t mag_id;
uint8_t mpu9255::who_am_i_ak8963(){
	write_byte(IMU_ADD, USER_CTRL, 0x20); //enable master mode, 0010 0000
	write_byte(IMU_ADD, I2C_MST_CTRL, 0x0D); //set multi-master i2c, 0000 1101
	write_byte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD | 0x80); //set slave address and set to read 0x80=i2c_slv_en, 0x81=enable slave and read one byte 1000 0001
	write_byte(IMU_ADD, I2C_SLV0_REG, AK8963_WHO_AM_I); //write magnetometer id register as place to begin data transfer
	write_byte(IMU_ADD, I2C_SLV0_CTRL, 0x81); //write i2c_slv0_en to enable reading data from slave (always EXT_SENS_DATA_00 for i2c slave 0) at desired sample rate. read one byte.
	usleep(100000);
	mag_id=read_byte(IMU_ADD, EXT_SENS_DATA_00);//read data from slave on IMU
	ROS_INFO("AK8963 address = 0x%X\n", mag_id);
}

//returns value in mG/LSB, multiply by raw data to get milli-gauss in desired resolution
float mag_res;
float mpu9255::set_m_res(uint8_t mag_scale){
	switch(mag_scale){
		case MFS_14BITS: 
			mag_res = 4912.0f/8192.0f;
			return mag_res;
			break;
		case MFS_16BITS:
			mag_res = 4912.0f/32768.0f; 
			return mag_res;
			break;
	}
}	

float mag_calibration[3];
uint8_t m_mode=0x06; //0010(0x02)=8hz, 0110(0x06)=100hz sample rate
void mpu9255::init_ak(){

	  uint8_t m_scale=set_m_res(MFS_16BITS);
 	  uint8_t rawData[3]={0,0,0};  // x/y/z gyro calibration data stored here
 	  
	  write_byte(IMU_ADD, AK_CTRL, 0x00); // Power down magnetometer  
	  usleep(100000);
	  
	  write_byte(IMU_ADD, AK_CTRL, 0x0F); // Enter Fuse ROM access mode
	  usleep(100000);
	  
	  read_bytes(IMU_ADD, AK_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	  mag_calibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	  mag_calibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
	  mag_calibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
	  
	  //printf("raw mag calibration data = %3f, %3f, %3f\n\n", mag_calibration[0], mag_calibration[1], mag_calibration[2]);
	  write_byte(IMU_ADD, AK_CTRL, 0x00); // Power down magnetometer  
	  usleep(100000);
	  
	  // Configure the magnetometer for continuous read and highest resolution
	  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	  write_byte(IMU_ADD, AK_CTRL, m_scale << 4 | m_mode); // Set magnetometer data resolution and sample ODR
	  usleep(100000);
}

void mpu9255::init_ak_slave(){
	uint8_t raw_data[3];
	write_byte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD);//set the i2c address to ak8963 and set for write
	write_byte(IMU_ADD, I2C_SLV0_REG, AK_CTRL2); //i2c slave 0 register from where to begin data transfer
	write_byte(IMU_ADD, I2C_SLV0_DO, 0X01); //reset ak8963
	write_byte(IMU_ADD, I2C_SLV0_CTRL, 0x81); //enable i2c and write one byte
	usleep(50000);
	
	write_byte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD);//set the i2c address to ak8963 and set for write
	write_byte(IMU_ADD, I2C_SLV0_REG, AK_CTRL); //i2c slave 0 register from where to begin data transfer
	write_byte(IMU_ADD, I2C_SLV0_DO, 0x00); //power dowm magnetometer
	write_byte(IMU_ADD, I2C_SLV0_CTRL, 0x81); //enable i2c and write one byte
	usleep(50000);	
	
	write_byte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD);//set the i2c address to ak8963 and set for write
	write_byte(IMU_ADD, I2C_SLV0_REG, AK_CTRL); //i2c slave 0 register from where to begin data transfer
	write_byte(IMU_ADD, I2C_SLV0_DO, 0x0F); //enter fuse mode
	write_byte(IMU_ADD, I2C_SLV0_CTRL, 0x81); //enable i2c and write one byte
	usleep(50000);
	
	write_byte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD | 0x80); //set i2c slave address and set for read
	write_byte(IMU_ADD, I2C_SLV0_REG, AK_ASAX); //i2c slave register from where to begin data transfer
	write_byte(IMU_ADD, I2C_SLV0_CTRL, 0x83); //enable data transfer and write 3 bytes
	usleep(50000);
	
	read_bytes(IMU_ADD, EXT_SENS_DATA_00, 3, &raw_data[0]); // read xyz calibration values
	mag_calibration[0] = (float)(raw_data[0]-128)/256.0f + 1.0f; //return x_axis sensitivity adjustment values 
	mag_calibration[1] = (float)(raw_data[1]-128)/256.0f + 1.0f;
	mag_calibration[2] = (float)(raw_data[2]-128)/256.0f + 1.0f;
	printf("raw mag calibration data = %3f, %3f, %3f\n\n", mag_calibration[0], mag_calibration[1], mag_calibration[2]);

	write_byte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD);//set the i2c address to ak8963 and set for write
	write_byte(IMU_ADD, I2C_SLV0_REG, AK_CTRL); //i2c slave 0 register from where to begin data transfer
	write_byte(IMU_ADD, I2C_SLV0_DO, 0x00); //power_down magnetometer
	write_byte(IMU_ADD, I2C_SLV0_CTRL, 0x81); //enable i2c and write one byte
	usleep(50000);
	
	write_byte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD);//set the i2c address to ak8963 and set for write
	write_byte(IMU_ADD, I2C_SLV0_REG, AK_CTRL); //i2c slave 0 register from where to begin data transfer
	
	//configure for continuous read at highest resolution 
	uint8_t m_scale=set_m_res(MFS_16BITS);
	write_byte(IMU_ADD, I2C_SLV0_DO, m_scale << 4 | m_mode); 
	write_byte(IMU_ADD, I2C_SLV0_CTRL, 0x81); 
	usleep(50000);
}

int16_t mag_xyz[3];
int mpu9255::mag_read(){

	uint8_t mag_raw[7];
	write_byte(IMU_ADD, I2C_SLV0_ADD, 0x00);//set slave address to read
	write_byte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD | 0x80);//set slave address to read
	write_byte(IMU_ADD, I2C_SLV0_REG, AK_HXL);
	write_byte(IMU_ADD, I2C_SLV0_CTRL, 0x87);
	read_bytes(IMU_ADD, EXT_SENS_DATA_00, 7, &mag_raw[0]);
	
	uint8_t c = mag_raw[6]; //end data transfer by reading St2 register
	if(!(c & 0x08)){   //check for magnetic overflow otherwise read in values
		mag_xyz[0]=((int16_t)mag_raw[1]<<8) | mag_raw[0]; //read high byte onto low byte
		mag_xyz[1]=((int16_t)mag_raw[3]<<8) | mag_raw[2];
		mag_xyz[2]=((int16_t)mag_raw[5]<<8) | mag_raw[4];

	}
ROS_INFO("x magnetic field is = %3f uG, y magnetic field is = %3f uG, z magnetic field is = %3f uG", mag_xyz[0]*mag_calibration[0]*set_m_res(MFS_16BITS), mag_xyz[1]*set_m_res(MFS_16BITS), mag_xyz[2]*set_m_res(MFS_16BITS));

	return 0;
}


int mpu_id, ak_id;
int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "imu_accel_data"); 
	mpu9255 nine_axis;
	mpu_id=nine_axis.who_am_i();
	ak_id=nine_axis.who_am_i_ak8963();

	ros::Rate lr(10);
	while(ros::ok()){
		nine_axis.temp_read();
		nine_axis.accel_read();
		nine_axis.gyro_read();
		nine_axis.mag_read();
		
		usleep(100000);
		lr.sleep();
		ros::spinOnce();
	}
	return 0;
}
