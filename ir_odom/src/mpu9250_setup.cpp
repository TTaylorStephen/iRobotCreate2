#include "mpu9250.h"
/*******************************************************************************************************************/
/*        adapted from mkriswiner AK896t3 as slave to MPU9250 driver - for use with jetson nano			           */ 
/*        original source code found at guthub.com/kriswiner/MPU9250/tree/master/AK8963_as_slave                   */ 
/*******************************************************************************************************************/  


//initialize i2c communication with board 
int adc::mpu9250::imuInit(){ 
	if(0==fd){
	  	fd = open("/dev/i2c-1",O_RDWR);
  	}
	if(fd < 0){
		perror("issue initializing i2c comm\n"); //check for error
		return -1;
	}
	tcflush(fd,TCIOFLUSH);//flush buffer
	delay(100);
	return fd;
}



//reset MPU9250
void adc::mpu9250::reset(){
	writeByte(IMU_ADD, PWR_MGMT_1, 0x80);
	delay(100);
}



//get resolution for accerometer - possible option +-2G, 4G, 8G, 16G for 16 bit FS reading
//returns value in G's
float adc::mpu9250::getAres(uint8_t ascale){
	switch(ascale){
		case AFS_2G: 
			_a_res = 2.0/32768.0; //peak to peak @ +-250, resolution is 2^16=65536 bc 16 bit data
			return _a_res;
			break;
		case AFS_4G: 
			_a_res = 4.0/32768.0;
			return _a_res;
			break;
		case AFS_8G: 
			_a_res = 8.0/32768.0;
			return _a_res;
			break;
		case AFS_16G: 
			_a_res = 16.0/32768.0;
			return _a_res;
			break;
	}
}	
	
	
	
//set resolution for gyro - possible options = +-250dps, 500dps, 1000dps, 2000dps for 16 bit FS reading 
//returns value deg/sec 
float adc::mpu9250::getGres(uint8_t gscale){
	switch(gscale){
		case GFS_250DPS: 
			_g_res = 250.0/32768.0; //peak to peak @ +-250, resolution is 2^16=65536 bc 16 bit data
			return _g_res;
			break;
		case GFS_500DPS: 
			_g_res = 500.0/32768.0;
			return _g_res;
			break;
		case GFS_1000DPS: 
			_g_res = 1000.0/32768.0;
			return _g_res;
			break;
		case GFS_2000DPS: 
			_g_res = 2000.0/32768.0;
			return _g_res;
			break;
	}
}	



//get resolution for magnetometer - possible option = 14 bits or 16 bits per FS reading
//returns value in uT/LSB, note:10mG = 1uT
float adc::mpu9250::getMres(uint8_t mscale){
	switch(mscale){
		case MFS_14BITS: 
			_m_res = 10.0*4912.0/8192.0; //peak to peak range +-4912 uT, resolution is 2^14=(8192*2)=16384
			return _m_res;
			break;
		case MFS_16BITS:
			_m_res = 10.0*4912.0/32768.0; //peak to peak range +-4912 uT, resolution is 2^16=(32768*2)=65536 
			return _m_res;
			break;
	}
}	


	
//calibrate mpu9250
void adc::mpu9250::calibrateMPU(float *dest1, float *dest2){

	uint8_t raw_data[12]; //will hold x,y,z accel and gyro data samples
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]={0,0,0}, accel_bias[3]={0,0,0}; 
	
	//reset device
	writeByte(IMU_ADD, PWR_MGMT_1, 0x80);
	delay(100);
	
	//get stable time source; auto select clock source to be PLL gyroscope reference if ready
	//else use the internal oscillator, bits 2:0 = 001
	writeByte(IMU_ADD, PWR_MGMT_1, 0x01);
	writeByte(IMU_ADD, PWR_MGMT_2, 0x00);
	delay(200);
	
	//configure device for bias calibration
	writeByte(IMU_ADD, INT_ENABLE, 0x00);   //disable interrupts
	writeByte(IMU_ADD, FIFO_EN, 0x00);      //disable fifo (first in first out reading)
	writeByte(IMU_ADD, PWR_MGMT_1, 0x00);   // turn on internal clock source
	writeByte(IMU_ADD, I2C_MST_CTRL, 0x00); //disable i2c master
	writeByte(IMU_ADD, USER_CTRL, 0x00);	 //disable fifo and i2c master modes
	writeByte(IMU_ADD, USER_CTRL, 0x0C);    //reset fifo and dmp
	delay(15);
	
	//configure MPU6050 gyo and accel calibration
	writeByte(IMU_ADD, CONFIG, 0x01);       //set low-pass filter to 188 hz
	writeByte(IMU_ADD, SMPLRT_DIV, 0x00);
	writeByte(IMU_ADD, GYRO_CONFIG, 0x00);  //set gyro full scale @ 250 degrees/sec max sensitivity
	writeByte(IMU_ADD, ACCEL_CONFIG, 0x00); //set accelerometer full-scale to 2 g, maximum sensitivity
	
	uint16_t gyro_sensitivity  = 131;         // LSB/deg/sec
	uint16_t accel_sensitivity = 16384;       // LSB/g
	
	writeByte(IMU_ADD, USER_CTRL, 0x40);    //enable fifo, 0x40=01000000 
	writeByte(IMU_ADD, FIFO_EN, 0x78);      //enable gyro and accelerometer sensors for fifo, 0x78=01111000
	delay(40);  //accumulate 40 samples in 40 millisecond = 480samples/12bytes
	
	//configure FIFO to capture accel and gyro data for bias calibration
	writeByte(IMU_ADD, FIFO_EN, 0x00); //disable gyro/accel sensors
	readBytes(IMU_ADD, FIFO_COUNTH, 2, &raw_data[0]); //read from the fifo buffer	
	fifo_count = ((uint16_t)raw_data[0]<<8) | raw_data[1]; 
	packet_count = fifo_count/12; //12 full sets of gyro/accel data 6 bytes each so divide by 12 to get amount of fifo sent

	for(ii=0; ii<packet_count; ii++){
		int16_t accel_temp[3] = {0} , gyro_temp[3] = {0}; //store temp values
		readBytes(IMU_ADD, FIFO_R_W, 12, &raw_data[0]); //read data setup for averaging	
		accel_temp[0] = (int16_t) (((int16_t)raw_data[0]) | raw_data[1]  ); //form a 16bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)raw_data[2]) | raw_data[3]  ); 
		accel_temp[2] = (int16_t) (((int16_t)raw_data[4]) | raw_data[5]  );
		gyro_temp[0]  = (int16_t) (((int16_t)raw_data[6]) | raw_data[7]  );
		gyro_temp[1]  = (int16_t) (((int16_t)raw_data[8]) | raw_data[9]  );
		gyro_temp[2]  = (int16_t) (((int16_t)raw_data[10]) | raw_data[11]);
		
		//sum individual signed 16bit biases to get int value 
		accel_bias[0] += (int32_t)accel_temp[0]; 
		accel_bias[1] += (int32_t)accel_temp[1];
		accel_bias[2] += (int32_t)accel_temp[2];
		gyro_bias[0]  += (int32_t)gyro_temp[0];
		gyro_bias[1]  += (int32_t)gyro_temp[1];
		gyro_bias[2]  += (int32_t)gyro_temp[2];
	}
	
	//normalize sums to get average count biases
	accel_bias[0]/=(int32_t)packet_count;
	accel_bias[1]/=(int32_t)packet_count;
	accel_bias[2]/=(int32_t)packet_count;
	gyro_bias[0]/=(int32_t)packet_count;
	gyro_bias[1]/=(int32_t)packet_count;
	gyro_bias[2]/=(int32_t)packet_count;

	//remove gravity from the z-accelerometer bias calc (L means long)
	if( accel_bias[2] > 0L) { accel_bias[2] -= (int32_t)accel_sensitivity; }
	else { accel_bias[2] += (int32_t)accel_sensitivity; }	
	
	//construct gyro biases for push to hardware gyro bias registers which are reset upon device startup
	raw_data[0]=(-gyro_bias[0]/4 >> 8) & 0xFF; //divide 131 by 4 to get 32.9 LSB/deg/s to conform to expected bias input, more info in X_OFFS_USR
	raw_data[1]=(-gyro_bias[0]/4)      & 0xFF; //biases are additive, so change sign on calculated gyro biases
	raw_data[2]=(-gyro_bias[1]/4 >> 8) & 0xFF; //set up high byte
	raw_data[3]=(-gyro_bias[1]/4)      & 0xFF;	    
	raw_data[4]=(-gyro_bias[2]/4 >> 8) & 0xFF; //set up low byte 
	raw_data[5]=(-gyro_bias[2]/4)      & 0xFF;
	
	//push gyro biases to the hardware registers
	writeByte(IMU_ADD, XG_OFFSET_H, raw_data[0]); 
	writeByte(IMU_ADD, XG_OFFSET_L, raw_data[1]);
	writeByte(IMU_ADD, YG_OFFSET_H, raw_data[2]);
	writeByte(IMU_ADD, YG_OFFSET_L, raw_data[3]);
	writeByte(IMU_ADD, ZG_OFFSET_H, raw_data[4]);
	writeByte(IMU_ADD, ZG_OFFSET_L, raw_data[5]);
	
	//save biases for display or use in program
	dest1[0] = (float)gyro_bias[0]/(float)gyro_sensitivity;
	dest1[1] = (float)gyro_bias[1]/(float)gyro_sensitivity;
	dest1[2] = (float)gyro_bias[2]/(float)gyro_sensitivity;
	
	//construct the accelerometer biases for push to the hardware accelerometer bias registers
	int32_t accel_bias_reg[3] = {0};
	readBytes(IMU_ADD, XA_OFFSET_H, 2, &raw_data[0]); //read factory trim values
	accel_bias_reg[0] = (int32_t)(((int16_t)raw_data[0] << 8) | raw_data[1]); //read bytes into an int
	readBytes(IMU_ADD, YA_OFFSET_H, 2, &raw_data[0]); 
	accel_bias_reg[1] = (int32_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
	readBytes(IMU_ADD, ZA_OFFSET_H, 2, &raw_data[0]); //read factory trim values
	accel_bias_reg[2] = (int32_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
	
	uint32_t mask = 1uL; //unsigned long assigned for mask of temp comp, bit 0 of lower byte of accel bias registers
	uint8_t mask_bit[3] = {0};
	
	for(ii<0; ii<3; ii++){
		if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; //if temperature comp bit is set, record that fact in mask_bit
	}
	
	//construct total accelerometer bias, including calculated average accel bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); //subtract calculated avg bias scaled to 2048 LSB/g (16g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);
	
	raw_data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	raw_data[1] = (accel_bias_reg[0])      & 0xFF;
	raw_data[1] = raw_data[1] | mask_bit[0]; //preserve temp compensation bit when writing back to acc registers
	raw_data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	raw_data[3] = (accel_bias_reg[1])      & 0xFF;
	raw_data[3] = raw_data[3] | mask_bit[1];
	raw_data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	raw_data[5] = (accel_bias_reg[2])      & 0xFF;
	raw_data[5] = raw_data[5] | mask_bit[2];
	
	//some writing is here in the original code and noted to not be working so offsets never get pushed to the register
	
	//output scaled accelerometer values
	dest2[0] = (float)accel_bias[0]/(float)accel_sensitivity;
	dest2[1] = (float)accel_bias[1]/(float)accel_sensitivity;
	dest2[2] = (float)accel_bias[2]/(float)accel_sensitivity;
	
	ROS_INFO("gx_bias=%3f, gy_bias=%3f, gz_bias=%3f\n ax_bias=%3f, ay_bias=%3f, az_bias=%3f\n", dest1[0], dest1[1], dest1[2], dest2[0], dest2[1], dest2[2]);
}



//initialize mpu with original or configured settings
void adc::mpu9250::mpuInit(uint8_t g_scale, uint8_t a_scale, uint8_t a_rate){
	//general configuration for board	
	writeByte(IMU_ADD, PWR_MGMT_1, 0x00); //enable all sensors
	delay(100);
	writeByte(IMU_ADD, PWR_MGMT_1, 0x01); //2nd bit sets PLL (best clock source) 
	delay(200);
	writeByte(IMU_ADD, CONFIG, 0x03); //dlpf_cfg = bits 2:0 = 011(3) = 0x03
	writeByte(IMU_ADD, SMPLRT_DIV, a_rate); //set sample rate to 200 Hz 
	
	//configure gyro
	 //get desired resolution
	uint8_t c = readByte(IMU_ADD, GYRO_CONFIG); //get current configuration
	c = c & ~0x02; //clears Fchoice bits 00000011=0x02
	c = c & ~0x18; //clears full scale settings 00011000=0x18
	c = c | g_scale << 3; //set full scale range by shifting 3 bits left
	writeByte(IMU_ADD, GYRO_CONFIG, c);
	
	//configure accelerometer fs range
	//get desired resolution
	c = readByte(IMU_ADD, ACCEL_CONFIG); //get current accelerometer configuration
	c = c & ~0x18; //clear FS accel settings 
	c = c | a_scale << 3; //bitwise or to set new FS range
	writeByte(IMU_ADD, ACCEL_CONFIG,c);
	
	//set accelerometer sample rate config
	c = readByte(IMU_ADD, ACCEL_CONFIG2);
	c = c & ~0x0F; //clear lower 4 bits for accel_fchoiceb and a_dlpfg 
	c = c & 0x03; //set a_dlpfg to 3 for 41hz bandwidth and 1000kHz sample rate
	writeByte(IMU_ADD, ACCEL_CONFIG2, c);
	
	//set pins to high(00000010=0x10) and enable raw sensor data ready (bit 0 00000001=0x01) interrupt
	writeByte(IMU_ADD, INT_PIN_CFG, 0x10);
	writeByte(IMU_ADD, INT_ENABLE, 0x01);	
	delay(100);
	
	writeByte(IMU_ADD, USER_CTRL, 0x20); //enable I2C master mode
	writeByte(IMU_ADD, I2C_MST_CTRL, 0x1D); //bit 4 controls i2c master's transition from one slave to next if set to 1 there is a pause b/w reads
											 //if set to 0 there is a reset inbetween reads, dec 13=1101 for bits [3:0] 
	writeByte(IMU_ADD, I2C_MST_DELAY_CTRL, 0x81); //use blocking data retrieval and enable delay for mag sample rate mismatch
	writeByte(IMU_ADD, I2C_SLV4_CTRL, 0x01); //delay mag data retriveal to once every other accel/gyro data sample	
}	


//get accelorometer and gyroscope address
uint8_t adc::mpu9250::idMPU9250(){ 
	uint8_t address=readByte(IMU_ADD, 0x75);
	ROS_INFO("MPU9250 address = 0x%X\n", address);
	return address;
}



void adc::mpu9250::mpu9250ReadAll(std::vector<int16_t> &destination){
	
	uint8_t raw_data[14] = {0}; 
	readBytes(IMU_ADD, ACCEL_XOUT_H, 14, &raw_data[0]); //read the 14 raw data registers into a signed 16-bit value
	destination[0] = ((int16_t)raw_data[0]  << 8) | raw_data[1]; //acceleration
	destination[1] = ((int16_t)raw_data[2]  << 8) | raw_data[3]; 
	destination[2] = ((int16_t)raw_data[4]  << 8) | raw_data[5]; 
	destination[3] = ((int16_t)raw_data[6]  << 8) | raw_data[7]; //temperature
	destination[4] = ((int16_t)raw_data[8]  << 8) | raw_data[9]; //gyroscope 
	destination[5] = ((int16_t)raw_data[10] << 8) | raw_data[11]; 
	destination[6] = ((int16_t)raw_data[12] << 8) | raw_data[13]; 
}



int16_t adc::mpu9250::tempRead(){

	int16_t temp_raw; 
	float temp_c, temp_f;
	uint8_t raw_data[2];
	readBytes(IMU_ADD,TEMP_OUT_H,2,&raw_data[0]);
	temp_raw = ((int16_t)raw_data[0] << 8) | raw_data[1];
	return temp_raw;
}


	
void adc::mpu9250::accelRead(int16_t *destination){
		
		uint8_t raw_data[6] = {0};	
		//read xyz acceleration into array passed to function
		readBytes(IMU_ADD, ACCEL_XOUT_H, 6, &raw_data[0]); 
		destination[0] = ((int16_t)raw_data[0] << 8) | raw_data[1];
		destination[1] = ((int16_t)raw_data[2] << 8) | raw_data[3];
		destination[2] = ((int16_t)raw_data[4] << 8) | raw_data[5];
}



void adc::mpu9250::gyroRead(int16_t *destination){
	
	uint8_t gyro_raw[6]={0};
	//read x, y, z angular values -> read into gyro_raw starting at gyro_xout_h until all 6 bytes read`
	readBytes(IMU_ADD, GYRO_XOUT_H, 6, &gyro_raw[0]);
	destination[0]=((int16_t)gyro_raw[0] << 8) | gyro_raw[1];
	destination[1]=((int16_t)gyro_raw[2] << 8) | gyro_raw[3];
	destination[2]=((int16_t)gyro_raw[4] << 8) | gyro_raw[5];
}



/*........................................................................................................
					Access Magnetometer
........................................................................................................*/


//echo register address, should read 0x71
uint8_t adc::mpu9250::idAK8963(){
	writeByte(IMU_ADD, USER_CTRL, 0x20); //enable master mode, 0010 0000
	writeByte(IMU_ADD, I2C_MST_CTRL, 0x0D); //set multi-master i2c, 0000 1101
	writeByte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD | 0x80); //set slave address and set to read 0x80=i2c_slv_en, 0x81=enable slave and read one byte 1000 0001
	writeByte(IMU_ADD, I2C_SLV0_REG, AK8963_WHO_AM_I); //write magnetometer id register as place to begin data transfer
	writeByte(IMU_ADD, I2C_SLV0_CTRL, 0x81); //write i2c_slv0_en to enable reading data from slave (always EXT_SENS_DATA_00 for i2c slave 0) at desired sample rate. read one byte.
	delay(10);
	uint8_t address=readByte(IMU_ADD, EXT_SENS_DATA_00);//read data from slave on IMU
	ROS_INFO("AK8963 address = 0x%X\n", address);
	return address;
}



//initialize magnetometer
void adc::mpu9250::initAKslave(uint8_t m_scale, uint8_t m_rate, float *mag_calibration){

	uint8_t raw_data[3];
	writeByte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD);//set the i2c address to ak8963 and set for write
	writeByte(IMU_ADD, I2C_SLV0_REG, AK_CNTL2); //i2c slave 0 register from where to begin data transfer
	writeByte(IMU_ADD, I2C_SLV0_DO, 0X01); //reset ak8963
	writeByte(IMU_ADD, I2C_SLV0_CTRL, 0x81); //enable i2c and write one byte
	delay(50);
	
	writeByte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD);//set the i2c address to ak8963 and set for write
	writeByte(IMU_ADD, I2C_SLV0_REG, AK_CNTL); //i2c slave 0 register from where to begin data transfer
	writeByte(IMU_ADD, I2C_SLV0_DO, 0x00); //power dowm magnetometer
	writeByte(IMU_ADD, I2C_SLV0_CTRL, 0x81); //enable i2c and write one byte
	delay(50);	
	
	writeByte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD);//set the i2c address to ak8963 and set for write
	writeByte(IMU_ADD, I2C_SLV0_REG, AK_CNTL); //i2c slave 0 register from where to begin data transfer
	writeByte(IMU_ADD, I2C_SLV0_DO, 0x0F); //enter fuse mode
	writeByte(IMU_ADD, I2C_SLV0_CTRL, 0x81); //enable i2c and write one byte
	delay(50);
	
	writeByte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD | 0x80); //set i2c slave address and set for read
	writeByte(IMU_ADD, I2C_SLV0_REG, AK_ASAX); //i2c slave register from where to begin data transfer
	writeByte(IMU_ADD, I2C_SLV0_CTRL, 0x83); //enable data transfer and write 3 bytes
	delay(50);
	readBytes(IMU_ADD, EXT_SENS_DATA_00, 3, &raw_data[0]); // read xyz calibration values
	mag_calibration[0] = (float)(raw_data[0]-128)/256.0f + 1.0f; //return x_axis sensitivity adjustment values 
	mag_calibration[1] = (float)(raw_data[1]-128)/256.0f + 1.0f;
	mag_calibration[2] = (float)(raw_data[2]-128)/256.0f + 1.0f;
	_mag_calibration[0] = mag_calibration[0]; //saves input variable as private class variable for use in program
	_mag_calibration[1] = mag_calibration[1];
	_mag_calibration[2] = mag_calibration[2];
	_m_rate = m_rate;

	writeByte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD);//set the i2c address to ak8963 and set for write
	writeByte(IMU_ADD, I2C_SLV0_REG, AK_CNTL);   //i2c slave 0 register from where to begin data transfer
	writeByte(IMU_ADD, I2C_SLV0_DO, 0x00);       //power_down magnetometer
	writeByte(IMU_ADD, I2C_SLV0_CTRL, 0x81);     //enable i2c and write one byte
	delay(50);
	
	writeByte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD);//set the i2c address to ak8963 and set for write
	writeByte(IMU_ADD, I2C_SLV0_REG, AK_CNTL);   //i2c slave 0 register from where to begin data transfer
	
	//configure for continuous read at highest resolution 
	writeByte(IMU_ADD, I2C_SLV0_DO, m_scale << 4 | m_rate); //set magnetometer data res and sample odr
	writeByte(IMU_ADD, I2C_SLV0_CTRL, 0x81); 				 //enable i2c and transfer 1 byte
	delay(50);
	
	writeByte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD | 0x80);//set i2c slave address of AK and set for read
	writeByte(IMU_ADD, I2C_SLV0_REG, AK_CNTL);       //i2c slave 0 register address from where to begin data transfer
	writeByte(IMU_ADD, I2C_SLV0_CTRL, 0x81);             //enable i2c and transfer 1 byte
	delay(50);
}



//calibrate AK8963 magnetometer 
void adc::mpu9250::calibrateMag(float *dest1, float *dest2){

	uint16_t ii = 0, sample_count = 0; 
	int32_t mag_bias[3] = {0}, mag_scale[3] = {0};
	int16_t mag_max[3] = {-32767, -32676, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
	
	ROS_INFO("Mag Calibration: Wave device in a figure eight until done!");
	delay(4000);
	
	//aiming for ~15 seconds of data
	if(_m_rate == 0x02) sample_count = 128;
	if(_m_rate == 0x06) sample_count = 1500; 
	//ROS_INFO("m_rate=0x0%f", _m_rate);
	for(ii = 0; ii<sample_count; ii++){	
		magRead(mag_temp); 			//read mag data
		for(int jj = 0; jj < 3; jj++){  //split into sets of 3 (x,y,z)
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj]; 	
		}
		if(_m_rate == 0x02) delay(135); //at 8hz ODR, new mag data avail every 125 ms
		if(_m_rate == 0x06) delay(12);  //at 100hz ODR, new mag data avail every 10 ms
	}
	
	/*ROS_INFO("mag x max=%d and min=%d\n", mag_max[0], mag_min[0]);
	ROS_INFO("mag y max=%d and min=%d\n", mag_max[1], mag_min[1]);
	ROS_INFO("mag z max=%d and min=%d\n", mag_max[2], mag_min[2]);*/
	
	//get hard iron correction
	mag_bias[0] = (mag_max[0] + mag_min[0])/2; //get average x mag bias in counts 
	mag_bias[1] = (mag_max[1] + mag_min[1])/2; //get average y mag bias in counts 
	mag_bias[2] = (mag_max[2] + mag_min[2])/2; //get average z mag bias in counts 
	//save mag biases in G for main program
	dest1[0] = (float)mag_bias[0]*_m_res*_mag_calibration[0];
	dest1[1] = (float)mag_bias[1]*_m_res*_mag_calibration[1];
	dest1[2] = (float)mag_bias[2]*_m_res*_mag_calibration[2];
	ROS_INFO("mag bias=%3f, m_res=%3f, mag_calibration=%3f", float(mag_bias[0]), _m_res, _mag_calibration[0]);
	for(int i = 0 ; i<sizeof(dest1); i++ ){
		ROS_INFO("mag biases at i=%d are x=%3f, y=%3f, z=%3f \n", i, dest1[0], dest1[1], dest1[2]);
	}
	
	//get soft iron correction estimate
	mag_scale[0] = (mag_max[0] - mag_min[0])/2; //get average x axis max chord length in counts
	mag_scale[1] = (mag_max[1] - mag_min[1])/2; //get average y axis max chord length in counts
	mag_scale[2] = (mag_max[2] - mag_min[2])/2; //get average z axis max chord length in counts
	
	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2]; 
	avg_rad /= 3.0;
	
	dest2[0] = avg_rad/((float)mag_scale[0]);
	dest2[1] = avg_rad/((float)mag_scale[1]);
	dest2[2] = avg_rad/((float)mag_scale[2]);	
	for(int i = 0 ; i<sizeof(dest2); i++ ){
		ROS_INFO("mag scales at i=%d are x=%3f, y=%3f, z=%3f \n", i, dest2[0],  dest2[1],  dest2[2]);
	}
}



//read from the magnetometer
void adc::mpu9250::magRead(int16_t * destination){

	uint8_t mag_raw[7];
	//writeByte(IMU_ADD, I2C_SLV0_ADD, 0x00);//set slave address to read
	writeByte(IMU_ADD, I2C_SLV0_ADD, AK8963_ADD | 0x80);//set slave address to read
	writeByte(IMU_ADD, I2C_SLV0_REG, AK_HXL);	
	writeByte(IMU_ADD, I2C_SLV0_CTRL, 0x87);
	delay(2);
	readBytes(IMU_ADD, EXT_SENS_DATA_00, 7, &mag_raw[0]);
	
	uint8_t c = mag_raw[6]; //end data transfer by reading St2 register
	if(!(c & 0x08)){   //check for magnetic overflow is set otherwise read in values
		destination[0]=((int16_t)mag_raw[1] << 8) | mag_raw[0]; //read high byte onto low byte
		destination[1]=((int16_t)mag_raw[3] << 8) | mag_raw[2];
		destination[2]=((int16_t)mag_raw[5] << 8) | mag_raw[4];
		/*for(int i = 0; i<3; i++){
			ROS_INFO("raw readings from Magnetometer at i=%d: %d", i, destination[i]);
		}*/
	}
	
}



/*........................................................................................................
    Generic Functions / Reading and Writing to Jetson Nano using linux/i2c-dev.h and i2c/smbus.h
........................................................................................................*/


uint8_t adc::mpu9250::readByte(uint8_t dev_add, uint8_t reg_add){
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

uint8_t adc::mpu9250::readBytes(uint8_t dev_add, uint8_t reg_add, uint8_t num_bytes, uint8_t * buf){
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
		//printf("multiple bytes read = %X\n\n", response[i]);	
		buf[i]=response[i];
	}
	//tcflush(fd, TCIOFLUSH);
	return 0;
}

uint8_t adc::mpu9250::writeByte(uint8_t dev_add, uint8_t reg_add, uint8_t data){
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

//pauses program for designated time in ms - made to correspond with arduino and help readability		
void adc::mpu9250::delay(auto time_ms){ 
	usleep(time_ms*1000);
}

/*void adc::mpu9250::low_pass(std::vector<float> &imu_data){
	float alpha = 0.15
	std::vector<float> lp_data(imu_data.size());  
	for(int i = 0; i<imu_data.size(); ++i){
		lp_data[i] = (1-alpha)*lp_data[i]+alpha*imu_data[i];
		ROS_INFO("filtered value at i=%d is %3f\n", lp_data[i]);
	}
}*/



































