#ifndef MPU9250_H
#define MPU9250_H


#include <math.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <termios.h>

#include <iostream>
#include <vector>

#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "std_msgs/Float32MultiArray.h"
#include <ir_odom/Euler.h>
#include <ir_odom/Magnet.h>

#define BAUD_RATE 115200
#define IMU_ADD 0x68 //MPU9250 register address
#define IMU_WHO_AM_I 0x75

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define CONFIG 0x1A
#define USER_CTRL 0x6A
#define FIFO_EN 0x23
#define FIFO_COUNTH 0x72
#define FIFO_R_W 0x74
#define I2C_MST_CTRL 0x24
#define I2C_SLV0_ADD 0x25 
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define I2C_SLV4_CTRL 0x52
#define I2C_SLV0_DO 0x63
#define I2C_MST_DELAY_CTRL 0x67
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B

#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42

#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define ACCEL_XOUT_H 0x3B //x accel high byte
#define ACCEL_XOUT_L 0x3C //x accel low byte
#define ACCEL_YOUT_H 0x3D //y accel high byte
#define ACCEL_YOUT_L 0x3E //y accel low byte
#define ACCEL_ZOUT_H 0x3F //z accel high byte
#define ACCEL_ZOUT_L 0x40 //z accel low byte

#define XA_OFFSET_H 0x77
#define XA_OFFSET_L 0x78
#define YA_OFFSET_H 0x7A
#define YA_OFFSET_L 0x7B
#define ZA_OFFSET_H 0x7D
#define ZA_OFFSET_L 0x7E

#define XG_OFFSET_H 0x13
#define XG_OFFSET_L 0x14
#define YG_OFFSET_H 0x15
#define YG_OFFSET_L 0x16
#define ZG_OFFSET_H 0x17
#define ZG_OFFSET_L 0x18

#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43 //gyro high byte
#define GYRO_XOUT_L 0x44 //gyro low byte
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define AK8963_ADD 0x0C
#define AK8963_WHO_AM_I 0x00
#define AK_HXL 0x03
#define AK_HXH 0x04
#define AK_HYL 0x05
#define AK_HYH 0x06
#define AK_HZL 0x07
#define AK_HZH 0x08
#define AK_ST2 0x09
#define AK_CNTL 0x0A
#define AK_CNTL2 0x0B 
#define AK_ASAX 0x10
#define AK_ASAY 0x11
#define AK_ASAZ 0x12
#define SMPLRT_DIV 0x19


#define AFS_2G 0
#define AFS_4G 1
#define AFS_8G 2
#define AFS_16G 3
#define GFS_250DPS 0
#define GFS_500DPS 1
#define GFS_1000DPS 2
#define GFS_2000DPS 3
#define MFS_14BITS 0
#define MFS_16BITS 1

#define GRAVITY 9.80665
#define PI 3.14159265359


namespace adc{

	class mpu9250{

		public:
			mpu9250();
			uint8_t idMPU9250();
			uint8_t idAK8963();
			int imuInit();
			int fd;
			
			float getMres(uint8_t mscale);
			float getAres(uint8_t ascale);
			float getGres(uint8_t gscale);
			
			int16_t tempRead();	
			void mpu9250ReadAll(std::vector<int16_t> &destination);
			void accelRead(int16_t *destination);
			void gyroRead(int16_t *destination);	
			void magRead(int16_t *destination);
			void readAll();
			
			
			void mpuInit(uint8_t g_scale, uint8_t a_scale, uint8_t a_rate);
			void initAKslave(uint8_t m_scale, uint8_t m_rate, float * mag_calibration);

			void calibrateMPU(float *dest1, float *dest2);
			void calibrateMag(float *dest1, float *dest2);		
			void reset();

			uint8_t readByte(uint8_t dev_add, uint8_t reg_add);
			uint8_t readBytes(uint8_t dev_add, uint8_t num_bytes, uint8_t reg_add, uint8_t * buf);
			uint8_t writeByte(uint8_t dev_add, uint8_t reg_add, uint8_t data);
			void delay(auto time_ms);
			 __attribute__((optimize("O3"))) void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float del_t, float * q);
			
			//float getTime();
			void convert();
			float avg(float ang_accel);
			float RMS(float data);	
			  
		private:
			ros::NodeHandle nh;
			ros::Publisher imu_pub, accel_pub, ang_pub, mag_pub;
			float _mag_calibration[3];
			float _a_res, _g_res, _m_res;
			float _m_rate;
			float _a_mode;
			int _fd;
	};
}

#endif
