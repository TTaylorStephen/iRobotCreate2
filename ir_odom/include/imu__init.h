#ifndef imu__init_h
#define imu__init_h

#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <fcntl.h>
#include <iostream>
#include <sys/ioctl.h>
#include <unistd.h>
#include <termios.h>
#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include <math.h>

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
#define AK_CTRL 0x0A
#define AK_CTRL2 0x0B 
#define AK_ASAX 0x10
#define AK_ASAY 0x11
#define AK_ASAZ 0x12
#define SMPLRT_DIV 0x19

#define GRAVITY 9.80665
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


class mpu9255{

	public:
		mpu9255();
		int imu_init();
		uint8_t read_byte(uint8_t dev_add, uint8_t reg_add);
		float accel_offset(float accel, float size);
		int accel_read();
		int gyro_read();
		float avg(float ang_accel);
		float RMS(float data);
		uint8_t who_am_i();
		uint8_t write_byte(uint8_t dev_add, uint8_t reg_add, uint8_t data);
		uint8_t who_am_i_ak8963();
		float set_m_res(uint8_t mag_scale);
		uint8_t read_bytes(uint8_t dev_add, uint8_t num_bytes, uint8_t reg_add, uint8_t * buf);
		int mag_read();
		void init_ak();
		int mpu_init(uint8_t g_scale, uint8_t a_scale);
		void init_ak_slave();
		float temp_read();
		void calibrate_mpu();
		uint8_t g_scale;
		uint8_t a_scale;

	private:
		ros::NodeHandle nh;
		ros::Publisher accel_data;


};

#endif
