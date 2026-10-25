
//#include "main.h"
#include "stm32f3xx_hal.h"

#define MPU6050_ADDR 0xD0


#define MPU6050_SMPRT_DIV	0X19
#define MPU6050_CONFIG		0X1A
#define MPU6050_GYRO_CONFIG	0X1B
#define MPU6050_ACCEL_CONFIG	0X1C
#define MPU6050_INT_PIN_CFG	0X37
#define MPU6050_INT_ENABLE	0X38
#define MPU6050_INT_STATUS	0X3A
#define MPU6050_ACCEL_XOUT_H	0X3B
#define MPU6050_ACCEL_XOUT_L	0X3C
#define MPU6050_PWR_MGMT_1	0X6B //most important
#define MPU6050_PWR_MGMT_2	0X6C
#define MPU6050_WHO_AM_I	0X75

#define DLPF_44HZ_42HZ		0x03
#define SMPLRT_100		0x09
#define DEVICE_RESET		0x80
#define SLEEP_OFF		0x00
#define SLEEP_ON		0x40
#define TEMP_DIS		0x08
#define CLKSEL_PLLX		0x01
#define DATA_RDY_INT		0x01
#define GYRO_DIS		0x07

#define MPU6050_INT_PORT 	GPIOB
#define MPU6050_INT_PIN 	GPIO_PIN_5


typedef struct _MPU6050{
	short acc_x_raw;
	short acc_y_raw;
	short acc_z_raw;
	short temperature_raw;
	short gyro_x_raw;
	short gyro_y_raw;
	short gyro_z_raw;

	float acc_x;
	float acc_y;
	float acc_z;
	float temperature;
	float gyro_x;
	float gyro_y;
	float gyro_z;
}Struct_MPU6050;

extern Struct_MPU6050 MPU6050;

void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val);
void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data);
void MPU6050_Readbyte(uint8_t reg_addr, uint8_t* data);
void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data);
void MPU6050_Initialization(void);
void MPU6050_Get6AxisRawData(Struct_MPU6050* mpu6050);
int MPU6050_DataReady(void);
void MPU6050_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC);
void MPU6050_DataConvert(Struct_MPU6050* mpu6050);
void MPU6050_ProcessData(Struct_MPU6050* mpu6050);

