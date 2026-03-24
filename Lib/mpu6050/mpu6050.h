#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

// MPU6050 I2C Address (AD0 low)
#define MPU6050_ADDR 0xD0

// MPU6050 Registers
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_WHO_AM_I     0x75

#define MPU6050_ACCEL_XOUT_H 0x3B

typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    float Ax;
    float Ay;
    float Az;
    float Gx;
    float Gy;
    float Gz;
} MPU6050_t;

extern volatile uint8_t mpu6050_data_ready;
extern volatile MPU6050_t mpu6050;

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Start_Read(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);
void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);

#ifdef __cplusplus
}
#endif

#endif
