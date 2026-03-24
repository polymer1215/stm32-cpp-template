#include "mpu6050.h"
#include "i2c.h"
#include <math.h>

volatile uint8_t mpu6050_data_ready = 0;
static uint8_t mpu6050_buffer[14];
volatile MPU6050_t mpu6050;

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check;
    uint8_t Data;

    // Check device ID
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &check, 1, 1000);

    if (check == 0x68) {
        // Wake up MPU6050
        Data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &Data, 1, 1000);

        // Set Data Rate 1kHz
        Data = 0x07;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, &Data, 1, 1000);

        // Set Accelerometer Configuration (+/- 2g)
        Data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &Data, 1, 1000);

        // Set Gyroscope Configuration (+/- 250 dps)
        Data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &Data, 1, 1000);
        
        return 0; // OK
    }
    return 1; // Error
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, Rec_Data, 6, 1000);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Convert raw values to g (assuming +/- 2g range)
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0f;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0f;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0f;


}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register (0x43)
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 1000);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // Convert raw values to dps (assuming +/- 250 dps range)
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0f;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0f;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0f;
}

void MPU6050_Start_Read(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_Mem_Read_IT(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, mpu6050_buffer, 14);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        mpu6050.Accel_X_RAW = (int16_t)(mpu6050_buffer[0] << 8 | mpu6050_buffer[1]);
        mpu6050.Accel_Y_RAW = (int16_t)(mpu6050_buffer[2] << 8 | mpu6050_buffer[3]);
        mpu6050.Accel_Z_RAW = (int16_t)(mpu6050_buffer[4] << 8 | mpu6050_buffer[5]);
        
        mpu6050.Gyro_X_RAW = (int16_t)(mpu6050_buffer[8] << 8 | mpu6050_buffer[9]);
        mpu6050.Gyro_Y_RAW = (int16_t)(mpu6050_buffer[10] << 8 | mpu6050_buffer[11]);
        mpu6050.Gyro_Z_RAW = (int16_t)(mpu6050_buffer[12] << 8 | mpu6050_buffer[13]);

        mpu6050.Ax = mpu6050.Accel_X_RAW / 16384.0f;
        mpu6050.Ay = mpu6050.Accel_Y_RAW / 16384.0f;
        mpu6050.Az = mpu6050.Accel_Z_RAW / 16384.0f;

        mpu6050.Gx = mpu6050.Gyro_X_RAW / 131.0f;
        mpu6050.Gy = mpu6050.Gyro_Y_RAW / 131.0f;
        mpu6050.Gz = mpu6050.Gyro_Z_RAW / 131.0f;
        
        mpu6050_data_ready = 1;
    }
}
