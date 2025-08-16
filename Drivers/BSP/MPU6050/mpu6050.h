/*
 * mpu6050.h
 *
 *  Created on: Aug 10, 2025
 *      Author: MXQ11
 */

#ifndef BSP_MPU6050_MPU6050_H_
#define BSP_MPU6050_MPU6050_H_

#define MPU6050_Location	0xD0				//从机地址

#include "../../Core/Inc/main.h"
#include "../../Core/Inc/i2c.h"

void MPU6050_WriteReg(uint8_t Reg, uint8_t value);
uint8_t MPU6050_ReadReg(uint8_t Reg);
void MPU6050_Init();
void MPU6050_UpdateData();
void MPU6050_ZeroOffsetCalibrated();
void MPU6050_GetEularAngle();
float MPU6050_GetAx();
float MPU6050_GetAy();
float MPU6050_GetAz();
float MPU6050_GetGx();
float MPU6050_GetGy();
float MPU6050_GetGz();
float MPU6050_GetYaw();
float MPU6050_GetPitch();
float MPU6050_GetRoll();

#endif /* BSP_MPU6050_MPU6050_H_ */
