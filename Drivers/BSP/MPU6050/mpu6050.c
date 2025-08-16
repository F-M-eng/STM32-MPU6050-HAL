/*
 * mpu6050.c
 *
 *  Created on: Aug 10, 2025
 *      Author: MXQ11
 */

#include "mpu6050.h"
#include "math.h"

static float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;			//MPU6050计算结果
static float yaw_a = 0, roll_a = 0, pitch_a = 0, yaw_g = 0, roll_g = 0, pitch_g = 0;
static float gx_offset = 0, gy_offset = 0, gz_offset = 0;				//零偏校准结果
static float yaw = 0, roll = 0, pitch = 0;
uint32_t timer;


/*
 * @brief 在指定寄存器写数据
 * @param Reg:寄存器地址
 * @param value:写入的值
 * @retval None
 * */
void MPU6050_WriteReg(uint8_t Reg, uint8_t value)
{
	uint8_t SendData[] = {Reg, value};
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_Location, SendData, sizeof(SendData)/sizeof(SendData[0]), HAL_MAX_DELAY);
}

/*
 * @brief 在指定寄存器读数据
 * @param Reg:寄存器地址
 * @retval 读取结果
 * */
uint8_t MPU6050_ReadReg(uint8_t Reg)
{
	uint8_t Value;

	//发送要读取的寄存器地址
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_Location, &Reg, 1, HAL_MAX_DELAY);

	//读取寄存器的值
	HAL_I2C_Master_Receive(&hi2c1, MPU6050_Location, &Value, 1, HAL_MAX_DELAY);

	return Value;
}



/*
 * @brief MPU6050初始化
 * @param None
 * @retval None
 * */
void MPU6050_Init()
{
	MPU6050_WriteReg(0x6b, 0x80);		//复位
	HAL_Delay(100);

	MPU6050_WriteReg(0x6B, 0x00);		//关闭睡眠模式

	MPU6050_WriteReg(0x1B, 0x10);		//陀螺仪量程，+-1000°/s
	MPU6050_WriteReg(0x1C, 0x00);		//加速度计量程，+-2g
}

/*
 * @brief 更新MPU6050数据
 * @param None
 * @retval None
 * */
void MPU6050_UpdateData()
{
	uint8_t ax_h = MPU6050_ReadReg(0x3B);			//获取ax高8位
	uint8_t ax_l = MPU6050_ReadReg(0x3C);			//获取ax低8位
	int16_t ax_raw = (ax_h << 8) + ax_l;			//计算ax原始值
	ax = ax_raw / 16384.0f;							//计算ax最终结果

	uint8_t ay_h = MPU6050_ReadReg(0x3D);			//获取ay高8位
	uint8_t ay_l = MPU6050_ReadReg(0x3E);			//获取ay低8位
	int16_t ay_raw = (ay_h << 8) + ay_l;			//计算ay原始值
	ay = ay_raw / 16384.0f;

	uint8_t az_h = MPU6050_ReadReg(0x3F);			//获取az高8位
	uint8_t az_l = MPU6050_ReadReg(0x40);			//获取az低8位
	int16_t az_raw = (az_h << 8) + az_l;			//计算az原始值
	az = az_raw / 16384.0f;

	uint8_t gx_h = MPU6050_ReadReg(0x43);			//获取gx高8位
	uint8_t gx_l = MPU6050_ReadReg(0x44);			//获取gx低8位
	int16_t gx_raw = (gx_h << 8) + gx_l;			//计算gx原始值
	gx = gx_raw / 32.8f;

	uint8_t gy_h = MPU6050_ReadReg(0x45);			//获取gy高8位
	uint8_t gy_l = MPU6050_ReadReg(0x46);			//获取gy低8位
	int16_t gy_raw = (gy_h << 8) + gy_l;			//计算gy原始值
	gy = gy_raw / 32.8f;

	uint8_t gz_h = MPU6050_ReadReg(0x47);			//获取gz高8位
	uint8_t gz_l = MPU6050_ReadReg(0x48);			//获取gz低8位
	int16_t gz_raw = (gz_h << 8) + gz_l;			//计算gz原始值
	gz = gz_raw / 32.8f;
}

void MPU6050_ZeroOffsetCalibrated()
{
	// 陀螺仪零偏校准
	float gx_sum = 0, gy_sum = 0, gz_sum = 0;
	const int samples = 200;

	for(int i = 0; i < samples; i++)
	{
		MPU6050_UpdateData();
		gx_sum += gx;
		gy_sum += gy;
		gz_sum += gz;
		HAL_Delay(5);
	}

	gx_offset = gx_sum / samples;
	gy_offset = gy_sum / samples;
	gz_offset = gz_sum / samples;
}

/*
 * @brief 计算MPU6050 欧拉角
 * @param None
 * @retval None
 * */
void MPU6050_GetEularAngle()
{
	static uint8_t first_call = 1;
	if (first_call)
	{
		timer = HAL_GetTick();
		first_call = 0;
	  	return; // 第一次调用更新timer的值，不计算，跳过
	}

	float dt = (float)(HAL_GetTick() - timer) / 1000;
	timer = HAL_GetTick();

	MPU6050_UpdateData();

	// 应用零偏校准
	float calibrated_gx = gx - gx_offset;
	float calibrated_gy = gy - gy_offset;
	float calibrated_gz = gz - gz_offset;

	float a = 0.95238;

	//通过陀螺仪（角加速度计）计算欧拉角
	yaw_g = yaw_g + calibrated_gz * dt;
	pitch_g = pitch_g + calibrated_gx * dt;
	roll_g = roll_g + calibrated_gy * dt;

	//通过加速度计计算欧拉角
	yaw_a = 0;
	pitch_a = atan2(ay,az) / 3.14159 * 180.0f;
	roll_a = - atan2(ax,az) / 3.14159 * 180.0f;

	//互补滤波
	yaw = yaw_g;
	pitch = a * pitch_g+ (1 - a) * pitch_a;
	roll = a * roll_g + (1-a) * roll_a;
}

/*
 * @brief 获取MPU6050 Ax数据
 * @param None
 * @retval Ax
 * */
float MPU6050_GetAx()
{
	return ax;
}

/*
 * @brief 获取MPU6050 Ay数据
 * @param None
 * @retval Ay
 * */
float MPU6050_GetAy()
{
	return ay;
}

/*
 * @brief 获取MPU6050 Az数据
 * @param None
 * @retval Az
 * */
float MPU6050_GetAz()
{
	return az;
}

/*
 * @brief 获取MPU6050 Gx数据
 * @param None
 * @retval Gx
 * */
float MPU6050_GetGx()
{
	return gx;
}

/*
 * @brief 获取MPU6050 Gy数据
 * @param None
 * @retval Gy
 * */
float MPU6050_GetGy()
{
	return gy;
}

/*
 * @brief 获取MPU6050 Gz数据
 * @param None
 * @retval Gz
 * */
float MPU6050_GetGz()
{
	return gz;
}


/*
 * @brief 获取MPU6050 Yaw数据
 * @param None
 * @retval Yaw
 * */
float MPU6050_GetYaw()
{
	return yaw;
}

/*
 * @brief 获取MPU6050 Pitch数据
 * @param None
 * @retval Pitch
 * */
float MPU6050_GetPitch()
{
	return pitch;
}

/*
 * @brief 获取MPU6050 Roll数据
 * @param None
 * @retval Roll
 * */
float MPU6050_GetRoll()
{
	return roll;
}
