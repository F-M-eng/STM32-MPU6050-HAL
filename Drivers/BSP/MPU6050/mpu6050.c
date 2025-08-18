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
static float complemantary_yaw = 0, complemantary_roll = 0, complemantary_pitch = 0;	//互补滤波结果
static float kalman_yaw, kalman_roll, kalman_pitch;										//卡尔曼滤波结果
static float DMP_yaw, DMP_roll, DMP_pitch;												//DMP四元数计算结果

float dt;				//采样时间
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
#if GetEularAngle !=2

	MPU6050_WriteReg(0x6b, 0x80);		//复位
	HAL_Delay(100);

	MPU6050_WriteReg(0x6B, 0x00);		//关闭睡眠模式

	MPU6050_WriteReg(0x1B, 0x10);		//陀螺仪量程，+-1000°/s
	MPU6050_WriteReg(0x1C, 0x00);		//加速度计量程，+-2g

#else
	MPU6050_DMP_init();

#endif
}


/*
 * @brief 更新MPU6050数据
 * @param None
 * @retval None
 * */
void MPU6050_UpdateData()
{
#if GetEularAngle !=2

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

#endif
}


void MPU6050_ZeroOffsetCalibrated()
{
#if GetEularAngle !=2

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

#endif
}


/*
 * @brief 计算MPU6050 欧拉角
 * @param None
 * @retval None
 * */
void MPU6050_GetEularAngle()
{
#if GetEularAngle != 2

	static uint8_t first_call = 1;
	if (first_call)
	{
		timer = HAL_GetTick();
		first_call = 0;
	  	return; // 第一次调用更新timer的值，不计算，跳过
	}

	MPU6050_UpdateData();

	dt = (float)(HAL_GetTick() - timer) / 1000.0f;
	timer = HAL_GetTick();

	//通过加速度计计算欧拉角
	yaw_a = 0;
	pitch_a = atan2(ay,az) / 3.14159 * 180.0f;
	roll_a = - atan2(ax,az) / 3.14159 * 180.0f;

	// 应用零偏校准
	float calibrated_gx = gx - gx_offset;
	float calibrated_gy = gy - gy_offset;
	float calibrated_gz = gz - gz_offset;

	//通过陀螺仪（角加速度计）计算欧拉角
	yaw_g = yaw_g + calibrated_gz * dt;
	pitch_g = pitch_g + calibrated_gx * dt;
	roll_g = roll_g + calibrated_gy * dt;

  #if GetEularAngle == 0

	float a = 0.95238;

	complemantary_yaw = yaw_g;
	complemantary_roll = Complementary_Filter(roll_a, roll_g);
	complemantary_pitch = Complementary_Filter(pitch_a, pitch_g);

  #endif

  #if GetEularAngle == 1

	kalman_pitch = Kalman_Filter_x(pitch_a, calibrated_gx);
	kalman_roll = -Kalman_Filter_y(roll_a, calibrated_gy);
	kalman_yaw = yaw_g;

  #endif

#else GetEularAngle == 2

	MPU6050_DMP_Get_Date(&DMP_pitch, &DMP_roll, &DMP_yaw);

#endif

}


/*
 * @brief 获取MPU6050 Ax数据
 * @param None
 * @retval Ax
 * */
float MPU6050_GetAx()
{
#if GetEularAngle !=2
	return ax;
#endif
}

/*
 * @brief 获取MPU6050 Ay数据
 * @param None
 * @retval Ay
 * */
float MPU6050_GetAy()
{
#if GetEularAngle !=2
	return ay;
#endif
}

/*
 * @brief 获取MPU6050 Az数据
 * @param None
 * @retval Az
 * */
float MPU6050_GetAz()
{
#if GetEularAngle !=2
	return az;
#endif
}

/*
 * @brief 获取MPU6050 Gx数据
 * @param None
 * @retval Gx
 * */
float MPU6050_GetGx()
{
#if GetEularAngle !=2
	return gx;
#endif
}

/*
 * @brief 获取MPU6050 Gy数据
 * @param None
 * @retval Gy
 * */
float MPU6050_GetGy()
{
#if GetEularAngle !=2
	return gy;
#endif
}

/*
 * @brief 获取MPU6050 Gz数据
 * @param None
 * @retval Gz
 * */
float MPU6050_GetGz()
{
#if GetEularAngle !=2
	return gz;
#endif
}


/*
 * @brief 获取MPU6050 Yaw数据
 * @param None
 * @retval Yaw
 * */
float MPU6050_GetYaw()
{
#if GetEularAngle == 0
	return complemantary_yaw;
#endif

#if GetEularAngle == 1
	return kalman_yaw;
#endif

#if GetEularAngle == 2
	return DMP_yaw;
#endif
}

/*
 * @brief 获取MPU6050 Pitch数据
 * @param None
 * @retval Pitch
 * */
float MPU6050_GetPitch()
{
#if GetEularAngle == 0
	return complemantary_pitch;
#endif

#if GetEularAngle == 1
	return kalman_pitch;
#endif

#if GetEularAngle == 2
	return DMP_pitch;
#endif

}

/*
 * @brief 获取MPU6050 Roll数据
 * @param None
 * @retval Roll
 * */
float MPU6050_GetRoll()
{
#if GetEularAngle == 0
	return complemantary_roll;
#endif

#if GetEularAngle == 1
	return kalman_roll;
#endif

#if GetEularAngle == 2
	return DMP_roll;
#endif
}

#if GetEularAngle == 0
/*
 * @brief 一阶互补滤波计算角度
 * @param Angle_a：加速度计计算获取的角度
 * @param Gyro：陀螺仪获取的角速度
 * @retval None
 * */
float Complementary_Filter(float Angle_a, float Angle_g)
{
	float a = 0.95238;
	return a * Angle_g+ (1 - a) * Angle_a;
}

#endif

#if GetEularAngle == 1
/*
 * @brief x轴卡尔曼滤波计算角度
 * @param Angle_a：加速度计计算获取的角度
 * @param Gyro：陀螺仪获取的角速度
 * @retval None
 * */
float Kalman_Filter_x(float Angle_a,float Gyro)
{
	static float angle_dot;
	static float angle;
	float Q_angle=0.001; // 过程噪声的协方差
	float Q_gyro=0.003;	//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
	float R_angle=3;		// 测量噪声的协方差 既测量偏差
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err = Angle_a - angle;	//zk-先验估计

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
	return angle;
}

/*
 * @brief y轴卡尔曼滤波计算角度
 * @param Angle_a：加速度计计算获取的角度
 * @param Gyro：陀螺仪获取的角速度
 * @retval None
 * */
float Kalman_Filter_y(float Angle_a,float Gyro)
{
	static float angle_dot;
	static float angle;
	float Q_angle=0.001; // 过程噪声的协方差
	float Q_gyro=0.003;	//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
	float R_angle=3;		// 测量噪声的协方差 既测量偏差
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err = Angle_a - angle;	//zk-先验估计

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
	return angle;
}
#endif

//计算量相对较大的卡尔曼滤波程序，不适合单片机使用，仅供参考学习
/*
	//通过加速度计计算欧拉角
	yaw_a = 0;
	pitch_a = atan2(ay,az) / 3.14159 * 180.0f;
	roll_a = - atan2(ax,az) / 3.14159 * 180.0f;

//****************************对roll进行卡尔曼滤波*********************************
	//参数初始化
	//初始时刻最优估计值
	kalman_roll = 0;				//估计角度
	static float bias_roll = 0;					//估计偏差
	//预测噪声方差
	static float Q_angle_roll = 0.02;				//预测角度方差 = 预测角度协方差
	static float Q_groy_roll = 0.01;				//预测偏差方差 = 预测偏差协方差
	//测量噪声方差
	static float R_angle_roll = 10;
	//预测协方差矩阵，初始值为单位阵
	static float PP_roll[2][2] = {{1, 0},{0, 1}};
	//测量值
	static float measure_roll = 0;
	//卡尔曼增益
	static float K_0_roll = 0;
	static float K_1_roll = 0;


	//------------进行先验估计运算--------------
	//公式：X(k|k-1) = F*X(k-1|k-1) + B*U(k) + w(k)
	//		当前先验结果 = 状态矩阵*上一时刻后验结果 + 控制矩阵 * 当前时刻控制输入 + 预测噪音
	//X = [angle_roll] 		F = [1, dt]		B = [dt]
	//	  [bias_roll ]			[0, 1 ]			[0 ]
	//angle_roll = angle_roll - dt * bias_roll + gy * dt	当前角度 = 上一时刻角度 - 偏差(bias_roll * dt) + 增加角度(gy * dt)
	//bias_roll = bias_roll
	kalman_roll = kalman_roll - dt * bias_roll + calibrated_gy * dt;
	bias_roll = bias_roll;

	//------预测协方差矩阵	反应预测结果可信度------
	//公式 P(k|k-1) = F*P(k-1|k-1)*Ft + Q
	//		当前先验结果协方差矩阵 = 状态矩阵*上一时刻后验结果协方差矩阵*状态矩阵的转置 + 预测结果协方差矩阵
	//Q =[ cov(angle_roll, angle_roll), cov(groy_roll, angle_roll)] = [Q_angle_roll, 0]
	//   [ cov(angle_roll, groy_roll) , cov(groy_roll, groy_roll) ]   [0, Q_groy_roll ]
	//设预测协方差矩阵为 PP[2][2] = [a, b]
	//							  [c, d]
	//带入公式计算可得：
	//a(k) = a(k - 1) - [c(k - 1) + b(k - 1)]*dt + d(k - 1)dt*dt + Q_angle_roll
	//b(k) = b(k - 1) - d(k - 1) * dt
	//c(k) = c(k - 1) - d(k - 1) * dt
	//d(k) = d(k - 1) + Q_groy_roll

	PP_roll[0][0] = PP_roll[0][0] - (PP_roll[1][0] + PP_roll[0][1]) * dt + Q_angle_roll;		//dt*dt忽略
	PP_roll[0][1] = PP_roll[0][1] - PP_roll[1][1] * dt;
	PP_roll[1][0] = PP_roll[1][0] - PP_roll[1][1] * dt;
	PP_roll[1][1] = PP_roll[1][1] + Q_groy_roll;

	//--------------建立测量方程-----------------
	//公式 Z(k) = H*x(k) + R
	//		当前时刻测量值 = 测量矩阵 * 当前时刻真实值 + 测量噪声协方差矩阵
	//		注：测量值其实是没有输入的，完全通过测量得到(即Z(k) = roll_a)，测量矩阵能够反映测量值的数据维度并提供数据维度转换的矩阵渠道，
	//			因为有时候预测值和测量值的数据维度不一样
	measure_roll = roll_a;

	//-------------计算卡尔曼增益----------------
	//				P(k|k - 1)*Ht
	//公式：	K = ------------------------
	//			H*P(k|k - 1)*Ht + R
	K_0_roll = PP_roll[0][0] / (PP_roll[0][0] + R_angle_roll);
	K_1_roll = PP_roll[1][0] / (PP_roll[0][0] + R_angle_roll);

	//-----------计算当前最优估计值---------------
	//公式 X(k|k) = X(k|k-1) + K[z(k) - HX(k|k-1)]
	kalman_roll = kalman_roll + K_0_roll * (measure_roll - kalman_roll);
	bias_roll = bias_roll + K_1_roll * (measure_roll - kalman_roll);


	//更新协方差矩阵
	//公式 P(k|k)=[I-K * H]P(k|k-1)
	PP_roll[0][0] = PP_roll[0][0] - K_0_roll * PP_roll[0][0];
	PP_roll[0][1] = PP_roll[0][1] - K_0_roll * PP_roll[0][1];
	PP_roll[1][0] = PP_roll[1][0] - K_1_roll * PP_roll[0][0];
	PP_roll[1][1] = PP_roll[1][1] - K_1_roll * PP_roll[0][1];

//****************************对pitch进行卡尔曼滤波*********************************

	//参数初始化
	//初始时刻最优估计值
	kalman_pitch = 0;				//估计角度
	static float bias_pitch = 0;				//估计偏差
	//预测噪声方差
	static float Q_angle_pitch = 0.1f;				//预测角度方差 = 预测角度协方差
	static float Q_groy_pitch = 0.01f;				//预测偏差方差 = 预测偏差协方差
	//测量噪声方差
	static float R_angle_pitch = 0.5f;
	//预测协方差矩阵，初始值为单位阵
	static float PP_pitch[2][2] = {{1, 0},{0, 1}};
	//测量值
	static float measure_pitch = 0;
	//卡尔曼增益
	static float K_0_pitch = 0;
	static float K_1_pitch = 0;

	static uint8_t first_kalman = 1;
	if (first_kalman)
	{
		kalman_pitch = pitch_a;
		first_kalman = 0;
	}

	//------------进行先验估计运算--------------
	kalman_pitch = kalman_pitch - dt * bias_pitch + calibrated_gx * dt;
	bias_pitch = bias_pitch;

	//------预测协方差矩阵	反应预测结果可信度------
	PP_pitch[0][0] = PP_pitch[0][0] - (PP_pitch[1][0] + PP_pitch[0][1]) * dt + Q_angle_pitch;		//dt*dt忽略
	PP_pitch[0][1] = PP_pitch[0][1] - PP_pitch[1][1] * dt;
	PP_pitch[1][0] = PP_pitch[1][0] - PP_pitch[1][1] * dt;
	PP_pitch[1][1] = PP_pitch[1][1] + Q_groy_pitch;

	//--------------建立测量方程-----------------
	measure_pitch = pitch_a;

	//-------------计算卡尔曼增益----------------
	K_0_pitch = PP_pitch[0][0] / (PP_pitch[0][0] + R_angle_pitch);
	K_1_pitch = PP_pitch[1][0] / (PP_pitch[0][0] + R_angle_pitch);

	//-----------计算当前最优估计值---------------
	kalman_pitch = kalman_pitch + K_0_pitch * (measure_pitch - kalman_pitch);
	bias_pitch = bias_pitch + K_1_pitch * (measure_pitch - kalman_pitch);


	//更新协方差矩阵
	PP_pitch[0][0] = PP_pitch[0][0] - K_0_pitch * PP_pitch[0][0];
	PP_pitch[0][1] = PP_pitch[0][1] - K_0_pitch * PP_pitch[0][1];
	PP_pitch[1][0] = PP_pitch[1][0] - K_1_pitch * PP_pitch[0][0];
	PP_pitch[1][1] = PP_pitch[1][1] - K_1_pitch * PP_pitch[0][1];

	kalman_yaw = 0;							//偏航角无法进行滤波

*/


