// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
#ifndef __Tangair_usb2can__
#define __Tangair_usb2can__

#include <assert.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <string>
#include <sys/mman.h>
#include <sys/timerfd.h>
#include <thread>
#include <sched.h>
#include <unistd.h>
#include "usb_can.h"


// 辅助函数
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);

// LK MG电机
#define P_ENCOD_PARA 16   // 编码器参数
#define V_ROIT (9.55f * 6.0f) // dps转rad/s
#define IQ_MIN -66.0f      // 最小电流
#define IQ_MAX 66.0f       // 最大电流



#define PI (3.1415926f)

typedef struct
{
	uint8_t id;

	float position;
	float speed;
	float kp;
	float kd;
	float torque;

	//发送的电流
	float send_iq_f;
	uint16_t send_iq;


} Motor_CAN_Send_Struct;

typedef struct
{
	uint8_t motor_id;
	uint8_t motor_status;


	uint16_t current_position; //编码器值 encoder（uint16_t 类型，14bit 编码器的数值范围 0~16383，15bit 编码器的数值范围0~32767，16bit 编码器的数值范围 0~65535）
	uint16_t current_speed;	   //电机转速 speed（int16_t 类型，1dps/LSB）
	uint16_t current_iq;       //MG 电机 iq 分辨率为(66/4096 A) / LSB
	uint16_t current_temp;	   //电机温度 temperature（int8_t 类型，1℃/LSB）

	float current_position_f; 
	float current_speed_f;	  
	float current_iq_f;	  
	float current_temp_f;	  

	

} Motor_CAN_Recieve_Struct;

typedef struct
{
	Motor_CAN_Send_Struct ID_1_motor_send, ID_2_motor_send, ID_3_motor_send;
	Motor_CAN_Recieve_Struct ID_1_motor_recieve, ID_2_motor_recieve, ID_3_motor_recieve;

} USB2CAN_CAN_Bus_Struct;



class Tangair_usb2can
{
public:
	
	bool all_thread_done_;
	bool running_;

	void Spin();

  

	Tangair_usb2can();

	~Tangair_usb2can();

  
	// CAN设备0
	int USB2CAN0_;
	std::thread _CAN_RX_device_0_thread;
	void CAN_RX_device_0_thread();


	// CAN设备1
	int USB2CAN1_;
	std::thread _CAN_RX_device_1_thread;
	void CAN_RX_device_1_thread();

    int can_dev0_rx_count;
	int can_dev0_rx_count_thread;
	int can_dev1_rx_count;
	int can_dev1_rx_count_thread;


	//can发送测试线程
	std::thread _CAN_TX_test_thread;
	//can发送测试线程函数
	void CAN_TX_test_thread();


    int speed_input;
    //can键盘输入线程
	std::thread _keyborad_input;
	//can发送测试线程函数
	void keyborad_input();




	/*****************************************************************************************************/
	/*********************************       ***电机相关***      ***********************************************/
	/*****************************************************************************************************/
	// 电机基本操作变量
	FrameInfo txMsg_CAN = {
		.canID = 0,
		.frameType = STANDARD,
		.dataLength = 8,
	};

	uint8_t Data_CAN[8];
	Motor_CAN_Send_Struct Motor_Data_Single;

	//接收暂存
	Motor_CAN_Recieve_Struct CAN_DEV0_RX;
	Motor_CAN_Recieve_Struct CAN_DEV1_RX;


	// 腿部数据
	USB2CAN_CAN_Bus_Struct USB2CAN0_CAN_Bus_1; // 模块0，can1
	USB2CAN_CAN_Bus_Struct USB2CAN0_CAN_Bus_2;  // 模块0，can2
	USB2CAN_CAN_Bus_Struct USB2CAN1_CAN_Bus_1;  // 模块1，can1
	USB2CAN_CAN_Bus_Struct USB2CAN1_CAN_Bus_2;	  // 模块1，can2



	//初始化
	void USB2CAN_CAN_Bus_inti_set(USB2CAN_CAN_Bus_Struct* Leg_Data);
	void USB2CAN_CAN_Bus_Init();

    void Motor_Read_State(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);


	void Motor_Enable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void Motor_Disable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void Motor_Zore(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

    void CAN_Send_Control(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data_Send,Motor_CAN_Recieve_Struct *Motor_Data_state) ;

	void Motor_Passive_SET(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	
    void READ_ALL_STATE(int delay_us);
	
	void ENABLE_ALL_MOTOR(int delay_us);

	void DISABLE_ALL_MOTOR(int delay_us);

	void ZERO_ALL_MOTOR(int delay_us);

	void PASSIVE_ALL_MOTOR(int delay_us);

	void CAN_TX_ALL_MOTOR(int delay_us);

private:


};

#endif

// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
