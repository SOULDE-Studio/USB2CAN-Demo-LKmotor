// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
#include "Tangair_usb2can.h"


// 主函数循环
void Tangair_usb2can::Spin()
{
    while (all_thread_done_ != true)
    {
        sleep(1); // 延时1s
    }
    printf("~ ALL Exit ~\n");
}



/// @brief 构造函数，初始化
/// @return
Tangair_usb2can::Tangair_usb2can() 
{

    running_ = true;
    all_thread_done_ = false;

    USB2CAN0_ = openUSBCAN("/dev/USB2CAN0");
    if (USB2CAN0_ == -1)
        std::cout << std::endl
                  << "USB2CAN0 open INcorrect!!!" << std::endl;
    else
        std::cout << std::endl
                  << "USB2CAN0 opened ,num=" << USB2CAN0_ << std::endl;

    USB2CAN1_ = openUSBCAN("/dev/USB2CAN1");
    if (USB2CAN1_ == -1)
        std::cout << std::endl
                  << "USB2CAN1 open INcorrect!!!" << std::endl;
    else
        std::cout << std::endl
                  << "USB2CAN1 opened ,num=" << USB2CAN1_ << std::endl;

    // 电机ID配置
    USB2CAN_CAN_Bus_Init();

    
    // 启动成功
    std::cout << std::endl
              << "USB2CAN   NODE INIT__OK   by TANGAIR" << std::endl
              << std::endl
              << std::endl;

  
    // 创建CAN接收线程，设备1
    _CAN_RX_device_0_thread = std::thread(&Tangair_usb2can::CAN_RX_device_0_thread, this);

    // 创建CAN接收线程，设备2
    _CAN_RX_device_1_thread = std::thread(&Tangair_usb2can::CAN_RX_device_1_thread, this);

    //can 测试线程
    _CAN_TX_test_thread = std::thread(&Tangair_usb2can::CAN_TX_test_thread, this);

    //键盘输入线程
    _keyborad_input = std::thread(&Tangair_usb2can::keyborad_input, this);
}

/// @brief 析构函数
Tangair_usb2can::~Tangair_usb2can()
{

    running_ = false;

    /*注销线程*/

    // can接收设备0
    _CAN_RX_device_0_thread.join();
    // can接收设备1
    _CAN_RX_device_1_thread.join();

    //can发送测试线程
    _CAN_TX_test_thread.join();
    //键盘改速度
    _keyborad_input.join();

   

    // 关闭设备
    closeUSBCAN(USB2CAN0_);
    closeUSBCAN(USB2CAN1_);

    all_thread_done_ = true;
}



/// @brief can设备0，接收线程函数
void Tangair_usb2can::CAN_RX_device_0_thread()
{
    can_dev0_rx_count = 0;
    can_dev0_rx_count_thread=0;
    while (running_)
    {

        uint8_t channel;
        FrameInfo info_rx;
        uint8_t data_rx[8] = {0};

        //
        can_dev0_rx_count_thread++;

        // 阻塞1s接收
        int recieve_re = readUSBCAN(USB2CAN0_, &channel, &info_rx, data_rx, 1e6);


        // 接收到数据
        if (recieve_re != -1)
        {
            // can_dev0_rx_count++;
            // 解码
           
            CAN_DEV0_RX.motor_id = info_rx.canID -0x140;
            CAN_DEV0_RX.motor_status=data_rx[0];

           


            if((CAN_DEV0_RX.motor_status==0x9C)||(CAN_DEV0_RX.motor_status==0xA1))//只有符合协议的电机数据才进行解析
            {
                if(CAN_DEV0_RX.motor_status==0xA1) can_dev0_rx_count++;


                CAN_DEV0_RX.current_temp=data_rx[1];
                CAN_DEV0_RX.current_iq= data_rx[2]|(data_rx[3]<<8);
                CAN_DEV0_RX.current_speed= data_rx[4]|(data_rx[5]<<8);
                CAN_DEV0_RX.current_position= data_rx[6]|(data_rx[7]<<8);

                // 转换
                CAN_DEV0_RX.current_position_f = (float)CAN_DEV0_RX.current_position*P_RIOT; //rad
                CAN_DEV0_RX.current_speed_f = (float)CAN_DEV0_RX.current_speed/V_ROIT;   //rad/s
                CAN_DEV0_RX.current_iq_f = (float)CAN_DEV0_RX.current_iq*IQ_ROIT;//A
                CAN_DEV0_RX.current_temp_f = (float)CAN_DEV0_RX.current_temp;                 //摄氏度       

                if (channel == 1) // 模块0，can1
                {
                    switch (CAN_DEV0_RX.motor_id)
                    {
                    case 1:
                    {
                        USB2CAN0_CAN_Bus_1.ID_1_motor_recieve = CAN_DEV0_RX;
                    
                        break;
                    }
                    case 2:
                    {
                        USB2CAN0_CAN_Bus_1.ID_2_motor_recieve = CAN_DEV0_RX;
                    
                        break;
                    }
                    case 3:
                    {
                        USB2CAN0_CAN_Bus_1.ID_3_motor_recieve = CAN_DEV0_RX;
                        
                        break;
                    }
                
                    default:
                        break;
                    }
                }
                else if (channel == 2) // 模块0，can2
                {
                    switch (CAN_DEV0_RX.motor_id)
                    {
                    case 1:
                    {
                        USB2CAN0_CAN_Bus_2.ID_1_motor_recieve = CAN_DEV0_RX;
                    
                        break;
                    }
                    case 2:
                    {
                        USB2CAN0_CAN_Bus_2.ID_2_motor_recieve = CAN_DEV0_RX;
                    
                        break;
                    }
                    case 3:
                    {
                        USB2CAN0_CAN_Bus_2.ID_3_motor_recieve = CAN_DEV0_RX;
                    
                        break;
                    }
                
                    default:
                        break;
                    }
                }
            }
            
        }
    }
    std::cout << "CAN_RX_device_0_thread  Exit~~" << std::endl;
}

/// @brief can设备1，接收线程函数
void Tangair_usb2can::CAN_RX_device_1_thread()
{
    can_dev1_rx_count = 0;
    can_dev1_rx_count_thread=0;
    while (running_)
    {

        uint8_t channel;
        FrameInfo info_rx;
        uint8_t data_rx[8] = {0};

        // 接收有可能是CAN1，也有可能是CAN2，使用if进行分类
        int recieve_re = readUSBCAN(USB2CAN1_, &channel, &info_rx, data_rx, 1e6);

        can_dev1_rx_count_thread++;
        // 接收到数据
        if (recieve_re != -1)
        {
            // can_dev1_rx_count++;

            CAN_DEV1_RX.motor_id = info_rx.canID - 0x140;
            CAN_DEV1_RX.motor_status=data_rx[0];

            if(CAN_DEV1_RX.motor_status==0x9C||CAN_DEV1_RX.motor_status==0xA1)//只有符合协议的电机数据才进行解析
            {
                 if(CAN_DEV1_RX.motor_status==0xA1) can_dev1_rx_count++;

                CAN_DEV1_RX.current_temp=data_rx[1];
                CAN_DEV1_RX.current_iq= data_rx[2]|(data_rx[3]<<8);
                CAN_DEV1_RX.current_speed= data_rx[4]|(data_rx[5]<<8);
                CAN_DEV1_RX.current_position= data_rx[6]|(data_rx[7]<<8);

        
                // 转换
                CAN_DEV1_RX.current_position_f = (float)CAN_DEV1_RX.current_position*P_RIOT; //rad
                CAN_DEV1_RX.current_speed_f = (float)CAN_DEV1_RX.current_speed/V_ROIT;   //rad/s
                CAN_DEV1_RX.current_iq_f = (float)CAN_DEV1_RX.current_iq*IQ_ROIT;//A
                CAN_DEV1_RX.current_temp_f = (float)CAN_DEV1_RX.current_temp;                 //摄氏度     

                if (channel == 1)  // 模块1，can1
                {
                    switch (CAN_DEV1_RX.motor_id)
                    {
                    case 1:
                    {
                        USB2CAN1_CAN_Bus_1.ID_1_motor_recieve = CAN_DEV1_RX;
                        break;
                    }
                    case 2:
                    {
                        USB2CAN1_CAN_Bus_1.ID_2_motor_recieve = CAN_DEV1_RX;
                        
                        break;
                    }
                    case 3:
                    {
                        USB2CAN1_CAN_Bus_1.ID_3_motor_recieve = CAN_DEV1_RX;
                    
                        break;
                    }
            
                    default:
                        break;
                    }
                }
                else if (channel == 2)  // 模块1，can2
                {
                    switch (CAN_DEV1_RX.motor_id)
                    {
                    case 1:
                    {
                        USB2CAN1_CAN_Bus_2.ID_1_motor_recieve = CAN_DEV1_RX;
                    
                        break;
                    }
                    case 2:
                    {
                        USB2CAN1_CAN_Bus_2.ID_2_motor_recieve = CAN_DEV1_RX;
                        
                        break;
                    }
                    case 3:
                    {
                        USB2CAN1_CAN_Bus_2.ID_3_motor_recieve = CAN_DEV1_RX;
                        
                        break;
                    }
            
                    default:
                        break;
                    }
                }


            }
            
        }
    }
    std::cout << "CAN_RX_device_1_thread  Exit~~" << std::endl;
}


/*****************************************************************************************************/
/*********************************       ***测试相关***      ***********************************************/
/*****************************************************************************************************/
// can发送测试线程函数
void Tangair_usb2can::CAN_TX_test_thread()
{
    //发送计数
    uint32_t tx_count = 0;
    //键盘输入速度
    speed_input = 2;
    //电机控制参数配置，单纯给速度
    {
        
        USB2CAN0_CAN_Bus_1.ID_1_motor_send.position = 0;
        USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = 2;
        USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque = 0;
        USB2CAN0_CAN_Bus_1.ID_1_motor_send.kp = 0;
        USB2CAN0_CAN_Bus_1.ID_1_motor_send.kd = 1;

        USB2CAN0_CAN_Bus_1.ID_2_motor_send.position = 0;
        USB2CAN0_CAN_Bus_1.ID_2_motor_send.speed = 2;
        USB2CAN0_CAN_Bus_1.ID_2_motor_send.torque = 0;
        USB2CAN0_CAN_Bus_1.ID_2_motor_send.kp = 0;
        USB2CAN0_CAN_Bus_1.ID_2_motor_send.kd = 1;

        USB2CAN0_CAN_Bus_1.ID_3_motor_send.position = 0;
        USB2CAN0_CAN_Bus_1.ID_3_motor_send.speed = 2;
        USB2CAN0_CAN_Bus_1.ID_3_motor_send.torque = 0;
        USB2CAN0_CAN_Bus_1.ID_3_motor_send.kp = 0;
        USB2CAN0_CAN_Bus_1.ID_3_motor_send.kd = 1;

        USB2CAN0_CAN_Bus_2.ID_1_motor_send.position = 0;
        USB2CAN0_CAN_Bus_2.ID_1_motor_send.speed = 2;
        USB2CAN0_CAN_Bus_2.ID_1_motor_send.torque = 0;
        USB2CAN0_CAN_Bus_2.ID_1_motor_send.kp = 0;
        USB2CAN0_CAN_Bus_2.ID_1_motor_send.kd = 1;

        USB2CAN0_CAN_Bus_2.ID_2_motor_send.position = 0;
        USB2CAN0_CAN_Bus_2.ID_2_motor_send.speed = 2;
        USB2CAN0_CAN_Bus_2.ID_2_motor_send.torque = 0;
        USB2CAN0_CAN_Bus_2.ID_2_motor_send.kp = 0;
        USB2CAN0_CAN_Bus_2.ID_2_motor_send.kd = 1;

        USB2CAN0_CAN_Bus_2.ID_3_motor_send.position = 0;
        USB2CAN0_CAN_Bus_2.ID_3_motor_send.speed = 2;
        USB2CAN0_CAN_Bus_2.ID_3_motor_send.torque = 0;
        USB2CAN0_CAN_Bus_2.ID_3_motor_send.kp = 0;
        USB2CAN0_CAN_Bus_2.ID_3_motor_send.kd = 1;

        USB2CAN1_CAN_Bus_1.ID_1_motor_send.position = 0;
        USB2CAN1_CAN_Bus_1.ID_1_motor_send.speed = 2;
        USB2CAN1_CAN_Bus_1.ID_1_motor_send.torque = 0;
        USB2CAN1_CAN_Bus_1.ID_1_motor_send.kp = 0;
        USB2CAN1_CAN_Bus_1.ID_1_motor_send.kd = 1;

        USB2CAN1_CAN_Bus_1.ID_2_motor_send.position = 0;
        USB2CAN1_CAN_Bus_1.ID_2_motor_send.speed = 2;
        USB2CAN1_CAN_Bus_1.ID_2_motor_send.torque = 0;
        USB2CAN1_CAN_Bus_1.ID_2_motor_send.kp = 0;
        USB2CAN1_CAN_Bus_1.ID_2_motor_send.kd = 1;

        USB2CAN1_CAN_Bus_1.ID_3_motor_send.position = 0;
        USB2CAN1_CAN_Bus_1.ID_3_motor_send.speed = 2;
        USB2CAN1_CAN_Bus_1.ID_3_motor_send.torque = 0;
        USB2CAN1_CAN_Bus_1.ID_3_motor_send.kp = 0;
        USB2CAN1_CAN_Bus_1.ID_3_motor_send.kd = 1;

        USB2CAN1_CAN_Bus_2.ID_1_motor_send.position = 0;
        USB2CAN1_CAN_Bus_2.ID_1_motor_send.speed = 2;
        USB2CAN1_CAN_Bus_2.ID_1_motor_send.torque = 0;
        USB2CAN1_CAN_Bus_2.ID_1_motor_send.kp = 0;
        USB2CAN1_CAN_Bus_2.ID_1_motor_send.kd = 1;

        USB2CAN1_CAN_Bus_2.ID_2_motor_send.position = 0;
        USB2CAN1_CAN_Bus_2.ID_2_motor_send.speed = 2;
        USB2CAN1_CAN_Bus_2.ID_2_motor_send.torque = 0;
        USB2CAN1_CAN_Bus_2.ID_2_motor_send.kp = 0;
        USB2CAN1_CAN_Bus_2.ID_2_motor_send.kd = 1;

        USB2CAN1_CAN_Bus_2.ID_3_motor_send.position = 0;
        USB2CAN1_CAN_Bus_2.ID_3_motor_send.speed = 2;
        USB2CAN1_CAN_Bus_2.ID_3_motor_send.torque = 0;
        USB2CAN1_CAN_Bus_2.ID_3_motor_send.kp = 0;
        USB2CAN1_CAN_Bus_2.ID_3_motor_send.kd = 1;
    }
  
    //读取电机信息
    READ_ALL_STATE(100);
   
    //使能所有电机
    ENABLE_ALL_MOTOR(100);

    while (running_)
    {
        // 电机控制参数配置，单纯给速度，给ID为1的电机，设置键盘速度，速度单位为rad/s
        if (abs((int)speed_input) < 50)
        {
            USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = (int)speed_input;
            USB2CAN0_CAN_Bus_2.ID_1_motor_send.speed = (int)speed_input;
            USB2CAN1_CAN_Bus_1.ID_1_motor_send.speed = (int)speed_input;
            USB2CAN1_CAN_Bus_2.ID_1_motor_send.speed = (int)speed_input;
            
        }

        // CAN发送,发送频率为1000hz,实际间隔约为950us
        CAN_TX_ALL_MOTOR(75);
            

        // CAN发送计数
        tx_count++;
      
    
        std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tpMill =
            std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        time_t tp = tpMill.time_since_epoch().count();

        // 打印数据tp时间ms，1000hz的控制频率的话，1s一次，
        if (tx_count % 1000 == 0)
        {
            std::cout << std::endl
                      << "USB2CAN0_CAN1.current_speed_f=  " << USB2CAN0_CAN_Bus_1.ID_1_motor_recieve.current_speed_f << "  rad/s" << std::endl
                      << "USB2CAN0_CAN1.current_iq_f=  " << USB2CAN0_CAN_Bus_1.ID_1_motor_recieve.current_iq_f << " A" << std::endl
                      << "USB2CAN0_CAN1.current_position_f=  " << USB2CAN0_CAN_Bus_1.ID_1_motor_recieve.current_position_f << "  rad" << std::endl
                      << "USB2CAN0_CAN2.current_speed_f=  " << USB2CAN0_CAN_Bus_2.ID_1_motor_recieve.current_speed_f << "  rad/s" << std::endl
                      << "USB2CAN1_CAN1.current_speed_f=  " << USB2CAN1_CAN_Bus_1.ID_1_motor_recieve.current_speed_f << "  rad/s" << std::endl
                      << "USB2CAN1_CAN2.current_speed_f=  " << USB2CAN1_CAN_Bus_2.ID_1_motor_recieve.current_speed_f << "  rad/s" << std::endl;
            std::cout << "can_tx_count=" << tx_count << "     " << "can_dev0_rx_count=" << can_dev0_rx_count << "     "<< "can_dev1_rx_count=" << can_dev1_rx_count << "     "
                      << "TIME=" << (tp % 1000000) / 1000 << "." << tp % 1000 << "s" << std::endl;
        }
    }
     // 失能电机
    DISABLE_ALL_MOTOR(100);

    //程序终止时的提示信息
    std::cout << "CAN_TX_test_thread  Exit~~" << std::endl;
    std::cout << std::endl
              << "----------------请输入任意数字，按回车，以结束键盘进程  ----------------------" << std::endl
              << std::endl;
}

// 键盘输入线程，可以键盘修改电机速度
void Tangair_usb2can::keyborad_input()
{

    while (running_)
    {
        std::cin >> speed_input;
        std::cout << "speed_input=" << speed_input << std::endl;
    }
    std::cout << "keyborad_input_thread  Exit~~" << std::endl;
}

/*****************************************************************************************************/
/*********************************       ***电机相关***      ***********************************************/
/*****************************************************************************************************/

void Tangair_usb2can::USB2CAN_CAN_Bus_inti_set(USB2CAN_CAN_Bus_Struct *CAN_Bus)
{
    CAN_Bus->ID_1_motor_send.id = 1;
    CAN_Bus->ID_1_motor_recieve.motor_id= 1;

    CAN_Bus->ID_2_motor_send.id = 2;
    CAN_Bus->ID_2_motor_recieve.motor_id=2;
  

    CAN_Bus->ID_3_motor_send.id = 3;
    CAN_Bus->ID_3_motor_recieve.motor_id=3;

}

void Tangair_usb2can::USB2CAN_CAN_Bus_Init()
{
    USB2CAN_CAN_Bus_inti_set(&USB2CAN0_CAN_Bus_1);
    USB2CAN_CAN_Bus_inti_set(&USB2CAN0_CAN_Bus_2);
    USB2CAN_CAN_Bus_inti_set(&USB2CAN1_CAN_Bus_1);
    USB2CAN_CAN_Bus_inti_set(&USB2CAN1_CAN_Bus_2);
}


/// @brief 读取电机状态
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Read_State(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
    txMsg_CAN.canID = Motor_Data->id +0x140;
  
   
    Data_CAN[0] = 0x9C;
    Data_CAN[1] = 0x00;
    Data_CAN[2] = 0x00;
    Data_CAN[3] = 0x00;
    Data_CAN[4] = 0x00;
    Data_CAN[5] = 0x00;
    Data_CAN[6] = 0x00;
    Data_CAN[7] = 0x00;
    
    sendUSBCAN(dev, channel, &txMsg_CAN, Data_CAN);
}


/// @brief 使能
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Enable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
    txMsg_CAN.canID = Motor_Data->id +0x140;
   
    Data_CAN[0] = 0x88;
    Data_CAN[1] = 0x00;
    Data_CAN[2] = 0x00;
    Data_CAN[3] = 0x00;
    Data_CAN[4] = 0x00;
    Data_CAN[5] = 0x00;
    Data_CAN[6] = 0x00;
    Data_CAN[7] = 0x00;
    
    sendUSBCAN(dev, channel, &txMsg_CAN, Data_CAN);
}

/// @brief 电机失能
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Disable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
    txMsg_CAN.canID = Motor_Data->id +0x140;
   
    Data_CAN[0] = 0x80;
    Data_CAN[1] = 0x00;
    Data_CAN[2] = 0x00;
    Data_CAN[3] = 0x00;
    Data_CAN[4] = 0x00;
    Data_CAN[5] = 0x00;
    Data_CAN[6] = 0x00;
    Data_CAN[7] = 0x00;
    
    sendUSBCAN(dev, channel, &txMsg_CAN, Data_CAN);
}

/// @brief 设置零点,重新上电后生效
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Zore(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
      txMsg_CAN.canID = Motor_Data->id +0x140;
   
    Data_CAN[0] = 0x19;
    Data_CAN[1] = 0x00;
    Data_CAN[2] = 0x00;
    Data_CAN[3] = 0x00;
    Data_CAN[4] = 0x00;
    Data_CAN[5] = 0x00;
    Data_CAN[6] = 0x00;
    Data_CAN[7] = 0x00;
    
    sendUSBCAN(dev, channel, &txMsg_CAN, Data_CAN);
}

/// @brief 电机控制
/// @param dev 模块设备号
/// @param channel can1或者can2
/// @param Motor_Data 电机数据
void Tangair_usb2can::CAN_Send_Control(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data_Send,Motor_CAN_Recieve_Struct *Motor_Data_state) 
{
    // 运控模式专用的局部变
    FrameInfo txMsg_Control = {
        .canID = 0,
        .frameType = STANDARD,
        .dataLength = 8,
    };
    uint8_t Data_CAN_Control[8];

     //保险，确保状态和命令是给同一个电机的
    if(Motor_Data_Send->id==Motor_Data_state->motor_id)
    {
        txMsg_Control.canID = Motor_Data_Send->id +0x140;

        //解算电流
        Motor_Data_Send->send_iq_f = \
        Motor_Data_Send->kp*(Motor_Data_Send->position-Motor_Data_state->current_position_f) + \
        Motor_Data_Send->kd*(Motor_Data_Send->speed   -Motor_Data_state->current_speed_f)+\
        Motor_Data_Send->torque;

         Motor_Data_Send->send_iq_f*=100;


        Motor_Data_Send->send_iq= Motor_Data_Send->send_iq_f;//控制值 iqControl 为 int16_t 类型，数值范围-2048~ 2048，对应 MG 电机实际转矩电流范围-33A~33A
    
        if( Motor_Data_Send->send_iq>2048) Motor_Data_Send->send_iq=2048;
        if( Motor_Data_Send->send_iq<-2048) Motor_Data_Send->send_iq=-2048;


        Data_CAN_Control[0] = 0xA1;
        Data_CAN_Control[1] = 0x00;
        Data_CAN_Control[2] = 0x00;
        Data_CAN_Control[3] = 0x00;
        Data_CAN_Control[4] = Motor_Data_Send->send_iq&0XFF;
        Data_CAN_Control[5] = (Motor_Data_Send->send_iq>>8)&0XFF;
        Data_CAN_Control[6] = 0x00;
        Data_CAN_Control[7] = 0x00;

        int ret = sendUSBCAN(dev, channel, &txMsg_Control, Data_CAN_Control);
    } 
}




void Tangair_usb2can::READ_ALL_STATE(int delay_us)
{
    // 右前腿
    Motor_Read_State(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 右后腿
    Motor_Read_State(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左前腿
    Motor_Read_State(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左后腿
    Motor_Read_State(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // 右前腿
    Motor_Read_State(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 右后腿
    Motor_Read_State(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左前腿
    Motor_Read_State(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左后腿
    Motor_Read_State(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // 右前腿
    Motor_Read_State(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 右后腿
    Motor_Read_State(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左前腿
    Motor_Read_State(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左后腿
    Motor_Read_State(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

   
}


void Tangair_usb2can::ENABLE_ALL_MOTOR(int delay_us)
{
    // 右前腿
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 右后腿
    Motor_Enable(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左前腿
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左后腿
    Motor_Enable(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // 右前腿
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 右后腿
    Motor_Enable(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左前腿
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左后腿
    Motor_Enable(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // 右前腿
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 右后腿
    Motor_Enable(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左前腿
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左后腿
    Motor_Enable(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

   
}

void Tangair_usb2can::Tangair_usb2can::DISABLE_ALL_MOTOR(int delay_us)
{
    // 右前腿
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 右后腿
    Motor_Disable(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左前腿
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左后腿
    Motor_Disable(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // 右前腿
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 右后腿
    Motor_Disable(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左前腿
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左后腿
    Motor_Disable(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // 右前腿
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 右后腿
    Motor_Disable(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左前腿
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左后腿
    Motor_Disable(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

 
}

void Tangair_usb2can::ZERO_ALL_MOTOR(int delay_us)
{

    // 右前腿
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 右后腿
    Motor_Zore(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左前腿
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左后腿
    Motor_Zore(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // 右前腿
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 右后腿
    Motor_Zore(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左前腿
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左后腿
    Motor_Zore(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // 右前腿
    Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 右后腿
    Motor_Zore(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左前腿
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // 左后腿
    Motor_Zore(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

   
}


/// @brief can控制发送，12个电机的数据
// 目前能达到1000hz的控制频率--------3000hz的总线发送频率---------同一路can的发送间隔在300us
void Tangair_usb2can::CAN_TX_ALL_MOTOR(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now();//这一句耗时50us

    //**ID_1_motor_send *//
    //右前腿
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send,&USB2CAN0_CAN_Bus_1.ID_1_motor_recieve);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //右后腿
    CAN_Send_Control(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_1_motor_send,&USB2CAN1_CAN_Bus_1.ID_1_motor_recieve);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //左前腿
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send,&USB2CAN0_CAN_Bus_2.ID_1_motor_recieve);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //左后腿
    CAN_Send_Control(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_1_motor_send,&USB2CAN1_CAN_Bus_2.ID_1_motor_recieve);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    //**ID_2_motor_send *//
     // 右前腿
     CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send,&USB2CAN0_CAN_Bus_1.ID_2_motor_recieve);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
     // 右后腿
    CAN_Send_Control(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_2_motor_send,&USB2CAN1_CAN_Bus_1.ID_2_motor_recieve);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //左前腿
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send,&USB2CAN0_CAN_Bus_2.ID_2_motor_recieve);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //左后腿
    CAN_Send_Control(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_2_motor_send,&USB2CAN1_CAN_Bus_2.ID_2_motor_recieve);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    //**ID_3_motor_send *//
    // 右前腿
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send,&USB2CAN0_CAN_Bus_1.ID_3_motor_recieve);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    // 右后腿
    CAN_Send_Control(USB2CAN1_, 1, &USB2CAN1_CAN_Bus_1.ID_3_motor_send,&USB2CAN1_CAN_Bus_1.ID_3_motor_recieve);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //左前腿
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send,&USB2CAN0_CAN_Bus_2.ID_3_motor_recieve);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //左后腿
    CAN_Send_Control(USB2CAN1_, 2, &USB2CAN1_CAN_Bus_2.ID_3_motor_send,&USB2CAN1_CAN_Bus_2.ID_3_motor_recieve);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    
}

/// @brief 辅助函数
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0

