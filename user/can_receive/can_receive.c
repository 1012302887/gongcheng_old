#include "can_receive.h"
#include "stm32f4xx_hal.h"

//声明tof变量
//static tof_can_data_t tof_data;
//陀螺仪变量
static gyro_info_t gyro_info;
//声明电机变量
static motor_measure_t motor_chassis[8];
//CAN接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == (&hcan1))
	{
		uint8_t Data[8];
		CAN_RxHeaderTypeDef RxMeg;
		HAL_StatusTypeDef	HAL_RetVal;
		HAL_RetVal = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMeg, Data);
		if(HAL_OK==HAL_RetVal)
		{
			switch (RxMeg.StdId)
			{
				case CAN_3508_M1_ID:
				case CAN_3508_M2_ID:
				case CAN_3508_M3_ID:
				case CAN_3508_M4_ID:		
				case CAN_3508_M5_ID:
				case CAN_3508_M6_ID:
				case CAN_3508_M7_ID:
				case CAN_3508_M8_ID:
				{
					static uint8_t i = 0;
					//处理电机ID号
					i = RxMeg.StdId - 0x201;
					//处理电机数据宏函数
					get_motor_measuer(&motor_chassis[i], Data, RxMeg.StdId);
					break;
				}
				default:
				{
					break;
				}			
			}
		}
	}
	if(hcan == (&hcan2))
	{
		uint8_t Data[8];
		CAN_RxHeaderTypeDef RxMeg;
		HAL_StatusTypeDef	HAL_RetVal;
		HAL_RetVal = HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxMeg, Data);
		if(HAL_OK==HAL_RetVal)
		{
			switch (RxMeg.StdId)
			{
				case 101:
				{
					static int16_t pitch_connt = 0;
					static int16_t raw_pit,raw_v_z = 0;
					static float pitch_angle, last_pitch_angle = 0;
					raw_v_z = Data[2]<<8 | Data[3];
					raw_pit = Data[4]<<8 | Data[5];
					
					//陀螺仪原始数据是弧度，把弧度转换为角度
					gyro_info.v_z = (float)raw_v_z * 0.057295f;
					
					//陀螺仪原始数据被乘了100倍
					pitch_angle = (float)raw_pit/100;
					
					//pit角度没有负值
					if(pitch_angle < 0)
					{
						pitch_angle = pitch_angle + 360;
					}
					
					//将角度改成连续的，不是360°变回0°
					if((pitch_angle - last_pitch_angle) > 330)
						pitch_connt--;
					else if((pitch_angle - last_pitch_angle) < -330)
						pitch_connt++;
					
					gyro_info.pit = pitch_angle + pitch_connt * 360;
					last_pitch_angle = pitch_angle;
					break;
				}			
				case 0x300:
				{
//					if((Data[0]<<8 | Data[1]) < 1000)
//					tof_data.dis_r = Data[0]<<8 | Data[1];
//					if((Data[2]<<8 | Data[3]) < 1000)
//					tof_data.dis_l = Data[2]<<8 | Data[3];
					break;
				}
				case 0x401:
				{
					gyro_info.yaw = 0.01f*((int32_t)(Data[0]<<24)|(int32_t)(Data[1]<<16) | (int32_t)(Data[2]<<8) | (int32_t)(Data[3]))*0.8571428571f; 
					break;
				}
				default:
				{
					break;
				}
			}
		}
	}
}

//电机数据获取
void get_motor_measuer(motor_measure_t* ptr, uint8_t* Data, uint16_t Id)
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = Data[0] << 8 | Data[1];
	ptr->speed_rpm = (uint16_t)(Data[2] << 8 | Data[3]); 

	if (ptr->ecd - ptr->last_ecd > 4096)
	{
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
	
	else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

	int32_t temp_sum = 0;
  ptr->rate_buf[ptr->buf_cut++] = ptr->ecd_raw_rate;
  if (ptr->buf_cut >= 5)
    ptr->buf_cut = 0;
  for (uint8_t i = 0; i < 5; i++)
  {
    temp_sum += ptr->rate_buf[i];
  }
	ptr->last_filter_rate = ptr->filter_rate;

	ptr->filter_rate = (int32_t)(temp_sum/5);
	
	ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
	/* total angle, unit is degree */
	ptr->angle = ptr->total_ecd / (8192.0f/360.0f*19.0f);
}
//发送底盘电机控制命令
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint8_t Data[8];
	uint32_t pTxMailbox;
	CAN_TxHeaderTypeDef TxMeg;
	
	TxMeg.StdId = CAN_CHASSIS_ALL_ID;
	TxMeg.IDE = CAN_ID_STD;
	TxMeg.RTR = CAN_RTR_DATA;
	TxMeg.DLC = 0x08;
	Data[0] = motor1 >> 8;
	Data[1] = motor1;
	Data[2] = motor2 >> 8;
	Data[3] = motor2;
	Data[4] = motor3 >> 8;
	Data[5] = motor3;
	Data[6] = motor4 >> 8;
	Data[7] = motor4;

	HAL_CAN_AddTxMessage(&hcan1, &TxMeg, Data, &pTxMailbox);
}

//发送升降电机控制命令
void CAN_CMD_LIFT(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint8_t Data[8];
	uint32_t pTxMailbox;
	CAN_TxHeaderTypeDef TxMeg;
	
	TxMeg.StdId = 0x1ff;
	TxMeg.IDE = CAN_ID_STD;
	TxMeg.RTR = CAN_RTR_DATA;
	TxMeg.DLC = 0x08;
	Data[0] = motor1 >> 8;
	Data[1] = motor1;
	Data[2] = motor2 >> 8;
	Data[3] = motor2;
	Data[4] = motor3 >> 8;
	Data[5] = motor3;
	Data[6] = motor4 >> 8;
	Data[7] = motor4;

	HAL_CAN_AddTxMessage(&hcan1, &TxMeg, Data, &pTxMailbox);
}

//发送陀螺仪控制命令 陀螺仪接收ID：100; mode:0x30为校准模式，time为校准时间，1000ms左右就行。
void CAN_CMD_GYRO_CALI(uint8_t mode, uint16_t time)
{
	uint8_t Data[8];
	uint32_t pTxMailbox;
	CAN_TxHeaderTypeDef TxMeg;
	
	TxMeg.StdId = 100;
	TxMeg.IDE = CAN_ID_STD;
	TxMeg.RTR = CAN_RTR_DATA;
	TxMeg.DLC = 0x03;
  Data[0] = mode;
	Data[1] = time >> 8;
  Data[2] = time ;
	Data[3] = 0;
	Data[4] = 0;
	Data[5] = 0;
	Data[6] = 0;
	Data[7] = 0;
	
	//10ms内一直发送校准指令给陀螺仪
	uint32_t tickstart = HAL_GetTick();
  uint32_t wait = 10;

  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)1;
  }

  while((HAL_GetTick() - tickstart) < wait)
  {
	  HAL_CAN_AddTxMessage(&hcan2, &TxMeg, Data, &pTxMailbox);
  }
}

//返回电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Motor_Measure_Point(uint8_t i)
{
	return &motor_chassis[(i & 0x07)];//i与0x07只会是0~7，防止数组溢出
}
//返回陀螺仪变量地址，通过指针方式获取原始数据
const gyro_info_t *get_GYRO_Measure_Point(void)
{
	return &gyro_info;
}
