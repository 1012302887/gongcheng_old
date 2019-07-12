#include "can_receive.h"
#include "stm32f4xx_hal.h"

//����tof����
//static tof_can_data_t tof_data;
//�����Ǳ���
static gyro_info_t gyro_info;
//�����������
static motor_measure_t motor_chassis[8];
//CAN�����жϻص�����
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
					//������ID��
					i = RxMeg.StdId - 0x201;
					//���������ݺ꺯��
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
					
					//������ԭʼ�����ǻ��ȣ��ѻ���ת��Ϊ�Ƕ�
					gyro_info.v_z = (float)raw_v_z * 0.057295f;
					
					//������ԭʼ���ݱ�����100��
					pitch_angle = (float)raw_pit/100;
					
					//pit�Ƕ�û�и�ֵ
					if(pitch_angle < 0)
					{
						pitch_angle = pitch_angle + 360;
					}
					
					//���Ƕȸĳ������ģ�����360����0��
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

//������ݻ�ȡ
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
//���͵��̵����������
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

//�������������������
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

//���������ǿ������� �����ǽ���ID��100; mode:0x30ΪУ׼ģʽ��timeΪУ׼ʱ�䣬1000ms���Ҿ��С�
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
	
	//10ms��һֱ����У׼ָ���������
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

//���ص��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Motor_Measure_Point(uint8_t i)
{
	return &motor_chassis[(i & 0x07)];//i��0x07ֻ����0~7����ֹ�������
}
//���������Ǳ�����ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const gyro_info_t *get_GYRO_Measure_Point(void)
{
	return &gyro_info;
}
