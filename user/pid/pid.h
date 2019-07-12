#ifndef PID_H
#define PID_H
#include "stm32f4xx_hal.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
	uint16_t mode;
	//PID ������
	float Kp;
	float Ki;
	float Kd;

	float max_out;  //������
	float max_iout; //���������

	float set;
	float fdb;

	float out;
	float Pout;
	float Iout;
	float Dout;
	float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
	float error[3]; //����� 0���� 1��һ�� 2���ϴ�
} PidTypeDef;
extern void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
extern float PID_Calc(PidTypeDef *pid, float ref, float set);
extern void PID_clear(PidTypeDef *pid);
#endif
