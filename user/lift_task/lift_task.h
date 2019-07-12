#ifndef LIFT_TASK_H
#define LIFT_TASK_H

#include "rc.h"
#include "can_receive.h"
#include "pid.h"

/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
#define W 			0x0001		
#define S 			0x0002
#define A 			0x0004
#define D 			0x0008
#define SHIFT 	0x0010
#define CTRL 		0x0020
#define Q 			0x0040
#define E				0x0080
#define R 			0x0100
#define F 			0x0200
#define G 			0x0400
#define Z 			0x0800
#define X 			0x1000
#define C 			0x2000
#define V 			0x4000		
#define B				0x8000
/******************************************************/

typedef enum
{
	Init_MODE      = 0,      //��ʼ״̬
  Rc_MODE			   = 1,      //ң��״̬
	Key_MODE  		 = 2,      //����״̬
	Stop_MODE      = 3,      //ֹͣ״̬
} lift_mode_e;

typedef enum
{
	PINCH_INIT = 0,					//��ʼ״̬
	PINCH_RISE,							//����
	PINCH_GIVE,							//����
}
pinch_mode_e;

typedef struct
{
  const motor_measure_t *lift_motor_measure;
  float speed;
  float speed_set;
	float angle;
	float angle_set;
	float angle_start;
  int16_t give_current;
} Lift_Motor_t;

typedef struct
{
  const RC_ctrl_t *lift_RC;               //����ʹ�õ�ң����ָ��
  Lift_Motor_t motor_lift[2];          //���̵������
  PidTypeDef motor_speed_pid[2];             //��������ٶ�pid
	PidTypeDef motor_pos_pid[2];               //�������λ��pid
	lift_mode_e mode;
	
} lift_move_t;

//��������
void lift_task(void *pvParameters);
//������ʼ��
void lift_init(lift_move_t *lift_init);
//�������ݸ���
void lift_feedback_update(lift_move_t *lift_update);
//��������PID����
void lift_control_loop(lift_move_t *lift_control);
//����ȡ��״̬
uint8_t get_pinch_state(void);
#endif
