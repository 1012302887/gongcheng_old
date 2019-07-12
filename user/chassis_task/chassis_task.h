#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "rc.h"
#include "tof.h"

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

//#include "user_lib.h"

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0
//ң������������
#define CHASSIS_RC_DEADLINE 10
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.001f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.001f

//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 5000.0f

//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.9f
//���̵������ٶ�
#define MAX_WHEEL_SPEED 30.0f

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.00015597718771929826
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//1.0 in = 2.54 cm	
//240DPI	ÿ240��һӢ��
#define DPI_POINT_TO_CENTIMETER	(2.54f / 240.0f)

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  float accel;
  float speed;
  float speed_set;
	float angle;
	float angle_set;
	float angle_start;
  int16_t give_current;
} Chassis_Motor_t;

typedef struct
{
	const tof_data_t *tof_measure;
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��
	const gyro_info_t *gyro_data;						   //����������
  Chassis_Motor_t motor_chassis[4];          //���̵������
  PidTypeDef motor_speed_pid[4];             //���̵���ٶ�pid
	PidTypeDef motor_pos_pid[4];               //���̵��λ��pid
	PidTypeDef chassis_acc_pid;
	PidTypeDef chassis_gryo_pid;
	
	int32_t  tof_h;                              //���̺���tof
	uint32_t key_time;
	uint32_t last_press_time;
	float    vx;
	float    vx_offset;
	float    vy;
	float    vy_offset;
	int16_t  vw_mouse;
	float    vw;
	float    vw_offset;
	float    vw_set;
	float    gimbal_y_offset;
	float    gyro_angle_start;                   //���������ʼ�����ʱ���������ǵĽǶ�
} chassis_move_t;

typedef enum
{
	INIT_MODE = 0,
	RC_MODE,
	KEY_MODE,
	STOP_MODE,
} chassis_mode_e;

extern chassis_move_t chassis_move;

//��������
void chassis_task(void *pvParameters);
//���̳�ʼ������Ҫ��pid��ʼ��
void chassis_init(chassis_move_t *chassis_init);
//�������ݸ���
void chassis_feedback_update(chassis_move_t *chassis_update);
//���̿���������
void chassis_set_contorl(chassis_move_t *chassis_control);
//ң���������ݴ���ɵ��̵�ǰ��vx�ٶȣ�vy�ٶ�
void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector);
//���̿���PID����
void chassis_control_loop(chassis_move_t *chassis_control);
//���ص�������״̬
uint8_t get_chassis_state(void);
//����Z��PID��ʼ��
void CHISSIS_PID_Init(PidTypeDef *pid, float maxout, float max_iout, float kp, float ki, float kd);
#endif
