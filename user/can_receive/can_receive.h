#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "stm32f4xx_hal.h"
#include "can.h"

//电机can接收ID
typedef enum
{
	CAN_CHASSIS_ALL_ID = 0x200,
	CAN_3508_M1_ID = 0x201,    //底盘1号电机
	CAN_3508_M2_ID = 0x202,    //底盘2号电机
	CAN_3508_M3_ID = 0x203,    //底盘3号电机
	CAN_3508_M4_ID = 0x204,    //底盘4号电机
	CAN_3508_M5_ID = 0x205,    //底盘5号电机
	CAN_3508_M6_ID = 0x206,    //底盘6号电机
	CAN_3508_M7_ID = 0x207,    //升降7号电机
	CAN_3508_M8_ID = 0x208,    //升级8号电机
} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
	int16_t offset_ecd;
	float round_cnt;
	float ecd_raw_rate;
	float total_ecd;
	float angle;
	int32_t  rate_buf[5];          //buf，for filter
  uint8_t  buf_cut;                       //计算用
  int32_t  filter_rate;                   //角速度
	int32_t  last_filter_rate;              //上次角速度
	
	float speed_raw;
	float speed_set;
	
	uint16_t bullet_launch;								//拨盘启动标志
	uint8_t  chassis_mode;                //底盘模式
} motor_measure_t;

//陀螺仪数据结构体
typedef struct
{
	float v_x;
	float v_z;
	float pit;
	float yaw;
} gyro_info_t;

//tof数据结构体
typedef struct
{
	uint16_t dis_r;   //tof距离
	uint16_t dis_l;   //tof距离
	uint16_t str;     //tof信号强度
}tof_can_data_t;


//统一处理can中断函数
void CAN_hook(CAN_RxHeaderTypeDef *rx_message, uint8_t *Data);
//发送底盘电机控制命令
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//发送升降电机控制命令
void CAN_CMD_LIFT(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//发送陀螺仪控制命令
void CAN_CMD_GYRO_CALI(uint8_t mode, uint16_t time);
//电机数据获取
void get_motor_measuer(motor_measure_t* ptr, uint8_t* Data, uint16_t Id);
//返回底盘电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Motor_Measure_Point(uint8_t i);
//返回陀螺仪变量地址，通过指针方式获取原始数据
extern const gyro_info_t *get_GYRO_Measure_Point(void);
#endif
