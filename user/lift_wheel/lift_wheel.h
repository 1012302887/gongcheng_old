#ifndef LIFT_WHEEL_H
#define LIFT_WHEEL_H

#include "chassis_task.h"

//��ʼ���ǵ���
void lift_wheel_init(chassis_move_t *lift_wheel_init);
//�ǵ������ݸ���
void lift_wheel_feedback_update(chassis_move_t *lift_wheel_update);
//�ǵ��ֿ���PID����
void lift_wheel_control_loop(chassis_move_t *lift_wheel_control);

#endif
