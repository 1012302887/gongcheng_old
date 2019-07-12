#ifndef LIFT_WHEEL_H
#define LIFT_WHEEL_H

#include "chassis_task.h"

//初始化登岛轮
void lift_wheel_init(chassis_move_t *lift_wheel_init);
//登岛轮数据更新
void lift_wheel_feedback_update(chassis_move_t *lift_wheel_update);
//登岛轮控制PID计算
void lift_wheel_control_loop(chassis_move_t *lift_wheel_control);

#endif
