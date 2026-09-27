#ifndef __MOTOR_TASK_H
#define __MOTOR_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "rc_task.h"
#include "damiao_motor.h"

#define DAMIAO_MOTOR_NUM 3

extern DamiaoMotor_t damiao_motor[DAMIAO_MOTOR_NUM];

#endif /* __MOTOR_TASK_H */
