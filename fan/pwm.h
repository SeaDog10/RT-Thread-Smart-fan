/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-07-26     dog       the first version
 */
#ifndef PWM_H_
#define PWM_H_
#include "board.h"

#define Servo_MAX   30
#define Servo_MIN   10
#define Servo_MID   20

void Motor_PWMOut_Init(u16 arr, u16 psc, u16 ccp);
void Servo_PWMOut_Init(u16 arr, u16 psc, u16 ccp);

#endif /* PWM_H_ */
