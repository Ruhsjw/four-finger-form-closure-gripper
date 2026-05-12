/*
 * bsp_pid.h
 *
 *  Created on: Sep 17, 2020
 *      Author: raysw
 */

#ifndef INC_BSP_PID_H_
#define INC_BSP_PID_H_

#ifdef STM32F4
#include "stm32f4xx_hal.h"
#elif defined STM32F1
#include "stm32f1xx_hal.h"
#endif

#include "pid.h"

#define PID_COUNT   (1)

/* Extern  ------------------------------------------------------------------*/
extern PIDController pid_ctrl_speed[];
extern PIDController pid_ctrl_angle[];
extern PIDPtr speed_pid[];
extern PIDPtr angle_pid[];
extern float pid_input_angle[];
extern float pid_input_speed[];
extern float pid_output_angle[];
extern float pid_output_speed[];
extern float pid_setval_angle[];
extern float pid_setval_speed[];

void BSP_PID_Init_speed(void);
void BSP_PID_Init_angle(void);
void BSP_PID_StepOnce(void);

#endif /* INC_BSP_PID_H_ */
