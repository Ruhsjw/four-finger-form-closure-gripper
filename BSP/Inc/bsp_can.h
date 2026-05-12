/*
 * bsp_can.h
 *
 *  Created on: Nov 26, 2019
 *      Author: raysw
 */
/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#ifndef __BSP_CAN
#define __BSP_CAN

#ifdef STM32F4
#include "stm32f4xx_hal.h"
#elif defined STM32F1
#include "stm32f1xx_hal.h"
#endif
#include "mytype.h"
#include "can.h"

#define ABS(x)  ( (x>0) ? (x) : (-x) )

/* Motor ID on the CAN bus */
typedef enum
{

	CAN_2006Motor_ALL_ID = 0x200,
	CAN_2006Motor1_ID = 0x201,
	CAN_2006Motor2_ID = 0x202,
	CAN_2006Motor3_ID = 0x203,
	CAN_2006Motor4_ID = 0x204,

}CAN_Motor_ID;

/* Motor measurement struct */
typedef struct{
    float    angle;        // current motor angle (range:[0,8191])
     float    last_angle;   // last motor angle (range:[0,8191])

	float	 	speed_rpm;    // motor speed (rpm)

	int16_t  	torque;       // motor real current

	int32_t		round_cnt;    // total number of turns

	uint16_t	angle_init_offset;  // initial angle after motor power on
	int32_t		total_angle;        // total angle rotated
} MotorStatus;

/* Extern  ------------------------------------------------------------------*/
extern MotorStatus  motor_status[];


void  BSP_CAN1_Init(void);
void  BSP_CAN1_Filter_Config(CAN_HandleTypeDef* _hcan);
void  CAN1_msg_to_status(MotorStatus *ptr, uint8_t aData[]);
void  CAN1_set_motor_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
void  CAN1_get_angle_init_offset(MotorStatus *ptr, uint8_t aData[]);
float CAN1_status_to_input_speed(MotorStatus *ptr);
float CAN1_status_to_input_angle(MotorStatus *ptr);
#endif

#ifndef INC_BSP_CAN_H_
#define INC_BSP_CAN_H_



#endif /* INC_BSP_CAN_H_ */
