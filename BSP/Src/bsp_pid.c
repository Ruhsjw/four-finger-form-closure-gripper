/*
 * bsp_pid.c
 *
 *  Created on: Sep 17, 2020
 *      Author: raysw
 */
#include "bsp_pid.h"
#include "bsp_can.h"


float target_angle =270;
uint64_t target_speed = 30;
PIDController pid_ctrl_speed[PID_COUNT] = {0};
PIDController pid_ctrl_angle[PID_COUNT] = {0};
PIDPtr speed_pid[4];
PIDPtr angle_pid[4];

float pid_input_speed[PID_COUNT];
float pid_input_angle[PID_COUNT];
float pid_output_speed[PID_COUNT];
float pid_output_angle[PID_COUNT];
float pid_setval_speed[PID_COUNT];
float pid_setval_angle[PID_COUNT];

void BSP_PID_Init_speed(void)
{
    float kp = 1, ki = 0, kd = 0;
//    float kp = 1.0, ki = 0.0, kd = 0.00;

    for(int i = 0; i < PID_COUNT; i++)
    {
        pid_input_speed[i] = 0;
        pid_output_speed[i] = 0;
        pid_setval_speed[i] = 0;

        // Prepare PID controller for operation
        speed_pid[i] = pid_create(&pid_ctrl_speed[i], pid_input_speed[i],pid_output_speed[i], pid_setval_speed[i],
                kp, ki, kd);
        // Set controller output limits from 0 to 16384
       // pid_limits(pid[i], -16384, 16384);
        // Allow PID to compute and change output
        pid_auto(speed_pid[i]);
    }
}
void BSP_PID_Init_angle(void)
{
    float kp = 1, ki = 0, kd = 0;
//    float kp = 1.0, ki = 0.0, kd = 0.00;

    for(int i = 0; i < PID_COUNT; i++)
    {
        pid_input_angle[i] = 0;
        pid_output_angle[i] = 0;
        pid_setval_angle[i] = 0;

        // Prepare PID controller for operation
        angle_pid[i] = pid_create(&pid_ctrl_angle[i], pid_input_angle[i],pid_output_angle[i], pid_setval_angle[i],
                kp, ki, kd);
        // Set controller output limits from 0 to 16384
       // pid_limits(pid[i], -16384, 16384);
        // Allow PID to compute and change output
        pid_auto(angle_pid[i]);
    }
}
void BSP_PID_StepOnce(void)
{
    for(int i = 0; i < PID_COUNT; i++)
    {  angle_pid[i]->omin=-6000;
       angle_pid[i]->omax=6000;
       speed_pid[i]->omin=-6000;
       speed_pid[i]->omax=6000;
         pid_input_angle[i] = CAN1_status_to_input_angle(&motor_status[i]); // update this value from CAN bus
            pid_angle_comput(angle_pid[i],target_angle,pid_input_angle[i]);

            pid_input_speed[i] = CAN1_status_to_input_speed(&motor_status[i]);
            pid_speed_comput(speed_pid[i],angle_pid[i]->output,pid_input_speed[i]);
            speed_pid[i] = speed_pid[i]; // output is updated via pid_compute

    }
}

