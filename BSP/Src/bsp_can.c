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

#include "can.h"
#include "bsp_can.h"
#include "bsp_pid.h"
#include "oled.h"
#include "stdio.h"

char message[50];
MotorStatus motor_status[4] =
{ 0 };
uint8_t isInitialized = 0;

/*******************************************************************************************
 * @Func	my_can_filter_init
 * @Brief   CAN filter configuration
 * @Param	CAN_HandleTypeDef* hcan
 * @Retval	None
 * @Date    2019/11/27
 *******************************************************************************************/
void BSP_CAN1_Filter_Config(CAN_HandleTypeDef *_hcan)
{
    CAN_FilterTypeDef sCanFilterConf;

    sCanFilterConf.FilterBank = 0;
    sCanFilterConf.FilterMode = CAN_FILTERMODE_IDMASK;
    sCanFilterConf.FilterScale = CAN_FILTERSCALE_32BIT;
    sCanFilterConf.FilterIdHigh = 0x0000;
    sCanFilterConf.FilterIdLow = 0x0000;
    sCanFilterConf.FilterMaskIdHigh = 0x0000;
    sCanFilterConf.FilterMaskIdLow = 0x0000;
    sCanFilterConf.FilterFIFOAssignment = CAN_FilterFIFO0;
    sCanFilterConf.SlaveStartFilterBank = 14; //can1(0-13)和can2(14-27)分别得到一半的filter
    sCanFilterConf.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(_hcan, &sCanFilterConf) != HAL_OK)
    {
        //err_deadloop();
        //show error!
        Error_Handler();
    }
}

void BSP_CAN1_Init(void)
{
    BSP_CAN1_Filter_Config(&hcan1);

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
            != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }
}

uint32_t FlashTimer;
/*******************************************************************************************
 * @Func	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
 * @Brief   HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
 * @Param
 * @Retval	None
 * @Date    2019/11/27
 *******************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
    if (HAL_GetTick() - FlashTimer > 500)
    {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        FlashTimer = HAL_GetTick();
    }

    if (HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        Error_Handler();
    }

    if (isInitialized == 0)
    {
        isInitialized = 1;
        CAN1_get_angle_init_offset(&motor_status[0], RxData);
        CAN1_get_angle_init_offset(&motor_status[1], RxData);
        CAN1_get_angle_init_offset(&motor_status[2], RxData);
        CAN1_get_angle_init_offset(&motor_status[3], RxData);
    }

    switch (RxHeader.StdId)
    {
    case CAN_2006Motor1_ID:
    case CAN_2006Motor2_ID:
    case CAN_2006Motor3_ID:
    case CAN_2006Motor4_ID:
    {
        static u8 i;
        i = RxHeader.StdId - CAN_2006Motor_ALL_ID - 1;

        CAN1_msg_to_status(&motor_status[i], RxData);

    }
        break;
    }
}

/*******************************************************************************************
 * @Func	 void CAN1_msg_to_status(MotorStatus *ptr, uint8_t aData[])
 * @Brief    C610 data format via CAN bus
 *           aData[0] << 8 | aData[1]  motor angle  (range 0~8191)
 *           aData[2] << 8 | aData[3]  motor speed  (rpm)
 *           aData[4] << 8 | aData[5]  motor torque (Nm)
 *           aData[6] null
 *           aData[7] null
 * @Param
 * @Retval	 None
 * @Date     2019/11/27
 *******************************************************************************************/
#define SPD_FILTER_LEN 20
void CAN1_msg_to_status(MotorStatus *ptr, uint8_t aData[])
{
 /*   static int16_t spd_filter[SPD_FILTER_LEN] = {0};
    static uint8_t spd_idx = 0;
    int32_t spd_sum = 0;*/

    // motor angle ----------------------------------

    ptr->angle = (float)(aData[0] << 8 | aData[1]);
    ptr->angle = ptr->angle/0x1fff*360.f-180.f;
    ptr->last_angle = ptr->angle;
    // motor speed ----------------------------------
    ptr->speed_rpm=(int16_t)(aData[2] << 8 | aData[3]);

    // get current speed
    //spd_filter[spd_idx] = (int16_t) (aData[2] << 8 | aData[3]);
    //spd_idx++;
    //if(spd_idx == SPD_FILTER_LEN) spd_idx = 0;
    // get average speed
    /*for(int i=0; i<SPD_FILTER_LEN; i++)
    {
        spd_sum += spd_filter[i];
    }
    ptr->speed_rpm = spd_sum / SPD_FILTER_LEN;*/

   // ptr->speed_rpm = (int16_t) (aData[2] << 8 | aData[3]);

    // motor torque ---------------------------------
    ptr->torque = (int16_t) (aData[4] << 8 | aData[5]);

    // motor round count ----------------------------
    if (ptr->angle - ptr->last_angle > 360)
        ptr->round_cnt++;
    else if (ptr->angle - ptr->last_angle < -360)
        ptr->round_cnt--;

    // motor total angle ----------------------------
    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle
            - ptr->angle_init_offset;

}

float CAN1_status_to_input_speed(MotorStatus *ptr)
{
    // ptr->speed_rpm: speed control
    // ptr->angle:     position control
    // we want position PID controller therefore
    // we set angles as pid_input
    return(ptr->speed_rpm);

//    return(ptr->angle);
}
float CAN1_status_to_input_angle(MotorStatus *ptr)
{
    // ptr->speed_rpm: speed control
    // ptr->angle:     position control
    // we want position PID controller therefore
    // we set angles as pid_input
    //return(ptr->speed_rpm);

   return(ptr->angle);
}
/*******************************************************************************************
 * @Func     void get_moto_offset(moto_measure_t *ptr, uint8_t aData[])
 * @Brief    Get angle initial offset. This function should be called after
 *           system+can initialization
 *
 * @Param
 * @Retval   None
 * @Date     2019/11/27
 *******************************************************************************************/

void CAN1_get_angle_init_offset(MotorStatus *ptr, uint8_t aData[])
{
    ptr->angle = (uint16_t) (aData[0] << 8 | aData[1]);
    ptr->angle_init_offset = ptr->angle;
}

void CAN1_set_motor_current(CAN_HandleTypeDef *_hcan, s16 iq1, s16 iq2, s16 iq3,
        s16 iq4)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];
    uint32_t txMailbox;

    txHeader.StdId = 0x200;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 0x08;

    txData[0] = (iq1 >> 8);
    txData[1] = iq1;
    txData[2] = (iq2 >> 8);
    txData[3] = iq2;
    txData[4] = (iq3 >> 8);
    txData[5] = iq3;
    txData[6] = (iq4 >> 8);
    txData[7] = iq4;

    /* Start the Transmission process */
    if (HAL_CAN_AddTxMessage(_hcan, &txHeader, txData, &txMailbox) != HAL_OK)
    {
        /* Transmission request Error */
        Error_Handler();
    }
}
