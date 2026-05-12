/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"

#include "oled.h"
#include "bsp_can.h"
#include "bsp_pid.h"
#include "usbd_cdc_if.h"
//#include "remote_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE    50
#define UART_BUF_SIZE   50
#define VALUE_STEP      200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static int isKeyProcessed = 1;
int step_sign = +1;

uint8_t motor_id_sel = 3;
uint16_t motor_angles[4];

char message[50];
float pid_value = 0.9;
uint8_t recv_val[50] = {0};

uint16_t TIM_COUNT[2];
uint16_t ADC_Value[ADC_BUF_SIZE];

uint8_t uart_flag = 1;
uint8_t UART_Value1[UART_BUF_SIZE] = {0};
uint8_t UART_Value2[UART_BUF_SIZE];


//-------------------------------------------------------------------------------------------
//the following is MCU code for CRC check,please refer.
//-------------------------------------------------------------------------------------------
uint16_t CRC_CHECK(uint8_t *Buf, uint8_t CRC_CNT)
{
    uint16_t CRC_Temp;
    uint8_t i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

void SP_Send(uint16_t value)
{
//    Ch1Data_L, Ch1Data_H,
    UART_Value2[0] = (value & 0x00FF);
    UART_Value2[1] = (value & 0xFF00) >> 8;

    HAL_UART_Transmit(&huart8, (uint8_t *)&UART_Value2, 2, 100);
}

void SP_Transmit(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4)
{
//    Ch1Data_L, Ch1Data_H,
//    Ch2Data_L, Ch2Data_H,
//    Ch3Data_L, Ch3Data_H,
//    Ch4Data_L, Ch4Data_H,
//    ChkSum
    uint16_t crc = 0;

    UART_Value2[0] = ch1 & 0x00FF;  UART_Value2[1] = (ch1 & 0xFF00) >> 8;
    UART_Value2[2] = ch2 & 0x00FF;  UART_Value2[3] = (ch2 & 0xFF00) >> 8;
    UART_Value2[4] = ch3 & 0x00FF;  UART_Value2[5] = (ch3 & 0xFF00) >> 8;
    UART_Value2[6] = ch4 & 0x00FF;  UART_Value2[7] = (ch4 & 0xFF00) >> 8;
    crc = CRC_CHECK(UART_Value2, 8);
    UART_Value2[8] = crc & 0x00FF;  UART_Value2[9] = (crc & 0xFF00) >> 8;

    HAL_UART_Transmit_IT(&huart8, UART_Value2, 10);
}

void SP_Init(uint32_t ch1_addr, uint32_t ch2_addr, uint32_t ch3_addr, uint32_t ch4_addr)
{
    //    Ch1_Addr_LL, Ch1_Addr_LH,
    //    Ch1_Addr_HL, Ch1_Addr_HH,
    //    Ch2_Addr_LL, Ch2_Addr_LH,
    //    Ch2_Addr_HL, Ch2_Addr_HH,
    //    Ch3_Addr_LL, Ch3_Addr_LH,
    //    Ch3_Addr_HL, Ch3_Addr_HH,
    //    Ch4_Addr_LL, Ch4_Addr_LH,
    //    Ch4_Addr_HL, Ch4_Addr_HH,
    //    CRC16_L, CRC16_H
    uint16_t crc = 0;

    UART_Value2[0] = ch1_addr & 0x000000FF;   UART_Value2[1] = (ch1_addr & 0x0000FF00) >> 8;  UART_Value2[2] = (ch1_addr & 0x00FF0000) >> 16;   UART_Value2[3] = (ch1_addr & 0xFF000000) >> 24;
    UART_Value2[4] = ch2_addr & 0x000000FF;   UART_Value2[5] = (ch2_addr & 0x0000FF00) >> 8;  UART_Value2[6] = (ch2_addr & 0x00FF0000) >> 16;   UART_Value2[7] = (ch2_addr & 0xFF000000) >> 24;
    UART_Value2[8] = ch3_addr & 0x000000FF;   UART_Value2[9] = (ch3_addr & 0x0000FF00) >> 8;  UART_Value2[10] = (ch3_addr & 0x00FF0000) >> 16;  UART_Value2[11] = (ch3_addr & 0xFF000000) >> 24;
    UART_Value2[12] = ch4_addr & 0x000000FF;  UART_Value2[13] = (ch4_addr & 0x0000FF00) >> 8; UART_Value2[14] = (ch4_addr & 0x00FF0000) >> 16;  UART_Value2[15] = (ch4_addr & 0xFF000000) >> 24;

    crc = CRC_CHECK((uint8_t *)&UART_Value1, 16);
    UART_Value2[16] = crc & 0xFF;
    UART_Value2[17] = crc >> 8;

    HAL_UART_Transmit(&huart8, (uint8_t *)&UART_Value2, 18, 100);
}
//-------------------------------------------------------------------------------------------
//The above is MCU code for CRC check,please refer.
//-------------------------------------------------------------------------------------------


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Determine the button status via ADC voltage value
 * @note   2.5V idle(0)
 *         1.8V UP(1)   2.2V DOWN(2)
 *         0.7V LEFT(3) 1.3V RIGHT(4)
 * @param  None
 * @retval None
 */
uint8_t OLED_Key_Scan()
{
    uint32_t ad_ch6 = 0;
    float value = 0;

    for (int i = 0; i < ADC_BUF_SIZE; i++)
    {
        ad_ch6 += ADC_Value[i];
    }
    ad_ch6 = (ad_ch6 / ADC_BUF_SIZE);
    value = ad_ch6 * 3.3f / 4096;

    if (value>2.4 && value<2.6) // idle
    {
        return 0;
    }
    else if (value>2.1 && value<2.3) // down
    {
        return 2;
    }
    else if (value>1.2 && value<1.4) // up
    {
        return 1;
    }
    else if (value>0.6 && value<0.8) // right
    {
        return 4;
    }
    else if (ABS(value-0.7) < 0.1) // right
    {
        return 3;
    }

    return 255;
}

void Key_Scan()
{
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) // button pressed
    {
        if (isKeyProcessed == 0)
        {
            isKeyProcessed = 1;

            for (int i = 0; i < PID_COUNT; i++)
            {
                pid_setval_speed[i] += VALUE_STEP * step_sign;

                if (pid_setval_speed[i] > 3000)
                {
                    pid_setval_speed[i] = 3000;
                    step_sign = -1;
                }
                if (pid_setval_speed[i] < -3000)
                {
                    pid_setval_speed[i] = -3000;
                    step_sign = 1;
                }
            }
        }
    }
    else
    {
        isKeyProcessed = 0;
    }

    uint8_t oled_key = OLED_Key_Scan();
    switch (oled_key)
    {
    case 1:
        motor_id_sel += 1;
        if (motor_id_sel >= 4)
            motor_id_sel = 0;
        break;
    case 2:
        if (motor_id_sel == 0)
            motor_id_sel = 3;
        else
            motor_id_sel -= 1;
        break;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  MX_USB_DEVICE_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */
    // Power on the motor output 24V (PH2)
    HAL_GPIO_WritePin(PWR_GPIO_Port, PWR_Pin, GPIO_PIN_SET);
    // start OLED update timer
    HAL_TIM_Base_Start_IT(&htim10);

    // start ADC DMA conversion
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADC_Value, ADC_BUF_SIZE);

    HAL_UART_Receive_IT(&huart8, UART_Value1, 6);

    // Config & Start CAN1
    BSP_CAN1_Init();

    // OLED init
    led_off();
    oled_init();
    oled_clear(Pen_Clear);
    oled_refresh_gram();

    // PID init
    BSP_PID_Init_speed();
    BSP_PID_Init_angle();
    Key_Scan();

    // initialization for serial scope
    SP_Init(0x00000000, 0x00000001, 0x00000002, 0x00000003);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   // int a=0;

    while(1){//if(a<1)
    	{//a=a+1;
       //Key_Scan();

        BSP_PID_StepOnce();

        CAN1_set_motor_current(&hcan1,
                (int16_t) speed_pid[0]->output,
                (int16_t) speed_pid[1]->output,
                (int16_t) speed_pid[2]->output,
                (int16_t) speed_pid[3]->output);

        HAL_Delay(2);  }    //PID control frequency 100Hz

//        if(uart_flag == 0)
//        {
//            HAL_UART_Transmit_IT(&huart8, (uint8_t*)&UART_Value1, UART_BUF_SIZE);
//            uart_flag = 1;
//        }
//        HAL_UART_Transmit(&huart8, (uint8_t *)"Hello, World!\n", sizeof("Hello, World!\n"), 10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i = 0;
    while(UART_Value1[i] != 'f')
    {
        recv_val[i] = UART_Value1[i];
        i++;
    }
    recv_val[i] = '\0';

    pid_value = atof((const char *)recv_val);

    speed_pid[motor_id_sel]->Kp = pid_value;

//    HAL_UART_Transmit_IT(&huart8, UART_Value1, 4);
    HAL_UART_Receive_IT(&huart8, UART_Value1, 6);
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
    static uint8_t tim_div = 0;
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

    if (htim->Instance == TIM10)
    {
        if(tim_div == 9)
        {
            tim_div = 0;
            for(int i = 0; i<4; i++)
            {
                motor_angles[i] = motor_status[i].angle;
//                motor_angles[i] = motor_status[i].speed_rpm;
            }
            SP_Transmit(motor_angles[0],
                        motor_angles[1],
                        motor_angles[2],
                        motor_angles[3]);
        }
        else
        {
            tim_div++;
        }
//        uint32_t ad_ch6 = 0;
//
//        for(int i=0; i<ADC_BUF_SIZE; i++)
//        {
//            ad_ch6 += ADC_Value[i];
//        }
//        ad_ch6 = (ad_ch6 / ADC_BUF_SIZE);



        oled_clear(Pen_Clear);
        sprintf(message, "%d, kp=%f", motor_id_sel, pid_value);
        oled_showstring1(0, 2, message);
        sprintf(message, "%.3f", pid_setval_speed[motor_id_sel]);
        oled_showstring1(1, 2, message);
        sprintf(message, "%d", motor_status[motor_id_sel].angle);
        oled_showstring1(2, 2, message);
        sprintf(message, "%d", motor_status[motor_id_sel].speed_rpm);
        oled_showstring1(3, 2, message);
        sprintf(message, "%d", motor_status[motor_id_sel].torque);
        oled_showstring1(4, 2, message);
        oled_refresh_gram();
    }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
