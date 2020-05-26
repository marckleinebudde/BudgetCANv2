/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "usbd_def.h"
#include "usbd_desc.h"
#include "usbd_core.h"
#include "usbd_gs_can.h"
#include "gpio.h"
#include "queue.h"
#include "gs_usb.h"
#include "can.h"
#include "led.h"
#include "dfu.h"
#include "timer.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */
FDCAN_ProtocolStatusTypeDef FDCAN_Status;

USBD_HandleTypeDef hUSB;
led_data_t hLED;

queue_t *q_frame_pool;
queue_t *q_from_host;
queue_t *q_to_host;

uint32_t received_count = 0;

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxTxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

uint8_t myData[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x5A, 0xA5, 0x5A, 0xA5 };

uint8_t can_msg145[] = { 0x45, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t can_mg284On[] = { 0x00, 0x00, 0x00, 0x00, 0x10, 0x10 };
uint8_t can_msg4DADoorClosed[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t can_msg264[] = { 0x01, 0xFB, 0x07, 0x80, 0x41, 0xE3, 0xC0, 0x00 };
uint8_t can_msg40B[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t can_msg459[] = { 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31 };
uint8_t can_msg45A[] = { 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31 };
uint8_t can_msg4D6[] = { 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t can_msg534[] = { 0x0A, 0x36, 0x5A, 0xA1, 0x23, 0x00, 0x00, 0x00 };
uint8_t can_msg535[] = { 0x0A, 0x36, 0x5A, 0xA1, 0x23, 0x00, 0x00, 0x00 };
uint32_t rx_count = 0;
uint32_t tx_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
static void FDCAN_Config(void);
static bool send_to_host_or_enqueue(struct gs_host_frame *frame);
static void send_to_host();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
    MX_FDCAN1_Init();

    /* USER CODE BEGIN 2 */
    /* Configure the FDCAN peripheral */
    uint32_t last_can_error_status = 7;

    flash_load();

    gpio_init();

    led_init(&hLED, LED1_GPIO_Port, LED1_Pin, LED1_Active_High, LED2_GPIO_Port, LED2_Pin, LED2_Active_High);
    led_set_mode(&hLED, led_mode_off);
    timer_init();

    can_init(&hfdcan1, FDCAN1);
    can_disable(&hfdcan1);


    q_frame_pool = queue_create(CAN_QUEUE_SIZE);
    q_from_host  = queue_create(CAN_QUEUE_SIZE);
    q_to_host    = queue_create(CAN_QUEUE_SIZE);

    struct gs_host_frame *msgbuf = calloc(CAN_QUEUE_SIZE, sizeof(struct gs_host_frame));
    for (unsigned i = 0; i < CAN_QUEUE_SIZE; i++) {
        queue_push_back(q_frame_pool, &msgbuf[i]);
    }

    USBD_Init(&hUSB, &FS_Desc, DEVICE_FS);
    USBD_RegisterClass(&hUSB, &USBD_GS_CAN);
    USBD_GS_CAN_Init(&hUSB, q_frame_pool, q_from_host, &hLED);
    USBD_GS_CAN_SetChannel(&hUSB, 0, &hfdcan1);
    USBD_Start(&hUSB);
    
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {        
        struct gs_host_frame *frame = queue_pop_front(q_from_host);
        if (frame != 0) 
        {
            // send can message from host
            if(can_send(&hfdcan1, frame))
            {
                // Echo sent frame back to host
                frame->timestamp_us = timer_get();
                send_to_host_or_enqueue(frame);

                led_indicate_trx(&hLED, led_2);
            } 
            else 
            {
                queue_push_front(q_from_host, frame);   // retry later
            }
        }

        if (USBD_GS_CAN_TxReady(&hUSB))
        {
            send_to_host();
        }

        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) >= 1)
        {
            struct gs_host_frame *frame = queue_pop_front(q_frame_pool);
            if (frame != 0)
            {
                if (can_receive(&hfdcan1, frame))
                {
                    received_count++;

                    frame->timestamp_us = timer_get();
                    frame->echo_id = 0xFFFFFFFF;   // not a echo frame
                    frame->channel = 0;
                    frame->flags = 0;
                    frame->reserved = 0;

                    send_to_host_or_enqueue(frame);

                    led_indicate_trx(&hLED, led_1);
                }
                else
                {
                    queue_push_back(q_frame_pool, frame);
                }
            }
        }

        //TODO: Make this better - currently hacked together to get it to work - use the new error codes correctly
        can_get_error_status(&hfdcan1, &FDCAN_Status);
        if (FDCAN_Status.LastErrorCode != last_can_error_status) {
            struct gs_host_frame *frame = queue_pop_front(q_frame_pool);
            if (frame != 0) 
            {
                frame->timestamp_us = timer_get();
                if (can_parse_error_status(&FDCAN_Status, frame)) 
                {
                    send_to_host_or_enqueue(frame);
                    last_can_error_status = FDCAN_Status.LastErrorCode;
                }
                else 
                {
                    queue_push_back(q_frame_pool, frame);
                }

            }
        }

        led_update(&hLED);

        if (USBD_GS_CAN_DfuDetachRequested(&hUSB)) {
            dfu_run_bootloader();
        }
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
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    /** Configure the main internal regulator output voltage 
    */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN = 12;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the peripherals clocks 
    */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

    /* USER CODE BEGIN FDCAN1_Init 0 */

    /* USER CODE END FDCAN1_Init 0 */

    /* USER CODE BEGIN FDCAN1_Init 1 */

    /* USER CODE END FDCAN1_Init 1 */
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = DISABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    hfdcan1.Init.NominalPrescaler = 6;
    hfdcan1.Init.NominalSyncJumpWidth = 1;
    hfdcan1.Init.NominalTimeSeg1 = 13;
    hfdcan1.Init.NominalTimeSeg2 = 2;
    hfdcan1.Init.DataPrescaler = 6;
    hfdcan1.Init.DataSyncJumpWidth = 1;
    hfdcan1.Init.DataTimeSeg1 = 13;
    hfdcan1.Init.DataTimeSeg2 = 2;
    hfdcan1.Init.StdFiltersNbr = 0;
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN FDCAN1_Init 2 */

    /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
bool send_to_host_or_enqueue(struct gs_host_frame *frame)
{
    if (USBD_GS_CAN_GetProtocolVersion(&hUSB) == 2) {
        queue_push_back(q_to_host, frame);
        return true;

    }
    else {
        bool retval = false;
        if (USBD_GS_CAN_SendFrame(&hUSB, frame) == USBD_OK) {
            queue_push_back(q_frame_pool, frame);
            retval = true;
        }
        else {
            queue_push_back(q_to_host, frame);
        }
        return retval;
    }
}

void send_to_host()
{
    struct gs_host_frame *frame = queue_pop_front(q_to_host);

    if (!frame)
        return;

    if (USBD_GS_CAN_SendFrame(&hUSB, frame) == USBD_OK) {
        queue_push_back(q_frame_pool, frame);
    }
    else {
        queue_push_front(q_to_host, frame);
    }
}
/* USER CODE END 4 */

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
