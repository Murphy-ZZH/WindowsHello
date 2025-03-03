/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "rc522.h"
#include "usbd_hid.h"
#include "hc-sr04.h"
#include "usbd_conf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t KeyBoard[8] = {0,0,4,0,0,0,0,0};
uint8_t KeyBoard01[8] = {0,0,0,0,0,0,0,0};
extern USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_FS;
int sign2=0;//来人之后空格按下后锁定，人走后空格启用
int sign1;//人走后win+l，锁定快捷键，人来后快捷键启用
int sign3=0;//消除测距模块抖动
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void SendHIDReport(uint8_t keyValue) {
    KeyBoard[2] = keyValue;

    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&KeyBoard01, sizeof(KeyBoard));
    HAL_Delay(15);
    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&KeyBoard, sizeof(KeyBoard01));
    HAL_Delay(15);
    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&KeyBoard01, sizeof(KeyBoard));
    HAL_Delay(15);
}

void SendMultipleKeys(uint8_t key1, uint8_t key2) {
    KeyBoard[0] = key1; // 修饰键
    KeyBoard[2] = key2;      // 主键
    USBD_HID_SendReport(&hUsbDeviceFS, KeyBoard, sizeof(KeyBoard));
    HAL_Delay(15);

    // 清除按键状态
    KeyBoard[0] = 0;
    KeyBoard[2] = 0;
    USBD_HID_SendReport(&hUsbDeviceFS, KeyBoard, sizeof(KeyBoard));
    HAL_Delay(15);
}
void password(void)
{
SendHIDReport(44);
HAL_Delay(800);
SendHIDReport(31);
SendHIDReport(31); // 再次发送31
SendHIDReport(32);
SendHIDReport(39);
printf("unlocked!\n");
}

void lock(void)
{
	SendMultipleKeys(0x08,0x0F);
}
void WakeUP() {
        if (hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED) {
            // 使能远程唤醒
            USBD_LL_RemoteWakeup(&hUsbDeviceFS);
            HAL_Delay(10);
            // 禁用远程唤醒
            HAL_PCD_ActivateRemoteWakeup(&hpcd_USB_FS);
            HAL_Delay(10);
            HAL_PCD_DeActivateRemoteWakeup(&hpcd_USB_FS);
					printf("WakeUP!\n");
        }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  RC522_Init();
	Hcsr04Init(&htim4, TIM_CHANNEL_1);  //  超声波模块初始化
  Hcsr04Start();  //  开启超声波模块测距
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_Delay(500);
		Hcsr04Start();
		if(Hcsr04Read()<=75){
			if(sign==0){
			if(sign2==0&&sign1==1)
			{
				HAL_Delay(100);
				SendHIDReport(44);
				sign2=1;
			}
			ReaderCard();
			sign1=0;
		}
			sign3=0;
		WakeUP();
			printf("sign3=%d humans detected, waiting lock,distance:%.1fcm\n",sign3,Hcsr04Read());
		}
		else if(Hcsr04Read()>75){  
			sign3++;
			if(sign3==5)
			{
				printf("sign3=%d locked! distance too far! %.1f\n",sign3,Hcsr04Read());
				sign=0;
				sign2=0;
				if(sign1==0)
				{
					lock();
					sign1=1;
				}
			}
			else if(sign3>10) sign3=10;
		}
		printf("sign3=%d",sign3);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @description: 定时器输入捕获中断
 * @param {TIM_HandleTypeDef} *htim
 * @return {*}
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  Hcsr04TimIcIsr(htim);
}

/**
 * @description: 定时器溢出中断
 * @param {*}
 * @return {*}
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		Hcsr04TimOverflowIsr(htim);
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
