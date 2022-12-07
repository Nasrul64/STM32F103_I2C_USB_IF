/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_TX_BUFFER_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define USB_TX(A) strcpy(szUSBTxBuf, A); CDC_Transmit_FS((uint8_t*)szUSBTxBuf, (uint16_t) strlen(szUSBTxBuf)); HAL_Delay(1);
#define USB_TX_V(A,B) sprintf(szUSBTxBuf, A, B); CDC_Transmit_FS((uint8_t*)szUSBTxBuf, (uint16_t) strlen(szUSBTxBuf)); HAL_Delay(1);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char szUSBTxBuf[USB_TX_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void setLED(uint8_t bOn);
uint8_t ReadI2C(uint8_t slv7bitAddr, uint8_t* data, uint8_t len, uint8_t timeout);
uint8_t WriteI2C(uint8_t slv7bitAddr, uint8_t* data, uint8_t len, uint8_t timeout);
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
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  for(int i=0;i<3;i++)
  {
	  HAL_Delay(250); setLED(1);
	  HAL_Delay(250); setLED(0);
  }

  //USB_TX("I2C USB IF Started\r\n")

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#define CMD_REP_BUFSIZE 10

  uint8_t iBuf[CMD_REP_BUFSIZE];
  uint8_t oBuf[CMD_REP_BUFSIZE];
  uint8_t bSucc;

  /*
   * in[0] = 0x5A
   * in[1] = (7bit Addr << 1) + (1bit R/W)
   * in[2] = data length
   * in[3] = timeout (ms)
   * in[4] = Data 0 (For Write Only)
   * in[5] = Data 1 (For Write Only)
   * in[n] = Data N (For Write Only)
   *
   * out[0] = 0xA5
   * out[1] = 0x0F=OK / 0xF0=NOK
   * out[2] = Data 0 (For Read Only)
   * out[3] = Data 1 (For Read Only)
   * out[n] = Data N (For Read Only)
   *
   */

  uint8_t temp7bitAddr;
  uint8_t tempLength;
  uint8_t tempTimeOut;

  while (1)
  {
	  if(CDC_GetRxBufferBytesAvailable_FS()>0)
	  {
		  setLED(1);

		  memset(oBuf, 0, CMD_REP_BUFSIZE);

		  if(CDC_ReadRxBuffer_FS(iBuf, 1)==USB_CDC_RX_BUFFER_OK)
		  {
			  if(iBuf[0]==0x5A && CDC_GetRxBufferBytesAvailable_FS()>=3)
			  {
				  if(CDC_ReadRxBuffer_FS(iBuf, 3)==USB_CDC_RX_BUFFER_OK)
				  {
					  temp7bitAddr = iBuf[0]>>1;
					  tempLength = iBuf[1];
					  tempTimeOut = iBuf[2];

					  if(iBuf[0] & 0x1)
					  {
						  // I2C Read

						  bSucc = ReadI2C(temp7bitAddr, &oBuf[2], tempLength, tempTimeOut);

						  oBuf[0] = 0xA5;
						  oBuf[1] = bSucc?0x0F:0xF0;

						  CDC_Transmit_FS(oBuf, 2 + tempLength);
					  }
					  else if(CDC_ReadRxBuffer_FS(iBuf, tempLength)==USB_CDC_RX_BUFFER_OK)
					  {
						  // I2C Write

						  bSucc = WriteI2C(temp7bitAddr, iBuf, tempLength, tempTimeOut);

						  oBuf[0] = 0xA5;
						  oBuf[1] = bSucc?0x0F:0xF0;

						  CDC_Transmit_FS(oBuf, 2);
					  }
				  }
			  }
			  else if(iBuf[0]=='c'||iBuf[0]=='C')
			  {
				  CDC_FlushRxBuffer_FS();
			  }
			  else if(iBuf[0]=='h'||iBuf[0]=='H')
			  {
				  USB_TX("HelloWorld");
			  }
			  else
			  {
				  // Do nothing
			  }
		  }

		  setLED(0);
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
void setLED(uint8_t bOn)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, bOn? GPIO_PIN_RESET:GPIO_PIN_SET);
}

uint8_t ReadI2C(uint8_t slv7bitAddr, uint8_t* data, uint8_t len, uint8_t timeout)
{
	HAL_StatusTypeDef halStat = HAL_I2C_Master_Receive(&hi2c1, slv7bitAddr<<1, data, len, timeout);

	return halStat==HAL_OK?1:0;
}

uint8_t WriteI2C(uint8_t slv7bitAddr, uint8_t* data, uint8_t len, uint8_t timeout)
{
	HAL_StatusTypeDef halStat = HAL_I2C_Master_Transmit(&hi2c1, slv7bitAddr<<1, data, len, timeout);

	return halStat==HAL_OK?1:0;
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
