/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usb_host.h"
#include "gpio.h"
#include "stdbool.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct item item;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define DEBOUNCE_TIME 30 //100
#define AFTER_PUSH_TIME 1000
#define BLINK_TIME 200
uint32_t Tick, PrevStatus;
uint32_t RisingSlope=0, FallingSlope=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct item
{
	uint32_t Port;
	uint32_t Pin;
	bool Status;
	bool PrevStatus;
	bool Temp;
	uint32_t WhenOn;
	uint32_t WhenOff;
	uint8_t Count;
	bool UnLock;
};

item BlueButton;
item RedLED;
item BlueLED;

void Flash(int n, int time_ms) // only for debugging function
{
	for(int i=0; i<2*n; i++)
	{
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
		HAL_Delay(time_ms);
	}
}

void HardwareUpdate()
{


	  Tick=HAL_GetTick();

      BlueButton.Status=HAL_GPIO_ReadPin(BlueButton.Port, BlueButton.Pin);
	  HAL_GPIO_WritePin(RedLED.Port, RedLED.Pin, RedLED.Status);
	  HAL_GPIO_WritePin(BlueLED.Port, BlueLED.Pin, BlueButton.Status);

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

  void SetStartValues()
{
	   Tick=HAL_GetTick();

	   BlueButton.Port=(uint32_t) GPIOA;
	   BlueButton.Pin=GPIO_PIN_0;
	   BlueButton.Status=0;
	   BlueButton.PrevStatus=0;
	   BlueButton.Temp=0;
	   BlueButton.WhenOn=Tick;
	   BlueButton.WhenOff=Tick;
	   BlueButton.Count=0;
	   BlueButton.UnLock=1;

	   RedLED.Port=(uint32_t) GPIOD;
	   RedLED.Pin=GPIO_PIN_13;
	   RedLED.Status=0;
	   RedLED.PrevStatus=0;
	   RedLED.Temp=0;
	   RedLED.WhenOn=Tick;
	   RedLED.WhenOff=Tick;
	   RedLED.Count=0;
	   RedLED.UnLock=0;

	   BlueLED.Port=(uint32_t) GPIOD;
	   BlueLED.Pin=GPIO_PIN_15;
	   BlueLED.Status=0;
	   BlueLED.PrevStatus=0;
	   BlueLED.Temp=0;
	   BlueLED.WhenOn=Tick;
	   BlueLED.WhenOff=Tick;
	   BlueLED.Count=0;
	   BlueLED.UnLock=1;

}

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

	  SetStartValues();

   while(1)
   {
	  if (BlueButton.UnLock) BlueButton.PrevStatus=BlueButton.Status;

	  HardwareUpdate();

//******************* Counting *******************
	  if ((!BlueButton.PrevStatus) && (BlueButton.Status)) BlueButton.WhenOn=Tick;
	  if ((BlueButton.PrevStatus) && (!BlueButton.Status)) BlueButton.WhenOff=Tick;

	  if ((!BlueButton.Temp) && (BlueButton.PrevStatus!=BlueButton.Status))
		  BlueButton.Temp=1; // latch first slope

	  if ((BlueButton.Temp) &&
			  ((Tick- BlueButton.WhenOn)>DEBOUNCE_TIME) &&
			  ((Tick- BlueButton.WhenOff)>DEBOUNCE_TIME) &&
			  (!BlueButton.Status))
	  {
		  BlueButton.Count++;
	  	  BlueButton.Temp=0;
	  }

//******************* Blinking *******************
	  if (((Tick-BlueButton.WhenOff)>AFTER_PUSH_TIME) && (BlueButton.Count) && (!BlueButton.Status) && (!BlueButton.PrevStatus))
	  {
		  BlueButton.UnLock=0;
		  RedLED.UnLock=1;
		  RedLED.Count=2*BlueButton.Count;
		  BlueButton.Count=0;
	  }

	  if (RedLED.UnLock)
	  {
		  if ((RedLED.Status==1) && ((Tick-RedLED.WhenOn)>BLINK_TIME))
		  {
			  RedLED.Status=0;
			  RedLED.WhenOff=Tick;
			  RedLED.Count--;
		  }
		  if ((RedLED.Status==0) && ((Tick-RedLED.WhenOff)>BLINK_TIME))
		  {
			  RedLED.Status=1;
			  RedLED.WhenOn=Tick;
			  RedLED.Count--;
		  }
		  if (RedLED.Count==0)
			  {
			  	  RedLED.UnLock=0;
			  	  BlueButton.UnLock=1;
			  }
	  }
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
