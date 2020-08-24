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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/*!
 * System status
 */


/* USER CODE BEGIN PTD */
typedef enum SystemStatus
{
	STATE_READY = 0,
	STATE_CHANGE,
	STATE_UPDATED,
}SystemStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define F_PWM 		10000			//Tần số xung nhỏ
#define PWM_RELOAD 	4800			//Giá trị nạp timer (Max)
#define PI			3.141592
#define MAX_SIZE	2000
#define	START_FREQ	10				//Giá trị tần số mặc định
#define BUTTON_STEP	5				//Giá trị thay đổi mỗi lần ấn nút nhấn
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SystemStatus g_pwmstate=STATE_CHANGE;		//Trạng thái PWM (Thay đổi, Tính toán, cập nhật)
uint16_t g_freqspwm=START_FREQ;				//Tần số xung lớn
uint16_t g_numpul=0;						//Số xung nhỏ
uint16_t g_runningtable[MAX_SIZE]={0};		//Bảng chứa giá trị chạy
uint16_t g_caltable[MAX_SIZE]={0};			//Bảng chứa giá trị tính toán khi thay đổi
uint16_t g_countb1press=0;
float fl_radianperstep=0.00;				//Số radian (pi <=> 180 độ) mỗi xung
volatile bool g_b1pressed = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Sys_Init(void);
void UpdateSineTable(void);
void ButtonsHandler(void);
void SPWMValueHandler(void);
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
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  Sys_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	SPWMValueHandler();
	ButtonsHandler();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Khởi tạo Timer
 *
 */
void Sys_Init(void)
{
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1); 	//Start PWM
	HAL_TIM_Base_Start_IT(&htim14);				//Start timer interrupt overflow
}

/*
 * Xử lý tần số PWM
 */
void SPWMValueHandler(void)
{
	switch (g_pwmstate)
	{
		case STATE_CHANGE:	//Giá trị tần số thay đổi-> Tính lại số xung và bảng giá trị
			g_numpul = F_PWM / g_freqspwm;
			fl_radianperstep = PI / (g_numpul/2);
			UpdateSineTable();
			g_pwmstate = STATE_UPDATED;
			break;

		case STATE_UPDATED:	//Tính toán xong, copy giá trị từ bảng phụ -> bảng chạy
			memset(g_runningtable, 0, (sizeof(g_runningtable[0])*MAX_SIZE));
			memcpy(g_runningtable, g_caltable, sizeof(g_runningtable[0])*(g_numpul/2) );
			g_pwmstate = STATE_READY;
			break;

		default:
			break;
	}
}

/*
 *  Copy giá trị bảng phụ -> Bảng chạy
 */
void UpdateSineTable(void)
{
	memset(g_caltable, 0, (sizeof(g_caltable[0])*MAX_SIZE));
	for (int ui16index = 0; ui16index < g_numpul/2; ++ui16index)
	{
		g_caltable[ui16index] = (uint16_t) (PWM_RELOAD * sinf(fl_radianperstep * ui16index));
	}
}

/*
 * Xử lý nút nhấn
 */
void ButtonsHandler(void)
{
	if (g_b1pressed == true)
	{
		if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) //Release
		{
			if(g_countb1press <100)
			{

			}
			else
			{
				g_freqspwm += BUTTON_STEP;
				g_pwmstate = STATE_CHANGE;
			}
			g_b1pressed = false;
			g_countb1press = 0;
		}
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_Pin)
	{
		g_b1pressed = true;
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	UNUSED(htim);
	static uint16_t ui16count=0;
	uint16_t ui16reload = 0;
	if(ui16count < (g_numpul/2) )
	{
		ui16reload = g_runningtable[ui16count];
		__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, ui16reload );
	}
	else if (ui16count < g_numpul)
	{
		__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
	}
	else
	{
		ui16count = 0;
	}
	ui16count++;

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
