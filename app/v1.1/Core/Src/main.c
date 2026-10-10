/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "spi.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "LCD.h"
#include "app_ota.h"   /* app 侧的 OTA 触发口（轮询 SPI，看到 PC 的字节就复位回 boot） */
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


QueueHandle_t queue_n;




#define APP_HEALTH_ADDR   0x08020018u   /* 与 boot 的 flags.h FLAG_HEALTH_ADDR 一致 */
#define APP_HEALTH_MAGIC  0xA5A5A5A5u

static void App_MarkHealthy(void)
{
    /* boot 跳转前已把整个 S5 扇区擦过，这里直接编程一个字即可，无需擦除 */
    HAL_FLASH_Unlock();
    (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APP_HEALTH_ADDR, (uint64_t)APP_HEALTH_MAGIC);
    HAL_FLASH_Lock();
}

/* 跑满 2s 才算"确认健康"：写太早会把"起得来但跑一会就崩"的坏候选误认成好的 */
static void vHealthTask(void *arg)
{
    (void)arg;
    vTaskDelay(2000);
    App_MarkHealthy();
    vTaskDelete(NULL);
}






uint8_t Digit_Num(uint32_t Num)
{
	uint8_t n = 0;
	if(Num == 0)
	{return 1;}
	while(Num != 0)
	{
		Num = Num / 10;
		n++;
	}
	return n;

}


void vLCDTask1(void *pvParameters)
{
	uint32_t n = 0;
	BaseType_t ret;
	
	while(1)
	{
		ret = xQueueReceive(queue_n, &n, portMAX_DELAY);
		LCD_DisplayString(10 + Digit_Num(n)*12, 40, 24, "S");
		if(ret == pdTRUE)
		{
			
			LCD_DisplayNum(10, 40, n, Digit_Num(n), 24, 1);
		}
	}
}

void vLCDTask2(void *pvParameters)
{

	while(1)
	{

		LCD_DisplayString_color(10, 10, 24, "APP1.1 running.  ", 60000, 65535);
		vTaskDelay(700);
		LCD_DisplayString_color(10, 10, 24, "APP1.1 running.. ", 60000, 65535);
		vTaskDelay(700);
		LCD_DisplayString_color(10, 10, 24, "APP1.1 running...", 60000, 65535);
		vTaskDelay(700);
	}	
}


void vNumTask(void *pvParameters)
{
	uint32_t n = 0;
	BaseType_t ret;
	while(1)
	{
		ret = xQueueSend(queue_n, &n, portMAX_DELAY);
		vTaskDelay(1000);
		n++;
		
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

  delay_init();   /* enable the DWT cycle counter used by delay_us/delay_ms */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  AppOta_Init();   /* 配好片选 PB12（CubeMX 里没这个脚），清事务缓冲。必须在 MX_SPI2_Init 之后 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  
	queue_n = xQueueCreate(3, sizeof(uint32_t));
	if(queue_n != NULL)
	{
		xTaskCreate(vHealthTask, "Health", 128, NULL, 4, NULL);
		xTaskCreate(vLCDTask1, "LCD1", 128, NULL, 1, NULL);
		xTaskCreate(vLCDTask2, "LCD2", 128, NULL, 2, NULL);
	    xTaskCreate(vNumTask, "Num", 128, NULL, 3, NULL);
		/* OTA 触发任务：优先级最低（1），每 100ms 跑一笔 SPI 事务，约占 1.6% CPU。
		 * ★ 它走的是**静态分配**（不占 FreeRTOS 堆），栈也给到别的任务的两倍 ——
		 *   两条理由都写在 app_ota.c 里，这里不重复。一句话：本工程的堆只有 4KB，
		 *   而堆不够时 xTaskCreate 只是**静默返回失败**（任务从来不存在，
		 *   表现是"SPI 完全没反应"）。 */
		AppOta_CreateTask();
		
		vTaskStartScheduler();
	}
  
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
