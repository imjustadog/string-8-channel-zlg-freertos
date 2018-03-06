/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include "CW201x.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim2_ch3;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;
DMA_HandleTypeDef hdma_tim3_ch1_trig;
DMA_HandleTypeDef hdma_tim3_ch3;
DMA_HandleTypeDef hdma_tim3_ch4_up;
DMA_HandleTypeDef hdma_tim4_ch3;
DMA_HandleTypeDef hdma_tim8_ch3_up;
DMA_HandleTypeDef hdma_tim8_ch4_trig_com;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

osThreadId TASK_BATHandle;
osThreadId TASK_ZLGHandle;
osThreadId TASK_CAPTUREHandle;
osThreadId TASK_485Handle;
osTimerId Timer1sHandle;
osSemaphoreId BinarySem_captureHandle;
osSemaphoreId BinarySem_zlgHandle;
osSemaphoreId BinarySem_485Handle;
osSemaphoreId CountingSem_dmacpltHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/*********************** BATTERY *************************/	
extern STRUCT_CW_BATTERY   cw_bat;

/****************** CAPRURE FREQUENCY ********************/	

uint16_t input_capture[8][30];
float avrg_freq[8];
int dma_size = 25;

GPIO_TypeDef* STRING_GPIO[8] = {
	GPIOC, 
	GPIOC, 
	GPIOA,
	GPIOA,
  GPIOA, 
  GPIOA,
	GPIOB,
	GPIOB};

uint16_t STRING_PIN[8] = {
	GPIO_PIN_6,
  GPIO_PIN_7,
	GPIO_PIN_15,
	GPIO_PIN_7,
	GPIO_PIN_8,
	GPIO_PIN_11,
	GPIO_PIN_14,
	GPIO_PIN_15};
	
uint32_t TIM_CHANNEL[8] = {
	TIM_CHANNEL_3,
	TIM_CHANNEL_4,
	TIM_CHANNEL_3,
	TIM_CHANNEL_4,
	TIM_CHANNEL_3,
	TIM_CHANNEL_1,
	TIM_CHANNEL_4,
  TIM_CHANNEL_3};

TIM_HandleTypeDef *htim[8];
	
/************************* ADC ***************************/	
	
float adcbuf[8] = {0};
float	A = 0.0014051f;
float B = 0.0002369f;
float C = 0.0000001019f;

/************************* ADDR ***************************/	

uint8_t board_num1;
uint8_t board_num2;

GPIO_TypeDef* ADDR_GPIO[8] = {
	GPIOB, 
	GPIOB, 
  GPIOB, 
  GPIOB,
	GPIOB,
	GPIOC,
	GPIOC,
	GPIOA};

uint16_t ADDR_PIN[8] = {
	GPIO_PIN_4,
  GPIO_PIN_5,
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_9,
	GPIO_PIN_13,
	GPIO_PIN_12,
  GPIO_PIN_12};

/********************** ANALYSIS **************************/	

uint8_t data_buf[46] = {
	'S',
	0,0,//ID high, ID low
	0,0,0,0,0, //CH1 freq high, freq low, temp high, temp low
	1,0,0,0,0, //CH2 
	2,0,0,0,0, //CH3 
	3,0,0,0,0, //CH4 
	4,0,0,0,0, //CH5 
	5,0,0,0,0, //CH6 
	6,0,0,0,0, //CH7 
	7,0,0,0,0, //CH8 
	0,0,       //bat high, bat low
  'E'};

uint8_t reply_buf[36] = {
	'S',
	0,0,//ID
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0xff,
	0xff,
	0x00,
	0xff,
  'A'};

uint8_t order_buf[35];
	
/************************* UART ***************************/	
	
uint8_t data_zlg;
uint8_t len_zlg;
uint8_t rxbuf_zlg[100];

uint8_t data_485;
uint8_t len_485 = 1;
uint8_t rxbuf_485[100];	
	
/*********************** ZLG 5168 *************************/	

uint8_t channel = 11;
	
enum{
	GET_INFO = 0,
	SET_INFO,
	RESET_ZLG,
	SHOW_ADDR,
	WAITFORCMD,
	SLEEP,
};	
uint8_t ZLGSTATE = GET_INFO;

uint8_t cmd_cfg_get[5] = {0xAB,0xBC,0xCD,0xD1,0x05};
struct struct_recv
{
	uint8_t cmd[4];
	uint8_t dev_info[65];
	uint8_t status[5];  
};
union union_recv
{
	uint8_t recv_buf[74];
	struct struct_recv recv_analy;
}ack_cfg_get;

struct struct_send
{
	uint8_t cmd[4];
	uint8_t local_addr[2];
	uint8_t dev_info[65];
	uint8_t veri;  
};
union union_send
{
	uint8_t send_buf[72];
	struct struct_send send_set;
}cmd_cfg_set;
uint8_t ack_cfg_set[7] = {0xAB,0xBC,0xCD,0xD6,0x20,0x01,0x00};

uint8_t cmd_reset[9] = {0xAB,0xBC,0xCD,0xD9,0x20,0x01,0x00,0x01,0x2F};

uint8_t cmd_show_addr[5] = {0xDE, 0xDF, 0xEF, 0xD3, 0x01};
uint8_t ack_show_addr[5] = {0xDE, 0xDF, 0xEF, 0xD3, 0x00};

uint8_t cmd_sleep[5] = {0xDE, 0xDF, 0xEF, 0xD8, 0x01};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM8_Init(void);
void FUNC_BAT(void const * argument);
void FUNC_ZLG(void const * argument);
void FUNC_CAPTURE(void const * argument);
void FUNC_485(void const * argument);
void Callback_timer1s(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void read_ID()
{
	int i;
	board_num1 = 0;
	board_num2 = 0;
	for(i = 0;i < 4;i ++)
	{
		if(HAL_GPIO_ReadPin(ADDR_GPIO[i], ADDR_PIN[i]) == GPIO_PIN_SET)
		{
			board_num1 |= (1 << i);
		}
	}
	for(i = 0;i < 4;i ++)
	{
		if(HAL_GPIO_ReadPin(ADDR_GPIO[i + 4], ADDR_PIN[i + 4]) == GPIO_PIN_SET)
		{
			board_num2 |= (1 << i);
		}
	}
	board_num1=1;
	board_num2=1;
	
	data_buf[1] = board_num1;
	data_buf[2] = board_num2;
}

void write_to_data_buf(int num,uint16_t Freq, uint16_t Temp)
{
	data_buf[3 + num * 5 + 1] = (uint8_t)((Freq&0xff00)>>8);
	data_buf[3 + num * 5 + 2] = (uint8_t)(Freq&0x00ff);
	data_buf[3 + num * 5 + 3] = (uint8_t)((Temp&0xff00)>>8);
	data_buf[3 + num * 5 + 4] = (uint8_t)(Temp&0x00ff);
}

void get_temperature()
{
	int i;
	for(i = 0;i < 8;i ++)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,1000);
		adcbuf[i]=HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	int ret;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	htim[0] = &htim2;
	htim[1] = &htim2;
	htim[2] = &htim3;
	htim[3] = &htim3;
	htim[4] = &htim4;
	htim[5] = &htim3;
	htim[6] = &htim8;
	htim[7] = &htim8;
	
	cmd_cfg_set.send_set.cmd[0] = 0xAB;
  cmd_cfg_set.send_set.cmd[1] = 0xBC;
  cmd_cfg_set.send_set.cmd[2] = 0xCD;
  cmd_cfg_set.send_set.cmd[3] = 0xD6;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();

  /* USER CODE BEGIN 2 */
  ret = cw_bat_init();
	
	read_ID();
	cw_bat_work();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinarySem_capture */
  osSemaphoreDef(BinarySem_capture);
  BinarySem_captureHandle = osSemaphoreCreate(osSemaphore(BinarySem_capture), 1);

  /* definition and creation of BinarySem_zlg */
  osSemaphoreDef(BinarySem_zlg);
  BinarySem_zlgHandle = osSemaphoreCreate(osSemaphore(BinarySem_zlg), 1);

  /* definition and creation of BinarySem_485 */
  osSemaphoreDef(BinarySem_485);
  BinarySem_485Handle = osSemaphoreCreate(osSemaphore(BinarySem_485), 1);

  /* definition and creation of CountingSem_dmacplt */
  osSemaphoreDef(CountingSem_dmacplt);
  CountingSem_dmacpltHandle = osSemaphoreCreate(osSemaphore(CountingSem_dmacplt), 8);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of Timer1s */
  osTimerDef(Timer1s, Callback_timer1s);
  Timer1sHandle = osTimerCreate(osTimer(Timer1s), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of TASK_BAT */
  osThreadDef(TASK_BAT, FUNC_BAT, osPriorityLow, 0, 128);
  TASK_BATHandle = osThreadCreate(osThread(TASK_BAT), NULL);

  /* definition and creation of TASK_ZLG */
  osThreadDef(TASK_ZLG, FUNC_ZLG, osPriorityNormal, 0, 128);
  TASK_ZLGHandle = osThreadCreate(osThread(TASK_ZLG), NULL);

  /* definition and creation of TASK_CAPTURE */
  osThreadDef(TASK_CAPTURE, FUNC_CAPTURE, osPriorityBelowNormal, 0, 128);
  TASK_CAPTUREHandle = osThreadCreate(osThread(TASK_CAPTURE), NULL);

  /* definition and creation of TASK_485 */
  osThreadDef(TASK_485, FUNC_485, osPriorityNormal, 0, 64);
  TASK_485Handle = osThreadCreate(osThread(TASK_485), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA7 PA8 
                           PA9 PA10 PA11 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB14 PB15 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB4 PB5 
                           PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* FUNC_BAT function */
void FUNC_BAT(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* FUNC_ZLG function */
void FUNC_ZLG(void const * argument)
{
  /* USER CODE BEGIN FUNC_ZLG */
	BaseType_t pdsem = pdFALSE;
	uint8_t addr1_read, addr2_read, channel_read;
	uint8_t checksum;
	uint8_t i,len;
	HAL_UART_Receive_DMA(&huart3,ack_cfg_get.recv_buf,sizeof(ack_cfg_get.recv_buf));
	HAL_UART_Transmit(&huart3,cmd_cfg_get,sizeof(cmd_cfg_get),1000);
  /* Infinite loop */
  for(;;)
  {
		pdsem = xSemaphoreTake(BinarySem_zlgHandle,portMAX_DELAY);
    if(pdsem == pdTRUE)
		{
			switch(ZLGSTATE)
			{
				case GET_INFO:
				{
					channel_read = ack_cfg_get.recv_analy.dev_info[33];
					addr1_read = ack_cfg_get.recv_analy.dev_info[36];
					addr2_read = ack_cfg_get.recv_analy.dev_info[37];
					if(channel_read != channel || addr1_read != board_num1 || addr2_read != board_num2)
					{
						cmd_cfg_set.send_set.local_addr[0] = addr1_read;
						cmd_cfg_set.send_set.local_addr[1] = addr2_read;
						memcpy(cmd_cfg_set.send_set.dev_info, ack_cfg_get.recv_analy.dev_info, 65);
						cmd_cfg_set.send_set.dev_info[33] = channel;  //use channel 11
						cmd_cfg_set.send_set.dev_info[36] = board_num1;  //local address high 8
						cmd_cfg_set.send_set.dev_info[37] = board_num2;  //local address low 8
						cmd_cfg_set.send_set.dev_info[46] = 0x00;  //remote address high 8
						cmd_cfg_set.send_set.dev_info[47] = 0x00;  //remote address low 8
						checksum = 0;
						len = sizeof(cmd_cfg_set.send_buf) - 1;
						for(i = 0;i < len;i ++)
							checksum += cmd_cfg_set.send_buf[i];
						cmd_cfg_set.send_set.veri = checksum;
						HAL_UART_Receive_DMA(&huart3,ack_cfg_set,sizeof(ack_cfg_set));
						HAL_UART_Transmit(&huart3,cmd_cfg_set.send_buf,sizeof(cmd_cfg_set.send_buf),1000);
						ZLGSTATE = SET_INFO;
					}
					else
					{
						HAL_UART_Receive_DMA(&huart3,ack_show_addr,sizeof(ack_show_addr));
						HAL_UART_Transmit(&huart3,cmd_show_addr,sizeof(cmd_show_addr),1000);
						ZLGSTATE = SHOW_ADDR;
					}
					break;
				}
				case SET_INFO:
				{
					if(ack_cfg_set[sizeof(ack_cfg_set)-1] == 0x00)
					{
						HAL_UART_Transmit(&huart3,cmd_reset,sizeof(cmd_reset),1000);
						osDelay(500);
						ZLGSTATE = RESET_ZLG;
					}
					break;
				}
				case RESET_ZLG:
				{
					HAL_UART_Receive_DMA(&huart3,ack_show_addr,sizeof(ack_show_addr));
					HAL_UART_Transmit(&huart3,cmd_show_addr,sizeof(cmd_show_addr),1000);
					ZLGSTATE = SHOW_ADDR;
					break;
				}
				case SHOW_ADDR:
				{
					if(ack_show_addr[sizeof(ack_show_addr)-1] == 0x00)
						ZLGSTATE = WAITFORCMD;
					break;
				}
				case WAITFORCMD:
				{
					break;
				}
				default:
				{
					break;
				}
			}
			
		}
  }
  /* USER CODE END FUNC_ZLG */
}

/* FUNC_CAPTURE function */
void FUNC_CAPTURE(void const * argument)
{
  /* USER CODE BEGIN FUNC_CAPTURE */
	BaseType_t pdsem = pdFALSE;
	unsigned int i,j,k,channel;
	uint16_t interval[30] = {0};
	float Frequency = 0;
  float Temperature = 0;
	float temp_log;
	uint32_t sum = 0;
  /* Infinite loop */
  for(;;)
  {
		pdsem = xSemaphoreTake(BinarySem_captureHandle,portMAX_DELAY);
		if(pdsem == pdTRUE)
		{
			i = 1800;	
			while(i>=800)
			{
				j = i;
				HAL_GPIO_WritePin(STRING_GPIO[0], STRING_PIN[0], GPIO_PIN_SET);
				HAL_GPIO_WritePin(STRING_GPIO[1], STRING_PIN[1], GPIO_PIN_RESET);
				HAL_GPIO_WritePin(STRING_GPIO[2], STRING_PIN[2], GPIO_PIN_SET);
				HAL_GPIO_WritePin(STRING_GPIO[3], STRING_PIN[3], GPIO_PIN_RESET);
				HAL_GPIO_WritePin(STRING_GPIO[4], STRING_PIN[4], GPIO_PIN_SET);
				HAL_GPIO_WritePin(STRING_GPIO[5], STRING_PIN[5], GPIO_PIN_RESET);		
				HAL_GPIO_WritePin(STRING_GPIO[6], STRING_PIN[6], GPIO_PIN_SET);
				HAL_GPIO_WritePin(STRING_GPIO[7], STRING_PIN[7], GPIO_PIN_RESET);		
				while(j)
				{    
						__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
						j = j - 1;
				}  
				
				j = i;
				HAL_GPIO_WritePin(STRING_GPIO[0], STRING_PIN[0], GPIO_PIN_RESET);
				HAL_GPIO_WritePin(STRING_GPIO[1], STRING_PIN[1], GPIO_PIN_SET);
				HAL_GPIO_WritePin(STRING_GPIO[2], STRING_PIN[2], GPIO_PIN_RESET);
				HAL_GPIO_WritePin(STRING_GPIO[3], STRING_PIN[3], GPIO_PIN_SET);
				HAL_GPIO_WritePin(STRING_GPIO[4], STRING_PIN[4], GPIO_PIN_RESET);
				HAL_GPIO_WritePin(STRING_GPIO[5], STRING_PIN[5], GPIO_PIN_SET);	
				HAL_GPIO_WritePin(STRING_GPIO[6], STRING_PIN[6], GPIO_PIN_RESET);
				HAL_GPIO_WritePin(STRING_GPIO[7], STRING_PIN[7], GPIO_PIN_SET);					
				while(j)
				{  
					 __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
					 j = j - 1;
				}    
				i = i - 1; 
			}
			HAL_GPIO_WritePin(STRING_GPIO[1], STRING_PIN[1], GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STRING_GPIO[3], STRING_PIN[3], GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STRING_GPIO[5], STRING_PIN[5], GPIO_PIN_RESET);		
			HAL_GPIO_WritePin(STRING_GPIO[7], STRING_PIN[7], GPIO_PIN_RESET);	
			
			HAL_Delay(20);
			
			for(k = 0;k < 8;k ++)
			{
				HAL_TIM_IC_Start_DMA(htim[i], TIM_CHANNEL[i], (uint32_t *)input_capture[i], dma_size);
			}
			
			for(k = 0;k < 8;k ++)
			{
				pdsem = xSemaphoreTake(CountingSem_dmacpltHandle, 100 / portTICK_PERIOD_MS);
			}
			
			for(channel = 0; channel < 8; channel ++)
			{
				sum = 0;
				for(i = 1;i < dma_size - 1;i ++)
				{
					if (input_capture[channel][i + 1] > input_capture[channel][i])
					{
						interval[i] = input_capture[channel][i + 1] - input_capture[channel][i]; 
					}
					else
					{
						interval[i]= (0xFFFF - input_capture[channel][i]) + input_capture[channel][i + 1]; 
					}
					sum += interval[i];
				}
				avrg_freq[channel] = sum /(float)(dma_size - 2);
			}
			
			get_temperature();	
		
			for(i = 0;i < 8; i ++)
			{
				Frequency=240000000.0f/avrg_freq[i] + 0.5f;
				Temperature = adcbuf[i];
				temp_log = log(Temperature * 4.5185f);
				Temperature = (1.0f / (A + B * temp_log + C * temp_log * temp_log * temp_log) - 273.2f) * 100.0f; 
				write_to_data_buf(i,(uint16_t)(Frequency),(uint16_t)(Temperature));	
			}
		}
  }
  /* USER CODE END FUNC_CAPTURE */
}

/* FUNC_485 function */
void FUNC_485(void const * argument)
{
  /* USER CODE BEGIN FUNC_485 */
	BaseType_t pdsem = pdFALSE;
	HAL_UART_Receive_DMA(&huart4,rxbuf_485,len_485);
  /* Infinite loop */
  for(;;)
  {
		pdsem = xSemaphoreTake(BinarySem_485Handle,portMAX_DELAY);
		if(pdsem == pdTRUE) {
			HAL_UART_Transmit(&huart4,rxbuf_485,len_485,1000);
		}
  }
  /* USER CODE END FUNC_485 */
}

/* Callback_timer1s function */
void Callback_timer1s(void const * argument)
{
  /* USER CODE BEGIN Callback_timer1s */
  
  /* USER CODE END Callback_timer1s */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
