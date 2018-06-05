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
#include <string.h>
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
osThreadId TASK_STIMULATEHandle;
osTimerId TimerCaptureHandle;
osSemaphoreId BinarySem_captureHandle;
osSemaphoreId BinarySem_zlgHandle;
osSemaphoreId BinarySem_485Handle;
osSemaphoreId BinarySem_batHandle;
osSemaphoreId BinarySem_stimulateHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define TIMER_INTERVAL 200
#define FLASH_START_ADDR  0X08070000
/*********************** BATTERY *************************/	
extern STRUCT_CW_BATTERY   cw_bat;
int bat_count = 0;
int bat_interval = 20;

#define GPIO_BAT                      GPIOA
#define GPIO_PIN_BAT1                 GPIO_PIN_10
#define GPIO_PIN_BAT2                 GPIO_PIN_9

/****************** CAPRURE FREQUENCY ********************/	
#define GPIO_SWITCH                   GPIOA
#define GPIO_PIN_SWITCH               GPIO_PIN_4

uint16_t input_capture[8][50];
float avrg_freq[8];
int dma_size = 17;
int capture_count = 0;
int capture_interval = 3;

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
	GPIOA,
	GPIOC};

uint16_t ADDR_PIN[8] = {
	GPIO_PIN_4,
  GPIO_PIN_5,
	GPIO_PIN_13,
	GPIO_PIN_12,
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

uint8_t reply_buf[46] = {
	'S',
	0,0,//ID
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,0,0,
	0,0,
	0xff,
	0xff,
	0x00,
	0xff,
  'A'};

uint8_t order_buf[35];
	
typedef enum
{
	DEVICE_RUN,
	DEVICE_SLEEP
}Device_StateTypeDef;

Device_StateTypeDef Device_State = DEVICE_RUN;
	
/************************* UART ***************************/	
	
uint8_t data_zlg;
uint8_t len_zlg;
uint8_t rxbuf_zlg[100];

uint8_t data_485;
uint8_t len_485 = 1;
uint8_t rxbuf_485[100];	
	
/*********************** ZLG 5168 *************************/	

uint8_t channel = 11;
uint8_t addr1_coord = 0x80;
uint8_t addr2_coord = 0x00;
	
enum{
	GET_INFO = 0,
	SET_INFO,
	RESET_ZLG,
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
uint8_t ack_cfg_set[7];

uint8_t cmd_reset[9];

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
void FUNC_STIMULATE(void const * argument);
void Callback_timercapture(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void write_flash(uint16_t dat)
{
	FLASH_EraseInitTypeDef f;
	uint32_t PageError = 0;
	
	HAL_FLASH_Unlock();
	f.TypeErase = FLASH_TYPEERASE_PAGES;
	f.PageAddress = FLASH_START_ADDR;
	f.NbPages = 1;
	HAL_FLASHEx_Erase(&f, &PageError);
	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_START_ADDR, dat);
}

uint16_t read_flash()
{
	uint16_t value;
	value = *(uint16_t*)(FLASH_START_ADDR);
	return value;
}

void set_led()
{
		if(cw_bat.voltage == 0)
		{
			HAL_GPIO_TogglePin(GPIO_BAT, GPIO_PIN_BAT1);
			HAL_GPIO_TogglePin(GPIO_BAT, GPIO_PIN_BAT2);
		}
		else
		{
			if(cw_bat.capacity >= 80)
			{
				HAL_GPIO_WritePin(GPIO_BAT, GPIO_PIN_BAT1, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIO_BAT, GPIO_PIN_BAT2, GPIO_PIN_RESET);
			}
			else if(cw_bat.capacity >= 60)
			{
				HAL_GPIO_WritePin(GPIO_BAT, GPIO_PIN_BAT1, GPIO_PIN_RESET);
				HAL_GPIO_TogglePin(GPIO_BAT, GPIO_PIN_BAT2);
			}
			else if(cw_bat.capacity >= 40)
			{
				HAL_GPIO_WritePin(GPIO_BAT, GPIO_PIN_BAT1, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIO_BAT, GPIO_PIN_BAT2, GPIO_PIN_SET);
			}
			else if(cw_bat.capacity >= 20)
			{
				HAL_GPIO_TogglePin(GPIO_BAT, GPIO_PIN_BAT1);
				HAL_GPIO_WritePin(GPIO_BAT, GPIO_PIN_BAT2, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIO_BAT, GPIO_PIN_BAT1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIO_BAT, GPIO_PIN_BAT2, GPIO_PIN_SET);
			}
		}
}

void show_485(uint8_t *buf, int len)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); // 485 SEND ENABLE
	HAL_UART_Transmit(&huart4,buf,len,1000);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); // 485 RECV ENABLE
}

int calc_checksum(uint8_t *buf, int len)
{
	int checksum = 0;
	int i;
	for(i = 0;i < len;i ++)
		checksum += buf[i];
	return checksum;
}

void read_bat()
{
	bat_count ++;
	if(bat_count >= bat_interval)
	{
		bat_count = 0;
		cw_bat_work();
	}
}

void read_ID()
{
	int i;
	board_num1 = 0;
	board_num2 = 0;
	
	for(i = 0;i < 4;i ++)
	{
		if(HAL_GPIO_ReadPin(ADDR_GPIO[i], ADDR_PIN[i]) == GPIO_PIN_RESET)
		{
			board_num1 |= (1 << i);
		}
	}
	for(i = 0;i < 4;i ++)
	{
		if(HAL_GPIO_ReadPin(ADDR_GPIO[i + 4], ADDR_PIN[i + 4]) == GPIO_PIN_RESET)
		{
			board_num2 |= (1 << i);
		}
	}
	
	data_buf[1] = board_num1;
	data_buf[2] = board_num2;
	
	reply_buf[1] = board_num1;
	reply_buf[2] = board_num2;
}

void write_to_data_buf(int num,uint16_t Freq, uint16_t Temp)
{
	data_buf[3 + num * 5 + 1] = (uint8_t)((Freq&0xff00)>>8);
	data_buf[3 + num * 5 + 2] = (uint8_t)(Freq&0x00ff);
	data_buf[3 + num * 5 + 3] = (uint8_t)((Temp&0xff00)>>8);
	data_buf[3 + num * 5 + 4] = (uint8_t)(Temp&0x00ff);
}

void MEASUREMENT(int group)
{
	unsigned int i, j;
	unsigned int k;
	
	/*72Mhz
	i=
	5000---420hz
	4000---526hz
	3000---700hz
	2000---1064hz
	1000---2174hz
	800----2632hz
	700----3030hz
	*/
	
	/*32Mhz
	2200---495hz
	2000---543hz
	800----1389hz
	400----2778hz
	300----3571hz
	*/
	
	i = 2150;	//510hz
  while(i>=350) //2941hz
	{
		j = i;
		HAL_GPIO_WritePin(STRING_GPIO[group * 4], STRING_PIN[group * 4], GPIO_PIN_SET);  		
		HAL_GPIO_WritePin(STRING_GPIO[group * 4 + 1], STRING_PIN[group * 4 + 1], GPIO_PIN_RESET); 
		HAL_GPIO_WritePin(STRING_GPIO[group * 4 + 2], STRING_PIN[group * 4 + 2], GPIO_PIN_SET); 
		HAL_GPIO_WritePin(STRING_GPIO[group * 4 + 3], STRING_PIN[group * 4 + 3], GPIO_PIN_RESET); 		
		while(j)
		{    
				__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
				j = j - 1;
		}  
		
		j = i;
		HAL_GPIO_WritePin(STRING_GPIO[group * 4], STRING_PIN[group * 4], GPIO_PIN_RESET); 	
		HAL_GPIO_WritePin(STRING_GPIO[group * 4 + 1], STRING_PIN[group * 4 + 1], GPIO_PIN_SET); 
		HAL_GPIO_WritePin(STRING_GPIO[group * 4 + 2], STRING_PIN[group * 4 + 2], GPIO_PIN_RESET); 
		HAL_GPIO_WritePin(STRING_GPIO[group * 4 + 3], STRING_PIN[group * 4 + 3], GPIO_PIN_SET); 		
		while(j)
		{  
			 __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
			 j = j - 1;
		}    
		//k = k - 1;
		i = i - 3; 
	}   
	HAL_GPIO_WritePin(STRING_GPIO[group * 4 + 1], STRING_PIN[group * 4 + 1], GPIO_PIN_RESET); 
	HAL_GPIO_WritePin(STRING_GPIO[group * 4 + 3], STRING_PIN[group * 4 + 3], GPIO_PIN_RESET); 
  HAL_Delay(20);	
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
	
	cmd_reset[0] = 0xAB;
	cmd_reset[1] = 0xBC;
	cmd_reset[2] = 0xCD;
	cmd_reset[3] = 0xD9;
	
	capture_interval = capture_interval * 1000 / TIMER_INTERVAL;
	bat_interval = bat_interval * 1000 / TIMER_INTERVAL;

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
	HAL_GPIO_WritePin(GPIO_SWITCH, GPIO_PIN_SWITCH, GPIO_PIN_SET);
  ret = cw_bat_init();
	read_ID();
	cw_bat_work();
	set_led();
	
	if(read_flash() != 'Z')
		Device_State = DEVICE_RUN;
	else Device_State = DEVICE_SLEEP;
	
	HAL_Delay(200);
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

  /* definition and creation of BinarySem_bat */
  osSemaphoreDef(BinarySem_bat);
  BinarySem_batHandle = osSemaphoreCreate(osSemaphore(BinarySem_bat), 1);

  /* definition and creation of BinarySem_stimulate */
  osSemaphoreDef(BinarySem_stimulate);
  BinarySem_stimulateHandle = osSemaphoreCreate(osSemaphore(BinarySem_stimulate), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	xSemaphoreTake(BinarySem_captureHandle,portMAX_DELAY);
	xSemaphoreTake(BinarySem_zlgHandle,portMAX_DELAY);
	xSemaphoreTake(BinarySem_485Handle,portMAX_DELAY);
	xSemaphoreTake(BinarySem_stimulateHandle,portMAX_DELAY);
	
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of TimerCapture */
  osTimerDef(TimerCapture, Callback_timercapture);
  TimerCaptureHandle = osTimerCreate(osTimer(TimerCapture), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	osTimerStart(TimerCaptureHandle,TIMER_INTERVAL);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of TASK_BAT */
  osThreadDef(TASK_BAT, FUNC_BAT, osPriorityNormal, 0, 128);
  TASK_BATHandle = osThreadCreate(osThread(TASK_BAT), NULL);

  /* definition and creation of TASK_ZLG */
  osThreadDef(TASK_ZLG, FUNC_ZLG, osPriorityHigh, 0, 128);
  TASK_ZLGHandle = osThreadCreate(osThread(TASK_ZLG), NULL);

  /* definition and creation of TASK_CAPTURE */
  osThreadDef(TASK_CAPTURE, FUNC_CAPTURE, osPriorityLow, 0, 128);
  TASK_CAPTUREHandle = osThreadCreate(osThread(TASK_CAPTURE), NULL);

  /* definition and creation of TASK_485 */
  osThreadDef(TASK_485, FUNC_485, osPriorityHigh, 0, 128);
  TASK_485Handle = osThreadCreate(osThread(TASK_485), NULL);

  /* definition and creation of TASK_STIMULATE */
  osThreadDef(TASK_STIMULATE, FUNC_STIMULATE, osPriorityAboveNormal, 0, 128);
  TASK_STIMULATEHandle = osThreadCreate(osThread(TASK_STIMULATE), NULL);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  htim2.Init.Period = 65535;
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
  htim3.Init.Period = 65535;
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
  htim4.Init.Period = 65535;
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
  htim8.Init.Period = 65535;
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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 4, 0);
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11 
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6, GPIO_PIN_RESET);

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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(huart == &huart3)
	{
		xSemaphoreGiveFromISR(BinarySem_zlgHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	else if(huart == &huart4)
	{
		xSemaphoreGiveFromISR(BinarySem_485Handle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/* USER CODE END 4 */

/* FUNC_BAT function */
void FUNC_BAT(void const * argument)
{

  /* USER CODE BEGIN 5 */
	BaseType_t pdsem = pdFALSE;
  /* Infinite loop */
  for(;;)
  {			
		pdsem = xSemaphoreTake(BinarySem_batHandle,portMAX_DELAY);
		if(pdsem == pdTRUE)
		{
			read_bat();
		}
  }
  /* USER CODE END 5 */ 
}

/* FUNC_ZLG function */
void FUNC_ZLG(void const * argument)
{
  /* USER CODE BEGIN FUNC_ZLG */
	BaseType_t pdsem = pdFALSE;
	char mode;
	uint8_t dev_type;
	uint8_t channel_read;
	uint8_t addr1_read, addr2_read;
	uint8_t dest1_read, dest2_read;
	uint8_t dev_info1, dev_info2;

	HAL_UART_Receive_IT(&huart3,ack_cfg_get.recv_buf,sizeof(ack_cfg_get.recv_buf));
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
					//show_485(ack_cfg_get.recv_buf,sizeof(ack_cfg_get.recv_buf));
					
					dev_type = ack_cfg_get.recv_analy.dev_info[32];
					channel_read = ack_cfg_get.recv_analy.dev_info[33];
					addr1_read = ack_cfg_get.recv_analy.dev_info[36];
					addr2_read = ack_cfg_get.recv_analy.dev_info[37];
					dest1_read = ack_cfg_get.recv_analy.dev_info[46];
					dest2_read = ack_cfg_get.recv_analy.dev_info[47];
					dev_info1 = ack_cfg_get.recv_analy.status[1];
					dev_info2 = ack_cfg_get.recv_analy.status[2];
					if(channel_read != channel || addr1_read != board_num1 || addr2_read != board_num2 || dest1_read != addr1_coord || dest2_read != addr2_coord ||dev_type != 0)
					{
						cmd_cfg_set.send_set.local_addr[0] = addr1_read;
						cmd_cfg_set.send_set.local_addr[1] = addr2_read;
						memcpy(cmd_cfg_set.send_set.dev_info, ack_cfg_get.recv_analy.dev_info, 65);
						cmd_cfg_set.send_set.dev_info[32] = 0;  //work as enddevice
						cmd_cfg_set.send_set.dev_info[33] = channel;  //use channel 11
						cmd_cfg_set.send_set.dev_info[36] = board_num1;  //local address high 8
						cmd_cfg_set.send_set.dev_info[37] = board_num2;  //local address low 8
						cmd_cfg_set.send_set.dev_info[46] = addr1_coord;  //remote address high 8
						cmd_cfg_set.send_set.dev_info[47] = addr2_coord;  //remote address low 8
						cmd_cfg_set.send_set.veri = calc_checksum(cmd_cfg_set.send_buf, sizeof(cmd_cfg_set.send_buf) - 1);
						HAL_UART_Receive_IT(&huart3,ack_cfg_set,sizeof(ack_cfg_set));
						HAL_UART_Transmit(&huart3,cmd_cfg_set.send_buf,sizeof(cmd_cfg_set.send_buf),1000);
						ZLGSTATE = SET_INFO;
					}
					else
					{
						HAL_UART_Receive_IT(&huart3,order_buf,sizeof(order_buf));
						ZLGSTATE = WAITFORCMD;
					}
					break;
				}
				case SET_INFO:
				{
					//show_485(ack_cfg_set,sizeof(ack_cfg_set));
					
					if(ack_cfg_set[sizeof(ack_cfg_set)-1] == 0x00)
					{
						cmd_reset[4] = addr1_read;
						cmd_reset[5] = addr2_read;
						cmd_reset[6] = dev_info1;
						cmd_reset[7] = dev_info2;
						cmd_reset[8] = calc_checksum(cmd_reset, sizeof(cmd_reset) - 1);
						HAL_UART_Transmit(&huart3,cmd_reset,sizeof(cmd_reset),1000);
						
						//show_485(cmd_reset,sizeof(cmd_reset));
						
						osDelay(500);
						HAL_UART_Receive_IT(&huart3,order_buf,sizeof(order_buf));
						ZLGSTATE = WAITFORCMD;
					}
					break;
				}
				case WAITFORCMD:
				{
					if(order_buf[11] == board_num1 && order_buf[12] == board_num2)
					{
						/*if(order_buf[18] != 0xff)
						{
							capture_interval = order_buf[18];
							capture_interval = capture_interval * 1000 / TIMER_INTERVAL;
							reply_buf[32] = 0;
							reply_buf[33] = 0;
							reply_buf[34] = 0x55;
						}*/
						if(order_buf[16] == 0xff)
					  {
							mode = 'R';
							reply_buf[42] = 0xff;
						  reply_buf[43] = 0;
							reply_buf[44] = 0xff;
						}
						else if(order_buf[26] == 0)
						{
							mode = 'Z';
							reply_buf[42] = 0x55;
							reply_buf[43] = 0;
							reply_buf[44] = 0;
						}		
						HAL_UART_Transmit(&huart3,reply_buf,sizeof(reply_buf),1000);						
				    if(mode == 'R')
						{
							Device_State = DEVICE_RUN;
							write_flash('R');
						}
						else if(mode == 'Z')
						{
					    Device_State = DEVICE_SLEEP;
							write_flash('Z');
						}
					}
					HAL_UART_Receive_IT(&huart3,order_buf,sizeof(order_buf));
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
	int i, channel;
	uint16_t interval[50] = {0};
	float Frequency = 0;
  float Temperature = 0;
	float temp_log;
	float sum = 0;
  /* Infinite loop */
  for(;;)
  {
		pdsem = xSemaphoreTake(BinarySem_captureHandle,portMAX_DELAY);
		if(pdsem == pdTRUE)
		{
			//HAL_GPIO_WritePin(GPIO_SWITCH, GPIO_PIN_SWITCH, GPIO_PIN_SET);
			//HAL_Delay(20);
			
			xSemaphoreGive(BinarySem_stimulateHandle);
			
			//HAL_GPIO_WritePin(GPIO_SWITCH, GPIO_PIN_SWITCH, GPIO_PIN_RESET);
			
			for(channel = 0; channel < 8; channel ++)
			{
				sum = 0;
				for(i = 1;i < dma_size - 1;i ++)
				{
					if (input_capture[channel][i + 1] >= input_capture[channel][i])
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
				Frequency=320000000.0f/avrg_freq[i] + 0.5f;
				Temperature = adcbuf[i];
				temp_log = log(Temperature * 4.5185f);
				Temperature = (1.0f / (A + B * temp_log + C * temp_log * temp_log * temp_log) - 273.2f) * 100.0f; 
				write_to_data_buf(i,(uint16_t)(Frequency),(uint16_t)(Temperature));	
			}
			
			data_buf[43] = cw_bat.voltage;
			data_buf[44] = cw_bat.capacity;
			
			HAL_UART_Transmit(&huart3,data_buf,sizeof(data_buf),1000);
			//show_485(data_buf, sizeof(data_buf));
		}
  }
  /* USER CODE END FUNC_CAPTURE */
}

/* FUNC_485 function */
void FUNC_485(void const * argument)
{
  /* USER CODE BEGIN FUNC_485 */
	BaseType_t pdsem = pdFALSE;
	HAL_UART_Receive_IT(&huart4,rxbuf_485,len_485);
  /* Infinite loop */
  for(;;)
  {
		pdsem = xSemaphoreTake(BinarySem_485Handle,portMAX_DELAY);
		if(pdsem == pdTRUE) 
		{
			//xSemaphoreGive(BinarySem_captureHandle);
			/*HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); // 485 SEND ENABLE
			
			read_ID();
			HAL_UART_Transmit(&huart4,&board_num1,1,1000);
			HAL_UART_Transmit(&huart4,&board_num2,1,1000);*/
			
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); // 485 RECV ENABLE
			HAL_UART_Receive_IT(&huart4,rxbuf_485,len_485);
		}
  }
  /* USER CODE END FUNC_485 */
}

/* FUNC_STIMULATE function */
void FUNC_STIMULATE(void const * argument)
{
  /* USER CODE BEGIN FUNC_STIMULATE */
	BaseType_t pdsem = pdFALSE;
	int i, j;
  /* Infinite loop */
  for(;;)
  {
		pdsem = xSemaphoreTake(BinarySem_stimulateHandle,portMAX_DELAY);
		if(pdsem == pdTRUE) 
    {
			for(i = 0;i < 2;i ++)
		  {
			  MEASUREMENT(i);
			  for(j = 0;j < 4;j ++)
			  {
					HAL_TIM_IC_Start_DMA(htim[i * 4 + j], TIM_CHANNEL[i * 4 + j], (uint32_t *)input_capture[i * 4 + j], dma_size);
					HAL_Delay(40);
					HAL_TIM_IC_Stop_DMA(htim[i * 4 + j], TIM_CHANNEL[i * 4 + j]);
			  }
		  }
		}
  }
  /* USER CODE END FUNC_STIMULATE */
}

/* Callback_timercapture function */
void Callback_timercapture(void const * argument)
{
  /* USER CODE BEGIN Callback_timercapture */
	bat_count ++;
	capture_count ++;
	set_led();
	if(bat_count == bat_interval)
	{
		bat_count = 0;
		xSemaphoreGive(BinarySem_batHandle);
	}
	if(capture_count == capture_interval)
	{
		capture_count = 0;
		if(Device_State == DEVICE_RUN)
			xSemaphoreGive(BinarySem_captureHandle);
	}
  /* USER CODE END Callback_timercapture */
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
