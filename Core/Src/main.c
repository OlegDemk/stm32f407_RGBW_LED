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
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdlib.h>
#include <stdint.h>
#include <microSD/sd_spi.h>
#include <microSD/file_system.h>
#include "stdbool.h"

#include "light/led_stript/ARGB.h"
#include "light/led_stript/control.h"
#include "light/led_stop/stop_light.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void all_leds_animantion_ok_state(void);
void all_leds_animantion_error_state(void);

// SD Card ////////////////////////////////////////////////////////////////////
extern FATFS *fs;
extern FIL MyFile;
extern sd_info_ptr sdinfo;
extern FILINFO sFileInfo;
extern DIR sDirectory;
extern char aStringBuffer[60];
extern uint8_t aBuffer[512];

extern char USER_Path[4];
FATFS SDFatFs;
///////////////////////////////////////////////////////////////////////////////

// UART ///////////////////////////////////////////////////////////////////////
uint8_t uart_RX_data = 0;					// buffer for receive one char
uint8_t  command_from_uart = 0;
bool flag_received_command = false;

uint8_t frame_start_flag = 0;
bool interrupt_animation_flag = false;

char rx_buf_command[10] = {0};
uint8_t count_chars = 0;
////////////////////////////////////////////////////////////////////////////////


// Generate 25 Hz //////////////////////////////////////////////////////////////
uint8_t interrupt_flag = 0;
////////////////////////////////////////////////////////////////////////////////

uint8_t data_ready_flag = 0;
uint8_t data_write_flag = 1;

// data
uint8_t frame_buffer[949] = {0};
extern int frame_size ;			// How many bytes on the one frame
extern int start_evenled;

extern int strat_left_led;
extern int midle_left_led;
extern int end_left_led;

extern int strat_right_led;
extern int midle_right_led;
extern int end_ritht_led;

extern int how_many_leds_up_part;
extern int how_many_leds_down_part;

//extern int frame;
extern int how_many_frames;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#include "stop_light.h"
//#include "LED_WS2812B.h"


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim13;
DMA_HandleTypeDef hdma_tim8_ch3;
DMA_HandleTypeDef hdma_tim8_ch4_trig_com;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM13_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void init_tim_13(int msec)
{
	msec = msec*10;
	__HAL_TIM_SET_AUTORELOAD(&htim13, msec-1);

	HAL_TIM_Base_Start_IT(&htim13);
}
// ----------------------------------------------------------------------------
uint8_t read_framesfrom_bin_file(char* name)
{
	uint16_t vTemp = 0;
	uint32_t vIndex = 0;
	static uint32_t vFileSize = 0;
	uint32_t vBytesReadCounter;

	//uint8_t frame_buffer[949] = {0};									// Frame buffer
	int size_buf_for_read = sizeof(frame_buffer);

//	static int how_many_frames = 0;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	static bool open_file_flag = false;

	if(open_file_flag == false)		// if file wasn't opened before
	{
		if(f_mount(&SDFatFs, (TCHAR const*)USER_Path, 0))
		{
			SD_Error_Handler();
	  	}
		else
		{
			if(f_open(&MyFile, name, FA_READ))
			{
				SD_Error_Handler();
	  		}
			else
			{
				vFileSize = MyFile.obj.objsize;									// Get size of current file
				how_many_frames = vFileSize/frame_size;							// How many frames into current file

				open_file_flag = true;
				return 0;
	  		}
	  	}
	}
	else
	{
		static int frame = 0;

		if(frame >= how_many_frames)		// If all frames has been read
		{
			f_close(&MyFile);
			open_file_flag = false;
			frame = 0;

			return 1;
	  	}

		for(frame; ((frame < how_many_frames) && (data_write_flag == 1)); frame++)
		{
			//HAL_GPIO_TogglePin(GPIOE, TEST_OUTPUT_1_Pin);					// For measure
			if(interrupt_animation_flag == true)						// If was sent "STOP animation" command. 'z' key
			{
				interrupt_animation_flag = false;
				stop_light_all_turn_off();
				turn_off_left_and_right_dtript();
				HAL_Delay(100);

				break;
	  		}

			memset(frame_buffer, 0, sizeof(frame_buffer));		// must be 4 buffer

			f_lseek(&MyFile, frame + ((frame_size - 1)*frame));						// shift on one frame
			f_read(&MyFile, aBuffer, vTemp, (UINT *)&vBytesReadCounter);
			f_gets(frame_buffer, size_buf_for_read, &MyFile);     			// Read one fraime into buffer

			// SET Left RGBW LEDs
			uint16_t number_of_rgbw_leds = 0;
			int k = 0;
			for(k = end_left_led; k >= strat_left_led; k--)		// 84 LEDs
			{
				if(k%4 == 0)
				{
					set_left_one_rgbw_led(number_of_rgbw_leds, frame_buffer[k], frame_buffer[k+1], frame_buffer[k+2], frame_buffer[k+3]);
					number_of_rgbw_leds++;
	  			}
	  		}

			// SET Right RGBW LEDs
			number_of_rgbw_leds = 0;
			for(k = end_ritht_led; k >= strat_right_led; k--)
	  		{
	  			if(k%4 == 0)
	  			{
	  				set_right_one_rgbw_led(number_of_rgbw_leds, frame_buffer[k], frame_buffer[k+1], frame_buffer[k+2], frame_buffer[k+3]);
	  				number_of_rgbw_leds++;
	  			}
	  		}


			//ТУТ ПОСТАВИТИ ФЛАГ ГОТОВНОСТІ ДАНИХ  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>
			data_write_flag = 0;
			data_ready_flag = 1;
//
	  		}
	  		return 0;
	  	}
}

// uint8_t tim_increment = 0;
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
  MX_DMA_Init();
  MX_USB_HOST_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(100);

  // LEDs //////////////////////////////////////////////////////////////////
  ARGB_SetBrightness(255); 					 	// Set global brightness to 100%
  ARGB_Init();  								// Initialization
  turn_off_left_and_right_dtript();
  //////////////////////////////////////////////////////////////////////////

  // SD Card //////////////////////////////////////////////////////////////
  char msg_buf[30] = {0};
  if(disk_initialize(SDFatFs. drv) != 0)
  {
	  while(1)
	  {
		  strcat(msg_buf, "\n\r-> ERROR: NO SD CARD! \n\r");
		  HAL_UART_Transmit_IT(&huart3, msg_buf, sizeof(msg_buf));
		  HAL_Delay(100);

		  all_leds_animantion_error_state();
	  }
  }
  else
  {
	  SD_SPI_GetFileInfo();
	  FATFS_UnLinkDriver(USER_Path);

	  all_leds_animantion_ok_state();
  }
  //////////////////////////////////////////////////////////////////////////

  // UART //////////////////////////////////////////////////////////////////
  HAL_Delay(100);
  HAL_UART_Receive_IT(&huart3, &uart_RX_data, sizeof(uart_RX_data));		// Turn on receive on byte from UART in interrupt mode
  //////////////////////////////////////////////////////////////////////////

  HAL_TIM_Base_Start_IT(&htim5);       										//використати цей таймер для синхронізації

  init_tim_13(40);				// Set value in milisecond

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {
	  //test_double_buffer();
	  //test_function_generate_delay();

	  if(flag_received_command == true)
	  {
		  static char buf_str[10] = {0};
		  static bool flag_firt_command = true;

		  if(flag_firt_command == true)											// Read file first time
		  {
			  memset(buf_str, 0, sizeof(buf_str));
			  strcat(buf_str, rx_buf_command);
			  strcat(buf_str, ".bin");
			  flag_firt_command = false;
		  }
		  else
		  {
			  static bool print_flag = true;

			  if(read_framesfrom_bin_file(buf_str) == 0)										// Read file
			  {
				  if(print_flag == true)											// Print only one time
				  {
					  memset(msg_buf, 0, sizeof(msg_buf));
					  strcat(msg_buf, rx_buf_command);
					  strcat(msg_buf, ": working...  \n\r");
					  HAL_UART_Transmit_IT(&huart3, msg_buf, sizeof(msg_buf));

					  print_flag = false;
				  }
			  }
			  else																	// Print "DONE" if all file was read
			  {
				  memset(rx_buf_command, 0, sizeof(rx_buf_command));
				  memset(msg_buf, 0, sizeof(msg_buf));
				  strcat(msg_buf, "\n\r DONE \n\r");
				  HAL_UART_Transmit_IT(&huart3, msg_buf, sizeof(msg_buf));

				  flag_received_command = false;									// Out
				  flag_firt_command = true;
				  print_flag = true;
			  }
		  }


		 // тут зробити читання і підготовку буффера					// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		 // зробити два буфера, один готується, другий					// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		 //  можлива реальзація як черга з двома елементами  в ній   	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


	  }


	  // Вивести дані, якщо двні готові, і якщо таймер зробив тік
	  if((interrupt_flag == 1) && (data_ready_flag == 1))
	  {
		  if(update_all_leds(NULL, how_many_frames) == true)
		  {
			  data_ready_flag = 0;					// Data was showed
			  data_write_flag = 1;			// alowe read next frame
		  }

	  }




    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 250-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 250-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2100-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 250-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8400-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 8400-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 50000;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, TEST_OUTPUT_1_Pin|TEST_OUTPUT_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_SD_GPIO_Port, CS_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TEST_OUT_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD6_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TEST_OUTPUT_1_Pin */
  GPIO_InitStruct.Pin = TEST_OUTPUT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TEST_OUTPUT_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TEST_OUTPUT_2_Pin */
  GPIO_InitStruct.Pin = TEST_OUTPUT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEST_OUTPUT_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_SD_Pin */
  GPIO_InitStruct.Pin = CS_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_SD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TEST_OUT_Pin */
  GPIO_InitStruct.Pin = TEST_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TEST_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// ------------------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim13)
	{
		HAL_GPIO_TogglePin(GPIOE, TEST_OUTPUT_2_Pin);
		interrupt_flag = 1;
		//HAL_TIM_Base_Stop_IT(&htim13);
	}
}
// ------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart -> Instance == USART3)
	{
		if(uart_RX_data != 13)
		{
			rx_buf_command[count_chars] = uart_RX_data;
			count_chars++;
		}
		else
		{
			flag_received_command = true;
			count_chars= 0;
		}


		if(uart_RX_data == 'z')						// If was sent "STOP animation" command. 'z' key
		{
			interrupt_animation_flag = true;
		}

		HAL_UART_Receive_IT(&huart3, &uart_RX_data, 1);
		//HAL_UART_Transmit_IT(&huart3, &uart_RX_data, 1);
	}
}
// ------------------------------------------------------------------------------------
void all_leds_animantion_ok_state(void)
{
	turn_all_leds_from_centr(10, 0, 0, 150);
	test_from_midle_to_corner();

	HAL_Delay(500);

	turn_all_leds_from_centr(20, 0, 0, 0);
	stop_light_all_turn_off();
}
// -----------------------------------------------------------------------------
void all_leds_animantion_error_state(void)
{
	turn_all_leds_from_centr(7, 150, 0, 0);
	test_from_midle_to_corner();

	for(uint8_t i = 0; i < 85; i ++)
	{
		set_left_one_rgbw_led(i, 0, 0, 0, 0);
		set_right_one_rgbw_led(i, 0, 0, 0, 0);
		while (!ARGB_Show_left());  			// Update
		while (!ARGB_Show_right());  			// Update
		HAL_Delay(3);
	}

	stop_light_all_turn_off();
}
// ------------------------------------------------------------------------------------

//	  uint16_t vTemp = 0;
//	  uint32_t vIndex = 0;
//	  static uint32_t vFileSize = 0;
//	  uint32_t vBytesReadCounter;
//
//	  uint8_t frame_buffer[949] = {0};									// Frame buffer
//	  int size_buf_for_read = sizeof(frame_buffer);
//
//	  static int how_many_frames = 0;
//
//	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
//	  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
//	  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//	  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
//	  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//	  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//	  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
//	  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
//
//
//	  	static bool open_file_flag = false;
//
//	  	if(open_file_flag == false)		// if file wasn't opened before
//	  	{
//	  		if(f_mount(&SDFatFs, (TCHAR const*)USER_Path, 0))
//	  		{
//	  			SD_Error_Handler();
//	  		}
//	  		else
//	  		{
//	  			if(f_open(&MyFile, name, FA_READ))
//	  			{
//	  				SD_Error_Handler();
//	  			}
//	  			else
//	  			{
//	  				vFileSize = MyFile.obj.objsize;									// Get size of current file
//	  				how_many_frames = vFileSize/frame_size;							// How many frames into current file
//
//	  				open_file_flag = true;
//	  				return 0;
//	  			}
//	  		}
//	  	}
//	  	else
//	  	{
//	  		static int frame = 0;
//
//	  		if(frame >= how_many_frames)		// If all frames has been read
//	  		{
//	  			f_close(&MyFile);
//	  			open_file_flag = false;
//	  			frame = 0;
//
//	  			return 1;
//	  		}
//
//	  		for(frame; ((frame < how_many_frames) && (interrupt_flag == 1)); frame++)
//	  		{
//	  			//HAL_GPIO_TogglePin(GPIOE, TEST_OUTPUT_1_Pin);					// For measure
//
//	  			if(interrupt_animation_flag == true)						// If was sent "STOP animation" command. 'z' key
//	  			{
//	  				interrupt_animation_flag = false;
//	  				stop_light_all_turn_off();
//	  				turn_off_left_and_right_dtript();
//	  				HAL_Delay(100);
//
//	  				break;
//	  			}
//
//	  			memset(frame_buffer, 0, sizeof(frame_buffer));		// must be 4 buffer
//
//	  			f_lseek(&MyFile, frame + ((frame_size - 1)*frame));						// shift on one frame
//	  			f_read(&MyFile, aBuffer, vTemp, (UINT *)&vBytesReadCounter);
//	  			f_gets(frame_buffer, size_buf_for_read, &MyFile);     			// Read one fraime into buffer
//
//	  //			if((frame > 316) && (frame < 330))			// Place in 7.bin file where somsing wrong
//	  //			{
//	  //				int ggg = 99;
//	  //			}
//
//	  			// SET Left RGBW LEDs
//	  			uint16_t number_of_rgbw_leds = 0;
//	  			int k = 0;
//	  			for(k = end_left_led; k >= strat_left_led; k--)		// 84 LEDs
//	  			{
//	  				if(k%4 == 0)
//	  				{
//	  					set_left_one_rgbw_led(number_of_rgbw_leds, frame_buffer[k], frame_buffer[k+1], frame_buffer[k+2], frame_buffer[k+3]);
//	  					number_of_rgbw_leds++;
//	  				}
//	  			}
//
//	  			// SET Right RGBW LEDs
//	  			number_of_rgbw_leds = 0;
//	  			for(k = end_ritht_led; k >= strat_right_led; k--)
//	  			{
//	  				if(k%4 == 0)
//	  				{
//	  					set_right_one_rgbw_led(number_of_rgbw_leds, frame_buffer[k], frame_buffer[k+1], frame_buffer[k+2], frame_buffer[k+3]);
//	  					number_of_rgbw_leds++;
//	  				}
//	  			}
//
//	  			// make_delay(267);
//
//	  			// тут чекати на флаг 25 Гц  			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//
//	  			// SET RED LEDs
//	  			set_duty_cycle_stop_left_5(frame_buffer[start_evenled + 16 ]);
//	  			set_duty_cycle_stop_left_4(frame_buffer[start_evenled + 12 ]);
//	  			set_duty_cycle_stop_left_3(frame_buffer[start_evenled + 8 ]);
//	  			set_duty_cycle_stop_left_2(frame_buffer[start_evenled + 4 ]);
//	  			set_duty_cycle_stop_left_1(frame_buffer[start_evenled]);
//
//	  			set_duty_cycle_stop_ritht_1(frame_buffer[start_evenled + 20 ]);
//	  			set_duty_cycle_stop_ritht_2(frame_buffer[start_evenled + 24 ]);
//	  			set_duty_cycle_stop_ritht_3(frame_buffer[start_evenled + 28 ]);
//	  			set_duty_cycle_stop_ritht_4(frame_buffer[start_evenled + 32 ]);
//	  			set_duty_cycle_stop_ritht_5(frame_buffer[start_evenled + 36 ]);
//
//	  			// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//
//	  			while (!ARGB_Show_left());  		// Update    	(Takes time around 17 us)
//	  			while (!ARGB_Show_right());  		// Update		(Takes time around 17 us)
//
//	  			interrupt_flag = 0;				// Tim 13
//
//	  			// HAL_GPIO_WritePin(GPIOE, TEST_OUTPUT_2_Pin, GPIO_PIN_SET);
//	  			//HAL_GPIO_TogglePin(GPIOE, TEST_OUTPUT_1_Pin);					// For measure
//	  		}
//	  		return 0;
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
