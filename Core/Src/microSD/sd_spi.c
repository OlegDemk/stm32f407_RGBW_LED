/*
 * sd_spi.c
 *
 *  Created on: Nov 6, 2022
 *      Author: odemki
 */


#include <microSD/sd_spi.h>


#include <light/led_stript/ARGB.h>
#include <light/led_stript/control.h>
#include "light/led_stop/stop_light.h"

#define ON 1
#define OFF 0
#define OLD_VER_LEFT ON
#define OLD_VER_RIGHT ON

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;



extern uint8_t frame_start_flag;

char USER_Path[4];
FATFS *fs;
FIL MyFile;
sd_info_ptr sdinfo;
FILINFO sFileInfo;
DIR sDirectory;
char aStringBuffer[60];
uint8_t aBuffer[512];

int frame_size = 948;			// How many bytes on the one frame

int start_evenled = 228;

int strat_left_led = 268;
int midle_left_led  = 436;
int end_left_led = 604;

int strat_right_led = 608;
int midle_right_led  = 776;
int end_ritht_led = 944;

int how_many_leds_up_part = 41;
int how_many_leds_down_part = 42;

extern bool interrupt_animation_flag;

extern TIM_HandleTypeDef htim14;			// Для формування 25 Hz
extern uint8_t interrupt_flag;			// ставиться в htim13
extern TIM_HandleTypeDef htim13;


void make_delay(int delay);

/***	FUNCTIONS	***********************************************************************************************************************************************************************************/
/******************************************************************************************************************************************************************************************************/
uint8_t SD_SPI_Init(void)
{
	uint8_t vCmd;
	int16_t vCounter;
	//uint32_t vTmpPrc;
	sdinfo.type = 0;
	uint8_t aArray[4];

	HAL_Delay(250);														// SD voltage stability delay

	/*vTmpPrc = hspi2.Init.BaudRatePrescaler;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; 		//156.25 kbbs*/

	HAL_SPI_Init(&hspi1);

	SD_DESELECT;
	for(vCounter = 0; vCounter<10; vCounter++) // 80 pulse bit. Set SPI as SD card interface
		SPI_Release();

	/*hspi2.Init.BaudRatePrescaler = vTmpPrc;
	HAL_SPI_Init(&hspi2);*/

	SD_SELECT;
	if (SD_SPI_Cmd(CMD0, 0) == 1) // Enter Idle state
		{
			SPI_Release();
			if (SD_SPI_Cmd(CMD8, 0x1AA) == 1) // SDv2
				{
					for (vCounter = 0; vCounter < 4; vCounter++)
						aArray[vCounter] = SPI_ReceiveByte();
					if (aArray[2] == 0x01 && aArray[3] == 0xAA) // The card can work at vdd range of 2.7-3.6V
						{
							for (vCounter = 12000; (vCounter && SD_SPI_Cmd(ACMD41, 1UL << 30)); vCounter--)	{;}	 // Wait for leaving idle state (ACMD41 with HCS bit)
							if (vCounter && SD_SPI_Cmd(CMD58, 0) == 0)
								{ // Check CCS bit in the OCR
									for (vCounter = 0; vCounter < 4; vCounter++) 	aArray[vCounter] = SPI_ReceiveByte();
									sdinfo.type = (aArray[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2; // SDv2 (HC or SC)
								}
						}
				}
			else		//SDv1 or MMCv3
				{
					if (SD_SPI_Cmd(ACMD41, 0) <= 1)
						{
							sdinfo.type = CT_SD1; vCmd = ACMD41; // SDv1
						}
						else
						{
							sdinfo.type = CT_MMC; vCmd = CMD1; // MMCv3
						}
					for (vCounter = 25000; vCounter && SD_SPI_Cmd(vCmd, 0); vCounter--) ; // Wait for leaving idle state
					if ( ! vCounter || SD_SPI_Cmd(CMD16, 512) != 0) // Set R/W block length to 512
					sdinfo.type = 0;
				}
		}
	else
		{
			return 1;
		}

	return 0;
}

/****************************************************************************************************************/
/****************************************************************************************************************/
uint8_t SPIx_WriteRead(uint8_t byte)
{
  uint8_t vReceivedByte = 0;
  if(HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &byte, (uint8_t*) &vReceivedByte, 1, 0x1000) != HAL_OK)
  {
  	SD_Error_Handler();
  }
  return vReceivedByte;
}

/****************************************************************************************************************/
/****************************************************************************************************************/
void SPI_SendByte(uint8_t byte)
{
  SPIx_WriteRead(byte);
}

/****************************************************************************************************************/
/****************************************************************************************************************/
uint8_t SPI_ReceiveByte(void)
{
  uint8_t byte = SPIx_WriteRead(0xFF);
  return byte;
}

/****************************************************************************************************************/
/****************************************************************************************************************/
void SPI_Release(void)
{
  SPIx_WriteRead(0xFF);
}

/****************************************************************************************************************/
/****************************************************************************************************************/
uint8_t SD_SPI_WaitingForReadiness(void)
{
	uint8_t vResult;
	uint16_t vCount = 0;

	do {
		vResult = SPI_ReceiveByte();
		vCount++;
	} while ( (vResult != 0xFF) && (vCount < 0xFFFF) );

	if (vCount >= 0xFFFF) return ERROR;

	  return (vResult == 0xFF) ? OK: ERROR;
}

/****************************************************************************************************************/
/****************************************************************************************************************/
uint8_t SD_SPI_Cmd(uint8_t cmd, uint32_t argument)
{
  uint8_t vByte, vResult;

	// ACMD is the command sequence of CMD55-CMD?
	if (cmd & 0x80)
	{
		cmd &= 0x7F;
		vResult = SD_SPI_Cmd(CMD55, 0);
		if (vResult > 1) return vResult;
	}

	// Select the card
	SD_DESELECT;
	SPI_ReceiveByte();
	SD_SELECT;
	SPI_ReceiveByte();

	// Send a command packet
	SPI_SendByte(cmd); // Start + Command index
	SPI_SendByte((uint8_t)(argument >> 24)); // Argument[31..24]
	SPI_SendByte((uint8_t)(argument >> 16)); // Argument[23..16]
	SPI_SendByte((uint8_t)(argument >> 8)); // Argument[15..8]
	SPI_SendByte((uint8_t)argument); // Argument[7..0]
	vByte = 0x01; // Dummy CRC + Stop

	if (cmd == CMD0) {vByte = 0x95;} // Valid CRC for CMD0(0)
	if (cmd == CMD8) {vByte = 0x87;} // Valid CRC for CMD8(0x1AA)
	SPI_SendByte(vByte);

  // Receive a command response
  vByte = 10; // Wait for a valid response in timeout of 10 attempts
  do {
    		vResult = SPI_ReceiveByte();
  } while ((vResult & 0x80) && --vByte);

  return vResult;

}

/****************************************************************************************************************/
/****************************************************************************************************************/
uint8_t SD_SPI_Read_Block(uint8_t *buff, uint32_t lba)
{
  uint8_t vResult = 0;
  uint16_t vCounter = 0;

	vResult = SD_SPI_Cmd (CMD17, lba);
	if (vResult) return 5; //	Error

	SPI_Release();

  do{
				vResult=SPI_ReceiveByte();
				vCounter++;
  } while ((vResult != 0xFE) && (vCounter < 0xFFFF)); // Wait till mark(0xFE) is received
  if (vCounter >= 0xFFFF) return 5;	 //	 Error

  for (vCounter = 0; vCounter<512; vCounter++) buff[vCounter]=SPI_ReceiveByte(); // Write data to the buffer
  SPI_Release(); // Skip CRC
  SPI_Release();

  return 0;
}

/****************************************************************************************************************/
/****************************************************************************************************************/
uint8_t SD_SPI_Write_Block (uint8_t *buff, uint32_t lba)
{
  uint8_t vResult;
  uint16_t vCounter;

  vResult = SD_SPI_Cmd(CMD24, lba);

  if(vResult != 0x00) return 6; // Error

  SPI_Release();
  SPI_SendByte (0xFE); // Send transmission start mark
  for (vCounter = 0; vCounter<512; vCounter++) SPI_SendByte(buff[vCounter]); // Write data to the SD
  SPI_Release();  // Skip CRC
  SPI_Release();
  vResult = SPI_ReceiveByte();
  if((vResult & 0x05) != 0x05) return 6; // Error  (datasheet p. 111)

  vCounter = 0;
  do {
	  vResult=SPI_ReceiveByte();
	  vCounter++;
  } while ( (vResult != 0xFF)&&(vCounter<0xFFFF) );		//Wait till BUSY mode is finished
  if (vCounter>=0xFFFF) return 6;		// Error

  return 0;
}

/****************************************************************************************************************/
/****************************************************************************************************************/
uint8_t SD_SPI_ReadFile(void)
{
	if(f_mount(&SDFatFs, (TCHAR const*)USER_Path, 0))
	{
		SD_Error_Handler();
	}
	else
	{
		if(f_open(&MyFile, "STM32Second.txt", FA_READ))
		{
			SD_Error_Handler();
		}
		else
		{
			SD_SPI_ReadLongFile();
			f_close(&MyFile);
		}
	}
	return 0;
}

/****************************************************************************************************************/
/****************************************************************************************************************/
FRESULT SD_SPI_ReadLongFile(void)
{
  uint16_t vTemp = 0;
  uint32_t vIndex = 0;
  uint32_t vFileSize = MyFile.obj.objsize;
  // uint32_t vFileSize = MyFile.obj.fs->csize;
  uint32_t vBytesReadCounter;

  int i = 0;

	do
	{
		if(vFileSize < 512)
		{
			vTemp = vFileSize;
		}
		else
		{
			vTemp = 512;
		}
		vFileSize -= vTemp;

		f_lseek(&MyFile, vIndex);
		f_read(&MyFile, aBuffer, vTemp, (UINT *)&vBytesReadCounter); //  чому vBytesReadCounter == 4 завжди?????


		vIndex += vTemp;
	}
	while(vFileSize > 0);

  	return FR_OK;
}

/****************************************************************************************************************/
/****************************************************************************************************************/
uint8_t SD_SPI_WriteFile(void)
{
	FRESULT vResult;
	uint8_t aWriteBuffer[100] = {0};
	uint32_t vBytesWritteCounter;




	if(f_mount(&SDFatFs, (TCHAR const*)USER_Path, 0))
		{
			SD_Error_Handler();
		}
		else
		{
			if(f_open(&MyFile, "STM32Second.txt", FA_CREATE_ALWAYS | FA_WRITE))
			{
				SD_Error_Handler();
			}
			else
			{
				vResult = f_write(&MyFile, aWriteBuffer, sizeof(vBytesWritteCounter), (void*)&vBytesWritteCounter);
				if((vBytesWritteCounter == 0) || (vResult))
				{
					SD_Error_Handler();
				}
				f_close(&MyFile);
			}
		}
	return 0;
}

/****************************************************************************************************************/
/****************************************************************************************************************/
uint8_t SD_SPI_GetFileInfo(void)
{
	uint8_t vResult;
	DWORD free_clusters, free_sectors, total_sectors;

	if(f_mount(&SDFatFs, (TCHAR const*)USER_Path, 0))
	{
		SD_Error_Handler();
	}
	else
	{
		vResult = f_opendir(&sDirectory, "/");		// "/" - directory name to open

		if (vResult == FR_OK)
		{
			while(1)
			{
				vResult = f_readdir(&sDirectory, &sFileInfo);

				if ((vResult == FR_OK) && (sFileInfo.fname[0]))
				{
					HAL_UART_Transmit(&huart3, (uint8_t*)sFileInfo.fname, strlen((char*)sFileInfo.fname), 0x1000);

					if(sFileInfo.fattrib & AM_DIR)
					{
						HAL_UART_Transmit(&huart3, (uint8_t*)"  [DIR]", 7, 0x1000);
					}
				}
				else break;

				HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 0x1000);
			}
		}
		f_closedir(&sDirectory);
	}

	f_getfree("/", &free_clusters, &fs);

	sprintf(aStringBuffer, "free_clusters: %lu\r\n", free_clusters);
	HAL_UART_Transmit(&huart3, (uint8_t*)aStringBuffer, strlen(aStringBuffer), 0x1000);

	sprintf(aStringBuffer,"n_fatent: %lu\r\n",fs->n_fatent);
	HAL_UART_Transmit(&huart3, (uint8_t*)aStringBuffer, strlen(aStringBuffer), 0x1000);

	sprintf(aStringBuffer,"fs_csize: %d\r\n",fs->csize);
	HAL_UART_Transmit(&huart3, (uint8_t*)aStringBuffer, strlen(aStringBuffer), 0x1000);

	total_sectors = (fs->n_fatent - 2) * fs->csize;
	sprintf(aStringBuffer, "total_sectors: %lu\r\n", total_sectors);
	HAL_UART_Transmit(&huart3, (uint8_t*)aStringBuffer, strlen(aStringBuffer), 0x1000);

	free_sectors = free_clusters * fs->csize;
	sprintf(aStringBuffer, "free_sectors: %lu\r\n", free_sectors);
	HAL_UART_Transmit(&huart3, (uint8_t*)aStringBuffer, strlen(aStringBuffer), 0x1000);

	sprintf(aStringBuffer, "%lu KB total drive space.\r\n%lu KB available.\r\n", (free_sectors / 2), (total_sectors / 2));
	HAL_UART_Transmit(&huart3, (uint8_t*)aStringBuffer, strlen(aStringBuffer), 0x1000);

	return 0;
}

/****************************************************************************************************************/
/****************************************************************************************************************/
void SD_Error_Handler(void)
{
	LED_Red_ON;
	//while(1);
}
// -----------------------------------------------------------------------------------------------
void test_function_generate_delay(void)
{
	if(interrupt_flag == 1)
	{
		HAL_GPIO_TogglePin(GPIOE, TEST_OUTPUT_1_Pin);
		interrupt_flag = 0;
	}
}
// -----------------------------------------------------------------------------------------------



void test_double_buffer(char* name)
{
	uint16_t vTemp = 0;
	uint32_t vIndex = 0;
	uint32_t vFileSize = 0;
	uint32_t vBytesReadCounter;

	uint8_t frame_buffer_A[949] = {0};
	uint8_t frame_buffer_B[949] = {0};

	int size_buf_for_read = sizeof(frame_buffer_A);

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
			HAL_TIM_Base_Start_IT(&htim14);									// Для формування 25 Hz

			vFileSize = MyFile.obj.objsize;									// Get size of current file
			int how_many_frames = vFileSize/frame_size;						// How many frames into current file


				for(int h = 0; h < how_many_frames; h++)
				{
					HAL_GPIO_TogglePin(GPIOE, TEST_OUTPUT_1_Pin);

					memset(frame_buffer_A, 0, sizeof(frame_buffer_A));

					f_lseek(&MyFile, h + ((frame_size - 1)*h));						// shift on one frame
					f_read(&MyFile, aBuffer, vTemp, (UINT *)&vBytesReadCounter);
					f_gets(frame_buffer_A, size_buf_for_read, &MyFile);     			// Read one fraime into buffer


					// SET Left RGBW LEDs
					uint16_t number_of_rgbw_leds = 0;
					int k = 0;
					for(k = end_left_led; k >= strat_left_led; k--)		// 84 LEDs
					{
						if(k%4 == 0)
						{
							set_left_one_rgbw_led(number_of_rgbw_leds, frame_buffer_A[k], frame_buffer_A[k+1], frame_buffer_A[k+2], frame_buffer_A[k+3]);
							number_of_rgbw_leds++;
						}
					}
					// SET Right RGBW LEDs
					number_of_rgbw_leds = 0;

					for(k = end_ritht_led; k >= strat_right_led; k--)
					{
						if(k%4 == 0)
						{
							set_right_one_rgbw_led(number_of_rgbw_leds, frame_buffer_A[k], frame_buffer_A[k+1], frame_buffer_A[k+2], frame_buffer_A[k+3]);
							number_of_rgbw_leds++;
						}
					}

					// SET RED LEDs
					set_duty_cycle_stop_left_5(frame_buffer_A[start_evenled + 16 ]);
					set_duty_cycle_stop_left_4(frame_buffer_A[start_evenled + 12 ]);
					set_duty_cycle_stop_left_3(frame_buffer_A[start_evenled + 8 ]);
					set_duty_cycle_stop_left_2(frame_buffer_A[start_evenled + 4 ]);
					set_duty_cycle_stop_left_1(frame_buffer_A[start_evenled]);

					set_duty_cycle_stop_ritht_1(frame_buffer_A[start_evenled + 20 ]);
					set_duty_cycle_stop_ritht_2(frame_buffer_A[start_evenled + 24 ]);
					set_duty_cycle_stop_ritht_3(frame_buffer_A[start_evenled + 28 ]);
					set_duty_cycle_stop_ritht_4(frame_buffer_A[start_evenled + 32 ]);
					set_duty_cycle_stop_ritht_5(frame_buffer_A[start_evenled + 36 ]);

					while (!ARGB_Show_left());  		// Update
					while (!ARGB_Show_right());  		// Update

					frame_start_flag = 0;

					//flip_flag_interrupt = 0;


					//HAL_GPIO_TogglePin(GPIOE, TEST_OUTPUT_Pin);
					//HAL_GPIO_WritePin(GPIOE, TEST_OUTPUT_Pin, GPIO_PIN_RESET);   // For measure



					//while (!ARGB_Show_right());  		// Update

					//HAL_GPIO_WritePin(GPIOE, TEST_OUTPUT_Pin, GPIO_PIN_RESET);
					// HAL_GPIO_WritePin(GPIOE, TEST_OUTPUT_Pin, GPIO_PIN_RESET);   // For measure
					// HAL_Delay(27);
			}
		f_close(&MyFile);

		}
	}
	return 0;
}
// ------------------------------------------------------------------------------------------------
/*
	Open file, calculate how many frames in selected file and show all frames.
 */
uint8_t open_bin_file(char* name)
{
	int i = 0;

	uint16_t vTemp = 0;
	uint32_t vIndex = 0;
	static uint32_t vFileSize = 0;
	uint32_t vBytesReadCounter;

	uint8_t frame_buffer[949] = {0};
	int size_buf_for_read = sizeof(frame_buffer);
	static int how_many_frames = 0;

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


	///////////////////////////////////////////////////

	// 1. Відкрити файл
	// 2. ПОрахувати кількість фреймів
	// 3. Включити таймер

	// 		Прочитати один фрейм
	// 		Якщо є 0.04 Hz
	// 			Вигрузити (включити леди)
	// 			Якщо фрейм останній то закрити файл
	// 		Якщо нема
	//			вийти



	static int open_file_flag = 0;

	if(open_file_flag == 0)		// if file wasn't opened before
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

				open_file_flag = 1;
				return 0;
			}
		}
	}
	else
	{
		static int h = 0;

		if(h >= how_many_frames)
		{
			f_close(&MyFile);
			open_file_flag = 0;
			h = 0;

			return 1;
		}

		for( h; ((h < how_many_frames) && (interrupt_flag == 1)); h++)
		{
			HAL_GPIO_TogglePin(GPIOE, TEST_OUTPUT_1_Pin);					// For measure

			if(interrupt_animation_flag == true)						// If was sent "STOP animation" command. 'z' key
			{
				interrupt_animation_flag = false;
				stop_light_all_turn_off();
				turn_off_left_and_right_dtript();
				HAL_Delay(100);

				break;
			}

			memset(frame_buffer, 0, sizeof(frame_buffer));

			f_lseek(&MyFile, h + ((frame_size - 1)*h));						// shift on one frame
			f_read(&MyFile, aBuffer, vTemp, (UINT *)&vBytesReadCounter);
			f_gets(frame_buffer, size_buf_for_read, &MyFile);     			// Read one fraime into buffer

			if((h > 316) && (h < 330))			// Place in 7.bin file where somsing wrong
			{
				int ggg = 99;
			}

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

			// make_delay(267);

			// SET RED LEDs
			set_duty_cycle_stop_left_5(frame_buffer[start_evenled + 16 ]);
			set_duty_cycle_stop_left_4(frame_buffer[start_evenled + 12 ]);
			set_duty_cycle_stop_left_3(frame_buffer[start_evenled + 8 ]);
			set_duty_cycle_stop_left_2(frame_buffer[start_evenled + 4 ]);
			set_duty_cycle_stop_left_1(frame_buffer[start_evenled]);

			set_duty_cycle_stop_ritht_1(frame_buffer[start_evenled + 20 ]);
			set_duty_cycle_stop_ritht_2(frame_buffer[start_evenled + 24 ]);
			set_duty_cycle_stop_ritht_3(frame_buffer[start_evenled + 28 ]);
			set_duty_cycle_stop_ritht_4(frame_buffer[start_evenled + 32 ]);
			set_duty_cycle_stop_ritht_5(frame_buffer[start_evenled + 36 ]);

			while (!ARGB_Show_left());  		// Update
			while (!ARGB_Show_right());  		// Update

			interrupt_flag = 0;				// Tim 13
		}
		return 0;
	}
}

// -----------------------------------------------------------------------------------------------------------
void make_delay(int delay)
{
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	while(__HAL_TIM_GET_COUNTER(&htim5) < delay);
}
// -----------------------------------------------------------------------------------------------------------
uint8_t open_my_bin_file(char* name)
{
	int i = 0;

	uint16_t vTemp = 0;
	uint32_t vIndex = 0;
	uint32_t vFileSize = MyFile.obj.objsize;
	uint32_t vBytesReadCounter;


	// uint8_t frame_buffer[948] = {0};			// One frame buffer

	uint8_t frame_buffer[949] = {0};

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


	if(f_mount(&SDFatFs, (TCHAR const*)USER_Path, 1))
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
			memset(frame_buffer, 0, sizeof(frame_buffer));

			f_lseek(&MyFile, 0);			// shift on one frame
			f_read(&MyFile, aBuffer, vTemp, (UINT *)&vBytesReadCounter);

			int size_buf_for_read = sizeof(frame_buffer);
			f_gets(frame_buffer, size_buf_for_read, &MyFile);     // Read one fraime

			//ПЕРЕДАТИ ДАНІ В СВІТЛОДІОДИ <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>
			int k = 0;
			uint16_t number_of_rgbw_leds = 0;
			// SET RED LEDs

			set_duty_cycle_stop_left_5(frame_buffer[228 + 16]);
			set_duty_cycle_stop_left_4(frame_buffer[228 + 12]);
			set_duty_cycle_stop_left_3(frame_buffer[228 + 8]);
			set_duty_cycle_stop_left_2(frame_buffer[228 + 4]);
			set_duty_cycle_stop_left_1(frame_buffer[228]);

			set_duty_cycle_stop_ritht_1(frame_buffer[228 + 20]);
			set_duty_cycle_stop_ritht_2(frame_buffer[228 + 24]);
			set_duty_cycle_stop_ritht_3(frame_buffer[228 + 28]);
			set_duty_cycle_stop_ritht_4(frame_buffer[228 + 32]);
			set_duty_cycle_stop_ritht_5(frame_buffer[228 + 36]);

				// SET Left RGBW LEDs
				number_of_rgbw_leds = 0;
				for(k = 269; k <= 604; k++)		// 85 LEDs
				{
					if(k%4 == 0)
					{
						// не вмикається останній лед
						set_left_one_rgbw_led(number_of_rgbw_leds, frame_buffer[k], frame_buffer[k+1], frame_buffer[k+2], frame_buffer[k+3]);
						number_of_rgbw_leds++;
					}
				}
				while (!ARGB_Show_left());

				// SET Right RGBW LEDs
				number_of_rgbw_leds = 0;
				for(k = 609; k <= 944; k++)
				{
					if(k%4 == 0)
					{
						// не вмикається останній лед
						//set_right_one_rgbw_led(number_of_rgbw_leds, frame_buffer[k], frame_buffer[k+1], frame_buffer[k+2], frame_buffer[k+3]);
						set_right_one_rgbw_led(number_of_rgbw_leds, frame_buffer[k], frame_buffer[k+1], frame_buffer[k+2], frame_buffer[k+3]);
						number_of_rgbw_leds++;
					}
				}
				while (!ARGB_Show_right());

			f_close(&MyFile);

			f_mount(&SDFatFs, (TCHAR const*)USER_Path, 0);
		}
	}
	return 0;
}
// ------------------------------------------------------------------------------------------------
// Create simple one frame (for testing)
void create_example_bin(void)
{
	FRESULT vResult;
	uint8_t aWriteBuffer[949] = {0};
	uint32_t vBytesWritteCounter;

	int i = 0;

		for(i = 0; i <= 228 - 4; i++)		// Fill in unused bytes
		{
			int g = i%4;
			if(g == 0)
			{
				aWriteBuffer[i] = 1;
				aWriteBuffer[i+1] = 2;
				aWriteBuffer[i+2] = 3;
				aWriteBuffer[i+3] = 4;
			}
		}
		for(i = 228; i <= 267; i++)		// Fill in RED LEDs
		{
			int g = i%4;
			if(g == 0)
			{
				aWriteBuffer[i] = 100;
				aWriteBuffer[i+1] = 0;
				aWriteBuffer[i+2] = 0;
				aWriteBuffer[i+3] = 0;
			}
		}

		for(i = 268; i <= 608; i++)		// Fill in Left RGBW
		{
			int g = i%4;
			if(g == 0)
			{
				aWriteBuffer[i] = 0;
				aWriteBuffer[i+1] = 50;
				aWriteBuffer[i+2] = 0;
				aWriteBuffer[i+3] = 0;
			}
		}

		for(i = 609; i <= 948; i++)		// Fill in Right RGBW
		{
			int g = i%4;
			if(g == 0)
			{
				aWriteBuffer[i] = 0;
				aWriteBuffer[i+1] = 0;
				aWriteBuffer[i+2] = 60;
				aWriteBuffer[i+3] = 0;
			}
		}

	if(f_mount(&SDFatFs, (TCHAR const*)USER_Path, 0))
	{
		SD_Error_Handler();
	}
	else
	{
		if(f_open(&MyFile, "test_3.bin", FA_CREATE_ALWAYS | FA_WRITE))
		{
			SD_Error_Handler();
		}
		else
		{
			int size_of_buf = sizeof(aWriteBuffer);
			vResult = f_write(&MyFile, aWriteBuffer, size_of_buf, (void*)&vBytesWritteCounter);
			if((vBytesWritteCounter == 0) || (vResult))
			{
				SD_Error_Handler();
			}
			f_close(&MyFile);
		}
	}
}
// ------------------------------------------------------------------------------------------------


// ------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------
