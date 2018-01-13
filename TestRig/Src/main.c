/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void perform_calibration_sequence(void);
static uint16_t find_calibration_point(void);
static void print_instructions (void);
static void set_output_amp(int16_t gain_value);
static void user_pwm_setvalue(uint16_t value);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"



// calibration_constants
#define CALIB_THRESHOLD 500 // amount the ADC needs to change ( in one cycle ) to trigger
#define CALIB_AVERAGING 10 // amount of ADC averaging
#define PWM_DIVIDE 10 // set frequency of PWM output
#define TIME_TO_PRIME 5 // seconds after calibration before enabling threshold detection
#define ADC_MIDVALUE 0x7ff // 12 bit ADC, so max = 0xfff, set to mid-scale


char aTxBuffer[1024]; // maximum of 1024 chars out - no error checking
char aRxBuffer[10]; // maximum of 10 chars in - no error checking
int g_ADCValue; // last ADC value read

//uint16_t calibration_value;
uint8_t seconds_to_prime=TIME_TO_PRIME; // number of seconds after calibration before the threshold is activated


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	// SPI setup - talking to programmable gain amplifiers
	uint8_t spi_tx_data[2] = {0x00, 0x00}; //data,command = set to NOP command initially
	uint8_t spi_data=0;
	uint8_t spi_command=0;

	// buffer to receive commands - no error checking
	uint8_t command_buffer[1024]; // set up command buffer & write pointer to it
	uint8_t command_buffer_rx=0;
	uint8_t command_received = 0; // set to 1 when we get a CR to indicate a command


	uint8_t  pwm_value;
	uint8_t  gain_value;
	uint8_t	 channel_value;
	uint16_t divider_value;

	uint16_t adc_array[100]; // used to grab ADC values for averaging - no error checking
	uint16_t adc_write_pointer=0;
	uint16_t adc_max=0;
	uint16_t adc_min=0xfff;
	uint16_t adc_average=0;
	uint32_t adc_sum; // needs to be big enough for 100 adc values - no error checking

	uint16_t averaging;
	uint16_t threshold;
	uint16_t last_adc_average;

	uint8_t calibration_started;

//	uint16_t adc_readback;

	int f;
//	int g;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_ADC_Start(&hadc);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  spi_command=0;
  spi_data=0;
  spi_tx_data[1]=spi_command;
  spi_tx_data[0]=spi_data;

  HAL_GPIO_WritePin(AMP1_SS_GPIO_Port, AMP1_SS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AMP2_SS_GPIO_Port, AMP2_SS_Pin, GPIO_PIN_SET);
  //HAL_SPI_Transmit(&hspi1, spi_tx_data, 1, 1000);

	htim1.Init.Prescaler = 0; //set initial PWM divider = 0 ( fastest )
	HAL_TIM_Base_Init(&htim1);

	user_pwm_setvalue(50); // set pwm to mid-scale, 50%
	averaging = CALIB_AVERAGING; // set to average ADC over 10 readings to start
	gain_value = 5;
	set_output_amp(gain_value); //  set to unity system gain ( in-circuit attenuation of 10, amp value of 5 = 10x gain

	threshold=CALIB_THRESHOLD; //adc threshold - print out message if ADC changes by this much
	last_adc_average = 0;
	calibration_started=0;
	print_instructions();


	perform_calibration_sequence(); // do initial calibration
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  // look for calibration button

	  if ((calibration_started==0) & (HAL_GPIO_ReadPin(CALIBRATE_GPIO_Port, CALIBRATE_Pin) != GPIO_PIN_SET))
	  {
		  // button is pressed, we haven't done a calibration yet
		  perform_calibration_sequence();
		  calibration_started = 1;
	  }
	  if ((calibration_started==1) & (HAL_GPIO_ReadPin(CALIBRATE_GPIO_Port, CALIBRATE_Pin) != GPIO_PIN_RESET))
	  {
		  // button is not pressed, ensure we are ready to re-calibrate if needed
		  calibration_started = 0;
	  }

	  // if the ADC is ready to give us a value, grab it then get it to start conversion again
		if (HAL_ADC_PollForConversion(&hadc,1000000)==HAL_OK)
		{
			g_ADCValue = HAL_ADC_GetValue(&hadc);
			adc_array[adc_write_pointer++] = g_ADCValue;
			if (adc_write_pointer == averaging) // filled array
			{
				adc_max = 0;
				adc_min = 0xfff;
				adc_sum = 0;
				last_adc_average = adc_average;
				for (f=0;f<averaging;f++)
				{
					if (adc_array[f] > adc_max ) adc_max = adc_array[f];
					if (adc_array[f] < adc_min) adc_min = adc_array[f];
					adc_sum = adc_sum + adc_array[f];
//					sprintf(aTxBuffer, "ADC:%04x sum=%08x max=%04x min=%04x\n",adc_array[f], adc_sum, adc_max, adc_min);
//					HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
				}
				adc_average = (uint16_t) (adc_sum/averaging);
				adc_write_pointer=0; // reset
			}
			// re-start the conversion
			HAL_ADC_Start(&hadc);
		}

// check if we have an ADC value that has changed
		if (seconds_to_prime == 0) // only check after the calibration has primed
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET );
			if (abs(last_adc_average-adc_average) > threshold)
			{
				sprintf(aTxBuffer, "Threshold Detected: from %04x to %04x\n",last_adc_average, adc_average);
				HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
				last_adc_average = adc_average; // only detect the first time it happens
				// set the output to show a detection has happened
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET );
			}
		}
	// See if anything has been received by the serial port, if so store it.
	  if (HAL_UART_Receive(&huart2,(uint8_t *) aRxBuffer, 1, 10) == HAL_OK) // wait 10ms for a character
	  {
		  for (f=0;f<strlen(aRxBuffer);f++)
		  {
			  command_buffer[command_buffer_rx++] = aRxBuffer[f];
			  if (aRxBuffer[f] == 13) //CR
			  {
				  command_buffer[command_buffer_rx] = 0; // terminate string
				  command_received = 1; // indicate command ready to be processed
			  }
		  }
	  }

	  // if we have received a complete command ( terminated by CR ), echo it out
	 if (command_received != 0)
		  {
		 	// echo out the command
			sprintf(aTxBuffer, "Received: %s\n",command_buffer);
			HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);

			// no checking for command being formatted correctly
			// commands are:
			// pxx where xx = 00 to 99 - sets pwm width
			// ix where x = 0 to 7 - sets input amp gain
			// ox where x = 0 to 7 - sets output amp gain
			// cx where x = 0 or 1 - sets output amp channel
			// a - outputs current ADC value
			// dxxx where x = 0 to 999 changes PWM divider ( bigger number = slower )
			// l - outputs ADC value until a key is pressed
			// s - calibration sequence
			// txx  where xx = 0 to 99 - sets ADC capture threshold
			// vxx where xx = 0 to 99 - sets amount of ADC averaging
			// ? - prints help

			switch (command_buffer[0])
			{
			case 'p':
				pwm_value = ((command_buffer[1]-'0') * 10) + (command_buffer[2]-'0');
				user_pwm_setvalue(pwm_value);
				sprintf(aTxBuffer, "PWM set to: %d\n",pwm_value);
				HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
				break;
			case 'v':
				averaging = ((command_buffer[1]-'0') * 10) + (command_buffer[2]-'0');
				sprintf(aTxBuffer, "Averaging set to: %d\n",averaging);
				HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
				break;
			case 't':
				threshold = ((command_buffer[1]-'0') * 10) + (command_buffer[2]-'0');
				threshold = threshold*10; // decrease sensitivity
				sprintf(aTxBuffer, "Threshold set to: %d\n",threshold);
				HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
				break;
			case 'i':
				gain_value = command_buffer[1]-'0';
				spi_command = 0x40; // write to gain register
				spi_tx_data[1]=spi_command;
				spi_tx_data[0]=gain_value;
				//assert SS for input amp ( AMP1 )
				HAL_GPIO_WritePin(AMP1_SS_GPIO_Port, AMP1_SS_Pin, GPIO_PIN_RESET);
				HAL_SPI_Transmit(&hspi1, spi_tx_data, 1, 1000);
				HAL_GPIO_WritePin(AMP1_SS_GPIO_Port, AMP1_SS_Pin, GPIO_PIN_SET);
				sprintf(aTxBuffer, "Input Gain set to: %d\n",gain_value);
				HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
				break;
			case 'o':
				gain_value = command_buffer[1]-'0';
				set_output_amp(gain_value);
				sprintf(aTxBuffer, "Output Gain set to: %d\n",gain_value);
				HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
				break;
			case 'c':
				channel_value = command_buffer[1]-'0';
				spi_command = 0x41; // write to channel register
				spi_tx_data[1]=spi_command;
				spi_tx_data[0]=channel_value;
				//assert SS for output amp ( AMP1 )
				HAL_GPIO_WritePin(AMP2_SS_GPIO_Port, AMP2_SS_Pin, GPIO_PIN_RESET);
				HAL_SPI_Transmit(&hspi1, spi_tx_data, 1, 1000);
				HAL_GPIO_WritePin(AMP2_SS_GPIO_Port, AMP2_SS_Pin, GPIO_PIN_SET);
				sprintf(aTxBuffer, "Output channel set to: %d\n",channel_value);
				HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
				break;
			case 'a':
				sprintf(aTxBuffer, "ADC: %04x\tMAX:%04x\tMIN:%04x\tAVE:%04x\n",g_ADCValue,adc_max, adc_min, adc_average);
				HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
				break;
			case 'd':
					divider_value = ((command_buffer[1]-'0')*100)+((command_buffer[2]-'0')*10)+(command_buffer[3]-'0');
					htim1.Init.Prescaler = divider_value;
					HAL_TIM_Base_Init(&htim1);
					sprintf(aTxBuffer, "PWM divider set to %d\n",divider_value);
					HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
				break;
			case 'l':
				while(HAL_UART_Receive(&huart2,(uint8_t *) aRxBuffer, 1, 10) != HAL_OK)
				{
					if (HAL_ADC_PollForConversion(&hadc,1000000)==HAL_OK)
					{
						g_ADCValue = HAL_ADC_GetValue(&hadc);
						// re-start the conversion
						HAL_ADC_Start(&hadc);
						sprintf(aTxBuffer, "%04x\n",g_ADCValue);
						HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
					}
				}
				break;
			case 's': // calibration sequence
				perform_calibration_sequence();
				break;
			case '?':
				print_instructions();
				break;

			default: // not recognised
				break;

			}
			// reset command buffer ready for the next command
			command_received = 0;
			command_buffer_rx = 0;
		  }
	/* USER CODE END 3 */
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = PWM_DIVIDE;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT2_Pin|OUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|AMP2_SS_Pin|AMP1_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CALIBRATE_Pin */
  GPIO_InitStruct.Pin = CALIBRATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CALIBRATE_GPIO_Port, &GPIO_InitStruct);


  /*Configure GPIO pins : OUT2_Pin OUT1_Pin */
  GPIO_InitStruct.Pin = OUT2_Pin|OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin AMP2_SS_Pin AMP1_SS_Pin */
  GPIO_InitStruct.Pin = LED_Pin|AMP2_SS_Pin|AMP1_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// called once a second

	if (seconds_to_prime>0)
	{
		seconds_to_prime--;
	}
}

static void user_pwm_setvalue(uint16_t value)
{
	TIM_OC_InitTypeDef sConfigOC;

	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = value;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

static void print_instructions (void)
{
	sprintf(aTxBuffer, "pxx where xx = 00 to 99 - sets pwm width - pwm on pin 18 (PA8/D9)\n");
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
	sprintf(aTxBuffer, "ix where x = 0 to 7 - sets input amp gain\n");
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
	sprintf(aTxBuffer, "ox where x = 0 to 7 - sets output amp gain\n");
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
	sprintf(aTxBuffer, "0->1x, 1->2x, 2->4x, 3->5x, 4->8x, 5->10x, 6->16x, 7->32x\n");
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
	sprintf(aTxBuffer, "cx where x = 0 or 1 - sets output amp channel\n");
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
	sprintf(aTxBuffer, "a - outputs current ADC value - adc is on pin 7 (PA1/A1)\n");
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
	sprintf(aTxBuffer, "dxxx where x = 0 to 999 changes PWM divider ( bigger number = slower )\n");
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
	sprintf(aTxBuffer, "l - outputs ADC value until a key is pressed\n");
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
	sprintf(aTxBuffer, "s - Start Calibration sequence ( set ADC input to mid-level )\n");
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
	sprintf(aTxBuffer, "txx  where xx = 0 to 99 - sets ADC capture threshold\n");
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
	sprintf(aTxBuffer, "vxx where xx = 0 to 99 - sets amount of ADC averaging\n");
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
	sprintf(aTxBuffer, "? - prints help\n");
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
	if (seconds_to_prime == 0)
	{
		sprintf(aTxBuffer, "\nPRIMED\n");
		HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
	}
}

static void perform_calibration_sequence(void)
{
	uint8_t calibration_value;
	uint8_t gain_value;

	calibration_value = 0;
	// cycle through amplifier gain, looking for lowest gain that allows
	// calibration at PWM ranges of 25-75%
	for ( gain_value=0;gain_value<8;gain_value++)
	{
		set_output_amp(gain_value);
		calibration_value = find_calibration_point();
		if ((calibration_value < 75) & (calibration_value>25))
			break; // quit if we have found an appropriate calibration value
	}

	sprintf(aTxBuffer, "Pulse set to %d, gain set to %d\n",calibration_value, gain_value);
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
	// ensure that the detected output is primed and ready to go
	seconds_to_prime = TIME_TO_PRIME;
	// ensure LED is on until primed sequence is complete
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET );
}

static void set_output_amp(int16_t gain_value)
{
	uint8_t spi_tx_data[2];
	spi_tx_data[1]=0x40; // command to write to gain register
	spi_tx_data[0]=gain_value;
	//assert SS for output amp ( AMP2 )
	HAL_GPIO_WritePin(AMP2_SS_GPIO_Port, AMP2_SS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, spi_tx_data, 1, 1000);
	HAL_GPIO_WritePin(AMP2_SS_GPIO_Port, AMP2_SS_Pin, GPIO_PIN_SET);
}

static uint16_t find_calibration_point(void)
{
	uint16_t f;
	uint16_t g;
	uint16_t adc_readback;
	uint16_t calibration_value;

	calibration_value = 50; //midpoint for first guess
	user_pwm_setvalue(calibration_value);
	// slice the calibration values, and see if we are under/over 50% point - then adjust accordingly
	// as we have 0-100 as PWM values, we only need to do this 6 times:
	// 50, 25, 12, 6, 3, 1

	for (f=0;f<6;f++)
	{
		for (g=0;g<10;g++) // takes a while to settle, read the ADC a few times - is this needed?
		{
			HAL_ADC_PollForConversion(&hadc,1000000);
			adc_readback = HAL_ADC_GetValue(&hadc);
			//sprintf(aTxBuffer, "%04x\n",adc_readback);
			//HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
			HAL_ADC_Start(&hadc); // set ADC to convert for next cycle
		}
		if (abs(adc_readback - ADC_MIDVALUE) > 0x04) // accuracy
		{
			if (adc_readback > ADC_MIDVALUE)
			{
				calibration_value = calibration_value - (uint16_t)(50/(2<<f));
			}
			else
			{
				calibration_value = calibration_value + (uint16_t)(50/(2<<f));
			}
		}
//		sprintf(aTxBuffer, "%04x : %d\n",adc_readback,calibration_value);
//		HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);

		user_pwm_setvalue(calibration_value);
		// now wait for a while to let the output settle - Needed?
		for (g=0;g<10000;g++);
		HAL_ADC_Start(&hadc); // set ADC to convert for next cycle
	}
	return(calibration_value);
}
/* USER CODE END 4 */

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
