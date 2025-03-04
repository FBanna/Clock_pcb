/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

GPIO_TypeDef *pin_ports[] = {M1_GPIO_Port, M2_GPIO_Port, M3_GPIO_Port, M4_GPIO_Port, M5_GPIO_Port, M6_GPIO_Port, M7_GPIO_Port, M8_GPIO_Port};
const int pin_numbers[] = {0,5,6,7,1,13,10,2};
//int numbers[][8] = {
//		{0,1,2,3,4,5}, 		//0
//		{2,3},				//1
//		{1,2,4,5,6},		//2
//		{1,2,3,4,6},		//3
//		{0,2,3,6},			//4
//		{0,1,3,4,6},		//5
//		{0,1,3,4,5,6},		//6
//		{1,2,3},			//7
//		{0,1,2,3,4,5,6},	//8
//		{0,1,2,3,6}			//9
//};

const uint8_t numbers[10] = {
		0b01111110,
		0b00011000,
		0b00110111,
		0b00111101,
		0b01011001,
		0b01101101,
		0b01101111,
		0b00111000,
		0b01111111,
		0b01111001
};

const uint8_t time_out_val = 60; //seconds

enum menu_enum {
	TIME,
	SET_TIME,
	SLEEP
};

enum menu_enum menu = TIME;

int button_pressed = 0;
uint8_t time[4];

int selected_digit = 0;

RTC_TimeTypeDef currTime = {0};
RTC_DateTypeDef currDate = {0};

int time_out = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

void set_mode(GPIO_TypeDef *gpio, int pin, uint32_t mode);
void set_level(GPIO_TypeDef *gpio, int pin, int level);
void update_buttons(void);
int change_digit(int digit, int change);
void disable_battery(void);
void check_time_out(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void set_LED(int digit, int row){
	int i;

	for(i = 0; i < 8; i++){
		if(i == digit){
			//set pin high
			continue;

//			set_mode(pin_ports[i], pin_numbers[i], 0b01);
//			set_level(pin_ports[i], pin_numbers[i], 1);


		} else if(digit <= row && row == (i-1)){

			//set pin low

			set_mode(pin_ports[i], pin_numbers[i], 0b01);
			set_level(pin_ports[i], pin_numbers[i], 0);

			//set_mode(pin_ports[i], pin_numbers[i], 0b00);


			//i += 1;

		} else if(digit > row && row == i){

			//set pin low

			set_mode(pin_ports[i], pin_numbers[i], 0b01);
			set_level(pin_ports[i], pin_numbers[i], 0);

		} else {
			//set pin high z
			set_mode(pin_ports[i], pin_numbers[i], 0b00);
		}
	}
}


void set_mode(GPIO_TypeDef *gpio, int pin, uint32_t mode){
	uint32_t reg;

	reg = gpio->MODER;
	//reg &= ~(mode << (pin * 2u));

	reg &= ~(0b11 << (pin * 2));
	reg |= (mode) << (pin * 2);


	gpio->MODER=reg;

}

void set_level(GPIO_TypeDef *gpio, int pin, int level){
	uint32_t reg;

	//gpio->ODR |= (level<<pin);


	reg = gpio->ODR;

	reg &= ~(1 << pin);
	reg |= (level << pin);

	gpio->ODR = reg;

}


int change_digit(int digit, int change){
	int new_digit = digit + change;

	if(new_digit > 9){
		return 0;
	} else if (new_digit < 0){
		return 9;
	} else {
		return new_digit;
	}


}
void update_buttons(void){

	if (HAL_GPIO_ReadPin(SELECT_GPIO_Port, SELECT_Pin) == 0) {

		time_out = 0;

		if( button_pressed == 0) {
			button_pressed = 1;

			if(menu == SLEEP){

				menu = TIME;


			} else if(menu == TIME){
				time[0] = 0;
				time[1] = 0;
				time[2] = 0;
				time[3] = 0;

				selected_digit = 0;

				menu = SET_TIME;
			} else if (menu == SET_TIME){

				if(selected_digit < 3){
					selected_digit++;
				} else {


					currTime.Hours = (time[0] * 10) + time[1];
					currTime.Minutes = (time[2] * 10) + time [3];
					currTime.Seconds = 0;

					HAL_RTC_SetTime(&hrtc, &currTime, RTC_FORMAT_BIN);


					menu = TIME;

				}
			}
		}

	} else if (HAL_GPIO_ReadPin(LEFT_GPIO_Port, LEFT_Pin) == 0){

		time_out = 0;

		if(menu == SLEEP){

			menu = TIME;

		} else if( button_pressed == 0) {
			button_pressed = 1;

			time[selected_digit] = change_digit(time[selected_digit], -1);


		}

	} else if (HAL_GPIO_ReadPin(RIGHT_GPIO_Port, RIGHT_Pin) == 0){

		time_out = 0;

		if(menu == SLEEP){

			menu = TIME;

		} else if( button_pressed == 0) {
			button_pressed = 1;

			time[selected_digit] = change_digit(time[selected_digit], 1);


		}

	} else {

		button_pressed = 0;

	}
}

void disable_battery(void){

	if(HAL_GPIO_ReadPin(FV_TEST_GPIO_Port, FV_TEST_Pin) == 1) {

		//FV is on

		//Stat 1 is low when LBO meaning that LBO pin is low.

		if(HAL_GPIO_ReadPin(LBO_GPIO_Port, LBO_Pin) == 0){

			// low battery, plugged into 5V, want to cancel charge -> disable charge
			// turn CE on

			HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, 1);

		} else {

			// turn off CE, enabling charging

			HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, 0);
		}

	}

}

void check_time_out(void){
	time_out++;

	if(time_out >= time_out_val){
		menu = SLEEP;
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_PCD_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  //HAL_GPIO_WritePin(GPIOB, COLON_Pin, 1);



  //HAL_GPIO_WritePin(GPIOB, M5_Pin, 1);

  //HAL_GPIO_WritePin(GPIOB, M1_Pin, 0);

//  set_mode(M5_GPIO_Port, 1, 0b10);
//  set_level(M5_GPIO_Port, 1, 1);




//  reg = M2_GPIO_Port->PUPDR;
//  reg &= ~(GPIO_PUPDR_PUPDR0 << (position * 2u));
//  reg |= ((GPIO_Init->Pull) << (position * 2u));

//  set_mode(M2_GPIO_Port, 5, 0b11);
//  set_mode(M3_GPIO_Port, 6, 0b11);
//  set_mode(M4_GPIO_Port, 7, 0b11);
//  set_mode(M6_GPIO_Port, 13, 0b11);
//  set_mode(M7_GPIO_Port, 10, 0b11);
//  set_mode(M8_GPIO_Port, 2, 0b11);

  //set_LED(4, 2);

  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  //HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//  int digit = 0;
//  int row = 0;

  int i;
  int k;
  uint32_t tick;
  uint32_t elapsed_time;
  int colon = 0;



  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  tick = HAL_GetTick();

	  update_buttons();

	  disable_battery();

	  HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);

	  if(menu == TIME){


		  time[0] = currTime.Hours / 10;
		  time[1] = currTime.Hours % 10;
		  time[2] = currTime.Minutes / 10;
		  time[3] = currTime.Minutes % 10;


		  // COLON on if even seconds
		  if ((currTime.Seconds%10) & 1){
			  if(colon == 0) {
				  HAL_GPIO_WritePin(COLON_GPIO_Port, COLON_Pin, 1);
				  colon = 1;
				  check_time_out();
			  }
		  } else {
			  if (colon == 1) {
				  HAL_GPIO_WritePin(COLON_GPIO_Port, COLON_Pin, 0);
				  colon = 0;
				  check_time_out();
			  }
		  }
	  }



//	  set_LED(digit, row);
//
//	  HAL_Delay(500);
//
//	  row += 1;
//
//
//	  if (row > 6){
//		  row = 0;
//
//		  digit +=1;
//
//		  if (digit > 4){
//			  digit = 0;
//		  }
//	  }



	  if(menu == TIME || menu == SET_TIME){
		  for(i = 0; i < 4; i++){ // for each digit

		  		  if (menu == SET_TIME && i == selected_digit && (currTime.Seconds%10) & 1){
		  			  continue;
		  		  }

		  		  // turn on the digit pin

		  		  set_mode(pin_ports[i], pin_numbers[i], 0b01);
		  		  set_level(pin_ports[i], pin_numbers[i], 1);

		  		  // turn on the needed lights

		  		  for(k = 0; k < 7; k++){
		  			  if(numbers[time[i]] & (1 << (6-k))){
		  				  set_LED(i, k);
		  				  HAL_Delay(0.1);

		  			  }
		  		  }

		  		  //turn off the digit pin

		  		  set_level(pin_ports[i], pin_numbers[i], 0);
		  	  }

		  	  // turns last light off
		  	  set_level(pin_ports[3], pin_numbers[3], 0);
	  }







	  elapsed_time = HAL_GetTick() - tick; // Time taken in ms

	  float remaining_time = (1000.0/40.0) - elapsed_time;

	  if (remaining_time > 0){
		  //HAL_GPIO_WritePin(COLON_GPIO_Port, COLON_Pin, 1);
		  HAL_Delay((uint32_t)remaining_time);
	  } else {
		  //HAL_GPIO_WritePin(COLON_GPIO_Port, COLON_Pin, 0);
	  }




//	  time++;
//
//	  if(time == 10){
//		  time = 0;
//	  }








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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV32;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 1952;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUZZER_Pin|M2_Pin|M3_Pin|M4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M1_Pin|M5_Pin|M8_Pin|M7_Pin
                          |COLON_Pin|M6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : FV_TEST_Pin */
  GPIO_InitStruct.Pin = FV_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FV_TEST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin M2_Pin M3_Pin M4_Pin */
  GPIO_InitStruct.Pin = M2_Pin|M3_Pin|M4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_Pin M5_Pin M8_Pin M7_Pin
                           COLON_Pin M6_Pin */
  GPIO_InitStruct.Pin = M1_Pin|M5_Pin|M8_Pin|M7_Pin
                          |COLON_Pin|M6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin LBO_Pin */
  GPIO_InitStruct.Pin = CE_Pin|LBO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

  GPIO_InitStruct.Pin = RIGHT_Pin|SELECT_Pin|LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



/* USER CODE END MX_GPIO_Init_2 */
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
