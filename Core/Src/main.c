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
#include <stdio.h>
#include <stdbool.h>
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
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile bool timer10_interrupt;
volatile uint8_t DispCnt;
volatile uint8_t to_be_displayed[4];

const uint8_t SEGMENT_LUT[10] = {
	0x3F, // 0
	0x06, // 1
	0x5B, // 2
	0x4F, // 3
	0x66, // 4
	0x6D, // 5
	0x7D, // 6
	0x07, // 7
	0x7F, // 8
	0x6F  // 9
};

typedef struct {
	GPIO_TypeDef* port;
	uint16_t pin;
}pin_t;

static const pin_t Segments[] = {
	{ SEG_A_GPIO_Port, SEG_A_Pin },
	{ SEG_B_GPIO_Port, SEG_B_Pin },
	{ SEG_C_GPIO_Port, SEG_C_Pin },
	{ SEG_D_GPIO_Port, SEG_D_Pin },
	{ SEG_E_GPIO_Port, SEG_E_Pin },
	{ SEG_F_GPIO_Port, SEG_F_Pin },
	{ SEG_G_GPIO_Port, SEG_G_Pin },
	{ SEG_DP_GPIO_Port, SEG_DP_Pin },
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

int __io_putchar(int ch); // Ustawia printfa tak ze wysyla dane UARTEM

void delay_us(uint32_t us); //Potrzebyn do komunikacji 1wire, zeby odczekac odpowiednie chwile w mikros

HAL_StatusTypeDef wire_reset(void); //wykonuje sekwencje na pinie Reset

void send_bit(uint8_t bit); //wysyla pojedyczny bit
uint8_t read_bit(void); //odbiera pojedynczy bit

void send_byte(uint8_t byte); //przesyła bajt
uint8_t read_byte(void); //odbiera bajt

float read_temperature(void); //zczytuje temperature

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); //obsługuje przerwanie od timera

void turn_segments(uint8_t number); //włącza odpowiednie segmenty w zalezności od cyfry

void convert_number_to_digits(uint8_t *digits, float number); // rozbija liczbe float na pojedyczne cyfry i wpisuje do podanej tablicy

void select_display(uint8_t display); // wybieramy com na ktorym chcemy wyswietlac

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
  MX_USART2_UART_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  static uint8_t interrupt_counter = 0;
  HAL_TIM_Base_Start(&htim9); //licznik odpowiadający za zliczanie mikrosekund okres 1 us
  HAL_TIM_Base_Start_IT(&htim10); //licznik potrzebny do liczenia sekund w while - przerwa między odczytami temp, okres 1s
  HAL_TIM_Base_Start_IT(&htim11); //odpalamy licznik 11 z przerwaniami potrzebny do wysweitlacza;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (timer10_interrupt) {
		  timer10_interrupt = false;

		  if (interrupt_counter < 3) {
			  printf(".");
			  fflush(stdout);
			  interrupt_counter++;

		  } else {
			  float temp = read_temperature();
			  temp = roundf(temp * 100.0) / 100.0;
			  convert_number_to_digits(&to_be_displayed, temp);
			  printf("Temperatura: %.2f °C\n", temp);
			  interrupt_counter = 0;
		  }

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 84-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8400-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 10000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 8400-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 10-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SEG_G_Pin|SEG_D_Pin|SEG_E_Pin|SEG_C_Pin
                          |SEG_B_Pin|SEG_F_Pin|SEG_A_Pin|SEG_DP_Pin
                          |COM_4_Pin|COM_3_Pin|COM_2_Pin|COM_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_G_Pin SEG_D_Pin SEG_E_Pin SEG_C_Pin
                           SEG_B_Pin SEG_F_Pin SEG_A_Pin SEG_DP_Pin
                           COM_4_Pin COM_3_Pin COM_2_Pin COM_1_Pin */
  GPIO_InitStruct.Pin = SEG_G_Pin|SEG_D_Pin|SEG_E_Pin|SEG_C_Pin
                          |SEG_B_Pin|SEG_F_Pin|SEG_A_Pin|SEG_DP_Pin
                          |COM_4_Pin|COM_3_Pin|COM_2_Pin|COM_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DS_Pin */
  GPIO_InitStruct.Pin = DS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return 1;
}

void delay_us(uint32_t us) {

	__HAL_TIM_SET_COUNTER(&htim9, 0); //ustawia licznik na zero
	while (__HAL_TIM_GET_COUNTER(&htim9) < us) {} // zlicza ile trzeba mikrosekund
}

HAL_StatusTypeDef wire_reset(void) {

	HAL_StatusTypeDef answer = HAL_ERROR;
	HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
	delay_us(480);
	HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
	delay_us(70);

	if (HAL_GPIO_ReadPin(DS_GPIO_Port, DS_Pin) == GPIO_PIN_RESET) {
		answer = HAL_OK;
		delay_us(410);
	}
	return answer;
}

void send_bit(uint8_t bit) {

	if (bit == 1){
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
		delay_us(6);
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
		delay_us(64);
	} else {
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
		delay_us(60);
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
		delay_us(10);
	}
}

uint8_t read_bit(void){

	  int rc;
	  HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET);
	  delay_us(6);
	  HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET);
	  delay_us(9);
	  rc = HAL_GPIO_ReadPin(DS_GPIO_Port, DS_Pin);
	  delay_us(55);
	  return rc;
}


void send_byte(uint8_t byte) {

	uint8_t i;
	for (i = 0; i < 8; i++) {
		send_bit(byte & 0x01); //wyciąga ostatni bit
		byte >>= 1;  //bitowe przesunięcie w prawo
  }
}

uint8_t read_byte(void) {

	 uint8_t value = 0;
	 uint8_t i;
	 for (i = 0; i < 8; i++) {
	    value >>= 1; //przesuwa bity w prawo, poniewaz czujnik przesyła LSB jako pierwszy
	    if (read_bit() == 1)
	      value |= 0x80;  //jeśli odczytany bit to 1 to ustawia go na najstarszy
	 }
	 return value;
}

float read_temperature(void) {
	wire_reset();
	send_byte(0xcc); //SKIP ROM pomija adresowanie czujnika - mamy i tak tylko jeden
	send_byte(0x44); //uruchamia pomiar temperatury

	HAL_Delay(750);

    wire_reset();
    send_byte(0xcc);
    send_byte(0xbe); //Odczytuje rejestry czujnika

    int i;
    uint8_t scratchpad[9];
    for (i = 0; i < 9; i++)
	  scratchpad[i] = read_byte();

    int16_t value = ((int16_t)scratchpad[1] << 8) | scratchpad[0];

    return value / 16.0;
}


void turn_segments(uint8_t number) {

	uint8_t binary_number = SEGMENT_LUT[number];

	for (uint8_t i = 0; i < 8; i++){

		if (binary_number & (1 << i)){ //sprawdza czy i-ty element to jedynka
			HAL_GPIO_WritePin(Segments[i].port, Segments[i].pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(Segments[i].port, Segments[i].pin, GPIO_PIN_RESET);
		}
	}
	if (DispCnt == 1) {
		HAL_GPIO_WritePin(Segments[7].port, Segments[7].pin, GPIO_PIN_SET);
	}
}

void convert_number_to_digits(uint8_t *digits, float number){
	//zakładam że 100 > temp > 0

	int int_part = (int)number;
	float float_part = number - int_part*1.0;

	digits[0] = (int_part - int_part % 10) / 10;
	digits[1] = int_part % 10;

	float_part *= 100;
	int inted_float_part = (int)float_part;

	digits[2] = (inted_float_part - inted_float_part % 10) / 10;
	digits[3] = inted_float_part % 10;
}

void select_display(uint8_t display){
	switch(display){

	case 0:
		  HAL_GPIO_WritePin(COM_1_GPIO_Port, COM_1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(COM_2_GPIO_Port, COM_2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(COM_3_GPIO_Port, COM_3_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(COM_4_GPIO_Port, COM_4_Pin, GPIO_PIN_SET);
		  break;
	case 1:
		  HAL_GPIO_WritePin(COM_1_GPIO_Port, COM_1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(COM_2_GPIO_Port, COM_2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(COM_3_GPIO_Port, COM_3_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(COM_4_GPIO_Port, COM_4_Pin, GPIO_PIN_SET);
		  break;
	case 2:
		  HAL_GPIO_WritePin(COM_1_GPIO_Port, COM_1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(COM_2_GPIO_Port, COM_2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(COM_3_GPIO_Port, COM_3_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(COM_4_GPIO_Port, COM_4_Pin, GPIO_PIN_SET);
		  break;
	case 3:
		  HAL_GPIO_WritePin(COM_1_GPIO_Port, COM_1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(COM_2_GPIO_Port, COM_2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(COM_3_GPIO_Port, COM_3_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(COM_4_GPIO_Port, COM_4_Pin, GPIO_PIN_RESET);
		  break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim10) {
//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		timer10_interrupt = true;
	}

	if (htim == &htim11) {
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	        HAL_GPIO_WritePin(COM_1_GPIO_Port, COM_1_Pin, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(COM_2_GPIO_Port, COM_2_Pin, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(COM_3_GPIO_Port, COM_3_Pin, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(COM_4_GPIO_Port, COM_4_Pin, GPIO_PIN_SET);

			turn_segments(to_be_displayed[DispCnt]);

			select_display(DispCnt);

			if (DispCnt >= 3) {
				DispCnt = 0;
			} else {
				DispCnt++;
			}
		}
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
