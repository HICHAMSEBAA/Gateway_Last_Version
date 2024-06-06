/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MY_NRF24.h"
#include "RGB.h"
#include "stdio.h"
#include <stdbool.h> // Include for boolean type
#include <string.h>

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

// commands
#define LED_ON "LED_ON"
#define LED_OFF "LED_OFF"
#define POWER_1 "P_1"
#define POWER_2 "P_2"
#define POWER_3 "P_3"
// -----------------------LED commands-----------------------//
#define LED 4
#define LEDUP 0
#define LEDDOWN 1
// ---------------------------------------------------------//

// -----------------------------------------------------------------------//

char time[10];
char date[10];

#define TX_MODE 1
#define RX_MODE 0
#define DATARATE RF24_250KBPS


// AnlyseThePayload
volatile uint8_t Anlyse ;

// Define the transmission pipe address
uint64_t Addr = 0x11223344AA;

// Define an array to store transmission data with an initial value "SEND"
uint8_t RH1, RH2, TC1, TC2, SUM, CHECK;
float tCelsius, tFahrenheit, RH;
uint32_t Value = 0;
uint8_t R, G, B;
uint8_t payload[32];

// Define an array to store acknowledgment payload
char AckPayload[32];

// Define an array to store received data
char myRxData[32];

// Define an acknowledgment payload with an initial value "Ack by Master !!"
char myAckPayload[32] = "Ack by Node 0";

int count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to configure NRF24 module for transmit mode without acknowledgment
void nrf24_config_mode(bool transmit_mode) {
	// Print information about entering transmit mode without acknowledgment
	//printf("________________________Tx Mode________________________ \n\r");

	if (transmit_mode) {
		// Stop listening for incoming data
		NRF24_stopListening();

		// Set writing pipe address to TxpipeAddrs
		NRF24_openWritingPipe(Addr);
	} else{
		// Start listening for incoming data
		NRF24_startListening();
		// Open reading pipe with address RxpipeAddrs
		NRF24_openReadingPipe(1, Addr);

	}

	// Enable auto acknowledgment
	NRF24_setAutoAck(true);

	// Set channel to 52
	NRF24_setChannel(52);

	// Set payload size to 32 bytes
	NRF24_setPayloadSize(32);

	// Enable dynamic payloads
	NRF24_enableDynamicPayloads();

	// Enable acknowledgment payloads
	NRF24_enableAckPayload();

}

bool Payload_charge_simulate(uint8_t *payload, uint8_t len) {

	uint8_t test_bit = 0;
	// Check for valid payload pointer and length
	if (payload == NULL || len < 11) {
		return false; // Indicate error (invalid payload or insufficient size)
	}

	// Directly access payload elements without unnecessary array indexing
	if (dht22_readings()) {
		// Copy sensor data directly
		payload[1] = RH1;
		payload[2] = RH2;
		payload[3] = TC1;
		payload[4] = TC2;
		test_bit = test_bit + 1;
	}
	if (HAL_ADC_Start(&hadc1) == HAL_OK) {
		HAL_ADC_PollForConversion(&hadc1, 100);
		Value = HAL_ADC_GetValue(&hadc1);
		payload[5] = Value;
		test_bit = test_bit + 2;
	}
	if (getRGB(&R, &G, &B) != 0) {

		payload[6] = R;
		payload[7] = G;
		payload[8] = B;
		test_bit = test_bit + 3;
	}
	payload[0] = test_bit;

	return true; // Indicate successful payload population
}

bool Send_Data(void) {
	nrf24_config_mode(TX_MODE);
	// Variable to track the status of data transmission
	bool send_stat = false;

	// Loop indefinitely until acknowledgment is sent or a timeout occurs
	int coun = 0;
	while (coun < 5) {

		printf("Waiting for acknowledgement... (Attempt %d)\n", coun);
		HAL_Delay(10);

		// Attempt to write data to NRF24 module
		if (NRF24_write(payload, 32)) {
			// If data is successfully written, read acknowledgment payload
			NRF24_read(AckPayload, 32);
			if (strlen(AckPayload) != 0) {
				printf("Master acknowledgement : %s \r\n", AckPayload);
				send_stat = true;
				break;
			}

		}

		// If acknowledgment transmission fails, wait for a short period before retrying
		coun++;
	}

	// Return the status of data transmission
	return send_stat;
}

uint8_t AnalyseThePayload(uint8_t *rxPayload) {
	// Check the first byte of the payload for an error flag (assuming 0 indicates error)
	if (strcmp(rxPayload, LED_ON) == 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	}else if (strcmp(rxPayload, LED_OFF) == 0)
    {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
    }else if (strcmp(rxPayload, POWER_1) == 0)
    {
    	NRF24_setPALevel(RF24_PA_m18dB);
    }else if (strcmp(rxPayload, POWER_2) == 0)
    {
    	NRF24_setPALevel(RF24_PA_m6dB);
    }else if (strcmp(rxPayload, POWER_3) == 0)
    {
    	NRF24_setPALevel(RF24_PA_0dB);
    }else
    {
    	printf("No Command from the master \n");
	}
    return 1;
}


/*
 * Function: Receive_Data
 * Description: Receives data from Node 1 using NRF24 module.
 * Parameters: None
 * Returns: bool - Indicates whether data was received successfully or not
 */
bool Receive_Data(void) {

	nrf24_config_mode(RX_MODE);

	// Variable to track the status of data reception
	bool receive_stat = false;

	while (1) {
		//printf("Waiting for Master request ... \n");
		// Check if there is data available to read
		if (NRF24_available()) {
			// Read data from NRF24 module
			NRF24_read(myRxData, 32);

			// Send acknowledgment payload to Node 1
			NRF24_writeAckPayload(1, myAckPayload, 32);

			// Print the received data
			count++;

			// Set receive_stat to true to indicate successful data reception
			receive_stat = true;
			break;
		}

	}

	// Return the status of data reception
	return receive_stat;
}

void setup(void) {
	// Initialize NRF24 module
	NRF24_Init();

	nrf24_config_mode(RX_MODE);

	// Print information about entering receive mode with acknowledgment
	printf(
			"________________________Engaging communication channels...________________________ \n\r");

	// Print current radio settings
	printRadioSettings();
}

void loop(void) {

	Payload_charge_simulate(payload, 32);
	// Check if data is received successfully
	if (Receive_Data()) {
		printf("Payload : %d | %d | %d | %d | %d | %d | %d | %d | %d | \n\r", payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7], payload[8]);
		// Check if acknowledgment is sent successfully
			Send_Data();


		// Delay for 100 milliseconds before breaking out of the loop
	}
	AnalyseThePayload(myRxData);
	HAL_Delay(200);

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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_TIM6_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim6);
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2) {
		set_time();
	}
	/*
	 * Description: This block of code initializes the NRF24 module, enters receive mode,
	 *              and prints current radio settings.
	 */
	setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		loop();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
	while (1) {
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
