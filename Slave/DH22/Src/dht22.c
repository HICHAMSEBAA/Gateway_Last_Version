#include "dht22.h"

extern uint8_t RH1, RH2, TC1, TC2, SUM, CHECK;
extern float tCelsius, tFahrenheit, RH;
uint32_t pMillis, cMillis;

void microsecond_delay(uint16_t delay) {
	__HAL_TIM_SET_COUNTER(&htim6, 0); // Reset the timer counter
	while (__HAL_TIM_GET_COUNTER(&htim6) < delay)
		// Wait until the timer reaches the specified delay
		;
}

uint8_t dht22_init(void) {

	uint8_t Response = 0;
	GPIO_InitTypeDef GPIO_InitStructPrivate = { 0 };
	// Configure GPIO pin as output and pull it low
	GPIO_InitStructPrivate.Pin = DHT22_PIN;
	GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // Initialize GPIO
	HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, 0); // Pull the pin low
	microsecond_delay(1300); // Wait for 1300 microseconds

	// Pull the pin high and wait for 30 microseconds
	HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, 1);
	microsecond_delay(30);

	// Configure GPIO pin as input with pull-up
	GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // Initialize GPIO
	microsecond_delay(40);

	// Check if the sensor responds
	if (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) {
		microsecond_delay(80);
		if ((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)))
			Response = 1;
	}

	// Wait for the pin to go low or timeout
	pMillis = HAL_GetTick();
	cMillis = HAL_GetTick();
	while ((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis) {
		cMillis = HAL_GetTick();
	}

	return Response; // Return the response status
}

uint8_t dht22_read_data(void) {
	uint8_t a, b;
	for (a = 0; a < 8; a++) {
		pMillis = HAL_GetTick(); // Record current time
		cMillis = HAL_GetTick(); // Initialize comparison time
		// Wait for the pin to go high or timeout
		while (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))
				&& pMillis + 2 > cMillis) {
			cMillis = HAL_GetTick(); // Update comparison time
		}
		microsecond_delay(40); // Wait for 40 microseconds
		if (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) // If the pin is low
			b &= ~(1 << (7 - a)); // Clear the corresponding bit in 'b'
		else
			b |= (1 << (7 - a)); // Set the corresponding bit in 'b'
		pMillis = HAL_GetTick(); // Record current time
		cMillis = HAL_GetTick(); // Initialize comparison time
		// Wait for the pin to go low or timeout
		while ((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))
				&& pMillis + 2 > cMillis) {
			cMillis = HAL_GetTick(); // Update comparison time
		}
	}
	return b; // Return the received byte
}

uint8_t dht22_readings(void) {

	uint8_t ReadingStatus = 0;

	// Check if communication with DHT22 sensor is successful
	if (dht22_init()) {
		// Read data from DHT22 sensor
		RH1 = dht22_read_data(); // First 8 bits of humidity
		RH2 = dht22_read_data(); // Second 8 bits of Relative humidity
		TC1 = dht22_read_data(); // First 8 bits of Celsius
		TC2 = dht22_read_data(); // Second 8 bits of Celsius
		SUM = dht22_read_data(); // Check sum

		// Calculate checksum
		CHECK = RH1 + RH2 + TC1 + TC2;
		ReadingStatus = SUM == CHECK;
	}
	return ReadingStatus;
}

void dht22_display() {

	// Check if communication with DHT22 sensor is successful
	if (dht22_readings()) {

		// Verify checksum
		if (CHECK == SUM) {
			// Calculate temperature in Celsius
			if (TC1 > 127) // If TC1=10000000, negative temperature
					{
				tCelsius = (float) TC2 / 10 * (-1);
			} else {
				tCelsius = (float) ((TC1 << 8) | TC2) / 10;
			}

			// Convert temperature to Fahrenheit
			tFahrenheit = tCelsius * 9 / 5 + 32;

			// Calculate relative humidity
			RH = (float) ((RH1 << 8) | RH2) / 10;
		}
	}
	char str_t[20] = { 0 }; // String to store temperature in Celsius
	char str_tf[20] = { 0 }; // String to store temperature in Fahrenheit
	char str_r[20] = { 0 }; // String to store relative humidity

	// Format temperature string in Celsius
	sprintf(str_t, "TEMP : %.2f C", tCelsius);

	// Format temperature string in Fahrenheit
	sprintf(str_tf, "TEMP : %.2f F", tFahrenheit);

	// Format relative humidity string
	sprintf(str_r, "RH : %.2f %%", RH);

	printf("%s\n\r", str_t);
	printf("%s\n\r", str_tf);
	printf("%s\n\r", str_r);
}

