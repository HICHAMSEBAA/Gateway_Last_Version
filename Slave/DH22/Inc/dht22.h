/*
 * dh22.h
 *
 *  Created on: Apr 26, 2024
 *      Author: hicham
 *
 * This header file provides function prototypes for interfacing with
 * a DHT22 temperature and humidity sensor on an STM32F4xx microcontroller.
 */

#include "tim.h"  // Include for timer functionalities
#include "gpio.h" // Include for GPIO functionalities
#include "stdio.h"  // Include for standard input/output functionalities
#include "stm32f4xx_hal.h"  // Include for STM32F4xx specific functionalities


#ifndef DH22_H_
#define DH22_H_

// Define the port connected to the DHT22 sensor (modify as needed)
#define DHT22_PORT GPIOA

// Define the pin on the chosen port connected to the DHT22 sensor (modify as needed)
#define DHT22_PIN GPIO_PIN_1

// Function to generate a microsecond delay
void microsecond_delay(uint16_t delay);

// Function to initialize communication with the DHT22 sensor
uint8_t dht22_init(void);

// Function to read temperature and humidity data from the DHT22 sensor
uint8_t dht22_read_data(void);

uint8_t dht22_readings(void);

// Function to format and display the sensor's readings (temperature, Fahrenheit conversion, and humidity)
void dht22_display(void);




#endif /* DH22_H_ */
