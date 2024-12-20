/*
 * thermistor.h
 *
 *  Created on: Dec 16, 2024
 *      Author: GeorgeVigelette
 */

#ifndef INC_THERMISTOR_H_
#define INC_THERMISTOR_H_

#include "main.h"  // Replace with your MCU HAL header

// Define constants
#define BETA 3380.0       // Beta coefficient of the thermistor
#define T0 298.15         // 25°C in Kelvin
#define R0 10000.0        // Resistance at 25°C (10K thermistor)

// Function prototypes
void Thermistor_Start(ADC_HandleTypeDef *hadc, float vRef, float rPullUp);
void Thermistor_Stop(void);
float Thermistor_ReadTemperature(void);

#endif /* INC_THERMISTOR_H_ */
