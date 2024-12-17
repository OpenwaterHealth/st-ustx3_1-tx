/*
 * thermistor.c
 *
 *  Created on: Dec 16, 2024
 *      Author: GeorgeVigelette
 */
#include "thermistor.h"
#include <math.h>

// Private variables
static ADC_HandleTypeDef *adcHandle = NULL;  // Pointer to the ADC handle
static float referenceVoltage;        // ADC reference voltage
static float pullUpResistance;        // Pull-up resistance value

// Initialize the thermistor
void Thermistor_Start(ADC_HandleTypeDef *hadc, float vRef, float rPullUp)
{
    adcHandle = hadc;
    referenceVoltage = vRef;
    pullUpResistance = rPullUp;

    // Start the ADC in continuous mode
    HAL_ADC_Start(adcHandle);
}

void Thermistor_Stop()
{
    if(adcHandle)
    {
    	HAL_ADC_Stop(adcHandle);
    	adcHandle = NULL;
    }
}

// Read thermistor resistance
static float Thermistor_GetResistance(void)
{
    // Get ADC value
    uint16_t adcValue = HAL_ADC_GetValue(adcHandle);

    // Calculate voltage measured
    float vMeasured = (adcValue / 4095.0f) * referenceVoltage;

    // Calculate thermistor resistance
    return pullUpResistance * ((referenceVoltage / vMeasured) - 1.0f);
}

// Read temperature in Celsius
float Thermistor_ReadTemperature(void)
{
    if(adcHandle)
    {
		float resistance = Thermistor_GetResistance();

		// Use the Beta equation to calculate temperature
		float invT = (1.0f / T0) + (log(resistance / R0) / BETA);
		return (1.0f / invT) - 273.15f;  // Convert Kelvin to Celsius
    }

    return 0;
}

