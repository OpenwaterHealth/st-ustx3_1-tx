/*
 * trigger.c
 *
 *  Created on: Mar 14, 2024
 *      Author: gvigelet
 */

#include "main.h"
#include "trigger.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

extern OW_TimerData _timerDataConfig;
extern OW_TriggerConfig _triggerConfig;

volatile uint32_t currentPulseCount = 0;
volatile uint32_t currentTrainCount = 0;

static TriggerRunState _triggerRunState = TRIGGER_MODE_STOPPED;

static void OW_TIM_Init(void);

static void delay_us(uint32_t us)
{
    // Assuming 48MHz system clock
    // Each loop iteration takes ~4 cycles (measured experimentally)
    // 48MHz / 1us = 48 cycles per microsecond
    // So we need about (48 / 4) = 12 iterations per microsecond

    const uint32_t cycles_per_us = 12;
    volatile uint32_t count;

    while (us--)
    {
        count = cycles_per_us;
        while (count--) __NOP();
    }
}


static void timerDataToJson(char *jsonString, size_t max_length)
{
	memset(jsonString, 0, max_length);
	snprintf(jsonString, max_length,
			 "{"
			 "\"TriggerFrequencyHz\": %lu,"
			 "\"TriggerPulseCount\": %lu,"
			 "\"TriggerPulseWidthUsec\": %lu,"
			 "\"TriggerPulseTrainInterval\": %lu,"
			 "\"TriggerPulseTrainCount\": %lu,"
			 "\"TriggerMode\": %lu,"
			 "\"ProfileIndex\": %lu,"
			 "\"ProfileIncrement\": %lu,"
			 "\"TriggerStatus\": \"%s\""
			 "}",
			 _timerDataConfig.TriggerFrequencyHz,
			 _timerDataConfig.TriggerPulseCount,
			 _timerDataConfig.TriggerPulseWidthUsec,
			 _timerDataConfig.TriggerPulseTrainInterval,
			 _timerDataConfig.TriggerPulseTrainCount,
			 _timerDataConfig.TriggerMode,
			 _timerDataConfig.ProfileIndex,
			 _timerDataConfig.ProfileIncrement,
			 _timerDataConfig.TriggerStatus == HAL_TIM_CHANNEL_STATE_BUSY ? "RUNNING" : "STOPPED");
}

static int jsoneq(const char *json, jsmntok_t *tok, const char *s)
{
	if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
		strncmp(json + tok->start, s, tok->end - tok->start) == 0)
	{
		return 0;
	}
	return -1;
}

static int jsonToTimerData(const char *jsonString)
{
	int i, r;
	jsmn_parser parser;
	jsmntok_t t[32];

	printf("%s\r\n", jsonString);

	jsmn_init(&parser, NULL);
	r = jsmn_parse(&parser, jsonString, strlen(jsonString), t,
				   sizeof(t) / sizeof(t[0]), NULL);

	if (r < 0)
	{
		printf("jsonToTimerData Failed to parse JSON: %d\r\n", r);
		return 1;
	}

	if (r < 1 || t[0].type != JSMN_OBJECT)
	{
		printf("jsonToTimerData Object expected\r\n");
		return 1;
	}

	for (i = 1; i < r - 1; i++)
	{
		if (jsoneq(jsonString, &t[i], "TriggerFrequencyHz") == 0)
		{
			_timerDataConfig.TriggerFrequencyHz = strtol(jsonString + t[i + 1].start, NULL, 10);
			i++;
		}
		else if (jsoneq(jsonString, &t[i], "TriggerPulseCount") == 0)
		{
			_timerDataConfig.TriggerPulseCount = strtol(jsonString + t[i + 1].start, NULL, 10);
			i++;
		}
		else if (jsoneq(jsonString, &t[i], "TriggerPulseWidthUsec") == 0)
		{
			_timerDataConfig.TriggerPulseWidthUsec = strtol(jsonString + t[i + 1].start, NULL, 10);
			i++;
		}
		else if (jsoneq(jsonString, &t[i], "TriggerPulseTrainInterval") == 0)
		{
			_timerDataConfig.TriggerPulseTrainInterval = strtol(jsonString + t[i + 1].start, NULL, 10);
			i++;
		}
		else if (jsoneq(jsonString, &t[i], "TriggerPulseTrainCount") == 0)
		{
			_timerDataConfig.TriggerPulseTrainCount = strtol(jsonString + t[i + 1].start, NULL, 10);
			i++;
		}
		else if (jsoneq(jsonString, &t[i], "TriggerMode") == 0)
		{
			_timerDataConfig.TriggerMode = strtol(jsonString + t[i + 1].start, NULL, 10);
			i++;
		}
		else if (jsoneq(jsonString, &t[i], "ProfileIndex") == 0)
		{
			_timerDataConfig.ProfileIndex = strtol(jsonString + t[i + 1].start, NULL, 10);
			i++;
		}
		else if (jsoneq(jsonString, &t[i], "ProfileIncrement") == 0)
		{
			_timerDataConfig.ProfileIncrement = strtol(jsonString + t[i + 1].start, NULL, 10);
			i++;
		}
	}

	return 0;
}


// Function to configure TRIGGER_TIMER2 generic timer
static void configureTimer2(TIM_HandleTypeDef *timer, uint32_t interval)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	// setup generic timer for 1 usec pulse
	timer->Init.Prescaler = 48 - 1;
	timer->Init.CounterMode = TIM_COUNTERMODE_UP;
	timer->Init.Period = interval - 1;
	timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(timer) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_OnePulse_Init(timer, TIM_OPMODE_SINGLE) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(timer, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_TIM_ENABLE_IT(timer, TIM_IT_UPDATE);
}

// Function to configure TRIGGER_TIMER based on triggerFrequency and triggerPulseWidthUsec
static void configureTimer(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t triggerFrequency, uint32_t triggerPulseWidthUsec)
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	uint32_t period = (100000 / triggerFrequency) - 1;
	uint32_t pulse = (100 * triggerPulseWidthUsec) / 1000; // Convert usec to a factor of the base frequency
	if (pulse > period)
	{
		pulse = period - 2; // Ensure pulse does not exceed the period
	}

	timer->Init.Prescaler = 479;
	timer->Init.CounterMode = TIM_COUNTERMODE_UP;
	timer->Init.Period = period;
	timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	// Initialize the timer
	if (HAL_TIM_Base_Init(timer) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, channel) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(timer);
	__HAL_TIM_ENABLE_IT(timer, TIM_IT_UPDATE);
}

void init_trigger_pulse(TIM_HandleTypeDef *htim, uint32_t channel, TIM_HandleTypeDef *htim2)
{
	if (!_triggerConfig.configured)
	{
		OW_TIM_Init();
	}
	_triggerConfig.channel = channel;
	_triggerConfig.htim = htim;
	_triggerConfig.htim2 = htim2;
	_triggerConfig.configured = true;
}

void deinit_trigger_pulse(TIM_HandleTypeDef *htim, uint32_t channel, TIM_HandleTypeDef *htim2)
{
	if (_triggerConfig.configured)
	{
		// update with current settings setting configured to false
		OW_TIM15_DeInit();
		OW_TIM2_DeInit();
		_triggerConfig.configured = false;
	}
}

bool get_trigger_data(char *jsonString, size_t max_length)
{
	timerDataToJson(jsonString, max_length);
	return true;
}

bool stop_trigger_pulse()
{
    HAL_TIM_PWM_Stop_IT(_triggerConfig.htim, _triggerConfig.channel);
    HAL_TIM_Base_Stop_IT(_triggerConfig.htim2);

    currentPulseCount = 0;
    currentTrainCount = 0;
    _triggerRunState = TRIGGER_MODE_STOPPED;

    HAL_GPIO_WritePin(TRANSMIT_LED_GPIO_Port, TRANSMIT_LED_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	deinit_trigger_pulse(&htim15, TIM_CHANNEL_2, &htim2);
    return true;
}

bool start_trigger_pulse()
{
	HAL_StatusTypeDef status = HAL_OK;
	currentPulseCount = 0;
	currentTrainCount = 0;

	if (_triggerConfig.configured)
	{

		// Check mode and decide behavior
		if (_timerDataConfig.TriggerPulseCount == 0)
		{
			status = HAL_TIM_PWM_Start(_triggerConfig.htim, _triggerConfig.channel);
		}
		else
		{
			// Pulse counting mode
			// Start PWM for first train
        	__HAL_TIM_ENABLE_IT(_triggerConfig.htim, TIM_IT_UPDATE);
			HAL_TIM_PWM_Start_IT(_triggerConfig.htim, _triggerConfig.channel); // Enable update interrupt!
		}
	}
	else
	{
		init_trigger_pulse(_triggerConfig.htim, _triggerConfig.channel, _triggerConfig.htim2);
		configureTimer(_triggerConfig.htim, _triggerConfig.channel, _timerDataConfig.TriggerFrequencyHz, _timerDataConfig.TriggerPulseWidthUsec);

		if(_timerDataConfig.TriggerPulseTrainInterval > 0) {
			configureTimer2(_triggerConfig.htim2, _timerDataConfig.TriggerPulseTrainInterval);
		}

		status = HAL_TIM_PWM_Start(_triggerConfig.htim, _triggerConfig.channel);
	}
	if (status != HAL_OK)
	{
		return false;
	}

	HAL_GPIO_WritePin(TRANSMIT_LED_GPIO_Port, TRANSMIT_LED_Pin, GPIO_PIN_RESET);
    _triggerRunState = TRIGGER_MODE_RUNNING;
	return true;
}

bool set_trigger_data(char *jsonString, size_t str_len)
{
	uint8_t tempArr[255] = {0};
	bool ret = false;

	// Copy the JSON string to tempArr
	memcpy((char *)tempArr, (char *)jsonString, str_len);
	stop_trigger_pulse();

	if (jsonToTimerData((const char *)tempArr) == 0)
	{
		currentPulseCount = 0;
		currentTrainCount = 0;

		ret = true;
	}

	return ret;
}

/**
 * @brief TIM15 Deinitialization Function
 * This function deinitializes the TIM15 used for PWM generation and
 * reconfigures the associated GPIO pin to a standard output pin with a low state.
 * @param None
 * @retval None
 */
void OW_TIM15_DeInit(void)
{
	/* USER CODE BEGIN TIM15_DeInit 0 */

	/* USER CODE END TIM15_DeInit 0 */

	/* 1. Stop the PWM generation on TIM15 Channel 4 */
	if (HAL_TIM_PWM_Stop(&TRIGGER_TIMER, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}

	/* 2. Deinitialize the TIM15 peripheral */
	if (HAL_TIM_PWM_DeInit(&TRIGGER_TIMER) != HAL_OK)
	{
		Error_Handler();
	}

	/* 3. Deinitialize GPIO pin used for TIM15 Channel 4 */
	HAL_GPIO_DeInit(TRIGGER_GPIO_Port, TRIGGER_Pin);

	/* 4. Reconfigure the GPIO pin as a general output pin */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = TRIGGER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TRIGGER_GPIO_Port, &GPIO_InitStruct);

	/* 5. Set the pin to low */
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);

	/* USER CODE BEGIN TIM15_DeInit 1 */
	_triggerConfig.configured = false;

	/* USER CODE END TIM15_DeInit 1 */
}

void OW_TIM2_DeInit(void)
{
	/* 1. Stop the PWM generation on TIM15 Channel 4 */
	if (HAL_TIM_Base_Stop_IT(&TRIGGER_TIMER2) != HAL_OK)
	{
		Error_Handler();
	}

	/* 2. Deinitialize the TIM15 peripheral */
	if (HAL_TIM_PWM_DeInit(&TRIGGER_TIMER2) != HAL_OK)
	{
		Error_Handler();
	}
	_triggerConfig.configured = false;
}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void OW_TIM_Init(void)
{

	/* USER CODE BEGIN TIM15_Init 0 */

	// Calculate the period for the given frequency
	uint32_t period = (100000 / _timerDataConfig.TriggerFrequencyHz) - 1;

	// Calculate the pulse width in timer ticks (since the timer runs at 100000 KHz)
	uint32_t pulse = (100 * _timerDataConfig.TriggerPulseWidthUsec) / 1000; // Convert usec to a factor of the base frequency
	if (pulse > period)
	{
		pulse = period - 2; // Ensure pulse does not exceed the period
	}

	/* USER CODE END TIM15_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	if (_timerDataConfig.TriggerPulseTrainInterval > 0)
	{
		TIM_MasterConfigTypeDef sMasterConfig2 = {0};

		// setup generic timer for 1 usec pulse
		TRIGGER_TIMER2.Instance = TIM2;
		TRIGGER_TIMER2.Init.Prescaler = 48 - 1;
		TRIGGER_TIMER2.Init.CounterMode = TIM_COUNTERMODE_UP;
		TRIGGER_TIMER2.Init.Period = _timerDataConfig.TriggerPulseTrainInterval - 1;
		TRIGGER_TIMER2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&TRIGGER_TIMER2) != HAL_OK)
		{
			Error_Handler();
		}

		if (HAL_TIM_OnePulse_Init(&TRIGGER_TIMER2, TIM_OPMODE_SINGLE) != HAL_OK)
		{
			Error_Handler();
		}

		sMasterConfig2.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig2.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&TRIGGER_TIMER2, &sMasterConfig2) != HAL_OK)
		{
			Error_Handler();
		}
	}

	/* USER CODE BEGIN TIM15_Init 1 */

	/* USER CODE END TIM15_Init 1 */
	TRIGGER_TIMER.Instance = TIM15;
	TRIGGER_TIMER.Init.Prescaler = 479;
	TRIGGER_TIMER.Init.CounterMode = TIM_COUNTERMODE_UP;
	TRIGGER_TIMER.Init.Period = period; // 49999
	TRIGGER_TIMER.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TRIGGER_TIMER.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&TRIGGER_TIMER) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&TRIGGER_TIMER, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&TRIGGER_TIMER) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&TRIGGER_TIMER, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&TRIGGER_TIMER, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&TRIGGER_TIMER);
}

void TRIG_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM15)
    {
        if (_triggerRunState == TRIGGER_MODE_RUNNING)
        {

            currentPulseCount++;

            if (currentPulseCount >= (currentTrainCount <= 1? (_timerDataConfig.TriggerPulseCount-1) : _timerDataConfig.TriggerPulseCount))
            {

            	__HAL_TIM_DISABLE_IT(_triggerConfig.htim, TIM_IT_UPDATE);
            	delay_us(_timerDataConfig.TriggerPulseWidthUsec/2);
                // Stop PWM cleanly after the last pulse completes
                HAL_TIM_PWM_Stop_IT(_triggerConfig.htim, _triggerConfig.channel);

                currentPulseCount = 0; // Reset for next train if needed

                if (_timerDataConfig.TriggerMode == TRIGGER_MODE_SINGLE)
                {
                    _triggerRunState = TRIGGER_MODE_STOPPED;
                    trigger_pulse_CpltCallback();
                    trigger_sequence_CpltCallback();
                    return;
                }
                else if (_timerDataConfig.TriggerMode == TRIGGER_MODE_SEQUENCE)
                {
                    // Start train interval (TIM2)
                    __HAL_TIM_SET_COUNTER(_triggerConfig.htim2, 0);
                    HAL_TIM_Base_Start_IT(_triggerConfig.htim2);
                    _triggerRunState = TRIGGER_MODE_WAITING_TRAIN;
                }
                else if (_timerDataConfig.TriggerMode == TRIGGER_MODE_CONTINUOUS)
                {
                    // Immediately restart the train
                	__HAL_TIM_ENABLE_IT(_triggerConfig.htim, TIM_IT_UPDATE);
                    HAL_TIM_PWM_Start_IT(_triggerConfig.htim, _triggerConfig.channel);
                }

                trigger_pulse_CpltCallback();
            }
        }
    }
}


void TRIG2_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        HAL_TIM_Base_Stop_IT(_triggerConfig.htim2);

        if (_timerDataConfig.TriggerMode == TRIGGER_MODE_SEQUENCE &&
            currentTrainCount >= _timerDataConfig.TriggerPulseTrainCount)
        {
            // All trains completed
            _triggerRunState = TRIGGER_MODE_STOPPED;

            // Stop PWM in case it's still running (safety)
            HAL_TIM_PWM_Stop_IT(_triggerConfig.htim, _triggerConfig.channel);

            trigger_sequence_CpltCallback();
        }
        else
        {
            // Start next pulse train
            currentPulseCount = 0;
            _triggerRunState = TRIGGER_MODE_RUNNING;

        	__HAL_TIM_ENABLE_IT(_triggerConfig.htim, TIM_IT_UPDATE);
            HAL_TIM_PWM_Start_IT(_triggerConfig.htim, _triggerConfig.channel);
        }
        currentTrainCount++;

    }
}


__weak void trigger_sequence_CpltCallback(void)
{
}

__weak void trigger_pulse_CpltCallback(void)
{
}
