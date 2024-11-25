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

OW_TimerData _timerDataConfig;
OW_TriggerConfig _triggerConfig;


//--static void OW_TIM4_Init(void);
//--static void OW_TIM4_DeInit(void);


static void updateTimerDataFromPeripheral(TIM_HandleTypeDef *htim, uint32_t channel)
{
	// Assuming you have the timer configuration and status
	uint32_t preScaler = htim->Instance->PSC;
	uint32_t timerClockFrequency = HAL_RCC_GetPCLK1Freq() / (preScaler + 1); //-- HAL_RCC_GetPCLK2Freq(
	uint32_t TIM_ARR = htim->Instance->ARR;
	uint32_t TIM_CCRx = HAL_TIM_ReadCapturedValue(htim, channel);
	_timerDataConfig.TriggerMode = 0;
	_timerDataConfig.TriggerPulseCount = 0;
	_timerDataConfig.TriggerFrequencyHz = timerClockFrequency / (TIM_ARR + 1);

	uint32_t pulseWidthUs = (TIM_CCRx * 100000) / timerClockFrequency;

	_timerDataConfig.TriggerPulseWidthUsec = pulseWidthUs * 10; // Set the pulse width as needed

	// Check the timer status to determine if it's running
	_timerDataConfig.TriggerStatus = TIM_CHANNEL_STATE_GET(htim, channel);
}


static void timerDataToJson(char *jsonString, size_t max_length)
{
	memset(jsonString, 0, max_length);
	snprintf(jsonString, max_length,
			 "{"
			 "\"TriggerFrequencyHz\": %lu,"
			 "\"TriggerMode\": \"%s\","
			 "\"TriggerPulseCount\": %lu,"
			 "\"TriggerPulseWidthUsec\": %lu,"
			 "\"TriggerStatus\": \"%s\""
			 "}",
			 _timerDataConfig.TriggerFrequencyHz, _timerDataConfig.TriggerMode > 0 ? "PULSECOUNT" : "CONTINUOUS",
			 _timerDataConfig.TriggerPulseCount, _timerDataConfig.TriggerPulseWidthUsec,
			 _timerDataConfig.TriggerStatus == HAL_TIM_CHANNEL_STATE_BUSY ? "RUNNING" : "STOPPED");
}

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
  if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
    return 0;
  }
  return -1;
}

static int jsonToTimerData(const char *jsonString)
{
	int i;
	int r;
    jsmn_parser parser;
    parser.size = sizeof(parser);
    jsmn_init(&parser, NULL);
    jsmntok_t t[16];

	// init parser
	jsmn_init(&parser, NULL);
	r = jsmn_parse(&parser, jsonString, strlen(jsonString), t,
				 sizeof(t) / sizeof(t[0]), NULL);
	if (r < 0) {
		printf("jsonToTimerData Failed to parse JSON: %d\n", r);
		return 1;
	}

	/* Assume the top-level element is an object */
	if (r < 1 || t[0].type != JSMN_OBJECT) {
		printf("jsonToTimerData Object expected\n");
		return 1;
	}


	/* Loop over all keys of the root object */
	for (i = 1; i < r; i++) {
	    if (jsoneq(jsonString, &t[i], "TriggerFrequencyHz") == 0) {
			/* We may use strndup() to fetch string value */
	    	_timerDataConfig.TriggerFrequencyHz = strtol(jsonString + t[i + 1].start, NULL, 10);
			//printf("- TriggerFrequencyHz: %.*s\r\n", t[i + 1].end - t[i + 1].start,
			//		jsonString + t[i + 1].start);
			i++;
	    } else if (jsoneq(jsonString, &t[i], "TriggerMode") == 0) {
	    	/* We may additionally check if the value is either "true" or "false" */
	    	_timerDataConfig.TriggerMode = strtol(jsonString + t[i + 1].start, NULL, 10);
	    	i++;
		} else if (jsoneq(jsonString, &t[i], "TriggerPulseCount") == 0) {
			/* We may want to do strtol() here to get numeric value */
			_timerDataConfig.TriggerPulseCount = strtol(jsonString + t[i + 1].start, NULL, 10);
			i++;
		} else if (jsoneq(jsonString, &t[i], "TriggerPulseWidthUsec") == 0) {
			/* We may want to do strtol() here to get numeric value */
			_timerDataConfig.TriggerPulseWidthUsec = strtol(jsonString + t[i + 1].start, NULL, 10);
			i++;
		}

	}

    return 0; // Successful parsing
}

#if 0
static int jsonToTimerData(const char *jsonString)
{
	// Parse the JSON string and extract values
	cJSON *rootTimerConfig = cJSON_Parse(jsonString);

	// Check if parsing was successful
	if (rootTimerConfig == NULL)
	{
		const char *error_ptr = cJSON_GetErrorPtr();
		if (error_ptr != NULL)
		{
			fprintf(stderr, "Error before: %s\n", error_ptr);
		}
		return 1;
	}

	// Access values from the cJSON object
	cJSON *triggerFrequency = cJSON_GetObjectItem(rootTimerConfig, "TriggerFrequencyHz");
	cJSON *triggerMode = cJSON_GetObjectItem(rootTimerConfig, "TriggerMode");
	cJSON *triggerPulseCount = cJSON_GetObjectItem(rootTimerConfig, "TriggerPulseCount");
	cJSON *triggerPulseWidthUsec = cJSON_GetObjectItem(rootTimerConfig, "TriggerPulseWidthUsec");

	_timerDataConfig.TriggerFrequencyHz = triggerFrequency->valueint;
	_timerDataConfig.TriggerMode = triggerMode->valueint;
	_timerDataConfig.TriggerPulseCount = triggerPulseCount->valueint;
	_timerDataConfig.TriggerPulseWidthUsec = triggerPulseWidthUsec->valueint;

	// Clean up cJSON object
	cJSON_Delete(rootTimerConfig);

	if ((1000000 / _timerDataConfig.TriggerFrequencyHz) <= _timerDataConfig.TriggerPulseWidthUsec)
	{
		// invalid pulsewidth
		return 1;
	}
	return 0; // Successful parsing
}
#endif

// Function to configure htim4 based on triggerFrequency and triggerPulseWidthUsec
static void configureTimer(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t triggerFrequency, uint32_t triggerPulseWidthUsec)
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	uint32_t period = (100000/triggerFrequency) - 1;
	uint32_t pulse = 25;

	timer->Init.Prescaler = 839;
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
}

void init_trigger_pulse(TIM_HandleTypeDef* htim, uint32_t channel)
{
	if(!_triggerConfig.configured)
	{
		OW_TIM15_Init();//OW_TIM4_Init()
	}
	_triggerConfig.channel = channel;
	_triggerConfig.htim = htim;
	_triggerConfig.configured = true;

	updateTimerDataFromPeripheral(htim, channel);
}

void deinit_trigger_pulse(TIM_HandleTypeDef* htim, uint32_t channel)
{
	if(_triggerConfig.configured)
	{
		// update with current settings setting configured to false
		updateTimerDataFromPeripheral(htim, channel);
		OW_TIM15_DeInit(); //-- OW_TIM4_DeInit()
		_triggerConfig.configured = false;
	}
}

bool get_trigger_data(char *jsonString, size_t max_length)
{
	if(!_triggerConfig.configured)
	{
		return false;
	}

	updateTimerDataFromPeripheral(_triggerConfig.htim , _triggerConfig.channel);
	timerDataToJson(jsonString, max_length);
	return true;
}

void stop_trigger_pulse()
{
	if(_triggerConfig.configured)
	{
		HAL_TIM_PWM_Stop(_triggerConfig.htim , _triggerConfig.channel);
		updateTimerDataFromPeripheral(_triggerConfig.htim , _triggerConfig.channel);
		deinit_trigger_pulse(_triggerConfig.htim , _triggerConfig.channel);
	}
}


void start_trigger_pulse()
{
	if(_triggerConfig.configured)
	{
		HAL_TIM_PWM_Start(_triggerConfig.htim , _triggerConfig.channel);
		updateTimerDataFromPeripheral(_triggerConfig.htim , _triggerConfig.channel);
	}else{
		init_trigger_pulse(_triggerConfig.htim , _triggerConfig.channel);
		configureTimer(_triggerConfig.htim, _triggerConfig.channel, _timerDataConfig.TriggerFrequencyHz, _timerDataConfig.TriggerPulseWidthUsec);
		HAL_TIM_PWM_Start(_triggerConfig.htim , _triggerConfig.channel);
		updateTimerDataFromPeripheral(_triggerConfig.htim , _triggerConfig.channel);
	}
}

bool set_trigger_data(char *jsonString, size_t str_len)
{
	uint8_t tempArr[255] = {0};
	bool ret = false;

    // Copy the JSON string to tempArr
    memcpy((char *)tempArr, (char *)jsonString, str_len);

	if(_timerDataConfig.TriggerStatus == HAL_TIM_CHANNEL_STATE_BUSY)
	{
		// stop timer pwm
		HAL_TIM_PWM_Stop(_triggerConfig.htim , _triggerConfig.channel);
		updateTimerDataFromPeripheral(_triggerConfig.htim , _triggerConfig.channel);
	}

	if (jsonToTimerData((const char *)tempArr) == 0)
	{
	 	configureTimer(_triggerConfig.htim , _triggerConfig.channel, _timerDataConfig.TriggerFrequencyHz, _timerDataConfig.TriggerPulseWidthUsec);
		ret = true;
	}

	return ret;
}

#if 0//--
/**
  * @brief TIM4 Deinitialization Function
  * This function deinitializes the TIM4 used for PWM generation and
  * reconfigures the associated GPIO pin to a standard output pin with a low state.
  * @param None
  * @retval None
  */
static void OW_TIM4_DeInit(void)
{
    /* USER CODE BEGIN TIM4_DeInit 0 */

    /* USER CODE END TIM4_DeInit 0 */

    /* 1. Stop the PWM generation on TIM4 Channel 4 */
    if (HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }

    /* 2. Deinitialize the TIM15 peripheral */
    if (HAL_TIM_PWM_DeInit(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    /* 3. Deinitialize GPIO pin used for TIM15 Channel 2 */
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

    /* USER CODE BEGIN TIM4_DeInit 1 */

    /* USER CODE END TIM4_DeInit 1 */
}


/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void OW_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  // Calculate the period for the given frequency
  uint32_t period = (100000 / _timerDataConfig.TriggerFrequencyHz) - 1;

  // Calculate the pulse width in timer ticks (since the timer runs at 100000 KHz)
  uint32_t pulse = 25;

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 839;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = period; //49999
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);

}
#endif
