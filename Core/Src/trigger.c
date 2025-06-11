#include "trigger.h"
#include "stm32f0xx_hal.h"
#include "main.h"

 #include "jsmn.h"

 #include <stdio.h>
 #include <string.h>
 #include <stdbool.h>
 #include <stdlib.h>

// Internal state variables
static volatile uint32_t _pulseCount = 0;
static volatile uint32_t _trainCount = 0;

static volatile OW_TimerData _timerDataConfig = {
		.TriggerFrequencyHz = 0,
		.TriggerPulseWidthUsec = 0,
		.TriggerPulseCount = 0,
		.TriggerMode = TRIGGER_MODE_SEQUENCE,
		.TriggerPulseTrainCount = 0,
		.TriggerPulseTrainInterval = 0,
		.ProfileIndex = 0,
		.ProfileIncrement = 0,
		.TriggerStatus = TRIGGER_STATUS_NOT_CONFIGURED
};

// Weak callback implementations
__weak void pulsetrain_complete_callback(uint32_t train_count) {
    // Default empty implementation - can be overridden by user
    (void)train_count;
}

__weak void sequence_complete_callback(void) {
    // Default empty implementation - can be overridden by user
}

__weak void pulse_complete_callback(uint32_t pulse_count) {
    // Default empty implementation - can be overridden by user
    (void)pulse_count;
}


static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
  if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
	   strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
	 return 0;
  }
  return -1;
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
			  _timerDataConfig.TriggerStatus == TRIGGER_STATUS_RUNNING ? "RUNNING" : "STOPPED");
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

	 if (r < 0) {
		 printf("jsonToTimerData Failed to parse JSON: %d\r\n", r);
		 return 1;
	 }

	 if (r < 1 || t[0].type != JSMN_OBJECT) {
		 printf("jsonToTimerData Object expected\r\n");
		 return 1;
	 }

	 for (i = 1; i < r - 1; i++) {
		 if (jsoneq(jsonString, &t[i], "TriggerFrequencyHz") == 0) {
			 _timerDataConfig.TriggerFrequencyHz = strtol(jsonString + t[i + 1].start, NULL, 10);
			 i++;
		 } else if (jsoneq(jsonString, &t[i], "TriggerPulseCount") == 0) {
			 _timerDataConfig.TriggerPulseCount = strtol(jsonString + t[i + 1].start, NULL, 10);
			 i++;
		 } else if (jsoneq(jsonString, &t[i], "TriggerPulseWidthUsec") == 0) {
			 _timerDataConfig.TriggerPulseWidthUsec = strtol(jsonString + t[i + 1].start, NULL, 10);
			 i++;
		 } else if (jsoneq(jsonString, &t[i], "TriggerPulseTrainInterval") == 0) {
			 _timerDataConfig.TriggerPulseTrainInterval = strtol(jsonString + t[i + 1].start, NULL, 10);
			 i++;
		 } else if (jsoneq(jsonString, &t[i], "TriggerPulseTrainCount") == 0) {
			 _timerDataConfig.TriggerPulseTrainCount = strtol(jsonString + t[i + 1].start, NULL, 10);
			 i++;
		 } else if (jsoneq(jsonString, &t[i], "TriggerMode") == 0) {
			 _timerDataConfig.TriggerMode = strtol(jsonString + t[i + 1].start, NULL, 10);
			 i++;
		 } else if (jsoneq(jsonString, &t[i], "ProfileIndex") == 0) {
			 _timerDataConfig.ProfileIndex = strtol(jsonString + t[i + 1].start, NULL, 10);
			 i++;
		 } else if (jsoneq(jsonString, &t[i], "ProfileIncrement") == 0) {
			 _timerDataConfig.ProfileIncrement = strtol(jsonString + t[i + 1].start, NULL, 10);
			 i++;
		 }
	 }

	 return 0;
}

static void Configure_TIMERS_Frequency(TIM_HandleTypeDef* htim, uint32_t frequencyHz, bool is32BIT)
{
    uint32_t timer_clk = 48000000;  // 48 MHz source clock
    uint32_t prescaler = 0;
    uint32_t arr = 0;

    // Try to find prescaler and arr such that arr <= 0xFFFF
    for (prescaler = 0; prescaler <= 0xFFFF; prescaler++) {
        uint32_t temp_arr = (timer_clk / (frequencyHz * (prescaler + 1))) - 1;

        if (is32BIT) {
            if (temp_arr <= 0xFFFFFFFF) {
                arr = temp_arr;
                break;
            }
        } else {
            if (temp_arr <= 0xFFFF) {
                arr = temp_arr;
                break;
            }
        }
    }

    // If we reach max prescaler without a valid arr, fallback
    if (prescaler > 0xFFFF) {
        prescaler = 0xFFFF;
        arr = 0xFFFF;
    }

    // Reset and prepare TIM15
    __HAL_TIM_DISABLE(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);

    htim->Instance->PSC = prescaler;
    htim->Instance->ARR = arr;

    // Clear interrupt flags
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
}


static void Configure_ONESHOT_Timer(TIM_HandleTypeDef* htim, uint16_t pulsewidth)
{

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_OC_InitTypeDef sConfigOC = {0};
	  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	  __HAL_TIM_DISABLE(htim);  // Stop timer if running

	  htim->Init.Prescaler = 48-1;
	  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim->Init.Period = (pulsewidth *2) - 1;
	  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim->Init.RepetitionCounter = 0;
	  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&TRIGGER_TIMER) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_TIM_PWM_Init(htim) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_TIM_OnePulse_Init(htim, TIM_OPMODE_SINGLE) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
	  if (HAL_TIM_SlaveConfigSynchro(htim, &sSlaveConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = pulsewidth;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  if (HAL_TIM_PWM_ConfigChannel(&TRIGGER_TIMER, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
	  if (HAL_TIMEx_ConfigBreakDeadTime(&TRIGGER_TIMER, &sBreakDeadTimeConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN TIM15_Init 2 */

	  /* USER CODE END TIM15_Init 2 */
	  HAL_TIM_MspPostInit(htim);

}

void print_OW_TimerData(const OW_TimerData *data) {
    printf("TriggerFrequencyHz: %lu\r\n", data->TriggerFrequencyHz);
    printf("TriggerPulseWidthUsec: %lu\r\n", data->TriggerPulseWidthUsec);
    printf("TriggerPulseCount: %lu\r\n", data->TriggerPulseCount);
    printf("TriggerPulseTrainInterval: %lu\r\n", data->TriggerPulseTrainInterval);
    printf("TriggerPulseTrainCount: %lu\r\n", data->TriggerPulseTrainCount);

    switch(data->TriggerMode){
    case TRIGGER_MODE_CONTINUOUS:
        printf("TriggerMode: CONTINUOUS\r\n");
    	break;
    case TRIGGER_MODE_SEQUENCE:
        printf("TriggerMode: SEQUENCE\r\n");
    	break;
    default:
        printf("TriggerMode: SINGLE\r\n");
    	break;
    }

    printf("ProfileIndex: %lu\r\n", data->ProfileIndex);
    printf("ProfileIncrement: %lu\r\n", data->ProfileIncrement);

    switch(data->TriggerStatus){
    case TRIGGER_STATUS_READY:
        printf("TriggerStatus: READY\r\n");
    	break;
    case TRIGGER_STATUS_RUNNING:
        printf("TriggerStatus: RUNNING\r\n");
    	break;
    case TRIGGER_STATUS_ERROR:
        printf("TriggerStatus: ERROR\r\n");
    	break;
    default:
        printf("TriggerStatus: NOT_CONFIGURED\r\n");
    	break;
    }
}


bool get_trigger_data(char *jsonString, size_t max_length)
{
	 timerDataToJson(jsonString, max_length);
	 return true;
}

bool set_trigger_data(char *jsonString, size_t str_len)
{
	 uint8_t tempArr[255] = {0};
	 bool ret = false;

	 // Copy the JSON string to tempArr
	 memcpy((char *)tempArr, (char *)jsonString, str_len);
	 if(_timerDataConfig.TriggerStatus == TRIGGER_STATUS_RUNNING){
		 stop_trigger_pulse();
	 }

	 if (jsonToTimerData((const char *)tempArr) == 0)
	 {
		 ret = true;
	 }

	 return ret;
}

void deinit_trigger(void)
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

	 /* USER CODE END TIM15_DeInit 1 */
 }

void init_trigger_pulse(OW_TimerData new_timerDataConfig) {
    memcpy((void *)&_timerDataConfig, &new_timerDataConfig, sizeof(OW_TimerData));

    Configure_TIMERS_Frequency(&LORES_TIMER, new_timerDataConfig.TriggerFrequencyHz, false);

    _timerDataConfig.TriggerState = TRIGGER_STATE_READY;
    _timerDataConfig.TriggerStatus = TRIGGER_STATUS_READY;

}


uint8_t get_trigger_status(void)
{
	return (uint8_t)_timerDataConfig.TriggerStatus;
}

uint8_t start_trigger_pulse(void) {
    if (_timerDataConfig.TriggerStatus != TRIGGER_STATUS_READY) return _timerDataConfig.TriggerStatus;


    // Compute period from frequency (in microseconds)
    uint32_t triggerPeriodUsec = 1000000 / _timerDataConfig.TriggerFrequencyHz;

    // Validate: Pulse width must be less than the period
    if (_timerDataConfig.TriggerPulseWidthUsec >= triggerPeriodUsec) {
        _timerDataConfig.TriggerStatus = TRIGGER_STATUS_ERROR;
        return TRIGGER_STATUS_ERROR;
    }

    // Validate: Pulse train interval must be 0 or greater than the period
    if (_timerDataConfig.TriggerPulseTrainInterval > 0 &&
        _timerDataConfig.TriggerPulseTrainInterval <= triggerPeriodUsec) {
        _timerDataConfig.TriggerStatus = TRIGGER_STATUS_ERROR;
        return TRIGGER_STATUS_ERROR;
    }

    _pulseCount = 0;
    _trainCount = 0;

    Configure_ONESHOT_Timer(&TRIGGER_TIMER, _timerDataConfig.TriggerPulseWidthUsec);
    Configure_TIMERS_Frequency(&LORES_TIMER, _timerDataConfig.TriggerFrequencyHz, false);

    __HAL_TIM_DISABLE(&HIRES_TIMER);
    __HAL_TIM_SET_COUNTER(&HIRES_TIMER, 0);

    HIRES_TIMER.Instance->ARR = _timerDataConfig.TriggerPulseTrainInterval - 1;

    // Clear interrupt flags
    __HAL_TIM_CLEAR_FLAG(&HIRES_TIMER, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&LORES_TIMER, TIM_FLAG_UPDATE);

    __HAL_TIM_SET_COUNTER(&TRIGGER_TIMER, 0);  // Just in case
    __HAL_TIM_SET_COUNTER(&LORES_TIMER, 0);
    __HAL_TIM_SET_COUNTER(&HIRES_TIMER, 0);

    __HAL_TIM_ENABLE_IT(&LORES_TIMER, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&HIRES_TIMER, TIM_IT_UPDATE);


    // Start PWM
    HAL_TIM_PWM_Start(&TRIGGER_TIMER, TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&LORES_TIMER);
    _timerDataConfig.TriggerStatus = TRIGGER_STATUS_RUNNING;
    return TRIGGER_STATUS_RUNNING;
}

uint8_t stop_trigger_pulse(void) {
	if(_timerDataConfig.TriggerStatus != TRIGGER_STATUS_RUNNING) return _timerDataConfig.TriggerStatus;

    HAL_TIM_PWM_Stop(&TRIGGER_TIMER, TIM_CHANNEL_2);
    HAL_TIM_Base_Stop_IT(&LORES_TIMER);
    HAL_TIM_Base_Stop_IT(&HIRES_TIMER);
    _timerDataConfig.TriggerStatus = TRIGGER_STATUS_READY;
    return TRIGGER_STATUS_READY;
}

void TRIG_TIM2_IRQHandler(void) {

	if(_timerDataConfig.TriggerStatus != TRIGGER_STATUS_RUNNING) return;
    __HAL_TIM_DISABLE_IT(&HIRES_TIMER, TIM_IT_UPDATE);
    HAL_TIM_Base_Stop_IT(&HIRES_TIMER);

    HAL_TIM_PWM_Stop(&TRIGGER_TIMER, TIM_CHANNEL_2);

	_trainCount++;
    pulsetrain_complete_callback(_trainCount);
    if(_timerDataConfig.TriggerMode == TRIGGER_MODE_SINGLE) {
        stop_trigger_pulse();
        sequence_complete_callback();
        return;
    }else if(_trainCount>=_timerDataConfig.TriggerPulseTrainCount &&  _timerDataConfig.TriggerMode != TRIGGER_MODE_CONTINUOUS) {
        stop_trigger_pulse();
        sequence_complete_callback();
	}else{
	    _pulseCount = 0;
	    HAL_TIM_PWM_Start(&TRIGGER_TIMER, TIM_CHANNEL_2);
	    __HAL_TIM_ENABLE_IT(&LORES_TIMER, TIM_IT_UPDATE);
	    HAL_TIM_Base_Start_IT(&LORES_TIMER);
	}
}

void TRIG_TIM3_IRQHandler(void) {
	if(_timerDataConfig.TriggerStatus != TRIGGER_STATUS_RUNNING) return;

    _pulseCount++;

	if(_timerDataConfig.TriggerPulseTrainInterval == 0 && _timerDataConfig.TriggerMode == TRIGGER_MODE_CONTINUOUS){
		// do anything needed here
	}
	else if(_pulseCount>=_timerDataConfig.TriggerPulseCount)
    {

        __HAL_TIM_DISABLE_IT(&LORES_TIMER, TIM_IT_UPDATE);
        HAL_TIM_Base_Stop_IT(&LORES_TIMER);
        if(_timerDataConfig.TriggerPulseTrainInterval>0) {

            // Re-enable interrupts
            __HAL_TIM_ENABLE_IT(&HIRES_TIMER, TIM_IT_UPDATE);

            // Start PWM
            HAL_TIM_Base_Start_IT(&HIRES_TIMER);
        }else{
			_trainCount++;
        	if(_timerDataConfig.TriggerMode == TRIGGER_MODE_SINGLE)
        	{
				pulsetrain_complete_callback(_trainCount);
		        stop_trigger_pulse();
		        sequence_complete_callback();
        		return;
        	} else {
				HAL_TIM_PWM_Stop(&TRIGGER_TIMER, TIM_CHANNEL_2);
				_pulseCount = 0;
				HAL_TIM_PWM_Start(&TRIGGER_TIMER, TIM_CHANNEL_2);
				__HAL_TIM_ENABLE_IT(&LORES_TIMER, TIM_IT_UPDATE);
				HAL_TIM_Base_Start_IT(&LORES_TIMER);
				pulsetrain_complete_callback(_trainCount);
        	}
    	}
    }
    pulse_complete_callback(_pulseCount);
}
