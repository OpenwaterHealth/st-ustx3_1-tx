/*
 * trigger.h
 *
 *  Created on: Mar 14, 2024
 *      Author: gvigelet
 */

#ifndef INC_TRIGGER_H_
#define INC_TRIGGER_H_

#include "jsmn.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint32_t TriggerFrequencyHz;
    uint32_t TriggerPulseWidthUsec;
    uint32_t TriggerPulseCount;
    uint32_t TriggerPulseTrainInterval;
    uint32_t TriggerPulseTrainCount;
    uint32_t TriggerMode;
    uint32_t ProfileIndex;
    uint32_t ProfileIncrement;
    uint32_t TriggerStatus;
} OW_TimerData;


typedef enum {
    TRIGGER_MODE_STOPPED = 0,
    TRIGGER_MODE_RUNNING,
    TRIGGER_MODE_WAITING_TRAIN
} TriggerRunState;

typedef enum {
	TRIGGER_MODE_SEQUENCE = 0,
	TRIGGER_MODE_CONTINUOUS = 1,
	TRIGGER_MODE_SINGLE = 2
} TriggerSequenceMode;


typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    TIM_HandleTypeDef *htim2;
    bool configured;
} OW_TriggerConfig;

void OW_TIM15_DeInit(void);
void OW_TIM2_DeInit(void);
void init_trigger_pulse(TIM_HandleTypeDef *htim, uint32_t channel, TIM_HandleTypeDef *htim2);
void deinit_trigger_pulse(TIM_HandleTypeDef *htim, uint32_t channel, TIM_HandleTypeDef *htim2);
bool start_trigger_pulse();
bool stop_trigger_pulse();
bool get_trigger_data(char *jsonString, size_t max_length);
bool set_trigger_data(char *jsonString, size_t str_len);

void trigger_sequence_CpltCallback(void);
void trigger_pulse_CpltCallback(void);

__weak void TRIG_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
__weak void TRIG2_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_TRIGGER_H_ */
