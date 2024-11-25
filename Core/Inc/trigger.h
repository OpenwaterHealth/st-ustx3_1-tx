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

typedef struct {
    uint32_t TriggerFrequencyHz;
    uint32_t TriggerMode;
    uint32_t TriggerPulseCount;
    uint32_t TriggerPulseWidthUsec;
    uint32_t TriggerStatus;
} OW_TimerData;

typedef struct {
	TIM_HandleTypeDef* htim;
    uint32_t channel;
    bool configured;
} OW_TriggerConfig;

void init_trigger_pulse(TIM_HandleTypeDef* htim, uint32_t channel);
void deinit_trigger_pulse(TIM_HandleTypeDef* htim, uint32_t channel);
void start_trigger_pulse();
void stop_trigger_pulse();
bool get_trigger_data(char *jsonString, size_t max_length);
bool set_trigger_data(char *jsonString, size_t str_len);

#endif /* INC_TRIGGER_H_ */
