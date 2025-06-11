#ifndef __TRIGGER_H
#define __TRIGGER_H

#include "stm32f0xx_hal.h"
#include <stdbool.h>

typedef enum {
    TRIGGER_MODE_SEQUENCE = 0,
    TRIGGER_MODE_CONTINUOUS = 1,
    TRIGGER_MODE_SINGLE = 2
} TriggerSequenceMode;

typedef enum {
    TRIGGER_STATUS_READY = 0,
	TRIGGER_STATUS_RUNNING = 1,
	TRIGGER_STATUS_ERROR = 2,
	TRIGGER_STATUS_NOT_CONFIGURED = 3
} TriggerStatus;

typedef enum {
    TRIGGER_STATE_READY = 0,
	TRIGGER_STATE_PULSE = 1,
	TRIGGER_STATE_PULSE_INTERVAL = 2,
	TRIGGER_STATE_TRAIN_INTERVAL = 3
} TriggerState;

typedef struct {
    uint32_t TriggerFrequencyHz;
    uint32_t TriggerPulseWidthUsec;
    uint32_t TriggerPulseCount;
    uint32_t TriggerPulseTrainInterval;  // in microseconds
    uint32_t TriggerPulseTrainCount;
    uint32_t TriggerMode;
    uint32_t ProfileIndex;
    uint32_t ProfileIncrement;
    uint32_t TriggerState;
    uint32_t TriggerStatus;
} OW_TimerData;

extern volatile uint8_t _running;

// Function prototypes
void deinit_trigger(void);
void init_trigger_pulse(OW_TimerData _timerDataConfig);
uint8_t get_trigger_status(void);
uint8_t start_trigger_pulse(void);
uint8_t stop_trigger_pulse(void);
bool get_trigger_data(char *jsonString, size_t max_length);
bool set_trigger_data(char *jsonString, size_t str_len);

void TRIG_TIM2_IRQHandler(void);
void TRIG_TIM3_IRQHandler(void);
void print_OW_TimerData(const OW_TimerData *data);

// Weak callback functions
__weak void pulse_complete_callback(uint32_t pulse_count);
__weak void pulsetrain_complete_callback(uint32_t train_count);
__weak void sequence_complete_callback(void);

#endif /* __TRIGGER_H */
