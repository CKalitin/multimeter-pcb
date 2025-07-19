#pragma once

#include "FreeRTOS.h"
#include "task.h"

/* External task handles */
extern TaskHandle_t blink01Handle;
extern TaskHandle_t blink02Handle;

/* Function prototypes */
void StartBlink01(void *argument);
void StartBlink02(void *argument);
void ADS1115_Tasks_Init(void);