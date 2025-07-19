#include "task_ads1115.h"

/* Task handles and attributes */
TaskHandle_t blink01Handle = NULL;
TaskHandle_t blink02Handle = NULL;

/**
  * @brief  Function implementing the blink01 thread.
  * @param  argument: Not used
  * @retval None
  */
void StartBlink01(void *argument)
{
    while (1) {
        // Task code
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
  * @brief  Function implementing the blink02 thread.
  * @param  argument: Not used
  * @retval None
  */
void StartBlink02(void *argument)
{
    while (1) {
        // Task code
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/**
  * @brief  Create and start ADS1115 related tasks
  * @param  None
  * @retval None
  */
void ADS1115_Tasks_Init(void)
{
    /* Create blink01 task */
    xTaskCreate(StartBlink01,           // Task function
                "blink01",              // Task name
                128,                    // Stack size in words
                NULL,                   // Task parameter
                tskIDLE_PRIORITY + 2,   // Priority (normal)
                &blink01Handle);        // Task handle

    /* Create blink02 task */
    xTaskCreate(StartBlink02,           // Task function
                "blink02",              // Task name
                128,                    // Stack size in words
                NULL,                   // Task parameter
                tskIDLE_PRIORITY + 1,   // Priority (below normal)
                &blink02Handle);        // Task handle
}