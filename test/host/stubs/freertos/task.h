#pragma once
#include "freertos/FreeRTOS.h"
void vTaskDelay(TickType_t ticks);
TickType_t xTaskGetTickCount(void);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name, uint32_t stack,
                                   void *arg, UBaseType_t prio, TaskHandle_t *out, BaseType_t core);
BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack,
                       void *arg, UBaseType_t prio, TaskHandle_t *out);
void vTaskDelete(TaskHandle_t t);
#define taskYIELD() ((void)0)
