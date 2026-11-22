#pragma once
/* PC stand-in for ESP-IDF's FreeRTOS additions: tasks with caps-allocated
 * stacks (main/task_util.h). sim.c runs them like any other task */
#include "freertos/FreeRTOS.h"
typedef uint8_t StackType_t;
typedef struct { int unused; } StaticTask_t;
BaseType_t xTaskCreatePinnedToCoreWithCaps(TaskFunction_t fn, const char *name, uint32_t stack,
                                           void *arg, UBaseType_t prio, TaskHandle_t *out,
                                           BaseType_t core, UBaseType_t caps);
void vTaskDeleteWithCaps(TaskHandle_t t);
BaseType_t xTaskGetStaticBuffers(TaskHandle_t t, StackType_t **stack, StaticTask_t **tcb);
