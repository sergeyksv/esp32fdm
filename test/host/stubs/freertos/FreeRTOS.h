/* PC stand-in for FreeRTOS: just what printer_comm.c and host_print.c use.
 * Time is the simulation clock (sim.c); nothing really blocks. */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef uint32_t TickType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;
typedef void *TaskHandle_t;
typedef void *QueueHandle_t;
typedef void *SemaphoreHandle_t;
typedef void *StreamBufferHandle_t;
typedef struct { int unused; } StaticStreamBuffer_t;
typedef struct { int unused; } portMUX_TYPE;
typedef void (*TaskFunction_t)(void *);

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdFAIL 0
#define portMAX_DELAY 0xffffffffu
#define portTICK_PERIOD_MS 1
#define configTICK_RATE_HZ 1000
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portMUX_INITIALIZER_UNLOCKED {0}
#define taskENTER_CRITICAL(m) ((void)(m))
#define taskEXIT_CRITICAL(m) ((void)(m))
#define tskNO_AFFINITY (-1)
