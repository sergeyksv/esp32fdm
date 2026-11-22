#pragma once
#include "freertos/FreeRTOS.h"
StreamBufferHandle_t xStreamBufferCreate(size_t size, size_t trigger);
StreamBufferHandle_t xStreamBufferCreateWithCaps(size_t size, size_t trigger, uint32_t caps);
size_t xStreamBufferSend(StreamBufferHandle_t s, const void *data, size_t len, TickType_t wait);
size_t xStreamBufferReceive(StreamBufferHandle_t s, void *buf, size_t len, TickType_t wait);
BaseType_t xStreamBufferReset(StreamBufferHandle_t s);
