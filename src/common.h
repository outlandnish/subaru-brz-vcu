#pragma once
#include <STM32FreeRTOS.h>

static SemaphoreHandle_t spi_lock = xSemaphoreCreateMutex();

inline void takeSPI() {
  // acquire mutex
  xSemaphoreTake(spi_lock, portMAX_DELAY);
}

inline void releaseSPI() {
  // release mutex
  xSemaphoreGive(spi_lock);
}