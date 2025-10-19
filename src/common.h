#pragma once

#ifdef ARDUINO_ARCH_STM32
#include <STM32FreeRTOS.h>
#elif defined(ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#else
#include <FreeRTOS.h>
#include <semphr.h>
#endif

static SemaphoreHandle_t spi_lock = xSemaphoreCreateMutex();

inline void takeSPI() {
  // acquire mutex
  xSemaphoreTake(spi_lock, portMAX_DELAY);
}

inline void releaseSPI() {
  // release mutex
  xSemaphoreGive(spi_lock);
}