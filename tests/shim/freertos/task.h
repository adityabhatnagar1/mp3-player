#pragma once
#include "FreeRTOS.h"
#include <thread>
#include <chrono>
typedef void (*TaskFunction_t)(void*);
typedef void* TaskHandle_t;
inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char*, uint32_t, void* arg, unsigned, TaskHandle_t*, int) {
    std::thread([fn, arg] { fn(arg); }).detach();
    return pdPASS;
}
inline void vTaskDelay(TickType_t ticks) { std::this_thread::sleep_for(std::chrono::milliseconds(ticks)); }
