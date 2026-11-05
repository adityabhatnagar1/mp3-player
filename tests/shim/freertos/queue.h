#pragma once
#include "FreeRTOS.h"
#include <deque>
#include <mutex>
#include <vector>
#include <string.h>
#include <chrono>
#include <thread>
struct SimQueue { std::mutex m; std::deque<std::vector<uint8_t>> q; size_t item; size_t len; };
typedef SimQueue* QueueHandle_t;
inline QueueHandle_t xQueueCreate(size_t len, size_t item) { auto* q = new SimQueue(); q->item = item; q->len = len; return q; }
inline BaseType_t xQueueSend(QueueHandle_t q, const void* it, TickType_t) {
    std::lock_guard<std::mutex> g(q->m);
    if (q->q.size() >= q->len) return pdFALSE;
    q->q.emplace_back((const uint8_t*)it, (const uint8_t*)it + q->item);
    return pdTRUE;
}
inline BaseType_t xQueueReceive(QueueHandle_t q, void* it, TickType_t) {
    std::lock_guard<std::mutex> g(q->m);
    if (q->q.empty()) return pdFALSE;
    memcpy(it, q->q.front().data(), q->item);
    q->q.pop_front();
    return pdTRUE;
}
