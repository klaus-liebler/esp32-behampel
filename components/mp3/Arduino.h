#pragma once
//Arduino Fake compatibility
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <string.h>
#include <esp_log.h>
#include "pgmspace.h"
#define ESP32
void delay(uint32_t ms);
void yield(void);