#pragma once
// Minimal fake Arduino-ESP32 core: records pin writes / LEDC calls for the adapter host test.
#include <stdint.h>
#include <vector>
#include <string>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
extern std::vector<std::string> g_log;
extern int g_pin[48];
void pinMode(uint8_t,uint8_t);
void digitalWrite(uint8_t,uint8_t);
void delayMicroseconds(unsigned);
#if FAKE_ESP_MAJOR>=3
bool ledcAttachChannel(uint8_t,uint32_t,uint8_t,uint8_t);
bool ledcWrite(uint8_t,uint32_t);
uint32_t ledcWriteTone(uint8_t,uint32_t);
#else
uint32_t ledcSetup(uint8_t,uint32_t,uint8_t);
void ledcAttachPin(uint8_t,uint8_t);
void ledcWrite(uint8_t,uint32_t);
uint32_t ledcWriteTone(uint8_t,uint32_t);
#endif
