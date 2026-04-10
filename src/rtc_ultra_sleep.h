// src/rtc_ultra_sleep.h
#pragma once
#include <stdint.h>

bool rtc_ultra_begin();
bool rtc_ultra_has_valid_time();
uint32_t rtc_ultra_now_epoch();
bool rtc_ultra_set_time_from_system();               // rtc.setEpoch(time(nullptr))
uint32_t rtc_ultra_compute_next_wake_epoch(uint32_t refreshSeconds);
bool rtc_ultra_program_next_wake(uint32_t refreshSeconds); // applies quiet-hours, sets alarm/countdown