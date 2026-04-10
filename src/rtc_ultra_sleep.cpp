// src/rtc_ultra_sleep.cpp
#include "rtc_ultra_sleep.h"

#include <Arduino.h>
#include <Wire.h>
#include <time.h>

#include <bb_rtc.h>
#include <config.h>   // SENSOR_SDA, SENSOR_SCL

static BBRTC g_rtc;

// Quiet hours: 23:00 -> 07:00 (local time)
static constexpr int QUIET_START_HOUR = 23;
static constexpr int QUIET_END_HOUR   = 7;

// Consider RTC "valid" if >= 2024-01-01
static constexpr uint32_t MIN_VALID_EPOCH = 1704067200UL; // 2024-01-01 UTC
static constexpr uint32_t MAX_VALID_EPOCH = 4102444800UL; // 2100-01-01 UTC

static bool epoch_sane(uint32_t e) {
  return (e >= MIN_VALID_EPOCH && e <= MAX_VALID_EPOCH);
}

bool rtc_ultra_begin()
{
  // init(iSDA=-1, iSCL=-1, bWire=true, speed=100k)
  int rc = g_rtc.init(SENSOR_SDA, SENSOR_SCL, true, 100000);
  if (rc != 0) {
    // Using Serial because this is Arduino-style; replace with Log_info if you prefer
    Serial.printf("[RTC] g_rtc.init failed rc=%d type=%d status=%d\n",
                  rc, g_rtc.getType(), g_rtc.getStatus());
    return false;
  }

  Serial.printf("[RTC] init OK type=%d status=%d epoch=%lu\n",
                g_rtc.getType(), g_rtc.getStatus(), (unsigned long)g_rtc.getEpoch());
  return true;
}

uint32_t rtc_ultra_now_epoch()
{
  return g_rtc.getEpoch();
}

bool rtc_ultra_has_valid_time()
{
  uint32_t e = rtc_ultra_now_epoch();
  bool ok = epoch_sane(e);
  Serial.printf("[RTC] epoch=%lu sane=%d\n", (unsigned long)e, ok);
  return ok;
}

bool rtc_ultra_set_time_from_system()
{
  time_t now = time(nullptr);
  if (now < (time_t)MIN_VALID_EPOCH) {
    Serial.printf("[RTC] system time invalid, now=%ld\n", (long)now);
    return false;
  }

  g_rtc.setEpoch((uint32_t)now);
  Serial.printf("[RTC] setEpoch(%lu)\n", (unsigned long)now);
  return true;
}

static bool in_quiet_hours_local(time_t now)
{
  struct tm lt;
  localtime_r(&now, &lt);
  return (lt.tm_hour >= QUIET_START_HOUR) || (lt.tm_hour < QUIET_END_HOUR);
}

static void compute_next_7am_local_tm(time_t now, struct tm *out)
{
  struct tm lt;
  localtime_r(&now, &lt);

  lt.tm_hour = QUIET_END_HOUR;
  lt.tm_min  = 0;
  lt.tm_sec  = 0;

  time_t wake_t = mktime(&lt);
  if (wake_t <= now) {
    lt.tm_mday += 1;
    (void)mktime(&lt); // normalize
  }

  *out = lt;
}

bool rtc_ultra_program_next_wake(uint32_t refreshSeconds)
{
  // Always clear alarm flags first so INT isn't stuck low from a previous alarm
  g_rtc.clearAlarms();

  time_t now = time(nullptr);

  // If system time is valid and we are in quiet hours -> set alarm for 07:00 local
  if (now >= (time_t)MIN_VALID_EPOCH && in_quiet_hours_local(now))
  {
    struct tm wake;
    compute_next_7am_local_tm(now, &wake);

    // RV3032: no seconds precision; bb_rtc doc says ALARM_TIME = hour:second match.
    // In practice for RV3032 you’ll get hour+minute granularity (library adapts).
    // We'll set tm_sec=0; tm_min=0; tm_hour=7.
    g_rtc.setAlarm(ALARM_TIME, &wake);

    Serial.printf("[RTC] quiet hours: setAlarm(ALARM_TIME) for next 07:00 local\n");
    return true;
  }

  // Normal mode: countdown alarm for refreshSeconds
  g_rtc.setCountdownAlarm((int)refreshSeconds);
  Serial.printf("[RTC] normal hours: setCountdownAlarm(%u seconds)\n", (unsigned)refreshSeconds);
  return true;
}