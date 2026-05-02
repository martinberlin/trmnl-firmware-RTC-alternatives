// src/rtc_ultra_sleep.cpp
#include "rtc_ultra_sleep.h"

#include <ArduinoLog.h>
#include <Arduino.h>
#include <Wire.h>
#include <time.h>

#include <bb_rtc.h>
#include <config.h>   // SENSOR_SDA, SENSOR_SCL

static BBRTC g_rtc;

// RV3032 register / bit constants used outside the bb_rtc library
static constexpr uint8_t RV3032_CTRL2_REG  = 0x11; // Control 2 register
static constexpr uint8_t RV3032_CTRL2_ILP  = 0x80; // bit 7: Interrupt Level/Pulse (1=level, 0=pulse)

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
  // Use the board-defined I2C pins (SENSOR_SDA=39, SENSOR_SCL=40 for SENSORIAS3)
  int rc = g_rtc.init(SENSOR_SDA, SENSOR_SCL, true, 100000);
  if (rc != 0) {
    // Using Serial because this is Arduino-style; replace with Log_info if you prefer
    Log.info("[RTC] g_rtc.init failed rc=%d type=%d status=%d\n",
                  rc, g_rtc.getType(), g_rtc.getStatus());
    return false;
  }

  Log.info("[RTC] init OK type=%d status=%d epoch=%lu\n",
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
  Log.info("[RTC] epoch=%lu sane=%d\n", (unsigned long)e, ok);
  return ok;
}

bool rtc_ultra_set_time_from_system()
{
  time_t now = time(nullptr);
  if (now < (time_t)MIN_VALID_EPOCH) {
    Log.info("[RTC] system time invalid, now=%ld\n", (long)now);
    return false;
  }

  g_rtc.setEpoch((uint32_t)now);
  Log.info("[RTC] setEpoch(%lu)\n", (unsigned long)now);
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

uint32_t rtc_ultra_compute_next_wake_epoch(uint32_t refreshSeconds)
{
  time_t now = time(nullptr);

  if (now >= (time_t)MIN_VALID_EPOCH && in_quiet_hours_local(now))
  {
    struct tm wake;
    compute_next_7am_local_tm(now, &wake);
    return (uint32_t)mktime(&wake);
  }

  return (uint32_t)(now + (time_t)refreshSeconds);
}

// Set RV3032 ILP=1 (interrupt level/pulse bit) so the INT pin stays asserted LOW
// until cleared by software (level mode). The default (ILP=0) is pulse mode where
// INT is only LOW for ~7.8 ms -- far too short for the ESP32-S3 to boot (~300-500 ms)
// and latch IO21 HIGH via the Q3 power-hold transistor before power is cut.
static void rv3032_set_ilp_level()
{
  BBI2C *pBB = g_rtc.getBB();
  uint8_t ctrl2 = 0;
  I2CReadRegister(pBB, RTC_RV3032_ADDR, RV3032_CTRL2_REG, &ctrl2, 1);
  uint8_t buf[2] = {RV3032_CTRL2_REG, (uint8_t)(ctrl2 | RV3032_CTRL2_ILP)};
  I2CWrite(pBB, RTC_RV3032_ADDR, buf, 2);
  Log.info("[RTC] ILP=1 level mode set (ctrl2=0x%02X)\n", buf[1]);
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

    // RV3032: ALARM_TIME matches hour+minute; set tm_sec=0, tm_min=0, tm_hour=7.
    g_rtc.setAlarm(ALARM_TIME, &wake);

    Log.info("[RTC] quiet hours: setAlarm(ALARM_TIME) for next 07:00 local\n");
  }
  else
  {
    // Normal mode: countdown alarm for refreshSeconds
    g_rtc.setCountdownAlarm((int)refreshSeconds);
    Log.info("[RTC] normal hours: setCountdownAlarm(%u seconds)\n", (unsigned)refreshSeconds);
  }

  // Keep INT asserted (level mode) until software clears it on next boot.
  // This is required so the power-hold circuit (Q1->Q2) stays enabled long enough
  // for the MCU to boot and set IO21 HIGH to latch the Q3 power-hold transistor.
  rv3032_set_ilp_level();

  return true;
}