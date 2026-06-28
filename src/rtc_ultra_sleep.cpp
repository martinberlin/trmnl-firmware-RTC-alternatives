// src/rtc_ultra_sleep.cpp
#include "rtc_ultra_sleep.h"

#include <ArduinoLog.h>
#include <Arduino.h>
#include <Wire.h>
#include <time.h>
#include <sys/time.h>

#include <bb_rtc.h>
#include <config.h>   // SENSOR_SDA, SENSOR_SCL

// STATUS_IRQ1_TRIGGERED was added in bb_rtc 1.2.0; define a fallback so the
// code compiles cleanly even if an older cached version of the library is
// installed (the numeric value matches the upstream definition).
#ifndef STATUS_IRQ1_TRIGGERED
#define STATUS_IRQ1_TRIGGERED 2
#endif

static BBRTC g_rtc;

// RV3032 register / bit constants used outside the bb_rtc library
static constexpr uint8_t RV3032_CTRL2_REG       = 0x11; // Control 2 register
static constexpr uint8_t RV3032_CTRL2_ILP       = 0x80; // bit 7: Interrupt Level/Pulse (1=level, 0=pulse)
static constexpr uint8_t RV3032_CTRL3_REG       = 0x12; // Control 3 register
static constexpr uint8_t RV3032_CTRL3_BSM_MASK  = 0x0C; // bits [3:2]: Backup Switch Mode
static constexpr uint8_t RV3032_CTRL3_BSM_DIRECT = 0x04; // BSM[1:0]=01 → direct VBACKUP switching

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

  int status = g_rtc.getStatus();
  Log.info("[RTC] init OK type=%d status=%d epoch=%lu\n",
                g_rtc.getType(), status, (unsigned long)g_rtc.getEpoch());
  if (status & STATUS_IRQ1_TRIGGERED) {
    Log.info("[RTC] timer/alarm flag was set -> this wakeup was triggered by the RTC alarm\n");
    // Important: release the RTC interrupt line after a wake so the next alarm
    // can be armed cleanly.
    g_rtc.clearAlarms();
    Log.info("[RTC] cleared alarm flags after RTC wake\n");
  }
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

bool rtc_ultra_sync_system_clock()
{
  uint32_t e = rtc_ultra_now_epoch();
  if (!epoch_sane(e)) {
    Log.info("[RTC] sync_system_clock: epoch %lu not sane, skipping\n", (unsigned long)e);
    return false;
  }
  struct timeval tv = { (time_t)e, 0 };
  settimeofday(&tv, nullptr);
  Log.info("[RTC] system clock set from RTC: epoch=%lu\n", (unsigned long)e);
  return true;
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

  // Keep this calculation aligned with rtc_ultra_program_next_wake(), which
  // rounds normal wake times up to the next whole minute before programming
  // ALARM_TIME on the RV3032.
  time_t wake_t = now + (time_t)refreshSeconds;
  if ((wake_t % 60) != 0) {
    wake_t += (60 - (wake_t % 60));
  }
  return (uint32_t)wake_t;
}

// The bb_rtc setCountdownAlarm() explicitly zeros CTRL3, which sets
// BSM[1:0]=00 (VBACKUP switchover disabled).  Without backup power the
// RV3032 loses its clock the moment VDD is cut by the power-hold circuit,
// so the countdown timer never reaches zero and the INT pin never asserts.
// This function restores BSM=01 (direct switching) so the chip continues
// running from VBACKUP after the main supply is removed.
static void rv3032_restore_backup_switch()
{
  Wire.beginTransmission(RTC_RV3032_ADDR);
  Wire.write(RV3032_CTRL3_REG);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)RTC_RV3032_ADDR, (uint8_t)1);
  uint8_t ctrl3 = Wire.available() ? Wire.read() : 0;

  ctrl3 = (ctrl3 & ~RV3032_CTRL3_BSM_MASK) | RV3032_CTRL3_BSM_DIRECT;

  Wire.beginTransmission(RTC_RV3032_ADDR);
  Wire.write(RV3032_CTRL3_REG);
  Wire.write(ctrl3);
  Wire.endTransmission();

  Log.info("[RTC] CTRL3 BSM=01 (direct VBACKUP switching): 0x%02X\n", ctrl3);
}

// Set RV3032 ILP=1 (interrupt level/pulse bit) so the INT pin stays asserted LOW
// until cleared by software (level mode). The default (ILP=0) is pulse mode where
// INT is only LOW for ~7.8 ms -- far too short for the ESP32-S3 to boot (~300-500 ms)
// and latch IO21 HIGH via the Q3 power-hold transistor before power is cut.
// Uses Wire directly because the installed bb_rtc version does not expose getBB().
static void rv3032_set_ilp_level()
{
  // Read current CTRL2 value
  Wire.beginTransmission(RTC_RV3032_ADDR);
  Wire.write(RV3032_CTRL2_REG);
  Wire.endTransmission(false); // repeated-start so the slave keeps the register pointer
  Wire.requestFrom((uint8_t)RTC_RV3032_ADDR, (uint8_t)1);
  uint8_t ctrl2 = Wire.available() ? Wire.read() : 0;

  // Write back with ILP bit set
  Wire.beginTransmission(RTC_RV3032_ADDR);
  Wire.write(RV3032_CTRL2_REG);
  Wire.write((uint8_t)(ctrl2 | RV3032_CTRL2_ILP));
  Wire.endTransmission();

  Log.info("[RTC] ILP=1 level mode set (ctrl2=0x%02X)\n", ctrl2 | RV3032_CTRL2_ILP);
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

    // Minimal alarm path: clearAlarms() + setAlarm(ALARM_TIME, &wake).
    g_rtc.setAlarm(ALARM_TIME, &wake);
    Log.info("[RTC] quiet hours: setAlarm(ALARM_TIME) for next 07:00 local\n");
  }
  else
  {
    // Normal mode: use a time alarm instead of the RV3032 countdown timer.
    // This relies on the RTC wall-clock time (kept in sync from NTP) and avoids
    // uncertainty around countdown-timer state after full power removal.
    time_t wake_t = now + (time_t)refreshSeconds;
    if ((wake_t % 60) != 0) {
      wake_t += (60 - (wake_t % 60)); // round up so the wake is never early
    }

    struct tm wake;
    localtime_r(&wake_t, &wake);
    wake.tm_sec = 0;

    // Minimal alarm path: clearAlarms() + setAlarm(ALARM_TIME, &wake).
    g_rtc.setAlarm(ALARM_TIME, &wake);
    char wake_hhmm[8];
    snprintf(wake_hhmm, sizeof(wake_hhmm), "%02d:%02d", wake.tm_hour, wake.tm_min);
    Log.info("[RTC] normal hours: setAlarm(ALARM_TIME) for %s local\n", wake_hhmm);
  }


  return true;
}