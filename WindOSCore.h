#ifndef WINDOS_CORE_H
#define WINDOS_CORE_H

#include <stdint.h>
#include <math.h>
#include <string.h>

namespace windos {
// ESC values retain the existing Gale configuration; verify them before use
// with another controller. A stopped command is distinct from running idle.
const int kStopPulseUs = 700;
const int kIdlePulseUs = 1200;
const int kMaxPulseUs = 1900;
const uint32_t kAirflowTimeoutMs = 3000;
const uint32_t kArmingMs = 7000;
const uint32_t kBetweenTrialsMs = 10000;

inline uint32_t elapsed(uint32_t now, uint32_t start) { return now - start; }

struct AirflowSample {
  bool valid;
  uint32_t sampledAt;
  float velocity;
  AirflowSample() : valid(false), sampledAt(0), velocity(0) {}
  void observe(bool ready, float value, uint32_t now) {
    // A not-ready response can mean warm-up or no new sample. It must not
    // renew freshness. Invalid numeric readings invalidate feedback at once.
    if (!isfinite(value) || value < 0 || value > 24.5f) {
      valid = false;
    } else if (ready) {
      valid = true;
      sampledAt = now;
      velocity = value;
    }
  }
  bool fresh(uint32_t now) const {
    return valid && elapsed(now, sampledAt) < kAirflowTimeoutMs;
  }
};

inline int boundedPulse(int requested, int configuredMaximum) {
  int maximum = configuredMaximum;
  if (maximum > kMaxPulseUs) maximum = kMaxPulseUs;
  if (maximum < kIdlePulseUs) maximum = kIdlePulseUs;
  if (requested < kIdlePulseUs) return kIdlePulseUs;
  return requested > maximum ? maximum : requested;
}

struct TrialClock {
  uint32_t startedAt;
  uint32_t durationMs;
  TrialClock() : startedAt(0), durationMs(0) {}
  void start(uint32_t now, uint32_t seconds) {
    startedAt = now;
    durationMs = seconds * 1000UL;
  }
  uint32_t elapsedMs(uint32_t now) const { return elapsed(now, startedAt); }
  bool complete(uint32_t now) const { return elapsedMs(now) >= durationMs; }
  uint32_t remainingSeconds(uint32_t now) const {
    const uint32_t spent = elapsedMs(now);
    return spent >= durationMs ? 0 : (durationMs - spent + 999UL) / 1000UL;
  }
};

const uint32_t kCalibrationMagic = 0x574F5343UL;
const uint16_t kCalibrationVersion = 1;
struct CalibrationRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t checksum;
  float factor;
};
static_assert(sizeof(CalibrationRecord) == 12, "Calibration EEPROM layout changed");

inline bool validCalibrationFactor(float factor) {
  return isfinite(factor) && fabs(factor) >= 1.0f && fabs(factor) <= 1000000.0f;
}

inline uint16_t calibrationChecksum(float factor) {
  uint8_t bytes[sizeof(float)];
  memcpy(bytes, &factor, sizeof(bytes));
  uint16_t crc = 0xffff;
  for (unsigned int i = 0; i < sizeof(bytes); ++i) {
    crc ^= uint16_t(bytes[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}

inline CalibrationRecord calibrationRecord(float factor) {
  CalibrationRecord record = {kCalibrationMagic, kCalibrationVersion,
                               calibrationChecksum(factor), factor};
  return record;
}

inline bool validCalibration(const CalibrationRecord &record) {
  return record.magic == kCalibrationMagic && record.version == kCalibrationVersion &&
         validCalibrationFactor(record.factor) &&
         record.checksum == calibrationChecksum(record.factor);
}
} // namespace windos
#endif
