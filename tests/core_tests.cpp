#include "WindOSCore.h"
#include "TestChecks.h"
#include <stdio.h>
#include <limits>

int main() {
  using namespace windos;
  AirflowSample sample;
  CHECK(!sample.fresh(0));
  sample.observe(true, 4.25f, 100);
  CHECK(sample.fresh(3099));
  sample.observe(false, 4.25f, 2000);
  CHECK(!sample.fresh(3100)); // Failed updates do not renew the deadline.
  sample.observe(true, 4.25f, 4000);
  sample.observe(true, -255, 4001);
  CHECK(!sample.fresh(4001));
  sample.observe(true, std::numeric_limits<float>::infinity(), 4002);
  CHECK(!sample.fresh(4002));
  sample.observe(true, NAN, 4003);
  CHECK(!sample.fresh(4003));
  sample.observe(true, 24.51f, 4004);
  CHECK(!sample.fresh(4004));
  sample.observe(true, 0, 0xfffffff0UL);
  CHECK(sample.fresh(0x10));
  CHECK(!sample.fresh(uint32_t(0xfffffff0UL + kAirflowTimeoutMs)));

  CHECK(boundedPulse(-20000, 1700) == 1200);
  CHECK(boundedPulse(1600, 1700) == 1600);
  CHECK(boundedPulse(1710, 1700) == 1700);
  CHECK(boundedPulse(30000, 2500) == 1900);
  CHECK(boundedPulse(1300, 0) == 1200);

  TrialClock clock;
  clock.start(1000, 15);
  CHECK(clock.remainingSeconds(1000) == 15);
  CHECK(clock.remainingSeconds(2001) == 14);
  CHECK(clock.remainingSeconds(9000) == 7); // Missed loop iterations count fully.
  CHECK(clock.complete(16000));
  CHECK(clock.remainingSeconds(16000) == 0);
  clock.start(50000, 15); // Previous trial/menu time cannot affect a new trial.
  CHECK(clock.elapsedMs(50000) == 0);
  clock.start(0xfffffff0UL, 1);
  CHECK(clock.elapsedMs(0x10) == 32);
  CHECK(!clock.complete(0x10));
  CHECK(clock.complete(uint32_t(0xfffffff0UL + 1000)));

  CalibrationRecord record = calibrationRecord(-1200.25f);
  CHECK(validCalibration(record));
  record.factor += 1;
  CHECK(!validCalibration(record));
  record = calibrationRecord(0);
  CHECK(!validCalibration(record));
  record = calibrationRecord(NAN);
  CHECK(!validCalibration(record));
  record = calibrationRecord(-1200);
  ++record.version;
  CHECK(!validCalibration(record));
  record = calibrationRecord(-1200);
  record.magic = 0;
  CHECK(!validCalibration(record));
  memset(&record, 0xff, sizeof(record));
  CHECK(!validCalibration(record));
  puts("Core regressions passed: freshness, actuator bounds, clocks, calibration records.");
}
