#include "TestChecks.h"
#include <stdio.h>
#include <algorithm>

// Each named case runs in a fresh process, preserving actual sketch defaults.
static void readyAirflow(float velocity = 3.25f) {
  cgAnem.airflowRate = velocity;
  cgAnem.updateOK = true;
  cgAnem.unsteadyProcess = cgAnem.timeoutOnUpdate = false;
  cgAnem.statusReadOK = true;
  updateAirflow();
}
static void configuredRun(int trials = 1) {
  active = runDataEntryComplete = trialDataEntryComplete = true;
  originalRunTime = 15;
  numTrials = trials;
  numTrialsObserved = 1;
  velocityTarget = 5;
  baroyes = loadcellyes = false;
  readyAirflow();
}
static void assertStopped() {
  CHECK(stopLatched && !active && !motorArming && !betweenTrials && !motorInitComplete);
  CHECK(motor.lastPulse == windos::kStopPulseUs);
}
static std::vector<std::string> fields(const std::string &line) {
  std::vector<std::string> result;
  size_t start = 0;
  for (size_t pos = 0; pos < line.size(); ++pos) {
    if (line[pos] == ',') { result.push_back(line.substr(start, pos - start)); start = pos + 1; }
  }
  result.push_back(line.substr(start));
  return result;
}
static std::vector<std::string> lines(const std::string &text) {
  std::vector<std::string> result;
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) result.push_back(line);
  return result;
}
static void boot() {
  // Absent HX711 must not block setup; erased EEPROM must not become valid.
  setup();
  CHECK(motor.attached && !motor.pulses.empty());
  for (int pulse : motor.pulses) CHECK(pulse == windos::kStopPulseUs);
  CHECK(!calibrationValid[0] && !calibrationValid[1]);
  CHECK(LoadCell_1.readCalls == 0 && LoadCell_2.readCalls == 0);
  CHECK(Wire.timeout == 25000);
}
static void stop_latch() {
  configuredRun();
  motorInitComplete = true;
  motorGo(1600);
  mockButton = LOW;
  stopRun("User abort");
  assertStopped();
  for (int i = 0; i < 3; ++i) { loop(); assertStopped(); }
  motorGo(1900);
  assertStopped();
  mockButton = HIGH;
  loop();
  assertStopped();
  mockButton = LOW;
  loop();
  CHECK(!stopLatched && !active && !motorInitComplete && !motorArming);
  CHECK(!runDataEntryComplete && !trialDataEntryComplete);
  CHECK(originalRunTime == 0 && numTrialsObserved == 1);
  CHECK(motor.lastPulse == windos::kStopPulseUs);
  mockButton = HIGH;
  loop();
  CHECK(!motorInitComplete && !motorArming && motor.lastPulse == windos::kStopPulseUs);
}
static void arming_abort() {
  configuredRun();
  mockButton = LOW; // The initiating click cannot immediately cancel arming.
  motorInit();
  loop();
  CHECK(motorArming && !stopLatched && motor.lastPulse == windos::kStopPulseUs);
  mockButton = HIGH;
  loop();
  mockButton = LOW;
  loop();
  assertStopped();
}
static void running_abort() {
  configuredRun();
  beginTrial();
  run_interactive = true;
  runInteractiveFirstRun = false;
  run_hover_1 = false;
  run_hover_2 = true;
  mockButton = LOW;
  loop();
  assertStopped();
  CHECK(lcd.output.find("User abort") != std::string::npos);
}
static void arming_sensor_fault() {
  configuredRun();
  motorInit();
  cgAnem.statusReadOK = false;
  loop();
  assertStopped();
}
static void arming_clock() {
  mockMillis = 0xfffff000UL;
  configuredRun();
  motorInit();
  const uint32_t start = mockMillis;
  mockMillis = start + windos::kArmingMs - 1;
  loop();
  CHECK(motorArming && !motorInitComplete);
  mockMillis = start + windos::kArmingMs;
  loop();
  CHECK(!motorArming && motorInitComplete && !stopLatched);
  CHECK(trialClock.startedAt == mockMillis && trialClock.elapsedMs(mockMillis) == 0);
  CHECK(runTime == 15 && motor.lastPulse == windos::kIdlePulseUs);
  CHECK(Serial.output.find(",0.000,") != std::string::npos);
}
static void airflow_faults() {
  configuredRun();
  beginTrial();
  motorGo(1500);
  cgAnem.updateOK = false;
  mockMillis = 2999;
  updateAirflow();
  motorGovernor();
  CHECK(!stopLatched && motor.lastPulse == 1500 && airflowSample.sampledAt == 0);
  mockMillis = 3000;
  loop();
  assertStopped();
  // Recovery of the sensor alone must not resume the stopped run.
  readyAirflow();
  loop();
  assertStopped();
}
static void airflow_transport_timeout() {
  configuredRun();
  beginTrial();
  cgAnem.timeoutOnUpdate = true;
  loop();
  assertStopped();
}
static void invalid_airflow() {
  configuredRun();
  beginTrial();
  cgAnem.airflowRate = NAN;
  loop();
  assertStopped();
}
static void start_refused() {
  configuredRun();
  cgAnem.unsteadyProcess = true;
  updateAirflow();
  motorInit();
  assertStopped();
  CHECK(Serial.output.empty());
}
static void governor_bounds() {
  configuredRun();
  beginTrial();
  motorpwmlimit = 1700;
  motorGo(32767); CHECK(motor.lastPulse == 1700);
  motorGo(-32767); CHECK(motor.lastPulse == 1200);
  motorTarget = 1700;
  for (int i = 0; i < 100; ++i) {
    mockMillis += 1000;
    readyAirflow(0);
    motorGovernor();
    CHECK(motor.lastPulse == 1700);
  }
  motorTarget = 1200;
  mockMillis += 1000;
  readyAirflow(20);
  motorGovernor();
  CHECK(motor.lastPulse == 1200);
  readyAirflow(-255);
  motorGo(1700);
  CHECK(motor.lastPulse == windos::kStopPulseUs);
}
static void trial_sequence() {
  configuredRun(2);
  beginTrial();
  // Menu time remains measurement time; missed iterations count in full.
  run_interactive = sensorstats_interactive = anemstats_interactive = true;
  mockMillis = 8200;
  readyAirflow();
  serviceTrial();
  CHECK(runTime == 7 && sensorDumperMillis == 8200);
  mockMillis = 15000;
  loop();
  CHECK(betweenTrials && !motorInitComplete && numTrials == 1 && numTrialsObserved == 2);
  CHECK(motor.lastPulse == windos::kStopPulseUs);
  mockMillis = 24999;
  loop();
  CHECK(betweenTrials);
  mockMillis = 25000;
  loop();
  CHECK(motorInitComplete && !betweenTrials && runTime == 15);
  CHECK(trialClock.startedAt == 25000 && sensorDumperMillis == 0);
  CHECK(!run_interactive && !sensorstats_interactive && !anemstats_interactive);
  CHECK(Serial.output.find("Trial Number: 2,0.000,") != std::string::npos);
  mockMillis = 40000;
  loop();
  assertStopped();
  CHECK(numTrials == 0);
}
static void between_abort() {
  configuredRun(2);
  beginTrial();
  mockMillis = 15000;
  loop();
  CHECK(betweenTrials);
  mockButton = LOW;
  loop();
  assertStopped();
}
static void pressure_channels() {
  Adafruit_BMP085_Unified *sensors[] = {&bmp1,&bmp2,&bmp3,&bmp4,&bmp5,&bmp6,&bmp7};
  for (uint8_t i = 0; i < 7; ++i) {
    pressureInitialized[i] = true;
    sensors[i]->pressure = 1000.25f + i;
    baroSelect = i;
    baroRead();
    CHECK(pressureValid[i] && Wire.selectedChannel == i);
    CHECK(sensors[i]->beginCalls == 1 && sensors[i]->eventCalls == 1);
  }
  CHECK(mockBmpWrongCalibrationReads == 0 && baro0 == 1000.25f && baro6 == 1006.25f);
  baroSelect = 0;
  Wire.selectionFails = true;
  baroRead();
  CHECK(!pressureValid[0] && isnan(baro0) && bmp1.eventCalls == 1);
  Wire.selectionFails = false;
  bmp1.beginOK = false;
  baroRead();
  CHECK(!pressureValid[0] && bmp1.eventCalls == 1);
  bmp1.beginOK = true;
  bmp1.timeoutOnRead = true;
  baroRead();
  CHECK(!pressureValid[0] && isnan(baro0));
  bmp1.timeoutOnRead = false;
  bmp1.pressure = NAN;
  baroRead();
  CHECK(!pressureValid[0]);
  bmp1.pressure = 299.9f;
  baroRead();
  CHECK(!pressureValid[0]);
  bmp1.pressure = 1000.75f;
  baroRead();
  CHECK(pressureValid[0] && baro0 == 1000.75f);
}
static void load_availability() {
  CHECK(isnan(readLoadCell(LoadCell_1)) && LoadCell_1.readCalls == 0);
  LoadCell_1.ready = true;
  CHECK(isnan(readLoadCell(LoadCell_1)) && LoadCell_1.tareCalls == 1);
  CHECK(loadCellZeroed[0] && !loadCellZeroed[1]);
  LoadCell_1.ready = true;
  LoadCell_1.units = 12.75f;
  CHECK(readLoadCell(LoadCell_1) == 12.75f);
  CHECK(isnan(readLoadCell(LoadCell_1)));
  LoadCell_1.ready = true;
  LoadCell_1.units = NAN;
  CHECK(isnan(readLoadCell(LoadCell_1)));
  LoadCell_2.ready = true;
  LoadCell_2.units = 22.5f;
  CHECK(isnan(displayLoadCell(LoadCell_2))); // Uncalibrated cannot look valid.
}
static void calibration_persistence() {
  float factor = -1200;
  loadCalibration(0, factor);
  CHECK(!calibrationValid[0] && factor == -1200);
  CHECK(!saveCalibration(0, NAN) && !saveCalibration(0, 0));
  EEPROM.failWrites = true;
  CHECK(!saveCalibration(0, -1500) && !calibrationValid[0]);
  EEPROM.failWrites = false;
  CHECK(saveCalibration(0, -1500));
  loadCalibration(0, factor);
  CHECK(factor == -1500 && calibrationValid[0]);
  EEPROM.failWrites = true;
  CHECK(!saveCalibration(0, -2000));
  CHECK(calibrationValid[0] && factor == -1500);
  EEPROM.failWrites = false;
  EEPROM.corruptWrites = true;
  CHECK(!saveCalibration(0, -2000));
  CHECK(calibrationValid[0] && factor == -1500); // RAM survives a corrupted write too.
  loadCalibration(0, factor);
  CHECK(!calibrationValid[0]); // Corruption must be rejected on reload.
}
static void calibration_capture() {
  calibration_factor_1 = -1200;
  LoadCell_1.set_scale(-1200);
  calibrationValid[0] = loadCellZeroed[0] = true;
  loadcellcalibratedmass = 100;
  loadCellGo1Active = true;
  EEPROM.failWrites = true;
  for (int i = 0; i < 16; ++i) {
    LoadCell_1.ready = true;
    LoadCell_1.units = -200;
    calibrateLoadCell(0);
  }
  assertStopped();
  CHECK(calibration_factor_1 == -1200 && LoadCell_1.scale == -1200 && calibrationValid[0]);
  CHECK(!calibrationCaptureActive && !loadCellGo1Active);
}
static void calibration_success() {
  loadCellZeroed[1] = true;
  loadcellcalibratedmass = 100;
  for (int i = 0; i < 16; ++i) {
    LoadCell_2.ready = true;
    LoadCell_2.units = -150;
    calibrateLoadCell(1);
  }
  CHECK(!stopLatched && calibrationValid[1] && !calibrationValid[0]);
  CHECK(calibration_factor_2 == -1800 && LoadCell_2.scale == -1800);
  windos::CalibrationRecord stored;
  EEPROM.get(calibrationAddress[1], stored);
  CHECK(windos::validCalibration(stored) && stored.factor == -1800);
}
static void calibration_timeout() {
  loadcellcalibratedmass = 100;
  calibrateLoadCell(0);
  mockMillis = 10000;
  calibrateLoadCell(0);
  assertStopped();
  CHECK(LoadCell_1.readCalls == 0 && !calibrationCaptureActive);
}
static void csv_contract() {
  readyAirflow(3.25f);
  baroyes = loadcellyes = true;
  for (int i = 0; i < 7; ++i) pressureInitialized[i] = true;
  bmp2.pressure = NAN;
  calibrationValid[0] = loadCellZeroed[0] = true;
  LoadCell_1.ready = true;
  LoadCell_1.units = -12.75f;
  motorTarget = 1550;
  velocityTarget = 5;
  sensorDumperMillis = 1234;
  sensorDumperInit();
  sensorDumper();
  auto output = lines(Serial.output);
  CHECK(output.size() == 2);
  auto header = fields(output[0]);
  auto row = fields(output[1]);
  CHECK(header.size() == 20 && row.size() == header.size());
  CHECK(row[0] == "Trial Number: 1" && row[1] == "1.234");
  CHECK(row[2] == "1000.25" && row[3].empty());
  CHECK(row[9] == "3.25" && row[12] == "-12.75" && row[13].empty());
  CHECK(row[14] == "50.00" && row[15] == "1" && row[16] == "ok");
  CHECK(row[17] == "1011111" && row[18] == "ok" && row[19] == "uncalibrated");
  Serial.clear();
  cgAnem.updateOK = false;
  updateAirflow();
  sensorDumper();
  row = fields(lines(Serial.output)[0]);
  CHECK(row.size() == 20 && row[0].empty() && row[16] == "held");
  CHECK(row[11].empty() && row[12].empty() && row[18] == "unavailable");
  Serial.clear();
  readyAirflow();
  mockMillis = 3000;
  baroyes = loadcellyes = false;
  sensorDumper();
  row = fields(lines(Serial.output)[0]);
  CHECK(row.size() == 20 && row[9].empty() && row[11].empty() && row[16] == "invalid");
  CHECK(row[17] == "disabled" && row[18] == "disabled" && row[19] == "disabled");
  for (int i = 2; i <= 8; ++i) CHECK(row[i].empty());
}
static void encoder_direction() {
  for (int i = 0; i < 1024; ++i) {
    read_encoder_rot(1); CHECK(encoder_CW && !encoder_CCW);
  }
  for (int i = 0; i < 1024; ++i) {
    read_encoder_rot(-1); CHECK(!encoder_CW && encoder_CCW);
  }
}
struct Case { const char *name; void (*run)(); };
#define CASE(name) {#name, name}
static const Case cases[] = {
  CASE(boot), CASE(stop_latch), CASE(arming_abort), CASE(running_abort), CASE(arming_sensor_fault), CASE(arming_clock),
  CASE(airflow_faults), CASE(airflow_transport_timeout), CASE(invalid_airflow), CASE(start_refused), CASE(governor_bounds),
  CASE(trial_sequence), CASE(between_abort), CASE(pressure_channels), CASE(load_availability),
  CASE(calibration_persistence), CASE(calibration_capture), CASE(calibration_success),
  CASE(calibration_timeout), CASE(csv_contract), CASE(encoder_direction)
};
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  for (const Case &test : cases) {
    if (std::string(argv[1]) == "--list") puts(test.name);
    else if (std::string(argv[1]) == test.name) {
      // Non-boot scenarios begin after the operator acknowledges startup diagnostics.
      if (std::string(test.name) != "boot") startupComplete = true;
      test.run(); printf("PASS: sketch %s\n", test.name); return 0;
    }
  }
  return std::string(argv[1]) == "--list" ? 0 : 2;
}
