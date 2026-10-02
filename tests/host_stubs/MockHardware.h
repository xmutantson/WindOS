#ifndef WINDOS_MOCK_HARDWARE_H
#define WINDOS_MOCK_HARDWARE_H

#include <stdint.h>
#include <cmath>
#include <cstring>
#include <array>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <type_traits>
#include <stdexcept>

#define ARDUINO 10819
#define HIGH 1
#define LOW 0
#define INPUT_PULLUP 2
#define OUTPUT 1
#define DEC 10
#define HEX 16
#define WIRE_HAS_TIMEOUT 1
typedef uint8_t byte;

static uint32_t mockMillis = 0;
static int mockButton = HIGH;
static int mockEncoderA = HIGH;
static int mockEncoderB = HIGH;
inline unsigned long millis() { return mockMillis; }
inline void delay(unsigned long ms) { mockMillis += uint32_t(ms); }
inline int digitalRead(int pin) {
  return pin == 4 ? mockButton : (pin == 3 ? mockEncoderA : (pin == 2 ? mockEncoderB : HIGH));
}
inline void pinMode(int, int) {}

class String {
public:
  std::string value;
  String(const char *s) : value(s) {}
  String(const std::string &s) : value(s) {}
  template<typename T> explicit String(T value_) {
    std::ostringstream text; text << value_; value = text.str();
  }
};
inline String operator+(const char *left, const String &right) {
  return String(std::string(left) + right.value);
}
inline String operator+(const String &left, const char *right) {
  return String(left.value + right);
}

class MockPrint {
public:
  std::string output;
  void clear() { output.clear(); }
  void print(const char *s) { output += s; }
  void print(char c) { output += c; }
  void print(const String &s) { output += s.value; }
  void print(const std::string &s) { output += s; }
  void print(double value, int places = 2) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(places) << value;
    output += text.str();
  }
  void print(float value, int places = 2) { print(double(value), places); }
  template<typename T>
  typename std::enable_if<std::is_integral<T>::value>::type
  print(T value, int base = DEC) {
    std::ostringstream text;
    if (base == HEX) text << std::hex;
    text << static_cast<long long>(value);
    output += text.str();
  }
  void println() { output += '\n'; }
  template<typename T> void println(const T &value) { print(value); println(); }
  template<typename T> void println(const T &value, int places) { print(value, places); println(); }
};

class MockSerial : public MockPrint {
public:
  long baud = 0;
  void begin(long value) { baud = value; }
  void flush() {}
  int available() { return 0; }
  int read() { return -1; }
};
static MockSerial Serial;

class LiquidCrystal : public MockPrint {
public:
  LiquidCrystal(int, int, int, int, int, int) {}
  void begin(int, int) {}
  void setCursor(int, int) {}
};

class Servo {
public:
  std::vector<int> pulses;
  int lastPulse = -1;
  bool attached = false;
  void attach(int) { attached = true; }
  void writeMicroseconds(int pulse) { lastPulse = pulse; pulses.push_back(pulse); }
};

class HX711 {
public:
  bool ready = false;
  float units = 0;
  float scale = 1;
  long offset = 0;
  int readCalls = 0;
  int tareCalls = 0;
  void begin(byte, byte, byte = 128) {}
  bool is_ready() { return ready; }
  void set_scale(float value = 1) { scale = value; }
  float get_scale() { return scale; }
  void set_offset(long value) { offset = value; }
  long get_offset() { return offset; }
  float get_units(byte count = 1) {
    if (!ready || count != 1) throw std::runtime_error("unguarded/multiple HX711 read");
    ready = false; ++readCalls; return units;
  }
  void tare(byte count = 10) {
    if (!ready || count != 1) throw std::runtime_error("unguarded/multiple HX711 tare");
    ready = false; ++tareCalls; offset = 42;
  }
};

class MockWire {
public:
  bool selectionFails = false;
  bool failAll = false;
  bool timeoutFlag = false;
  int selectedChannel = -1;
  int pendingAddress = 0;
  int pendingValue = 0;
  unsigned long timeout = 0;
  void begin() {}
  void beginTransmission(int address) { pendingAddress = address; }
  void write(int value) { pendingValue = value; }
  int endTransmission(bool = true) {
    if (failAll || (pendingAddress == 0x70 && selectionFails)) return 4;
    if (pendingAddress == 0x70) {
      selectedChannel = -1;
      for (int i = 0; i < 8; ++i) if (pendingValue == (1 << i)) selectedChannel = i;
    }
    return 0;
  }
  void setWireTimeout(unsigned long value, bool = false) { timeout = value; }
  bool getWireTimeoutFlag() { return timeoutFlag; }
  void clearWireTimeoutFlag() { timeoutFlag = false; }
};
static MockWire Wire;

#define ANEM_I2C_ADDR 0x11
#define i2c_reg_STATUS 0x06
#define STUP 0
class CG_Anem {
public:
  bool initOK = true;
  bool updateOK = true;
  bool unsteadyProcess = false;
  bool statusReadOK = true;
  bool timeoutOnUpdate = false;
  float airflowRate = 0;
  float temperature = 20;
  float airConsumption = 0;
  float ductArea = 0;
  explicit CG_Anem(uint8_t) {}
  bool init() { return initOK; }
  bool data_update() {
    if (timeoutOnUpdate) Wire.timeoutFlag = true;
    return updateOK;
  }
  bool register_read_byte(uint8_t reg, uint8_t *value) {
    if (reg != i2c_reg_STATUS) throw std::runtime_error("unexpected anemometer register");
    *value = unsteadyProcess ? (1 << STUP) : 0;
    return statusReadOK;
  }
  uint8_t getChipId() { return 0x11; }
  float getFirmwareVersion() { return 1.0f; }
  void set_duct_area(float value) { ductArea = value; }
};

struct sensors_event_t { float pressure; };
static int mockBmpCalibratedId = -1;
static int mockBmpWrongCalibrationReads = 0;
class Adafruit_BMP085_Unified {
public:
  int id;
  bool beginOK = true;
  bool eventOK = true;
  bool timeoutOnRead = false;
  float pressure = 1000.25f;
  int beginCalls = 0;
  int eventCalls = 0;
  explicit Adafruit_BMP085_Unified(int32_t value = -1) : id(value) {}
  bool begin() {
    ++beginCalls;
    if (beginOK) mockBmpCalibratedId = id;
    return beginOK;
  }
  bool getEvent(sensors_event_t *event) {
    ++eventCalls;
    if (mockBmpCalibratedId != id) ++mockBmpWrongCalibrationReads;
    event->pressure = pressure;
    if (timeoutOnRead) Wire.timeoutFlag = true;
    return eventOK;
  }
};

class MockEEPROM {
public:
  std::array<uint8_t, 4096> bytes;
  bool failWrites = false;
  bool corruptWrites = false;
  MockEEPROM() { bytes.fill(255); }
  template<typename T> T &get(int address, T &value) {
    if (address < 0 || size_t(address) + sizeof(T) > bytes.size())
      throw std::runtime_error("EEPROM read outside bounds");
    std::memcpy(&value, bytes.data() + address, sizeof(T)); return value;
  }
  template<typename T> const T &put(int address, const T &value) {
    if (address < 0 || size_t(address) + sizeof(T) > bytes.size())
      throw std::runtime_error("EEPROM write outside bounds");
    if (!failWrites) {
      std::memcpy(bytes.data() + address, &value, sizeof(T));
      if (corruptWrites) bytes[address + sizeof(T) - 1] ^= 1;
    }
    return value;
  }
};
static MockEEPROM EEPROM;
#endif
