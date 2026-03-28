# IMU Class Refactor Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Refactor `IMU.ino` into a single-header `IMU.h` class with non-blocking `update()` and portable I2C pins, so it integrates cleanly into a larger sketch (solenoid controller).

**Architecture:** Single-header class (`IMU/IMU.h`) containing the full `IMU` class definition and inline implementation. The main sketch (`IMU/IMU.ino`) becomes a thin demo that includes the header. All current globals become private members; all functions become methods. `delay()` in the main loop path is replaced with `millis()`-based gating.

**Tech Stack:** Arduino (AVR/Uno/Nano), Wire.h (I2C), MPU-6050

**Note:** This is an Arduino project -- there is no unit test framework. Verification is done via `arduino-cli compile`. If `arduino-cli` is not installed, verification is manual (review the code for correctness against the existing working sketch).

---

### Task 1: Create IMU.h with class skeleton and public API

**Files:**
- Create: `IMU/IMU.h`

**Step 1: Create the header file with include guard, class declaration, and all public/private members**

Write `IMU/IMU.h` with:
- Include guard (`#ifndef IMU_H` / `#define IMU_H`)
- `#include <Wire.h>`
- Full class declaration with:
  - Constructor: `IMU(uint8_t addr = 0x68)`
  - Public methods: `begin()`, `update()`, all getters, `isOk()`
  - Private members: all raw data, calibration offsets, computed values, timing, error tracking
  - Private methods: `initMPU()`, `calibrate()`, `readSensor()`, `writeRegister()`, `bitBangRecover()`, `recoverI2C()`
  - Static constants for register addresses, scale factors, thresholds, timing intervals
- Empty inline method bodies (just `return false;` / `return 0;` stubs)

```cpp
#ifndef IMU_H
#define IMU_H

#include <Wire.h>

class IMU {
public:
  IMU(uint8_t addr = 0x68)
    : _addr(addr), _rawAccX(0), _rawAccY(0), _rawAccZ(0),
      _rawGyroX(0), _rawGyroY(0), _rawGyroZ(0), _rawTemp(0),
      _axOff(0), _ayOff(0), _azOff(0), _gxOff(0), _gyOff(0), _gzOff(0),
      _accX(0), _accY(0), _accZ(0), _gyroX(0), _gyroY(0), _gyroZ(0),
      _roll(0), _pitch(0), _yaw(0), _tempC(0),
      _lastSampleTime(0), _lastRecoveryAttempt(0), _prevFilterTime(0),
      _i2cErrors(0), _staleCount(0),
      _prevAccX(0), _prevAccY(0), _prevAccZ(0),
      _prevGyroX(0), _prevGyroY(0), _prevGyroZ(0),
      _ok(false) {}

  bool begin();
  bool update();

  float roll()        { return _roll; }
  float pitch()       { return _pitch; }
  float yaw()         { return _yaw; }

  float accX()        { return _accX; }
  float accY()        { return _accY; }
  float accZ()        { return _accZ; }
  float gyroX()       { return _gyroX; }
  float gyroY()       { return _gyroY; }
  float gyroZ()       { return _gyroZ; }
  float temperature() { return _tempC; }

  int16_t rawAccX()   { return _rawAccX; }
  int16_t rawAccY()   { return _rawAccY; }
  int16_t rawAccZ()   { return _rawAccZ; }
  int16_t rawGyroX()  { return _rawGyroX; }
  int16_t rawGyroY()  { return _rawGyroY; }
  int16_t rawGyroZ()  { return _rawGyroZ; }
  int16_t rawTemp()   { return _rawTemp; }

  bool isOk()         { return _ok; }

private:
  static const uint8_t REG_PWR_MGMT_1  = 0x6B;
  static const uint8_t REG_WHO_AM_I    = 0x75;
  static const uint8_t REG_ACCEL_CONFIG = 0x1C;
  static const uint8_t REG_GYRO_CONFIG  = 0x1B;
  static const uint8_t REG_DLPF_CONFIG  = 0x1A;
  static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

  static constexpr float ACCEL_SCALE = 4096.0;
  static constexpr float GYRO_SCALE  = 32.8;
  static constexpr float ALPHA       = 0.96;

  static const unsigned long SAMPLE_INTERVAL_MS   = 100;
  static const unsigned long RECOVERY_INTERVAL_MS = 1000;
  static const uint8_t I2C_ERROR_THRESHOLD = 5;
  static const uint8_t STALE_THRESHOLD     = 10;

  uint8_t _addr;

  int16_t _rawAccX, _rawAccY, _rawAccZ;
  int16_t _rawGyroX, _rawGyroY, _rawGyroZ;
  int16_t _rawTemp;

  float _axOff, _ayOff, _azOff;
  float _gxOff, _gyOff, _gzOff;

  float _accX, _accY, _accZ;
  float _gyroX, _gyroY, _gyroZ;
  float _roll, _pitch, _yaw, _tempC;

  unsigned long _lastSampleTime;
  unsigned long _lastRecoveryAttempt;
  unsigned long _prevFilterTime;

  uint8_t _i2cErrors;
  uint8_t _staleCount;
  int16_t _prevAccX, _prevAccY, _prevAccZ;
  int16_t _prevGyroX, _prevGyroY, _prevGyroZ;
  bool _ok;

  bool initMPU();
  bool calibrate(int samples);
  bool readSensor();
  bool writeRegister(uint8_t reg, uint8_t value);
  void bitBangRecover();
  void recoverI2C();
};
```

Leave `begin()`, `update()`, and the private methods as declarations only (implementation in next tasks).

**Step 2: Verify it compiles**

Run: `arduino-cli compile --fqbn arduino:avr:uno IMU/`
If `arduino-cli` is not available, skip -- the file will be verified after implementation methods are added.

**Step 3: Commit**

```bash
git add IMU/IMU.h
git commit -m "Add IMU class skeleton with public API and private members"
```

---

### Task 2: Implement begin() -- Wire init, bus scan, MPU init, calibration

**Files:**
- Modify: `IMU/IMU.h`

**Step 1: Add inline implementation of `begin()`**

Port the logic from the current `setup()` function (lines 48-117 of `IMU.ino`) into `IMU::begin()`. This includes:
- `bitBangRecover()` call
- `Wire.begin()` and `Wire.setClock(100000)`
- Wire timeout setup (AVR only)
- I2C bus scan with Serial output
- MPU address detection (0x68 or 0x69)
- `initMPU()` call
- `calibrate(2000)` call
- Setting `_prevFilterTime = millis()`
- Return `true` on success, `false` on failure (instead of `while(1)`)

Add this after the class declaration, before `#endif`:

```cpp
inline bool IMU::begin() {
  Serial.println(F("Starting IMU..."));

  bitBangRecover();

  Wire.begin();
  Wire.setClock(100000);
#if defined(ARDUINO_ARCH_AVR)
  Wire.setWireTimeout(25000, true);
#endif
  delay(100);

  // Scan I2C bus
  Serial.println(F("Scanning I2C bus..."));
  uint8_t foundAddr = 0;
  uint8_t deviceCount = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  Device found at 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      deviceCount++;
      if (addr == 0x68 || addr == 0x69) {
        foundAddr = addr;
      }
    }
  }
  if (deviceCount == 0) {
    Serial.println(F("No I2C devices found! Check wiring: SDA->A4, SCL->A5, VCC->5V, GND->GND"));
    return false;
  }
  Serial.print(deviceCount);
  Serial.println(F(" device(s) found."));

  if (foundAddr == 0) {
    Serial.println(F("No MPU-6050 found (expected 0x68 or 0x69)."));
    return false;
  }

  _addr = foundAddr;
  if (_addr != 0x68) {
    Serial.print(F("MPU-6050 at alternate address 0x"));
    Serial.println(_addr, HEX);
  }

  if (!initMPU()) {
    Serial.println(F("MPU-6050 init failed!"));
    return false;
  }

  Serial.println(F("Calibrating... keep sensor FLAT and STILL."));
  if (!calibrate(2000)) {
    Serial.println(F("Calibration failed — no valid samples."));
    return false;
  }
  Serial.println(F("Calibration complete."));

  Serial.println(F("MPU-6050 ready."));
  _prevFilterTime = millis();
  _ok = true;
  return true;
}
```

**Key difference from original:** Returns `false` on error instead of `while(1)` -- the caller decides how to handle failure.

**Step 2: Commit**

```bash
git add IMU/IMU.h
git commit -m "Implement IMU::begin() with bus scan, init, and calibration"
```

---

### Task 3: Implement private helper methods (initMPU, calibrate, readSensor, writeRegister)

**Files:**
- Modify: `IMU/IMU.h`

**Step 1: Add inline implementations of the four helper methods**

Port directly from `IMU.ino`, replacing global variables with `_member` equivalents and `mpuAddr` with `_addr`:

```cpp
inline bool IMU::initMPU() {
  if (!writeRegister(REG_PWR_MGMT_1, 0x00)) return false;
  delay(100);

  Wire.beginTransmission(_addr);
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(_addr, (uint8_t)1) != 1) return false;

  uint8_t whoAmI = Wire.read();
  if (whoAmI != 0x68) {
    Serial.print(F("WHO_AM_I = 0x"));
    Serial.print(whoAmI, HEX);
    Serial.println(F(" (expected 0x68)"));
    return false;
  }
  Serial.println(F("WHO_AM_I verified (0x68)."));

  if (!writeRegister(REG_ACCEL_CONFIG, 0x10)) return false;
  if (!writeRegister(REG_GYRO_CONFIG, 0x10))  return false;
  if (!writeRegister(REG_DLPF_CONFIG, 0x03))  return false;

  return true;
}

inline bool IMU::calibrate(int samples) {
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;
  int valid = 0;

  for (int i = 0; i < samples; i++) {
    if (readSensor()) {
      axSum += _rawAccX; aySum += _rawAccY; azSum += _rawAccZ;
      gxSum += _rawGyroX; gySum += _rawGyroY; gzSum += _rawGyroZ;
      valid++;
    }
    delay(1);
  }

  if (valid == 0) return false;

  _axOff = (float)axSum / valid;
  _ayOff = (float)aySum / valid;
  _azOff = (float)azSum / valid - ACCEL_SCALE;
  _gxOff = (float)gxSum / valid;
  _gyOff = (float)gySum / valid;
  _gzOff = (float)gzSum / valid;

  Serial.print(F("Calibrated with "));
  Serial.print(valid);
  Serial.println(F(" samples."));

  return true;
}

inline bool IMU::readSensor() {
  Wire.beginTransmission(_addr);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t received = Wire.requestFrom(_addr, (uint8_t)14);
  if (received != 14) {
    while (Wire.available()) Wire.read();
    return false;
  }

  uint8_t hi, lo;
  hi = Wire.read(); lo = Wire.read(); _rawAccX  = (int16_t)(((uint16_t)hi << 8) | lo);
  hi = Wire.read(); lo = Wire.read(); _rawAccY  = (int16_t)(((uint16_t)hi << 8) | lo);
  hi = Wire.read(); lo = Wire.read(); _rawAccZ  = (int16_t)(((uint16_t)hi << 8) | lo);
  hi = Wire.read(); lo = Wire.read(); _rawTemp  = (int16_t)(((uint16_t)hi << 8) | lo);
  hi = Wire.read(); lo = Wire.read(); _rawGyroX = (int16_t)(((uint16_t)hi << 8) | lo);
  hi = Wire.read(); lo = Wire.read(); _rawGyroY = (int16_t)(((uint16_t)hi << 8) | lo);
  hi = Wire.read(); lo = Wire.read(); _rawGyroZ = (int16_t)(((uint16_t)hi << 8) | lo);

  return true;
}

inline bool IMU::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}
```

**Step 2: Commit**

```bash
git add IMU/IMU.h
git commit -m "Implement IMU private helpers: initMPU, calibrate, readSensor, writeRegister"
```

---

### Task 4: Implement bitBangRecover() and recoverI2C() with portable pins

**Files:**
- Modify: `IMU/IMU.h`

**Step 1: Add inline implementations using SDA/SCL constants**

```cpp
inline void IMU::bitBangRecover() {
  pinMode(SCL, OUTPUT);
  pinMode(SDA, INPUT_PULLUP);

  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
    if (digitalRead(SDA) == HIGH) break;
  }

  // Generate STOP: SDA low->high while SCL high
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(5);

  // Return pins to default before Wire.begin() reconfigures them
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
}

inline void IMU::recoverI2C() {
  bitBangRecover();
  Wire.begin();
  Wire.setClock(100000);
#if defined(ARDUINO_ARCH_AVR)
  Wire.setWireTimeout(25000, true);
#endif
  delay(50);

  _ok = initMPU();
  if (_ok) {
    _staleCount = 0;
  } else {
    Serial.println(F("Recovery failed — initMPU returned error"));
  }
}
```

**Key change:** `A5` -> `SCL`, `A4` -> `SDA` throughout. These are defined by the board's `pins_arduino.h`.

**Step 2: Commit**

```bash
git add IMU/IMU.h
git commit -m "Implement bitBangRecover with portable SDA/SCL pins and recoverI2C"
```

---

### Task 5: Implement update() with non-blocking timing

**Files:**
- Modify: `IMU/IMU.h`

**Step 1: Add inline implementation of `update()`**

This is the core non-blocking method. Port logic from `loop()` (lines 120-195 of `IMU.ino`), replacing `delay(100)` with `millis()` gating and making recovery non-blocking:

```cpp
inline bool IMU::update() {
  unsigned long now = millis();

  if (!_ok) {
    if (now - _lastRecoveryAttempt < RECOVERY_INTERVAL_MS) return false;
    _lastRecoveryAttempt = now;
    Serial.println(F("IMU offline — attempting recovery"));
    recoverI2C();
    return false;
  }

  if (now - _lastSampleTime < SAMPLE_INTERVAL_MS) return false;
  _lastSampleTime = now;

  if (!readSensor()) {
    _i2cErrors++;
    if (_i2cErrors >= I2C_ERROR_THRESHOLD) {
      Serial.println(F("I2C errors — recovering bus"));
      recoverI2C();
      _i2cErrors = 0;
    }
    return false;
  }
  _i2cErrors = 0;

  // Stale data detection
  if (_rawAccX == _prevAccX && _rawAccY == _prevAccY && _rawAccZ == _prevAccZ &&
      _rawGyroX == _prevGyroX && _rawGyroY == _prevGyroY && _rawGyroZ == _prevGyroZ) {
    _staleCount++;
  } else {
    _staleCount = 0;
  }
  _prevAccX = _rawAccX; _prevAccY = _rawAccY; _prevAccZ = _rawAccZ;
  _prevGyroX = _rawGyroX; _prevGyroY = _rawGyroY; _prevGyroZ = _rawGyroZ;

  if (_staleCount >= STALE_THRESHOLD) {
    Serial.println(F("Stale data — recovering bus"));
    recoverI2C();
    _staleCount = 0;
    return false;
  }

  // Convert to physical units
  _accX = (_rawAccX - _axOff) / ACCEL_SCALE;
  _accY = (_rawAccY - _ayOff) / ACCEL_SCALE;
  _accZ = (_rawAccZ - _azOff) / ACCEL_SCALE;

  _gyroX = (_rawGyroX - _gxOff) / GYRO_SCALE;
  _gyroY = (_rawGyroY - _gyOff) / GYRO_SCALE;
  _gyroZ = (_rawGyroZ - _gzOff) / GYRO_SCALE;

  _tempC = (_rawTemp / 340.0) + 36.53;

  // Time delta for complementary filter
  float dt = (now - _prevFilterTime) / 1000.0;
  _prevFilterTime = now;
  if (dt > 0.5) dt = 0.0;

  // Accelerometer-based angles
  float accelRoll  = atan2(_accY, sqrt(_accX * _accX + _accZ * _accZ)) * 180.0 / PI;
  float accelPitch = atan2(-_accX, sqrt(_accY * _accY + _accZ * _accZ)) * 180.0 / PI;

  // Complementary filter
  _roll  = ALPHA * (_roll  + _gyroX * dt) + (1.0 - ALPHA) * accelRoll;
  _pitch = ALPHA * (_pitch + _gyroY * dt) + (1.0 - ALPHA) * accelPitch;
  _yaw  += _gyroZ * dt;  // no accel correction (needs magnetometer)

  return true;
}
```

**Key differences from original `loop()`:**
- No `delay(100)` -- uses `millis()` interval check at top
- Recovery path is non-blocking (returns `false`, tries again next `RECOVERY_INTERVAL_MS`)
- No Serial output of data -- that's the caller's responsibility now
- Returns `true` only when new orientation data is ready

**Step 2: Commit**

```bash
git add IMU/IMU.h
git commit -m "Implement non-blocking IMU::update() with millis()-based timing"
```

---

### Task 6: Rewrite IMU.ino as thin demo sketch

**Files:**
- Modify: `IMU/IMU.ino`

**Step 1: Replace entire contents of `IMU.ino`**

The sketch becomes a thin consumer of the `IMU` class, demonstrating usage:

```cpp
#include "IMU.h"

IMU imu;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  if (!imu.begin()) {
    Serial.println(F("IMU init failed! Halting."));
    while (1);
  }

  Serial.println(F("AccX(g)\tAccY(g)\tAccZ(g)\tGx(d/s)\tGy(d/s)\tGz(d/s)\tRoll\tPitch\tYaw\tTemp(C)"));
}

void loop() {
  if (imu.update()) {
    Serial.print(imu.accX(), 2);  Serial.print('\t');
    Serial.print(imu.accY(), 2);  Serial.print('\t');
    Serial.print(imu.accZ(), 2);  Serial.print('\t');
    Serial.print(imu.gyroX(), 2); Serial.print('\t');
    Serial.print(imu.gyroY(), 2); Serial.print('\t');
    Serial.print(imu.gyroZ(), 2); Serial.print('\t');
    Serial.print(imu.roll(), 1);  Serial.print('\t');
    Serial.print(imu.pitch(), 1); Serial.print('\t');
    Serial.print(imu.yaw(), 1);   Serial.print('\t');
    Serial.println(imu.temperature(), 1);
  }
}
```

**Note:** No `delay()` anywhere in the sketch. The timing is handled internally by `imu.update()`.

**Step 2: Verify it compiles**

Run: `arduino-cli compile --fqbn arduino:avr:uno IMU/`
Expected: Compilation succeeds with no errors.

If `arduino-cli` is not available, manually verify:
- `IMU.h` is included
- All getter methods used in `loop()` match the class API
- No leftover references to old globals or functions

**Step 3: Commit**

```bash
git add IMU/IMU.ino
git commit -m "Rewrite IMU.ino as thin demo sketch using IMU class"
```

---

### Task 7: Final review and verification

**Files:**
- Review: `IMU/IMU.h`, `IMU/IMU.ino`

**Step 1: Verify no leftover globals or functions**

Check `IMU.ino` has no function definitions other than `setup()` and `loop()`. Check there are no global variables other than `IMU imu`.

**Step 2: Verify complete compilation**

Run: `arduino-cli compile --fqbn arduino:avr:uno IMU/`
Expected: Compilation succeeds.

**Step 3: Verify behavioral equivalence**

Manually compare these behaviors between old and new code:
- [ ] Same register configuration (PWR_MGMT_1=0x00, ACCEL_CONFIG=0x10, GYRO_CONFIG=0x10, DLPF=0x03)
- [ ] Same scale factors (ACCEL_SCALE=4096.0, GYRO_SCALE=32.8)
- [ ] Same complementary filter (ALPHA=0.96)
- [ ] Same calibration (2000 samples, subtract 1g on Z)
- [ ] Same stale detection (threshold=10)
- [ ] Same I2C error threshold (5)
- [ ] Same WHO_AM_I check (0x68)
- [ ] `SDA`/`SCL` used instead of `A4`/`A5` in bitBangRecover
- [ ] No `delay()` in update path

**Step 4: Commit if any fixes were needed**

```bash
git add IMU/IMU.h IMU/IMU.ino
git commit -m "Fix issues found during final review"
```
