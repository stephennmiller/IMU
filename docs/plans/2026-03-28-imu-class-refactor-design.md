# IMU Class Refactor Design

## Goal

Refactor the monolithic `IMU.ino` sketch into a reusable, non-blocking C++ class (`IMU.h`) that can be integrated into a larger project (solenoid controller with MOSFET on D5, driven by IMU orientation data).

## Scope

Two software-only improvements:

1. **Non-blocking architecture** -- replace all `delay()` usage in `loop()` and recovery with `millis()`-based timing
2. **Board-portable I2C pins** -- replace hardcoded `A4`/`A5` in `bitBangRecover()` with `SDA`/`SCL` constants

## Approach

Single-header class (`IMU.h`) alongside the main `.ino` sketch. Chosen over `.h`/`.cpp` split or full Arduino library because the code is ~300 lines and this is a single-project integration.

## API

```cpp
class IMU {
public:
  IMU(uint8_t addr = 0x68);  // Supports 0x68 (AD0 low) or 0x69 (AD0 high)

  bool begin();           // Wire init, bus scan, MPU init, calibration. Blocking (one-time).
  bool update();          // Non-blocking. Returns true when new data was processed.

  // Orientation (degrees)
  float roll();
  float pitch();
  float yaw();

  // Calibrated values (g / deg/s / Celsius)
  float accX();  float accY();  float accZ();
  float gyroX(); float gyroY(); float gyroZ();
  float temperature();

  // Raw 16-bit sensor values
  int16_t rawAccX();  int16_t rawAccY();  int16_t rawAccZ();
  int16_t rawGyroX(); int16_t rawGyroY(); int16_t rawGyroZ();
  int16_t rawTemp();

  // Status
  bool isOk();
};
```

### Usage in main sketch

```cpp
#include "IMU.h"

IMU imu;

void setup() {
  Serial.begin(115200);
  imu.begin();
  // ... solenoid pin setup, etc.
}

void loop() {
  if (imu.update()) {
    float r = imu.roll();
    // ... solenoid threshold logic
  }
  // ... other non-blocking tasks
}
```

## Non-blocking Architecture

### `update()` internals

- Uses `millis()` to gate sensor reads at `SAMPLE_INTERVAL_MS` (100ms)
- Returns `false` immediately if interval hasn't elapsed
- On I2C error, increments error counter; triggers recovery at threshold (5 errors)
- Recovery is also non-blocking: tries once per `RECOVERY_INTERVAL_MS` (1000ms)
- Returns `true` only when fresh orientation data was computed

### What stays blocking

- `begin()` -- calibration requires the sensor to be still; runs once in `setup()`
- This is intentional and standard practice for sensor initialization

## Portability Fix

`bitBangRecover()` replaces hardcoded pins:

- `A5` -> `SCL` (defined by board's `pins_arduino.h`)
- `A4` -> `SDA`

On Uno/Nano these resolve to A4/A5. On Mega they'd be 20/21, etc.

## Internal Structure

All current globals become private class members:

- Raw sensor data (`_rawAccX`, etc.)
- Calibration offsets (`_axOff`, etc.)
- Computed values cached for getters (`_roll`, `_pitch`, `_yaw`, `_accX`, etc.)
- Timing state (`_lastSampleTime`, `_lastRecoveryAttempt`, `_prevFilterTime`)
- Error tracking (`_i2cErrors`, `_staleCount`, `_ok`)

Constants become `static const` / `static constexpr` class members.

## What Does NOT Change

- Register addresses and configuration (PWR_MGMT_1, ACCEL_CONFIG, GYRO_CONFIG, DLPF)
- Scale factors (4096 LSB/g for +-8g, 32.8 LSB/(deg/s) for +-1000 deg/s)
- WHO_AM_I verification
- Complementary filter logic and ALPHA value (0.96)
- I2C bus scan in `begin()`
- Stale data detection algorithm
- Calibration algorithm (average N samples at rest, subtract 1g on Z)

## Verified Against

- Electronic Cats MPU6050 library (trust score 10) -- `initialize()` / `getMotion6()` pattern
- jrowberg/i2cdevlib -- constructor with address, `testConnection()`
- paulstoffregen/Wire -- `Wire.begin()`, `setClock()`, repeated start with `endTransmission(false)`
