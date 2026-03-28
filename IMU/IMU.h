#ifndef IMU_H
#define IMU_H

#include <Wire.h>

class IMU {
public:
    IMU(uint8_t addr = 0x68)
        : _addr(addr),
          _rawAccX(0), _rawAccY(0), _rawAccZ(0),
          _rawGyroX(0), _rawGyroY(0), _rawGyroZ(0),
          _rawTemp(0),
          _axOff(0), _ayOff(0), _azOff(0),
          _gxOff(0), _gyOff(0), _gzOff(0),
          _accX(0), _accY(0), _accZ(0),
          _gyroX(0), _gyroY(0), _gyroZ(0),
          _roll(0), _pitch(0), _yaw(0), _tempC(0),
          _lastSampleTime(0), _lastRecoveryAttempt(0), _prevFilterTime(0),
          _i2cErrors(0), _staleCount(0),
          _prevAccX(0), _prevAccY(0), _prevAccZ(0),
          _prevGyroX(0), _prevGyroY(0), _prevGyroZ(0),
          _ok(false)
    {}

    bool begin();
    bool update();

    float roll()        const { return _roll; }
    float pitch()       const { return _pitch; }
    float yaw()         const { return _yaw; }
    float accX()        const { return _accX; }
    float accY()        const { return _accY; }
    float accZ()        const { return _accZ; }
    float gyroX()       const { return _gyroX; }
    float gyroY()       const { return _gyroY; }
    float gyroZ()       const { return _gyroZ; }
    float temperature() const { return _tempC; }

    int16_t rawAccX()   const { return _rawAccX; }
    int16_t rawAccY()   const { return _rawAccY; }
    int16_t rawAccZ()   const { return _rawAccZ; }
    int16_t rawGyroX()  const { return _rawGyroX; }
    int16_t rawGyroY()  const { return _rawGyroY; }
    int16_t rawGyroZ()  const { return _rawGyroZ; }
    int16_t rawTemp()   const { return _rawTemp; }

    bool isOk()         const { return _ok; }

private:
    // Register addresses
    static const uint8_t REG_PWR_MGMT_1   = 0x6B;
    static const uint8_t REG_WHO_AM_I      = 0x75;
    static const uint8_t REG_ACCEL_CONFIG  = 0x1C;
    static const uint8_t REG_GYRO_CONFIG   = 0x1B;
    static const uint8_t REG_DLPF_CONFIG   = 0x1A;
    static const uint8_t REG_ACCEL_XOUT_H  = 0x3B;

    // Sensor scales and filter weight
    static constexpr float ACCEL_SCALE = 4096.0;
    static constexpr float GYRO_SCALE  = 32.8;
    static constexpr float ALPHA       = 0.96;

    // Timing and thresholds
    static const unsigned long SAMPLE_INTERVAL_MS   = 100;
    static const unsigned long RECOVERY_INTERVAL_MS = 1000;
    static const uint8_t I2C_ERROR_THRESHOLD        = 5;
    static const uint8_t STALE_THRESHOLD            = 10;

    // Private methods
    bool initMPU();
    bool calibrate(int samples);
    bool readSensor();
    bool writeRegister(uint8_t reg, uint8_t value);
    void bitBangRecover();
    void recoverI2C();

    // I2C address
    uint8_t _addr;

    // Raw sensor readings
    int16_t _rawAccX, _rawAccY, _rawAccZ;
    int16_t _rawGyroX, _rawGyroY, _rawGyroZ;
    int16_t _rawTemp;

    // Calibration offsets
    float _axOff, _ayOff, _azOff;
    float _gxOff, _gyOff, _gzOff;

    // Converted values
    float _accX, _accY, _accZ;
    float _gyroX, _gyroY, _gyroZ;

    // Orientation and temperature
    float _roll, _pitch, _yaw, _tempC;

    // Timing
    unsigned long _lastSampleTime;
    unsigned long _lastRecoveryAttempt;
    unsigned long _prevFilterTime;

    // Error tracking
    uint8_t _i2cErrors;
    uint8_t _staleCount;

    // Previous readings for stale detection
    int16_t _prevAccX, _prevAccY, _prevAccZ;
    int16_t _prevGyroX, _prevGyroY, _prevGyroZ;

    // Status
    bool _ok;
};

// ---------------------------------------------------------------------------
// Inline implementations
// ---------------------------------------------------------------------------

inline bool IMU::begin() {
    Serial.println(F("Starting IMU..."));

    // Recover I2C bus in case MPU is stuck from a previous session
    bitBangRecover();

    Wire.begin();
    Wire.setClock(100000);
#if defined(ARDUINO_ARCH_AVR)
    Wire.setWireTimeout(25000, true);
#endif
    delay(100);

    // Scan I2C bus for devices
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

    // Initialize MPU-6050
    if (!initMPU()) {
        Serial.println(F("MPU-6050 init failed!"));
        return false;
    }

    // Calibrate — keep sensor flat and still
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

inline bool IMU::initMPU() {
    if (!writeRegister(REG_PWR_MGMT_1, 0x00)) return false;
    delay(100);

    // Verify WHO_AM_I
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

    if (!writeRegister(REG_ACCEL_CONFIG, 0x10)) return false; // +-8g
    if (!writeRegister(REG_GYRO_CONFIG, 0x10))  return false; // +-1000 deg/s
    if (!writeRegister(REG_DLPF_CONFIG, 0x03))  return false; // DLPF ~44 Hz

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

    if (valid == 0) {
        return false;
    }

    _axOff = (float)axSum / valid;
    _ayOff = (float)aySum / valid;
    _azOff = (float)azSum / valid - ACCEL_SCALE; // subtract 1g on Z
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
        while (Wire.available()) Wire.read(); // flush partial data
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

inline void IMU::bitBangRecover() {
    pinMode(SCL, OUTPUT);
    pinMode(SDA, INPUT_PULLUP);

    for (uint8_t i = 0; i < 9; i++) {
        digitalWrite(SCL, LOW);
        delayMicroseconds(5);
        digitalWrite(SCL, HIGH);
        delayMicroseconds(5);
        if (digitalRead(SDA) == HIGH) break; // SDA released
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

#endif // IMU_H
