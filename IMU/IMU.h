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

#endif // IMU_H
