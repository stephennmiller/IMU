#include <Wire.h>

uint8_t mpuAddr = 0x68;

// MPU-6050 register addresses
const uint8_t REG_PWR_MGMT_1  = 0x6B;
const uint8_t REG_WHO_AM_I    = 0x75;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_DLPF_CONFIG  = 0x1A;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// +-8g (sensitivity 4096 LSB/g), +-1000 deg/s (sensitivity 32.8 LSB/(deg/s))
const float ACCEL_SCALE = 4096.0;
const float GYRO_SCALE  = 32.8;

int16_t rawAccX, rawAccY, rawAccZ;
int16_t rawGyroX, rawGyroY, rawGyroZ;
int16_t rawTemp;

// Calibration offsets
float axOffset = 0, ayOffset = 0, azOffset = 0;
float gxOffset = 0, gyOffset = 0, gzOffset = 0;

// Orientation angles
float roll  = 0.0;
float pitch = 0.0;
float yaw   = 0.0;

// Complementary filter weight (gyro vs accel)
const float ALPHA = 0.96;

// Timing
unsigned long prevTime = 0;

// Stale data detection
int16_t prevAccX, prevAccY, prevAccZ;
int16_t prevGyroX, prevGyroY, prevGyroZ;
uint8_t staleCount = 0;
const uint8_t STALE_THRESHOLD = 10;

// I2C error tracking
uint8_t i2cErrors = 0;
const uint8_t I2C_ERROR_THRESHOLD = 5;
bool imuOk = true;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  Serial.println(F("Starting IMU..."));

  // Recover I2C bus in case MPU is stuck from a previous session
  bitBangRecover();

  Wire.begin();
  Wire.setClock(100000);
#if defined(ARDUINO_ARCH_AVR)
  Wire.setWireTimeout(25000, true);  // 25ms timeout, reset bus on timeout
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
    while (1);
  }
  Serial.print(deviceCount);
  Serial.println(F(" device(s) found."));

  if (foundAddr == 0) {
    Serial.println(F("No MPU-6050 found (expected 0x68 or 0x69)."));
    while (1);
  }

  mpuAddr = foundAddr;
  if (mpuAddr != 0x68) {
    Serial.print(F("MPU-6050 at alternate address 0x"));
    Serial.println(mpuAddr, HEX);
  }

  // Initialize MPU-6050
  if (!initMPU()) {
    Serial.println(F("MPU-6050 init failed!"));
    while (1);
  }

  // Calibrate — keep sensor flat and still
  Serial.println(F("Calibrating... keep sensor FLAT and STILL."));
  if (!calibrate(2000)) {
    Serial.println(F("Calibration failed — no valid samples. Halting."));
    while (1);
  }
  Serial.println(F("Calibration complete."));

  Serial.println(F("MPU-6050 ready."));
  Serial.println(F("AccX(g)\tAccY(g)\tAccZ(g)\tGx(d/s)\tGy(d/s)\tGz(d/s)\tRoll\tPitch\tYaw\tTemp(C)"));

  prevTime = millis();
}

void loop() {
  if (!imuOk) {
    Serial.println(F("IMU offline — attempting recovery"));
    recoverI2C();
    delay(1000);
    return;
  }

  if (!readSensor()) {
    i2cErrors++;
    if (i2cErrors >= I2C_ERROR_THRESHOLD) {
      Serial.println(F("I2C errors — recovering bus"));
      recoverI2C();
      i2cErrors = 0;
    }
    return;
  }
  i2cErrors = 0;

  // Stale data detection
  if (rawAccX == prevAccX && rawAccY == prevAccY && rawAccZ == prevAccZ &&
      rawGyroX == prevGyroX && rawGyroY == prevGyroY && rawGyroZ == prevGyroZ) {
    staleCount++;
  } else {
    staleCount = 0;
  }
  prevAccX = rawAccX; prevAccY = rawAccY; prevAccZ = rawAccZ;
  prevGyroX = rawGyroX; prevGyroY = rawGyroY; prevGyroZ = rawGyroZ;

  if (staleCount >= STALE_THRESHOLD) {
    Serial.println(F("Stale data — recovering bus"));
    recoverI2C();
    staleCount = 0;
    return;
  }

  // Convert to physical units with calibration offsets
  float accX = (rawAccX - axOffset) / ACCEL_SCALE;
  float accY = (rawAccY - ayOffset) / ACCEL_SCALE;
  float accZ = (rawAccZ - azOffset) / ACCEL_SCALE;

  float gyroX = (rawGyroX - gxOffset) / GYRO_SCALE;
  float gyroY = (rawGyroY - gyOffset) / GYRO_SCALE;
  float gyroZ = (rawGyroZ - gzOffset) / GYRO_SCALE;

  // Temperature formula from datasheet
  float tempC = (rawTemp / 340.0) + 36.53;

  // Time delta
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;
  if (dt > 0.5) dt = 0.0; // reject unreasonable gaps

  // Accelerometer-based angles
  float accelRoll  = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / PI;
  float accelPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;

  // Complementary filter: trust gyro short-term, accel long-term
  roll  = ALPHA * (roll  + gyroX * dt) + (1.0 - ALPHA) * accelRoll;
  pitch = ALPHA * (pitch + gyroY * dt) + (1.0 - ALPHA) * accelPitch;
  yaw  += gyroZ * dt; // no accel correction for yaw (needs magnetometer)

  Serial.print(accX, 2);  Serial.print('\t');
  Serial.print(accY, 2);  Serial.print('\t');
  Serial.print(accZ, 2);  Serial.print('\t');
  Serial.print(gyroX, 2); Serial.print('\t');
  Serial.print(gyroY, 2); Serial.print('\t');
  Serial.print(gyroZ, 2); Serial.print('\t');
  Serial.print(roll, 1);  Serial.print('\t');
  Serial.print(pitch, 1); Serial.print('\t');
  Serial.print(yaw, 1);   Serial.print('\t');
  Serial.println(tempC, 1);

  delay(100);
}

// Initialize MPU-6050: wake, verify WHO_AM_I, configure ranges
bool initMPU() {
  if (!writeRegister(REG_PWR_MGMT_1, 0x00)) return false;
  delay(100);

  // Verify WHO_AM_I
  Wire.beginTransmission(mpuAddr);
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(mpuAddr, (uint8_t)1) != 1) return false;

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

// Calibrate by averaging samples at rest. Assumes sensor is flat (Z = +1g).
bool calibrate(int samples) {
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;
  int valid = 0;

  for (int i = 0; i < samples; i++) {
    if (readSensor()) {
      axSum += rawAccX; aySum += rawAccY; azSum += rawAccZ;
      gxSum += rawGyroX; gySum += rawGyroY; gzSum += rawGyroZ;
      valid++;
    }
    delay(1);
  }

  if (valid == 0) {
    return false;
  }

  axOffset = (float)axSum / valid;
  ayOffset = (float)aySum / valid;
  azOffset = (float)azSum / valid - ACCEL_SCALE; // subtract 1g on Z
  gxOffset = (float)gxSum / valid;
  gyOffset = (float)gySum / valid;
  gzOffset = (float)gzSum / valid;

  Serial.print(F("Calibrated with "));
  Serial.print(valid);
  Serial.println(F(" samples."));

  return true;
}

// Read all 6 axes + temperature, returns false on I2C error
bool readSensor() {
  Wire.beginTransmission(mpuAddr);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t received = Wire.requestFrom(mpuAddr, (uint8_t)14);
  if (received != 14) {
    while (Wire.available()) Wire.read(); // flush partial data
    return false;
  }

  rawAccX  = (Wire.read() << 8) | Wire.read();
  rawAccY  = (Wire.read() << 8) | Wire.read();
  rawAccZ  = (Wire.read() << 8) | Wire.read();
  rawTemp  = (Wire.read() << 8) | Wire.read();
  rawGyroX = (Wire.read() << 8) | Wire.read();
  rawGyroY = (Wire.read() << 8) | Wire.read();
  rawGyroZ = (Wire.read() << 8) | Wire.read();

  return true;
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

// Bit-bang SCL 9 times to release a slave holding SDA low,
// then generate a STOP condition. Per I2C spec.
void bitBangRecover() {
  pinMode(A5, OUTPUT);
  pinMode(A4, INPUT_PULLUP);

  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(A5, LOW);
    delayMicroseconds(5);
    digitalWrite(A5, HIGH);
    delayMicroseconds(5);
    if (digitalRead(A4) == HIGH) break; // SDA released
  }

  // Generate STOP: SDA low->high while SCL high
  pinMode(A4, OUTPUT);
  digitalWrite(A4, LOW);
  delayMicroseconds(5);
  digitalWrite(A5, HIGH);
  delayMicroseconds(5);
  digitalWrite(A4, HIGH);
  delayMicroseconds(5);

  // Return pins to default before Wire.begin() reconfigures them
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
}

// Full I2C recovery: bit-bang, reinit Wire, reinit MPU
void recoverI2C() {
  bitBangRecover();
  Wire.begin();
  Wire.setClock(100000);
#if defined(ARDUINO_ARCH_AVR)
  Wire.setWireTimeout(25000, true);  // 25ms timeout, reset bus on timeout
#endif
  delay(50);

  imuOk = initMPU();
  if (imuOk) {
    staleCount = 0;
  } else {
    Serial.println(F("Recovery failed — initMPU returned error"));
  }
}
