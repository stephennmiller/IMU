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
