# MPU-6050 IMU Reader

Arduino sketch that reads accelerometer, gyroscope, and temperature data from an MPU-6050 over I2C, computes roll/pitch/yaw orientation using a complementary filter, and streams the results over serial.

## Hardware

- Arduino Uno (or compatible AVR board)
- MPU-6050 breakout module

### Wiring

| MPU-6050 | Arduino Uno |
|----------|-------------|
| VCC      | 5V          |
| GND      | GND         |
| SDA      | A4          |
| SCL      | A5          |

The sketch auto-detects the MPU-6050 at address `0x68` or `0x69`.

## Features

- **I2C bus scan** — scans addresses 1-126 at startup, reports all detected devices, and auto-selects the MPU-6050 at `0x68` or `0x69`
- **WHO_AM_I verification** — confirms the device identity register reads `0x68` before proceeding
- **Startup calibration** — averages 2000 samples at rest to compute per-axis offsets; subtracts 1g from Z so gravity reads correctly
- **Complementary filter** — fuses gyro (short-term) and accelerometer (long-term) data with a 96/4 weighting for stable roll and pitch
- **Gyro-only yaw** — integrates Z-axis gyro for yaw (drifts over time without a magnetometer)
- **Temperature reading** — converts the on-chip temperature sensor to Celsius using the datasheet formula
- **I2C error tracking** — counts consecutive read failures; triggers bus recovery after 5 errors
- **Stale data detection** — flags when all 6 axes return identical values for 10 consecutive reads and triggers recovery
- **Bit-bang SCL recovery** — clocks SCL 9 times to release a slave holding SDA low, then generates a STOP condition per the I2C spec
- **Full bus recovery** — combines bit-bang recovery with Wire reinit and MPU reinitialization to restore communication
- **AVR Wire timeout** — sets a 25ms I2C timeout with automatic bus reset on AVR platforms to prevent hangs
- **Large dt rejection** — discards time deltas over 500ms to avoid orientation jumps after delays or recovery

## Sensor Configuration

| Parameter      | Setting        | Sensitivity        |
|----------------|----------------|--------------------|
| Accelerometer  | +/-8g          | 4096 LSB/g         |
| Gyroscope      | +/-1000 deg/s  | 32.8 LSB/(deg/s)   |
| DLPF           | ~44 Hz         |                    |

## Serial Output

115200 baud, tab-separated columns:

```text
AccX(g)  AccY(g)  AccZ(g)  Gx(d/s)  Gy(d/s)  Gz(d/s)  Roll  Pitch  Yaw  Temp(C)
```

Output rate is approximately 10 Hz (100ms loop delay).

## Build

Requires the [Arduino CLI](https://arduino.github.io/arduino-cli/) or Arduino IDE.

```bash
arduino-cli compile --fqbn arduino:avr:uno IMU/IMU.ino
arduino-cli upload --fqbn arduino:avr:uno --port /dev/ttyACM0 IMU/IMU.ino
arduino-cli monitor --port /dev/ttyACM0 --config baudrate=115200
```

## Calibration

On startup the sensor must be **flat and still** while calibration runs. The sketch averages 2000 readings to compute zero-offsets for all axes, subtracting 1g from the Z accelerometer so gravity reads as 1g at rest.
