#include <Arduino.h>

#include "MPU9250.h"

MPU9250 mpu;

// forward declarations
void print_calibration(void);
void print_roll_pitch_yaw(void);
void check_connections(void);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  while (!mpu.setup(0x68)) {  // change to your own address
    Serial.println("MPU connection failed. Checking connections:");
    check_connections();
  }

  int incomingByte = 0; // for incoming serial data
  while ('s' != incomingByte) {
    delay (1000);
    Serial.println("Press 's' to start calibration.");
    if (Serial.available() > 0) {
      // read the incoming byte:
      do {
        incomingByte = Serial.read();
        if (incomingByte != -1) {
          Serial.print("Incoming Byte: ");
          Serial.println(incomingByte, DEC);
        }
      } while (incomingByte != -1 && incomingByte != 's');

      if ('s' == incomingByte) {
        Serial.println("'s' detected, starting calibration.");

        // calibrate anytime you want to
        Serial.println("Accel Gyro calibration will start in 3sec.");
        Serial.println("Please leave the device still on the flat plane.");
        mpu.verbose(true);
        delay(3000);
        mpu.calibrateAccelGyro();

        Serial.println("Mag calibration will start in 3sec.");
        Serial.println("Please Wave device in a figure eight until done.");
        delay(3000);
        mpu.calibrateMag();

        print_calibration();
        mpu.verbose(false);
      } else {
        Serial.println("'s' not detected.");
      }
    }
  }
} 

void loop() {
  delay(100);
  mpu.calibrateAccelGyro();
  print_accel_calibration();
}

void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}

uint8_t addrs[7] = {0};
uint8_t device_count = 0;

template <typename WireType = TwoWire>
uint8_t readByte(uint8_t address, uint8_t subAddress, WireType& wire = Wire) {
    uint8_t data = 0;
    wire.beginTransmission(address);
    wire.write(subAddress);
    wire.endTransmission(false);
    wire.requestFrom(address, (size_t)1);
    if (wire.available()) data = wire.read();
    return data;
}

void print_accel_calibration() {
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    print_accel_calibration();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

template <typename WireType = TwoWire>
void scan_mpu(WireType& wire = Wire) {
    Serial.println("Searching for i2c devices...");
    device_count = 0;
    for (uint8_t i = 0x68; i < 0x70; ++i) {
        wire.beginTransmission(i);
        if (wire.endTransmission() == 0) {
            addrs[device_count++] = i;
            delay(10);
        }
    }
    Serial.print("Found ");
    Serial.print(device_count, DEC);
    Serial.println(" I2C devices");

    Serial.print("I2C addresses are: ");
    for (uint8_t i = 0; i < device_count; ++i) {
        Serial.print("0x");
        Serial.print(addrs[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void check_connections (void)
{
  delay(1000);
  scan_mpu();

  if (device_count == 0) {
      Serial.println("No device found on I2C bus. Please check your hardware connection");
  }

  // check WHO_AM_I address of MPU
  for (uint8_t i = 0; i < device_count; ++i) {
      Serial.print("I2C address 0x");
      Serial.print(addrs[i], HEX);
      byte ca = readByte(addrs[i], WHO_AM_I_MPU9250);
      if (ca == MPU9250_WHOAMI_DEFAULT_VALUE) {
          Serial.println(" is MPU9250 and ready to use");
      } else if (ca == MPU9255_WHOAMI_DEFAULT_VALUE) {
          Serial.println(" is MPU9255 and ready to use");
      } else if (ca == MPU6500_WHOAMI_DEFAULT_VALUE) {
          Serial.println(" is MPU6500 and ready to use");
      } else {
          Serial.println(" is not MPU series");
          Serial.print("WHO_AM_I is ");
          Serial.println(ca, HEX);
          Serial.println("Please use correct device");
      }
      static constexpr uint8_t AK8963_ADDRESS {0x0C};  //  Address of magnetometer
      static constexpr uint8_t AK8963_WHOAMI_DEFAULT_VALUE {0x48};
      byte cb = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);
      if (cb == AK8963_WHOAMI_DEFAULT_VALUE) {
          Serial.print("AK8963 (Magnetometer) is ready to use");
      } else {
          Serial.print("AK8963 (Magnetometer) was not found");
      }
  }
}
