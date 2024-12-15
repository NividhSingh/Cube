#include "MPUManager.h"

void MPUManager::init() {
  Wire.begin();
  if (!mpu.setup(0x68)) {
    while (1); // Halt if connection fails
  }
}

void MPUManager::calibrate() {
  const int numSamples = 2000;
  float sumZ = 0;
  for (int i = 0; i < numSamples; i++) {
    mpu.update();
    sumZ += mpu.getGyroZ();
  }
  gyroZOffset = sumZ / numSamples;
}

bool MPUManager::update() {
  return mpu.update();
}

float MPUManager::getGyroZ() {
  return mpu.getGyroZ() - gyroZOffset;
}

float MPUManager::getAccX() {
  return mpu.getAccX();
}

double MPUManager::calculateAngle(float gyroZ, double elapsedTime) {
  angle -= gyroZ * elapsedTime;
  return angle;
}

double MPUManager::calculateError(float accX, double angle) {
  return ((1 - alpha) * accX * 70) + (alpha * angle);
}
