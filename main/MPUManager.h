#ifndef MPU_MANAGER_H
#define MPU_MANAGER_H

#include <MPU9250.h>

class MPUManager {
public:
  void init();
  void calibrate();
  bool update();
  float getGyroZ();
  float getAccX();
  double calculateAngle(float gyroZ, double elapsedTime);
  double calculateError(float accX, double angle);

private:
  MPU9250 mpu;
  float gyroZOffset = 0;
  float angle = -45;
  float alpha = 0.99;
};

#endif
