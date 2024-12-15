#include "MPUManager.h"
#include "PIDController.h"
#include "MotorControl.h"
#include "Utils.h"

// IMU, PID Controller, and Motor instances
MPUManager imuManager;
PIDController pidController;
MotorControl motorControl;

unsigned long lastTime = 0;
unsigned long currentTime;
double elapsedTime;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting");

  imuManager.init();
  imuManager.calibrate();

  pidController.init(70, 0, 3, 100.0); // Initialize PID with Kp, Ki, Kd, and setpoint
  motorControl.init();
}

void loop() {
  if (imuManager.update()) {
    float gyroZ = imuManager.getGyroZ();
    float accX = imuManager.getAccX();

    currentTime = millis();
    elapsedTime = (currentTime - lastTime) / 1000.0;

    double angle = imuManager.calculateAngle(gyroZ, elapsedTime);
    double error = imuManager.calculateError(accX, angle);

    double output = pidController.compute(error, elapsedTime);
    motorControl.update(output);

    lastTime = currentTime;

    Serial.print(millis());
    Serial.print("\tError: ");
    Serial.print(error);
    Serial.print("\tOutput: ");
    Serial.println(output);
  }

  if (Serial.available()) {
    processSerialInput();
  }
}
