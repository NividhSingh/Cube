#include "PIDController.h"
#include <math.h>
#include <Arduino.h> // For delay and Arduino-specific functions

void PIDController::init(double Kp, double Ki, double Kd, double setpoint) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->setpoint = setpoint;
}

double PIDController::compute(double error, double elapsedTime) {
  double proportional = Kp * error;
  integral += error * elapsedTime;
  double integralTerm = Ki * integral;
  double derivative = (error - prevError) / elapsedTime;
  double derivativeTerm = Kd * derivative;
  prevError = error;
  return proportional + integralTerm + derivativeTerm;
}
