#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

class MotorControl {
public:
  void init();
  void update(double output);

private:
  const int pwmPin = 9;
  const int directionPin = 6;
  const int startstopPin = 12;
};

#endif
