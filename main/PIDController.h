#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
  void init(double Kp, double Ki, double Kd, double setpoint);
  double compute(double error, double elapsedTime);

private:
  double Kp, Ki, Kd;
  double setpoint;
  double prevError = 0.0;
  double integral = 0.0;
};

#endif
