#include <MPU9250.h>
#include <MadgwickAHRS.h>

// IMU
MPU9250 mpu;
Madgwick filter;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
float angle = 45; //-7.12; //-45;
float pitch = 0.0;
bool debug = false;

// 30

// Define PID parameters
double Kp = 140;  // Proportional gain
double Ki = 0.0008; //0.0001;//.000001;//0.00005;  // Integral gain
double Kd = 10.1; //100; //250;//.01;//0010;  // Derivative gain
int direction = 0;

float alpha = .99;

// Variables for PID calculations
double setpoint = 100.0;  // Desired value
double output = 0.0;      // Output signal to the actuator
double prevError = 0.0;   // Previous input value
double integral = 0.0;    // Integral accumulator

double lastPrint = millis();

// Timing variables
unsigned long lastTime = 0;
unsigned long currentTime;
double elapsedTime;

// Motor
const int pwmPin = 9;           // PWM pin for motor speed control
const int directionPin = 6;     // Direction pin for motor control
const int startstopPin = 12;    // Pin for motor enable/disable


void setup() {
  // Initialize serial communication for debugging and input
  //Serial.begin(9600);

  //Serial.println("Starting");
  // IMU
  Wire.begin();

  if (!mpu.setup(0x68)) {
    // // // Serial.println("MPU9250 connection failed!");
    while (1);
  }
  
  // // // Serial.println("Calibrating gyroscope...");
  calibrateGyro();
  filter.begin(100);



  // Motor
  pinMode(pwmPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pinMode(startstopPin, OUTPUT);

  // Enable the motor
  digitalWrite(startstopPin, HIGH);
  
  lastPrint = millis();
  // // // Serial.println("Motor enabled");
}

void loop() {
  // Get the current time
  currentTime = millis();
  mpu.update();
  // Check if 5 milliseconds have passed (1 / 200 Hz = 5 ms)
  if (currentTime - lastTime >= 10) {
    lastTime = currentTime;  // Update the last execution time

    if (mpu.update()) {
      // Apply calibration offset to gyroscope readings
      float gyroX = mpu.getGyroX() - gyroXOffset;
      float gyroY = mpu.getGyroY() - gyroYOffset;
      float gyroZ = mpu.getGyroZ() - gyroZOffset;

      float accX = mpu.getAccX();
      float accY = mpu.getAccY();
      float accZ = mpu.getAccZ();

      // Pass all gyroscope data
      filter.updateIMU(gyroX, gyroY, gyroZ, accX, accY, accZ);

      // Get orientation
      pitch = filter.getPitch();
    }

    // PID control logic
    double error = pitch;
    double proportional = Kp * error;

    integral += error * 0.005;  // Integrate over the interval (5 ms = 0.005 s)
    double integralTerm = Ki * integral;

    double derivative = (error - prevError) / 0.005;  // Derivative over 5 ms
    double derivativeTerm = Kd * derivative;

    output = proportional + integralTerm + derivativeTerm;
    output = constrain(output, -250, 250);

    prevError = error;

    // Control motor direction and speed
    if (output > 0) {
      digitalWrite(directionPin, HIGH);  // Reverse direction
      analogWrite(pwmPin, 255 - int(abs(output)));
    } else {
      digitalWrite(directionPin, LOW);  // Forward direction
      analogWrite(pwmPin, 255 - int(abs(output)));
    }

    // Optional debugging output
if (debug) {
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  Serial.print(derivativeTerm);
  Serial.print("\t");
  Serial.print(proportional);
  Serial.print("\t");
  Serial.print(integralTerm, 5);
  Serial.print("\t");
  Serial.print(output);

  // Ensure a newline at the end of the debug block
  Serial.println();
}
  }

  // Process serial input for tuning PID parameters
  if (Serial.available()) {
    processSerialInput();
  }
}

// Function to process serial input for tuning
void processSerialInput() {
  static String inputString = "";
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      parseCommand(inputString);
      inputString = "";  // Clear the string after processing
    } else {
      inputString += inChar;  // Append character to the input string
    }
  }
}

// Function to parse and execute commands
void parseCommand(String command) {
  if (command.startsWith("Kp ")) {
    Kp = command.substring(3).toFloat();
    // // Serial.print("Kp updated to: ");
    // // // Serial.println(Kp);
  } else if (command.startsWith("Ki ")) {
    Ki = command.substring(3).toFloat();
    // // Serial.print("Ki updated to: ");
    // // // Serial.println(Ki);
  } else if (command.startsWith("Kd ")) {
    Kd = command.substring(3).toFloat();
    // // Serial.print("Kd updated to: ");
    // // // Serial.println(Kd);
  } else {
    // Serial.println("Unknown command. Use Kp, Ki, or Kd followed by a value.");
  }
}

// Function to calibrate gyroscope
void calibrateGyro() {
  const int numSamples = 2000;
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < numSamples; i++) {
    // if (i % 100 == 0) {
    //   Serial.println(sumZ / i);
    // }
    mpu.update();
    sumX += mpu.getGyroX();
    sumY += mpu.getGyroY();
    sumZ += mpu.getGyroZ();
    //delay(10); // Small delay between samples
  }

  // gyroXOffset = sumX / numSamples;
  // gyroYOffset = sumY / numSamples;
  gyroYOffset = sumY / numSamples;
  //Serial.print("GyroOffset: ");
  //Serial.println(gyroYOffset);
  // // Serial.print(gyroXOffset);
  // // Serial.print(" ");
  // // Serial.print(gyroYOffset);
  // // Serial.print(" ");
  // // Serial.print(gyroZOffset);
  // // Serial.print(" ");
  // // // Serial.println("Gyroscope calibrated!");
  
}
