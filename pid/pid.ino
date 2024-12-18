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
double Kp = 60;  // Proportional gain
double Ki = 0.0; //0.0001;//.000001;//0.00005;  // Integral gain
double Kd = 4; //100; //250;//.01;//0010;  // Derivative gain
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
  Serial.begin(9600);

  // Serial.println("Starting");
  // IMU
  Wire.begin();

  if (!mpu.setup(0x68)) {
    // // // Serial.println("MPU9250 connection failed!");
    while (1);
  }
  
  // // // Serial.println("Calibrating gyroscope...");
  calibrateGyro();
  filter.begin(16);



  // PID Controller
  // // // Serial.println("PID Controller Initialized");
  // // // Serial.println("Commands:");
  // // // Serial.println("SET Kp value - e.g., Kp 2.5");
  // // // Serial.println("SET Ki value - e.g., Ki 0.7");
  // // // Serial.println("SET Kd value - e.g., Kd 1.2");



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
    // float roll = filter.getRoll();
    // float yaw = filter.getYaw();

     // // Serial.print("Calibrated Gyro X: "); // // Serial.print(gyroX);
     // // Serial.print("\tY: "); // // Serial.print(gyroY);
     // // Serial.print("\tZ: "); // // // Serial.println(gyroZ);
    // // // Serial.print("X: ");
    // // // Serial.print(mpu.getAccX());
    // // // Serial.print("\t"); // Add a tab for better alignment

    // // // Serial.print("Y: ");
    // // // Serial.print(mpu.getAccY());
    // // // Serial.print("\t"); // Add a tab for better alignment

    // // // Serial.print("Z: ");
    // // // // Serial.println(mpu.getAccZ()); // Use println to move to the next line

    //delay(10);
  }





  // Check for serial input
  if (Serial.available()) {
    processSerialInput();
  }

  // Get the current time
  currentTime = millis();
  elapsedTime = (currentTime - lastTime) / 1000.0;  // Convert to seconds

  // Read the sensor value (example: normalized between 0 and 255)
  angle -= (pitch) * elapsedTime;
  double error = pitch; //((1-alpha) * mpu.getAccX() * 70 + (alpha) * angle);
  // double error = pitch;
  // input = map(input, 0, 1023, 0, 255);  // Scale to match output range

  // Calculate the error
  // double error = setpoint - input;

  // Proportional term
  double proportional = Kp * error;

  // Integral term
  integral += error * elapsedTime;
  double integralTerm = Ki * integral;

  // update angle

  // Derivative term
  double derivative = (error - prevError) / elapsedTime;
  double derivativeTerm = Kd * derivative;

  lastTime = currentTime;
  
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(pitch);
  //Serial.print("\t");
  //Serial.print(angle);
  //Serial.print("\t");
  //Serial.print(mpu.getAccX() * 70);
  Serial.print("\t");
  Serial.print(derivativeTerm);
  Serial.print("\t");
  Serial.print(proportional);
  // Serial.print("\t");
  // Serial.print(integral);
  //Serial.print("\t");
  //Serial.print(error - prevError);
  //Serial.print("\t");
  //Serial.print(elapsedTime);
  
  output = proportional + integralTerm + derivativeTerm;
  output = constrain(output, -254 , 254);
  // output = constrain(output, -154 , 154);

  Serial.print("\t");
  // Serial.println(output);
  
  prevError = error;

  if (derivativeTerm != 0) {
    //// Serial.println(derivativeTerm);
  }

  // Calculate the output
  //output = proportional + integralTerm - derivativeTerm;


  int interval = 0;
  if (millis() - lastPrint >= interval and debug) {
    
    Serial.print("elapsed: ");
    Serial.print(elapsedTime);
    Serial.print("\tchange: ");
    Serial.print(angle - prevError);

    Serial.print("kp: ");
    Serial.print(proportional);
    Serial.print("\tkd: ");
    Serial.print(derivative);
    Serial.print("\t");
  }
  // Constrain the output to the valid range (e.g., 0-255 for PWM)

  // // Serial.print("output: ");
  // // // Serial.println(output);
  // // // Serial.println(abs(output));

  if (output < 0) {
    //// Serial.println(direction);
    // if (direction != -1) {
    //   analogWrite(pwmPin, 0);           // Stop motor
    //   direction = -1;
    // }
    // Spin the motor in the opposite direction at max speed
    digitalWrite(directionPin, HIGH);  // Set direction to reverse
    // // // Serial.println("Motor spinning in reverse at max speed");
    analogWrite(pwmPin, 255 - int(abs(output)));         // Max speed (100% duty cycle)
  }
  else {
    // if (direction != 1) {
    //   analogWrite(pwmPin, 0);           // Stop motor
    //   direction = 1;
    // }
    digitalWrite(directionPin, LOW); // Set direction to forward
    // // // Serial.println("Motor spinning forward at max speed");
    analogWrite(pwmPin, 255 - int(abs(output) ));         // Max speed (100% duty cycle)
  }

  // Debugging output
  // // Serial.print("Input: ");
  // // Serial.print(input);p
  if (millis() - lastPrint >= interval and debug) {
    Serial.print(mpu.getGyroY() - gyroYOffset);
    Serial.print("\tOutput: ");
    Serial.print(output);
    Serial.print("\tError: ");
    Serial.print(error);
    Serial.print("\tAngle: ");
    Serial.print(angle);
    // Serial.println();


    lastPrint = millis();
  }

  // // Serial.print("\tKp: ");
  // // Serial.print(Kp);
  // // Serial.print("\tKi: ");
  // // Serial.print(Ki);
  // // Serial.print("\tKd: ");
  // // // Serial.println(Kd);

  // Update previous values
  lastTime = currentTime;

  // Small delay for stability (adjust as needed)
  // delay(10);
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
    if (i % 100 == 0) {
      // Serial.println(sumZ / i);
    }
    mpu.update();
    sumX += mpu.getGyroX();
    sumY += mpu.getGyroY();
    sumZ += mpu.getGyroZ();
    //delay(10); // Small delay between samples
  }

  // gyroXOffset = sumX / numSamples;
  // gyroYOffset = sumY / numSamples;
  gyroYOffset = sumY / numSamples;
  Serial.print("GyroOffset: ");
  // Serial.println(gyroYOffset);
  // // Serial.print(gyroXOffset);
  // // Serial.print(" ");
  // // Serial.print(gyroYOffset);
  // // Serial.print(" ");
  // // Serial.print(gyroZOffset);
  // // Serial.print(" ");
  // // // Serial.println("Gyroscope calibrated!");
  
}