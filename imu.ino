#include <MPU9250.h>


MPU9250 mpu;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
float angle = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!mpu.setup(0x68)) {
    Serial.println("MPU9250 connection failed!");
    while (1);
  }

  Serial.println("Calibrating gyroscope...");
  calibrateGyro();
}

void loop() {
  if (mpu.update()) {
    // Apply calibration offset to gyroscope readings
    float gyroX = mpu.getGyroX() - gyroXOffset;
    float gyroY = mpu.getGyroY() - gyroYOffset;
    float gyroZ = mpu.getGyroZ() - gyroZOffset;

    //  Serial.print("Calibrated Gyro X: "); Serial.print(gyroX);
    //  Serial.print("\tY: "); Serial.print(gyroY);
     Serial.print("\tZ: "); Serial.println(gyroZ);
    // Serial.print("X: ");
    // Serial.print(mpu.getAccX());
    // Serial.print("\t"); // Add a tab for better alignment

    // Serial.print("Y: ");
    // Serial.print(mpu.getAccY());
    // Serial.print("\t"); // Add a tab for better alignment

    // Serial.print("Z: ");
    // Serial.println(mpu.getAccZ()); // Use println to move to the next line

    delay(1000);
  }
}

// Function to calibrate gyroscope
void calibrateGyro() {
  const int numSamples = 100;
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < numSamples; i++) {
    mpu.update();
    sumX += mpu.getGyroX();
    sumY += mpu.getGyroY();
    sumZ += mpu.getGyroZ();
    delay(10); // Small delay between samples
  }

  gyroXOffset = sumX / numSamples;
  gyroYOffset = sumY / numSamples;
  gyroZOffset = sumZ / numSamples;
  Serial.print(gyroXOffset);
  Serial.print(" ");
  Serial.print(gyroYOffset);
  Serial.print(" ");
  Serial.print(gyroZOffset);
  Serial.print(" ");
  Serial.println("Gyroscope calibrated!");
}