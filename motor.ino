const int pwmPin = 9;           // PWM pin for motor speed control
const int directionPin = 6;     // Direction pin for motor control
const int startstopPin = 12;    // Pin for motor enable/disable

void setup() {
    Serial.begin(9600);
    pinMode(pwmPin, OUTPUT);
    pinMode(directionPin, OUTPUT);
    pinMode(startstopPin, OUTPUT);

    // Enable the motor
    digitalWrite(startstopPin, HIGH);
    Serial.println("Motor enabled");
}

void loop() {

  int speed = 0;

  for (int i = 0; i < 255; i++) {
    digitalWrite(directionPin, HIGH);
    analogWrite(pwmPin, i);
    Serial.print("Speed: ");
    Serial.println(i);
    delay(500);
  }


  // // Spin the motor in one direction at max speed
  // digitalWrite(directionPin, HIGH); // Set direction to forward
  // Serial.println("Motor spinning forward at max speed");
  // analogWrite(pwmPin, 175);         // Max speed (100% duty cycle)
  // delay(5000);                      // Run for 5 seconds

  // // // Stop the motor briefly before switching directions
  // // Serial.println("Stopping motor briefly");
  // // analogWrite(pwmPin, 0);           // Stop motor
  // // delay(200);                       // Brief delay (0.2 seconds)

  // // Spin the motor in the opposite direction at max speed
  // digitalWrite(directionPin, LOW);  // Set direction to reverse
  // Serial.println("Motor spinning in reverse at max speed");
  // analogWrite(pwmPin, 175);         // Max speed (100% duty cycle)
  // delay(5000);                      // Run for 5 seconds

  // // Stop the motor briefly before switching directions
  // Serial.println("Stopping motor briefly");
  // analogWrite(pwmPin, 0);           // Stop motor
  // delay(200);                       // Brief delay (0.2 seconds)
}