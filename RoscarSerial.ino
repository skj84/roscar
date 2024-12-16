#include <Servo.h>

// Pin assignments
const int IN1 = 9;    // Motor control pin (PWM)
const int IN2 = 10;   // Motor control pin (Direction)
const int servoPin = 7; // Servo control pin

Servo steeringServo; // Create a Servo object

int motorSpeed = 0;  // Initialize motor speed (0-255)
int servoAngle = 90; // Initialize servo angle (centered)

void setup() {
  // Set up motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Attach the servo motor
  steeringServo.attach(servoPin);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize motor and servo to default states
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  steeringServo.write(servoAngle);
}

void loop() {
  if (Serial.available() > 0) {
    // Read input from Python
    String input = Serial.readStringUntil('\n'); // Read until newline
    input.trim(); // Remove extra whitespace or newline

    // Parse the input (expecting "SPEED,ANGLE")
    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      // Extract motor speed and servo angle
      motorSpeed = input.substring(0, commaIndex).toInt();
      servoAngle = input.substring(commaIndex + 1).toInt();

      // Apply motor speed
      if (motorSpeed > 0) {
        analogWrite(IN1, motorSpeed); // Set speed
        digitalWrite(IN2, LOW);      // Direction forward
      } else if (motorSpeed < 0) {
        analogWrite(IN2, -motorSpeed); // Reverse speed
        digitalWrite(IN1, LOW);        // Direction reverse
      } else {
        digitalWrite(IN1, LOW); // Stop motor
        digitalWrite(IN2, LOW);
      }

      // Apply servo angle
      steeringServo.write(servoAngle);
    }
  }
}
