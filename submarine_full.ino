#include <Servo.h>

const int servoPin1 = 62;
const int servoPin2 = 63;
const int servoPin3 = 64;

Servo servo1;
Servo servo2;
Servo servo3;

const int solenoidPin1 = 5;
const int solenoidPin2 = 7;
const int solenoidPin3 = 9;
const int solenoidPin4 = 11;

const int pumpPin1 = 4;
const int pumpPin2 = 6;
const int pumpPin3 = 8;
const int pumpPin4 = 10;

const int propeller = 3;

int option;
int time_op=10;
int angle;

void setup() {
  Serial.begin(9600);
  pinMode(solenoidPin1, OUTPUT);
  pinMode(solenoidPin2, OUTPUT);
  pinMode(solenoidPin3, OUTPUT);
  pinMode(solenoidPin4, OUTPUT);

  pinMode(pumpPin1, OUTPUT);
  pinMode(pumpPin2, OUTPUT);
  pinMode(pumpPin3, OUTPUT);
  pinMode(pumpPin4, OUTPUT);

  pinMode(propeller, OUTPUT);

  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);

  digitalWrite(solenoidPin1, HIGH);
  digitalWrite(solenoidPin2, HIGH);
  digitalWrite(solenoidPin3, HIGH);
  digitalWrite(solenoidPin4, HIGH);

  digitalWrite(pumpPin1, HIGH);
  digitalWrite(pumpPin2, HIGH);
  digitalWrite(pumpPin3, HIGH);
  digitalWrite(pumpPin4, HIGH);
}

void loop() {
  if (Serial.available() > 0) {
    // Read option and time_op from Python
    option = Serial.parseInt();

    // Clear the rest of the buffer
    while (Serial.available() > 0) {
      Serial.read();
    }

    // Perform actions based on the received option
    switch (option) {
      case 1:
        // Down Action
        digitalWrite(solenoidPin1, LOW);
        digitalWrite(solenoidPin2, LOW);
        digitalWrite(solenoidPin3, LOW);
        digitalWrite(solenoidPin4, LOW);
        delay(time_op*500);
        digitalWrite(solenoidPin1, HIGH);
        digitalWrite(solenoidPin2, HIGH);
        digitalWrite(solenoidPin3, HIGH);
        digitalWrite(solenoidPin4, HIGH);
        break;
      case 2:
        // Up action
        digitalWrite(pumpPin1, LOW);
        digitalWrite(pumpPin2, LOW);
        digitalWrite(pumpPin3, LOW);
        digitalWrite(pumpPin4, LOW);
        delay(time_op*5000);
        digitalWrite(pumpPin1, HIGH);
        digitalWrite(pumpPin2, HIGH);
        digitalWrite(pumpPin3, HIGH);
        digitalWrite(pumpPin4, HIGH);
        break;
      case 3:
        // Pitch Up
        while (Serial.available() == 0) {
          // Wait for user input
        }
        Serial.println("Enter angle : ");
        angle = Serial.parseInt();
        while (Serial.available() > 0) {
          Serial.read();  // Clear any unwanted characters
        }
        digitalWrite(pumpPin1, LOW);
        digitalWrite(pumpPin2, LOW);
        digitalWrite(solenoidPin3, LOW);
        digitalWrite(solenoidPin4, LOW);
        servo1.write(angle);
        servo2.write(angle);
        delay(time_op*500);
        servo1.write(90);
        servo2.write(90);
        digitalWrite(pumpPin1, HIGH);
        digitalWrite(pumpPin2, HIGH);
        digitalWrite(solenoidPin3, HIGH);
        digitalWrite(solenoidPin4, HIGH);
        break;
      case 4:
        // Pitch Down
        while (Serial.available() == 0) {
          // Wait for user input
        }
        Serial.println("Enter angle : ");
        angle = Serial.parseInt();
        while (Serial.available() > 0) {
          Serial.read();  // Clear any unwanted characters
        }
        digitalWrite(pumpPin3, LOW);
        digitalWrite(pumpPin4, LOW);
        digitalWrite(solenoidPin1, LOW);
        digitalWrite(solenoidPin2, LOW);
        servo1.write(180 - angle);
        servo2.write(180 - angle);
        delay(time_op*500);
        digitalWrite(pumpPin3, HIGH);
        digitalWrite(pumpPin4, HIGH);
        digitalWrite(solenoidPin1, HIGH);
        digitalWrite(solenoidPin2, HIGH);
        servo1.write(90);
        servo2.write(90);
        break;
      case 5:
        // Yaw Left
        Serial.println("Enter angle : ");
        while (Serial.available() == 0) {
          // Wait for user input
        }
        angle = Serial.parseInt();
        while (Serial.available() > 0) {
          Serial.read();  // Clear any unwanted characters
        }
        servo3.write(angle);
        digitalWrite(propeller, LOW);
        delay(time_op*500);
        digitalWrite(propeller, HIGH);
        servo3.write(90);
        break;
      case 6:
        // Yaw Right
        Serial.println("Enter angle : ");
        while (Serial.available() == 0) {
          // Wait for user input
        }
        angle = Serial.parseInt();
        while (Serial.available() > 0) {
          Serial.read();  // Clear any unwanted characters
        }
        servo3.write(angle);
        digitalWrite(propeller, LOW);
        delay(time_op);
        servo3.write(90);
        digitalWrite(propeller, HIGH);
        break;
      case 7:
        // Roll Anticlockwise
        digitalWrite(pumpPin1, LOW);
        digitalWrite(pumpPin2, LOW);
        delay(time_op*500);
        digitalWrite(pumpPin1, HIGH);
        digitalWrite(pumpPin2, HIGH);
        break;
      case 8:
        // Roll Clockwise
        digitalWrite(pumpPin3, LOW);
        digitalWrite(pumpPin4, LOW);
        delay(time_op*500);
        digitalWrite(pumpPin3, HIGH);
        digitalWrite(pumpPin4, HIGH);
        break;
      case 9:
        // Move Forward
        digitalWrite(propeller, HIGH);
        delay(time_op);
        digitalWrite(propeller, HIGH);
        break;
      default:
        // Handle unknown option
        break;
    }

    // Send a response back to Python
    Serial.println("Action completed");
  }
}
