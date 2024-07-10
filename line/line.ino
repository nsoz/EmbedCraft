#include <QTRSensors.h>
#include <AFMotor.h>

#define Kp 0.2 // Adjusted Proportional constant
#define Kd 1   // Adjusted Derivative constant
#define rightMaxSpeed 255
#define leftMaxSpeed 255
#define rightBaseSpeed 150 // Increased base speed
#define leftBaseSpeed 150  // Increased base speed

QTRSensors qtr;
const uint8_t SensorCount = 6; // Changed sensor count to 6
unsigned int sensorValues[SensorCount];

AF_DCMotor rightMotor(1); // Right motor connected to M1
AF_DCMotor leftMotor(2);  // Left motor connected to M2

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount); // Updated to 6 sensors

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (int i = 0; i < 100; i++) {
    if (i < 25 || i >= 75) {
      turn_right();
    } else {
      turn_left();
    }
    qtr.calibrate();
    delay(20); // Consider increasing this delay if calibration is not effective
  }
  wait();
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

int lastError = 0;

void loop() {
  unsigned int position = qtr.readLineBlack(sensorValues);

  // Handle specific sensor cases
  if (sensorValues[0] > 500 && sensorValues[1] <= 500 && sensorValues[2] <= 500 && sensorValues[3] <= 500 && sensorValues[4] <= 500 && sensorValues[5] <= 500) { // A0 sees white, others see black
    leftMotor.setSpeed(leftBaseSpeed);
    leftMotor.run(FORWARD);
    rightMotor.run(RELEASE); // Stop right motor
  } else if (sensorValues[1] > 500 && sensorValues[0] <= 500 && sensorValues[2] <= 500 && sensorValues[3] <= 500 && sensorValues[4] <= 500 && sensorValues[5] <= 500) { // A1 sees white, others see black
    leftMotor.setSpeed(leftBaseSpeed);
    leftMotor.run(FORWARD);
    rightMotor.setSpeed(rightBaseSpeed / 2);
    rightMotor.run(FORWARD);
  } else if ((sensorValues[2] > 500 || sensorValues[3] > 500) && sensorValues[0] <= 500 && sensorValues[1] <= 500 && sensorValues[4] <= 500 && sensorValues[5] <= 500) { // A2 or A3 sees white, others see black
    leftMotor.setSpeed(leftBaseSpeed);
    leftMotor.run(FORWARD);
    rightMotor.setSpeed(rightBaseSpeed);
    rightMotor.run(FORWARD);
  } else if (sensorValues[4] > 500 && sensorValues[0] <= 500 && sensorValues[1] <= 500 && sensorValues[2] <= 500 && sensorValues[3] <= 500 && sensorValues[5] <= 500) { // A4 sees white, others see black
    leftMotor.setSpeed(leftBaseSpeed / 2);
    leftMotor.run(FORWARD);
    rightMotor.setSpeed(rightBaseSpeed);
    rightMotor.run(FORWARD);
  } else if (sensorValues[5] > 500 && sensorValues[0] <= 500 && sensorValues[1] <= 500 && sensorValues[2] <= 500 && sensorValues[3] <= 500 && sensorValues[4] <= 500) { // A5 sees white, others see black
    rightMotor.setSpeed(rightBaseSpeed);
    rightMotor.run(FORWARD);
    leftMotor.run(RELEASE); // Stop left motor
  } else {
    // Normal line following behavior
    int error = position - 2500; // Adjusted this value based on the center position for 6 sensors
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;

    // Clamp motor speeds to valid range
    rightMotorSpeed = constrain(rightMotorSpeed, 0, rightMaxSpeed);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, leftMaxSpeed);

    rightMotor.setSpeed(rightMotorSpeed);
    rightMotor.run(FORWARD);
    leftMotor.setSpeed(leftMotorSpeed);
    leftMotor.run(FORWARD);
  }
}

void wait() {
  rightMotor.run(RELEASE);
  leftMotor.run(RELEASE);
}

void turn_left() {
  rightMotor.setSpeed(rightBaseSpeed);
  rightMotor.run(FORWARD);
  leftMotor.setSpeed(leftBaseSpeed);
  leftMotor.run(BACKWARD);
}

void turn_right() {
  rightMotor.setSpeed(rightBaseSpeed);
  rightMotor.run(BACKWARD);
  leftMotor.setSpeed(leftBaseSpeed);
  leftMotor.run(FORWARD);
}
