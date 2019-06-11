#include <ros.h>
#include <dyret_hardware/ActuatorBoardCommand.h>
#include <dyret_hardware/ActuatorBoardState.h>
#include <std_msgs/String.h>

const int actuatorEnablePins[]    = {11, 10,  7,  6,  5,  4,  3,  2};
const int actuatorDirectionPins[] = {34, 36, 42, 44, 46, 48, 50, 52};
const int limitSwitchInputPins[]  = {25, 29, 33, 37, 41, 45, 49, 53};
const int limitSwitchOutputPins[] = {23, 27, 31, 35, 39, 43, 47, 51};
const int encoderInputPins[]      = { 2,  3,  6,  7,  8,  9, 10, 11};

const int pValue = 500;

unsigned long sendMessagePreviousms = 0;
const int sendMessageIntervalms = 100; // 10hz
unsigned long motorControlPreviousms = 0;
const int motorControlIntervalms = 10; // 100hz
unsigned long errorDetectionPreviousms = 0;
const int errorDetectionIntervalms = 2000; // 2hz

const int femurLengthLimit = 50;
const int tibiaLengthLimit = 95;

bool initiated[8];

bool commandReceived[8];

float actuatorGoal[8];

float lastPos[8]; // Last recorded position, used to detect stuck motors or faulty encoders
bool currentlyRunning[8];
bool motorError[8];
float currentPos[8]; // Current position in MM

int lastMeasurement[8];
int turnCounter[8];

unsigned motorPwm[8];

int zeroOffset[8]; // Contains the offset when zero

ros::NodeHandle nh;

dyret_hardware::ActuatorBoardState stateMessage;
ros::Publisher statePub("dyret/actuator_board/state", &stateMessage);

void messageCb(const dyret_hardware::ActuatorBoardCommand& msg);
ros::Subscriber<dyret_hardware::ActuatorBoardCommand> s("dyret/actuator_board/command", messageCb);

bool limitSwitchActive(int givenActuatorNumber) {
  if (digitalRead(limitSwitchInputPins[givenActuatorNumber]) == LOW) return false;
  return true;
}

void expandActuator(int givenActuatorNumber) {
  digitalWrite(actuatorDirectionPins[givenActuatorNumber], LOW); // Set direction
  digitalWrite(actuatorEnablePins[givenActuatorNumber], HIGH); // Set enable bit
}

void retractActuator(int givenActuatorNumber) {
  digitalWrite(actuatorDirectionPins[givenActuatorNumber], HIGH); // Set direction
  digitalWrite(actuatorEnablePins[givenActuatorNumber], HIGH); // Set enable bit
}

void disableMotor(int givenActuatorNumber) {
  motorPwm[givenActuatorNumber] = 0;
  digitalWrite(actuatorEnablePins[givenActuatorNumber], LOW); // Set enable bit
}

int readRawEncoder(int givenActuatorNumber) { // Returns int 0-1023
  return analogRead(encoderInputPins[givenActuatorNumber]);
}

void setMotorZeroPoint(int givenActuatorNumber) {
  initiated[givenActuatorNumber] = true;
  commandReceived[givenActuatorNumber] = false;
  actuatorGoal[givenActuatorNumber] = 0.0;
  zeroOffset[givenActuatorNumber] = readRawEncoder(givenActuatorNumber);
  turnCounter[givenActuatorNumber] = 0;
}

void enableMotor(int givenActuatorNumber, int givenMotorSpeed, bool motorDirectionRetract) { // Speed 0-255, direction inward
  //int motorSpeed = min(givenMotorSpeed, 180); // 255 = 14.8V, 180 ~= 12V
  int motorSpeed = givenMotorSpeed; // For 12V operation

  // Set direction
  if (motorDirectionRetract) {
    digitalWrite(actuatorDirectionPins[givenActuatorNumber], HIGH); // Set direction
  } else {
    digitalWrite(actuatorDirectionPins[givenActuatorNumber], LOW); // Set direction
  }

  // Set speed (motor enable)
  analogWrite(actuatorEnablePins[givenActuatorNumber], motorSpeed);

}

void readPositions() {

  int measurements[8];

  // Read from encoders
  for (int i = 0; i < 8; i++) {
    measurements[i] = readRawEncoder(i);
  }

  // Handle turncounter
  for (int i = 0; i < 8; i++) {
    // Overflowed:
    if ((measurements[i] < 256) && (lastMeasurement[i] > 768)) {
      turnCounter[i]++;
    }

    // Underflowed:
    if ((measurements[i] > 768) && (lastMeasurement[i] < 256)) {
      turnCounter[i]--;
    }

    lastMeasurement[i] = measurements[i];
  }

  // Convert to mm
  for (int i = 0; i < 8; i++) {
    currentPos[i] = -((measurements[i] + (turnCounter[i] * 1024.0)) - zeroOffset[i]) * ((16.0 / 7.0) / 1024.0);
  }
}

void setupStateMessage() {

  for (int i = 0; i < 8; i++) {
    stateMessage.position[i] = currentPos[i];
    stateMessage.raw[i] = lastMeasurement[i];
    stateMessage.pwm[i] = motorPwm[i];

    if (motorError[i] == false && initiated[i] == true && commandReceived[i] == true) {
      stateMessage.status[i] = stateMessage.STATUS_RUNNING;
    } else if (motorError[i] == false) {
      stateMessage.status[i] = stateMessage.STATUS_RESTING;
    } else {
      stateMessage.status[i] = stateMessage.STATUS_ERROR;
    }
  }
}

void homeActuators() {
  for (int i = 0; i < 8; i++) {
    initiated[i] = false;
    currentlyRunning[i] = false;
    motorError[i] = false;
  }

  for (int i = 0; i < 8; i++) {
    commandReceived[i] = true;
    actuatorGoal[i] = -100;
  }

  for (int i = 0; i < 8; i++) {
    lastMeasurement[i] = readRawEncoder(i);
  }
}

void messageCb(const dyret_hardware::ActuatorBoardCommand& msg) {

  if (msg.length_length == 1) { // Special case for just one distance without ids
    if (msg.length[0] < 0) { // Zero actuators if receiving negative number
      homeActuators();
    } else {
      for (int i = 0; i < 8; i = i + 2) {
        if (initiated[i] == true) actuatorGoal[i] = min(msg.length[0], femurLengthLimit);
        commandReceived[i] = true;
      }

      for (int i = 1; i < 8; i = i + 2) {
        if (initiated[i] == true) actuatorGoal[i] = min(msg.length[0], tibiaLengthLimit);
        commandReceived[i] = true;
      }
    }
  } else if (msg.length_length == 2) { // Special case for just two distances without ids
    for (int i = 0; i < 8; i = i + 2) {
      if (initiated[i] == true) actuatorGoal[i] = min(msg.length[0], femurLengthLimit);
      commandReceived[i] = true;
    }

    for (int i = 1; i < 8; i = i + 2) {
      if (initiated[i] == true) actuatorGoal[i] = min(msg.length[1], tibiaLengthLimit);
      commandReceived[i] = true;
    }
  } else if (msg.length_length == 8) {
    for (int i = 0; i < 8; i = i + 2) {
      if (initiated[i] == true) actuatorGoal[i] = min(msg.length[i], femurLengthLimit);
      commandReceived[i] = true;
    }

    for (int i = 1; i < 8; i = i + 2) {
      if (initiated[i] == true) actuatorGoal[i] = min(msg.length[i], tibiaLengthLimit);
      commandReceived[i] = true;
    }
  }

}

void setup() {
  for (int i = 0; i < 8; i++) {
    actuatorGoal[i] = 512.0;
    motorPwm[i] = 0;
    zeroOffset[i] = 0;
    turnCounter[i] = 0;
    commandReceived[i] = false;
    initiated[i] = false;
    currentlyRunning[i] = false;
    motorError[i] = false;
  }

  for (int i = 0; i < sizeof(actuatorEnablePins) / sizeof(int); i++) {
    pinMode(actuatorEnablePins[i], OUTPUT);  // Enable pins
    digitalWrite(actuatorEnablePins[i], LOW);
  }
  for (int i = 0; i < sizeof(actuatorDirectionPins) / sizeof(int); i++) {
    pinMode(actuatorDirectionPins[i], OUTPUT);  // Direction pins
  }
  for (int i = 0; i < sizeof(limitSwitchOutputPins) / sizeof(int); i++) {
    pinMode(limitSwitchOutputPins[i], OUTPUT);  // Set pinmode and output ground to limit switch outputs
    digitalWrite(limitSwitchOutputPins[i], LOW);
  }
  for (int i = 0; i < sizeof(limitSwitchInputPins) / sizeof(int); i++) {
    pinMode(limitSwitchInputPins[i], INPUT);  // Set pinmode and connect pullup to limit switch inputs
    digitalWrite(limitSwitchInputPins[i], HIGH);
  }

  nh.initNode();

  homeActuators();

  nh.subscribe(s);
  nh.advertise(statePub);

}

void loop() {

  // Read positions:
  readPositions();

  // Send messages:
  unsigned long currentMillis = millis();

  if (currentMillis - sendMessagePreviousms >= sendMessageIntervalms) {
    sendMessagePreviousms = currentMillis;

    setupStateMessage();

    statePub.publish(&stateMessage);

  }

  // Do motor loop:
  currentMillis = millis();

  if (currentMillis - motorControlPreviousms >= motorControlIntervalms) {
    motorControlPreviousms = currentMillis;
    for (int i = 0; i < 8; i++) {

      if (commandReceived[i] == true && motorError[i] == false) {

        int motorSpeed = min(abs(actuatorGoal[i] - currentPos[i]) * pValue, 255);
        motorPwm[i] = motorSpeed;

        if (abs(actuatorGoal[i] - currentPos[i]) < 0.25) { // 0.25mm accuracy
          disableMotor(i);
          commandReceived[i] = false;
        } else {
          if (currentPos[i] > actuatorGoal[i]) { // Retract
            if (limitSwitchActive(i) == false) {
              enableMotor(i, motorSpeed, true);
            } else {
              setMotorZeroPoint(i);
              motorPwm[i] = 0;
            }

          } else {
            enableMotor(i, motorSpeed, false);
          }

        }

      } else {
        disableMotor(i);
      }
    }
  }

  // Detect stuck motors:
  currentMillis = millis();

  if (currentMillis - errorDetectionPreviousms >= errorDetectionIntervalms) {
    errorDetectionPreviousms = currentMillis;

    for (int i = 0; i < 8; i++) {
      if (currentlyRunning[i] && motorPwm[i] == 255) { // It has been running continously for a period
        if (abs(lastPos[i] - currentPos[i]) < 0.1) { // It has moved less than 0.1mm, even though it is enabled
          disableMotor(i);
          motorError[i] = true;
        }
      }

      // Record status for next loop:
      if (motorPwm[i] == 255) {
        currentlyRunning[i] = true;
      } else {
        currentlyRunning[i] = false;
      }
      lastPos[i] = currentPos[i];
    }

  }

  // Spin ROS:
  nh.spinOnce();

}
