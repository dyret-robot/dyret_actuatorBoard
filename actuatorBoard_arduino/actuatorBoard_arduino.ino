#include <ros.h>
#include <dyret_common/Configuration.h>
#include <std_msgs/String.h>

// TODO: Make sure no command is received before legs are zeroed

const int actuatorEnablePins[]    = {11, 10,  7,  6,  5,  4,  3,  2};
const int actuatorDirectionPins[] = {34, 36, 42, 44, 46, 48, 50, 52};
const int limitSwitchInputPins[]  = {25, 29, 33, 37, 41, 45, 49, 53}; 
const int limitSwitchOutputPins[] = {23, 27, 31, 35, 39, 43, 47, 51};
const int encoderInputPins[]      = { 2,  3,  6,  7,  8,  9, 10, 11};

const int pValue = 500;
const int sendMessageInterval = 50; // ~10hz
const int motorControlInterval = 50; // ~10hz

bool commandReceived[8];

int sendMessageCounter = 0;
int motorControlCounter = 0;

float actuatorGoal[8];
float currentPos[8]; // Current position in MM

int lastMeasurement[8];
int turnCounter[8];

int zeroOffset[8]; // Contains the offset when zero

ros::NodeHandle nh;

dyret_common::Configuration stateMessage;
ros::Publisher statePub("actuatorStates", &stateMessage);

dyret_common::Configuration debugMessage;
ros::Publisher debugPub("dyret_actuator/actuatorDebug", &stateMessage);

void messageCb(const dyret_common::Configuration& msg);
ros::Subscriber<dyret_common::Configuration> s("actuatorCommands",messageCb);

bool limitSwitchActive(int givenActuatorNumber){
  if (digitalRead(limitSwitchInputPins[givenActuatorNumber]) == LOW) return false;
  return true;
}

void expandActuator(int givenActuatorNumber){
  digitalWrite(actuatorDirectionPins[givenActuatorNumber], LOW); // Set direction
  digitalWrite(actuatorEnablePins[givenActuatorNumber], HIGH); // Set enable bit
}

void retractActuator(int givenActuatorNumber){
  digitalWrite(actuatorDirectionPins[givenActuatorNumber], HIGH); // Set direction
  digitalWrite(actuatorEnablePins[givenActuatorNumber], HIGH); // Set enable bit
}

void disableMotor(int givenActuatorNumber){
  digitalWrite(actuatorEnablePins[givenActuatorNumber], LOW); // Set enable bit
}

int readRawEncoder(int givenActuatorNumber){ // Returns int 0-1023
  return analogRead(encoderInputPins[givenActuatorNumber]);
}

void setMotorZeroPoint(int givenActuatorNumber){
  commandReceived[givenActuatorNumber] = false;  
  actuatorGoal[givenActuatorNumber] = 0.0;
  zeroOffset[givenActuatorNumber] = readRawEncoder(givenActuatorNumber);
  turnCounter[givenActuatorNumber] = 0;
}

void enableMotor(int givenActuatorNumber, int givenMotorSpeed, bool motorDirectionRetract){ // Speed 0-255, direction inward
  int motorSpeed = min(givenMotorSpeed, 255);

  // Set direction
  if (motorDirectionRetract){
    digitalWrite(actuatorDirectionPins[givenActuatorNumber], HIGH); // Set direction
  } else {
    digitalWrite(actuatorDirectionPins[givenActuatorNumber], LOW); // Set direction
  }

  // Set speed (motor enable)
  analogWrite(actuatorEnablePins[givenActuatorNumber], motorSpeed);
  
}

void readPositions(){

  int measurements[8];

  // Read from encoders
  for (int i = 0; i < 8; i++){
    measurements[i] = readRawEncoder(i);
  }

  // Handle turncounter
  for (int i = 0; i < 8; i++){
    // Overflowed:
    if ((measurements[i] < 256) && (lastMeasurement[i] > 768)){
      turnCounter[i]++;
    }

    // Underflowed:
    if ((measurements[i] > 768) && (lastMeasurement[i] < 256)){
      turnCounter[i]--;
    }

    lastMeasurement[i] = measurements[i];
  }

  // Convert to mm
  for (int i = 0; i < 8; i++){
    currentPos[i] = -((measurements[i] + (turnCounter[i] * 1024.0))-zeroOffset[i]) * ((16.0/7.0) / 1024.0);
  }
}

void setupDebugMessage(){
  static int32_t actuatorId[8];
  static float distances[8];

  for (int32_t i = 0; i < 8; i++){
      if (commandReceived[i] == true) actuatorId[i] = 1; else actuatorId[i] = 0;
      distances[i] = (float) actuatorGoal[i];
  }

  debugMessage.id = actuatorId;
  
  debugMessage.distance = distances;
  
  debugMessage.id_length = 8;
  debugMessage.distance_length = 8;

}

void setupStateMessage(){
  static int32_t actuatorId[8];
  //static float distances[8];

  for (int32_t i = 0; i < 8; i++){
      actuatorId[i] = i;
  }

  stateMessage.id = actuatorId;
  
  stateMessage.distance = currentPos;
  
  stateMessage.id_length = 8;
  stateMessage.distance_length = 8;

}

void messageCb(const dyret_common::Configuration& msg){

  // Special case for just one distance without ids
  if (msg.id_length == 0 && msg.distance_length == 1){
    for (int i = 0; i < 8; i++){
      actuatorGoal[i] = msg.distance[0];
      commandReceived[i] = true;
    }
  }

  // Special case for just two distances without ids
  if (msg.id_length == 0 && msg.distance_length == 2){
    for (int i = 0; i < 8; i = i+2){
      actuatorGoal[i] = msg.distance[0];
      commandReceived[i] = true;
    }
    
    for (int i = 1; i < 8; i = i+2){
      actuatorGoal[i] = msg.distance[1];
      commandReceived[i] = true;
    }

  } else {
  
    for (int i = 0; i < msg.id_length; i++){
      if (msg.id[i] >= 0 && msg.id[i] < 8){
        actuatorGoal[msg.id[i]] = msg.distance[i];
        commandReceived[msg.id[i]] = true;
      }
    }
  }  

//  debugMessage = msg;
//  debugPub.publish(&debugMessage);
}

void setup() {
  for (int i = 0; i < 8; i++){
    actuatorGoal[i] = 512.0;
    zeroOffset[i] = 0;
    turnCounter[i] = 0;
    commandReceived[i] = false;
  }

  for (int i = 0; i < sizeof(actuatorEnablePins)/sizeof(int); i++){ pinMode(actuatorEnablePins[i], OUTPUT); digitalWrite(actuatorEnablePins[i], LOW); }          // Enable pins
  for (int i = 0; i < sizeof(actuatorDirectionPins)/sizeof(int); i++){ pinMode(actuatorDirectionPins[i], OUTPUT); }                                              // Direction pins
  for (int i = 0; i < sizeof(limitSwitchOutputPins)/sizeof(int); i++){ pinMode(limitSwitchOutputPins[i], OUTPUT); digitalWrite(limitSwitchOutputPins[i], LOW); } // Set pinmode and output ground to limit switch outputs
  for (int i = 0; i < sizeof(limitSwitchInputPins)/sizeof(int); i++){ pinMode(limitSwitchInputPins[i], INPUT); digitalWrite(limitSwitchInputPins[i], HIGH); }    // Set pinmode and connect pullup to limit switch inputs

  nh.initNode(); 
  
  for (int i = 0; i < 8; i++){
    commandReceived[i] = true;
    actuatorGoal[i] = -100;
  }

  for (int i = 0; i < 8; i++){
    lastMeasurement[i] = readRawEncoder(i);
  }
  
  nh.subscribe(s);
  nh.advertise(statePub);
  nh.advertise(debugPub);

}

void loop() {

  // Read positions:
  readPositions();

  // Send messages:
  sendMessageCounter += 1;
  if (sendMessageCounter == sendMessageInterval+1){
    setupStateMessage();
    setupDebugMessage();
  
    statePub.publish(&stateMessage);
    debugPub.publish(&debugMessage);
    
    sendMessageCounter = 0;
  }
  
  // Do motor loop:
  motorControlCounter += 1;

  if (motorControlCounter == motorControlInterval+1){
    for (int i = 0; i < 8; i++){
  
      if (commandReceived[i] == true){
    
        int motorSpeed = min(abs(actuatorGoal[i] - currentPos[i]) * pValue, 255);
  
        if (abs(actuatorGoal[i] - currentPos[i]) < 0.1){
          disableMotor(i);
        } else {
          if (currentPos[i] > actuatorGoal[i]){ // Retract
            if (limitSwitchActive(i) == false){ 
              enableMotor(i, motorSpeed, true);
            } else {
              setMotorZeroPoint(i);
            }
            
          } else {
            enableMotor(i, motorSpeed, false);  
          }
          
        }
      
      } else {
        disableMotor(i);
      }
    }

    motorControlCounter = 0;
  }

  // Spin ROS:
  nh.spinOnce();

}
