#include <ros.h>
#include <dyret_common/Configuration.h>
#include <std_msgs/String.h>

bool commandReceived[8];

float actuatorGoal[8];
float currentPos[8]; // Current position in MM

int lastMeasurement[8];
int turnCounter[8];

int zeroOffset[8]; // Contains the offset when zero

ros::NodeHandle nh;

dyret_common::Configuration config_msg;
ros::Publisher p("actuatorStates", &config_msg);

const int actuatorEnablePins[]    = {11, 10,  7,  6,  5,  4,  3,  2};
const int actuatorDirectionPins[] = {34, 36, 42, 44, 46, 48, 50, 52};
const int limitSwitchInputPins[]  = {25, 29, 33, 37, 41, 45, 49, 53}; 
const int limitSwitchOutputPins[] = {23, 27, 31, 35, 39, 43, 47, 51};
const int encoderInputPins[]      = { 2,  3,  6,  7,  8,  9, 10, 11};

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

void zeroLegs(){
  for (int i = 0; i < 8; i++){
    Serial.print("Zeroing actuator ");
    Serial.println(i);
    if (limitSwitchActive(i) == false){ retractActuator(i); }
    while (limitSwitchActive(i) == false){ ; } // Wait until retracted fully
    disableMotor(i);
    zeroOffset[i] = readRawEncoder(i);
    Serial.print(i);
    Serial.print(": ");
    Serial.println(zeroOffset[i]);
  }
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
  }

  // Convert to mm
  for (int i = 0; i < 8; i++){
    currentPos[i] = -((measurements[i] + (turnCounter[i] * 1024.0))-zeroOffset[i]) * (2.5 / 1024.0);
  }
}

// Setup message to send out
void setupMessage(dyret_common::Configuration& msg){
  static int32_t actuatorId[8];
  static float distances[8];

  for (int32_t i = 0; i < 8; i++){
      actuatorId[i] = i;
  }

  static float tmp[8];
  for (int i = 0; i < 8; i++) tmp[i] = float(lastMeasurement[i]);

  actuatorGoal[2] = 15.0;
  
  config_msg.id = actuatorId;
  
  config_msg.distance = currentPos;
  //config_msg.distance = tmp;
  //config_msg.distance = actuatorGoal;
  
  config_msg.id_length = 8;
  config_msg.distance_length = 8;

}

void messageCb(const dyret_common::Configuration& msg){
   for (int i = 0; i < msg.id_length; i++){
     if (msg.distance[i] < 0){
      commandReceived[msg.id[i]] = false; // Disable regulation
    } else {
      if (msg.id[i] >= 0 && msg.id[i] < 8){
        actuatorGoal[msg.id[i]] = msg.distance[i];
        commandReceived[msg.id[i]] = true;
      }
    }
  }  
}

ros::Subscriber<dyret_common::Configuration> s("actuatorCommands",messageCb);

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

  zeroLegs();

  for (int i = 0; i < 8; i++){
    lastMeasurement[i] = readRawEncoder(i);
  }
  
  nh.initNode(); 

  nh.subscribe(s);
  nh.advertise(p);

}

void loop() {
  int pValue = 500;

  // Read positions:
  readPositions();
  
  setupMessage(config_msg);
  
  p.publish(&config_msg);
  nh.spinOnce();

  for (int i = 0; i < 8; i++){

    if (commandReceived[i] == true){
  
      int motorSpeed = min(abs(actuatorGoal[i] - currentPos[i]) * pValue, 255);
      bool motorDirectionRetract = false;

      if (abs(actuatorGoal[i] - currentPos[i]) < 0.05){
        disableMotor(i);
      } else {
        enableMotor(i, motorSpeed, (currentPos[i] > actuatorGoal[i]));
      
      }
    
    } else {
      disableMotor(i);
    }
  }


  for (int i = 0; i < 8; i++){
    lastMeasurement[i] = readRawEncoder(i);
  }

  delay(100);

}
