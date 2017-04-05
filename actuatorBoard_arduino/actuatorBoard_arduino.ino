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

bool limitSwitchActive(int givenActuatorNumber){
  if (digitalRead(25+(givenActuatorNumber*4)) == LOW) return false;
  return true;
}

void enableMotor(int givenActuatorNumber, int givenMotorSpeed, bool motorDirectionRetract){ // Speed 0-255, direction inward
  int motorSpeed = min(givenMotorSpeed, 255);

  // Set direction
  if (motorDirectionRetract){
    digitalWrite(38 + (2 * givenActuatorNumber), HIGH);
  } else {
    digitalWrite(38 + (2 * givenActuatorNumber), LOW);
  }

  // Set speed (motor enable)
  analogWrite(9 - givenActuatorNumber, motorSpeed);
  
}

void expandActuator(int givenActuatorNumber){
  digitalWrite(38 + (2 * givenActuatorNumber), LOW); // Set direction
  digitalWrite(9 - givenActuatorNumber, HIGH); // Set enable bit
}

void retractActuator(int givenActuatorNumber){
  digitalWrite(38 + (2 * givenActuatorNumber), HIGH); // Set direction
  digitalWrite(9 - givenActuatorNumber, HIGH); // Set enable bit
}

void disableMotor(int givenActuatorNumber){
  digitalWrite(9 - givenActuatorNumber, LOW); // Set enable bit
}

int readRawEncoder(int givenActuatorNumber){ // Returns int 0-1023
  return analogRead(givenActuatorNumber+4);
}

void zeroLegs(){
  for (int i = 0; i < 8; i++){
    if (limitSwitchActive(i) == false){ retractActuator(i); }
    while (limitSwitchActive(i) == false){ ; } // Wait until retracted fully
    disableMotor(i);
    zeroOffset[i] = readRawEncoder(i);
  }
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

  // Enable pins
  for(int i = 2; i <= 13; i++){ pinMode(i, OUTPUT); digitalWrite(i, LOW); }

  // Direction pins
  for(int i = 30; i <= 52; i = i+2){ pinMode(i, OUTPUT);}

  // Set pinmode and output ground to limit switch outputs
  for (int i = 23; i <= 53; i = i+4){
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  // Set pinmode and connect pullup to limit switch inputs
  for (int i = 25; i <= 53; i = i+4){
    pinMode(i, INPUT);
    digitalWrite(i, HIGH);
  }

  zeroLegs();
  
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
