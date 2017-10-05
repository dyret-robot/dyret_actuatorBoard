#include <stdio.h>

int zeroOffset[8]; // Contains the offset when zero

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

void setup() {

  for (int i = 0; i < sizeof(actuatorEnablePins)/sizeof(int); i++){ pinMode(actuatorEnablePins[i], OUTPUT); digitalWrite(actuatorEnablePins[i], LOW); }          // Enable pins
  for (int i = 0; i < sizeof(actuatorDirectionPins)/sizeof(int); i++){ pinMode(actuatorDirectionPins[i], OUTPUT); }                                              // Direction pins
  for (int i = 0; i < sizeof(limitSwitchOutputPins)/sizeof(int); i++){ pinMode(limitSwitchOutputPins[i], OUTPUT); digitalWrite(limitSwitchOutputPins[i], LOW); } // Set pinmode and output ground to limit switch outputs
  for (int i = 0; i < sizeof(limitSwitchInputPins)/sizeof(int); i++){ pinMode(limitSwitchInputPins[i], INPUT); digitalWrite(limitSwitchInputPins[i], HIGH); }    // Set pinmode and connect pullup to limit switch inputs
  
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  zeroLegs();

}

void loop() {


  for (int i = 0; i < 8; i++){

    char tbs[16];

    sprintf(tbs, "%04d ",readRawEncoder(i));
    Serial.print(tbs);

    if (limitSwitchActive(i)){
      Serial.print(" (L) ");
    } else {
      Serial.print(" (l) ");
    }
  }

  Serial.println();

  delay(200);
}
