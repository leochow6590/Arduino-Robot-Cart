#include <Wire.h>

///////////////////////////////////////////////////////
//interrupt pin
int interruptL1 = 2;
int interruptL2 = 3;

int interruptR1 = 18;
int interruptR2 = 19;
///////////////////////////////////////////////////////

//////////////////////////////////////////////////////
//motor driver LM298 control pin
int pin1R = 8;
int pin2R = 9;
int pin1L = 10;
int pin2L = 11;
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
// encoder parameter
long encoderValueR = 0;
long encoderValueL = 0;
//////////////////////////////////////////////////////

int forwardStart = 0;
int targetValue = 0;

void setup()
{
  //Serial Setup
  Serial.begin(115200);

  /////////////////////////////////////////
  //Motor
  pinMode(pin1L, OUTPUT);
  pinMode(pin2L, OUTPUT);
  pinMode(pin1R, OUTPUT);
  pinMode(pin2R, OUTPUT);
  /////////////////////////////////////////

  /////////////////////////////////////////
  //Encoder
  //https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  pinMode(interruptL1, INPUT_PULLUP);
  pinMode(interruptL2, INPUT_PULLUP);
  pinMode(interruptR1, INPUT_PULLUP);
  pinMode(interruptR2, INPUT_PULLUP);
  //setup interrupt
  attachInterrupt(digitalPinToInterrupt(interruptL1), countL, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptR1), countR, FALLING);
  /////////////////////////////////////////
}

/////////////////////////////
//Main Loop
void loop()
{
  MotorControl();
    Serial.print(" encoderValueL= ");
    Serial.print(encoderValueL);
    Serial.print(" encoderValueR= ");
    Serial.print(encoderValueR);
    Serial.println("");
}

//////////////////////////////////////////////////////
//Motor control
void MotorControl() {
    forward_stline();
}

//////////////////////////////////////////
void forward_stline() {
  //forward 100 click
  if (forwardStart == 0) {
    stopCar();
    targetValue = 100;
    encoderValueL = 0;
    encoderValueR = 0;
    forwardStart = 1;
  }
  if (encoderValueL > targetValue or encoderValueR > targetValue) {
    stopCar();
    forwardStart = 0;
  }
  else {
    if (encoderValueL > encoderValueR) {
      turnCarL();
    }
    else if (encoderValueR > encoderValueL) {
      turnCarR();
    }
    else {
      forwardCar();
    }
  }
}

//////////////////////////////////////////////////////
//Interrupt subroutine
void countL() {
  if (digitalRead(interruptL2)) {
    encoderValueL--;
  }
  else {
    encoderValueL++;
  }
}
void countR() {
  if (digitalRead(interruptR2)) {
    encoderValueR++;
  }
  else {
    encoderValueR--;
  }
}
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
// Motor subrountine
void turnCarL() {
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 0);
}
void turnCarR() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 1);
  digitalWrite(pin2L, 0);
}
void turnCarOnsiteL() {
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 1);
}
void turnCarOnsiteR() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 1);
  digitalWrite(pin1L, 1);
  digitalWrite(pin2L, 0);
}
void forwardCar() {
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 1);
  digitalWrite(pin2L, 0);
}
void backwardCar() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 1);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 1);
}
void stopCar() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 0);
}
void forwardRightWheel() {
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
}
void forwardLeftWheel() {
  digitalWrite(pin1L, 1);
  digitalWrite(pin2L, 0);
}
void backwardRightWheel() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 1);
}
void backwardLeftWheel() {
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 1);
}
void stopRightWheel() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 0);
}
void stopLeftWheel() {
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 0);
}
