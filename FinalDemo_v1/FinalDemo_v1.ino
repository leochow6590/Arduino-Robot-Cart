#include <Wire.h>
//Mega2560 pin 20 (SDA), pin 21 (SCL)
//SPI pin
// MOSI 51
//MISO 50
//SCK 52
//SS 53
#include "TimerOne.h"
#include <Adafruit_SSD1331.h>
///////////////////////////////////////////////////////
//OLED
int sclk = 52; //brown--- connect this to the display module CLK pin (Serial Clock)
int mosi = 51; //orange--- connect this to the display module DIN pin (Serial Data)
int rst  = 49; //yellow--- connect this to the display module RES pin (Reset)
int dc   = 50; //green--- connect this to the display module D/C  pin (Data or Command)
int cs   = 48; //blue--- connect this to the display module CS  pin (Chip Select)
// Color definitions

#define  BLACK           0x0000
#define BLUE            0x0006
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF
#define BACKGROUND      0x0000
Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, mosi, sclk, rst);

//Color sensor
#define colorpin_out 26 //brown
#define colorpin_s2 27 //orange
#define colorpin_s3 28 //yellow
#define colorpin_s1 29 // purple
#define colorpin_s0 30 //blue
#define colorpin_LED 31 //LED
int colorValueC = 0;
int colorValueR = 0;
int colorValueG = 0;
int colorValueB = 0;
int colorCnt = 0;
int colorCheckCntR = 0; //check several times before detection
int colorCheckCntG = 0;
int colorCheckCntB = 0;
int colorCheckCnt = 1;
int detectR = 0;
int detectG = 0;
int turn_cooldown = 0;

///////////////////////////////////////////////////////
//Black Line
//see black tape is 1, LED off
//see white is 0, LED on
#define blackLinePinTR 42         //Top Right
#define blackLinePinTL 43         //Top Left
#define blackLinePinFR 44         //Far Right (near the wheel)
#define blackLinePinFL 45         //Far Left (near the wheel)

bool blackLineTR = 0; //top right
bool blackLineTL = 0; // top left
bool blackLineFR = 0; //far right
bool blackLineFL = 0; //far left

///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
//Power detection
#define voltageValuePin A7
float  voltageValue = 0;
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
//Ultra Sonic
#define trigPin 13
#define echoPin 12

unsigned long UltraSonicStartTime = 0;
int UltraSonicDone = 1;
int    duration;
float  distance;
///////////////////////////////////////////////////////

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
//encoderValue
long encoderValueR = 0;
long encoderValueL = 0;

//Reset encoderValue
int encoderValueR_Reset = 0;
int encoderValueL_Reset = 0;

//encoderValue Diff
long encoderValueR_Diff = 0;
long encoderValueL_Diff = 0;
//////////////////////////////////////////////////////

int forwardStart = 0;
int rightStart = 0;
int leftStart = 0;
int targetValue = 0;

////////////////////////////////////////////
//For black line
int prev_blackLineFR = 0;
int rising_blackLineFR = 0;
int prev_blackLineFL = 0;
int rising_blackLineFL = 0;
int rising_blackLineFR_level = 0;
int rising_blackLineFL_level = 0;
int state = 0; //L = -1, forward = 0, R = 1
////////////////////////////////////////////

////////////////////////////////////////////
//For coordinate
int x = 0;
int y = 0;
int rotation = 0;
////////////////////////////////////////////

void setup()
{
  ///////////////////////////
  //OLED
  display.begin();
  display.fillScreen(BLACK);
  tftPrintTest();
  delay(2000);
  display.fillScreen(BACKGROUND);
  display.setCursor(0, 0);
  ///////////////////////////
  //Serial Setup
  Serial.begin(115200);
  /////////////////////////////////////////
  //Color sensor setup
  pinMode(colorpin_LED, OUTPUT);
  pinMode(colorpin_out, INPUT);
  pinMode(colorpin_s0, OUTPUT);
  pinMode(colorpin_s1, OUTPUT);
  pinMode(colorpin_s2, OUTPUT);
  pinMode(colorpin_s3, OUTPUT);
  // turn on LED
  digitalWrite(colorpin_LED, 1);
  //100% freq
  digitalWrite(colorpin_s0, 1);
  digitalWrite(colorpin_s1, 1);
  //detect red
  digitalWrite(colorpin_s2, 1);
  digitalWrite(colorpin_s3, 0);
  /////////////////////////////////////////

  /////////////////////////////////////////
  //Black Line sensor
  pinMode(blackLinePinTR, INPUT);
  pinMode(blackLinePinTL, INPUT);
  pinMode(blackLinePinFR, INPUT);
  pinMode(blackLinePinFL, INPUT);
  /////////////////////////////////////////


  /////////////////////////////////////////
  //Ultra Sonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  /////////////////////////////////////////

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

void loop() {
  InputCapture();
  MotorControl();
  Serial.print("|TR = ");
  Serial.print(blackLineTR);
  Serial.print("|TL = ");
  Serial.print(blackLineTL);
  Serial.print("|FR = ");
  Serial.print(blackLineFR);
  Serial.print("|FL = ");
  Serial.print(blackLineFL);
  Serial.print("|voltage = ");
  Serial.print(voltageValue);
  Serial.print("|ultra = ");
  Serial.print(distance);
  Serial.print(" encoderValueL= ");
  Serial.print(encoderValueL);
  Serial.print(" encoderValueR= ");
  Serial.print(encoderValueR);
  Serial.println("");
  
}

void MotorControl(){
  // Checking if it is on the intersection, turning condition should be prioritized. 
  // May have bug since both FR and FL may still detect blacklines after a 90 degree turn and may turn again.
  // if have bug, use blackline parameters instead
  if (rising_blackLineFL_level == 1 || rising_blackLineFR_level == 1){  // moving in the intersection , use || or && ???
    stopCar();          // When at intersections, it should stop immediately.
    ColorInput();       // Check color when the car is stopped.
    ColorCheck();
    if (detectR == 1){  // if red is detected -> turn right 90 degree
      turnLeft90();
    }
    if (detectG == 1){  // if green is detected -> turn left 90 degree
      turnRight90();
    }
  }
  if (blackLineTR || blackLineTL){
    forward_stline();
  }
}

/////////////////////////////////////////////////////////////////////////////////////
///Motor related functions
void turnLeft90(){
  targetValue = 30;
  encoderValueL = 0;
  encoderValueR = 0;
  for (int i = 0; i < targetValue; i++){
    turnCarOnsiteL();
  }
  stopCar();
}

void turnRight90(){
  targetValue = 30;
  encoderValueL = 0;
  encoderValueR = 0;
  for (int i = 0; i < targetValue; i++){
    turnCarOnsiteR();
  }
  stopCar(); 
}

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
///////////////////////////////////////////////////////////////////

void coordinate(){
  
}

void oledDisplay(){
  
}


///////////////////////////////////////////////////////////////////
void blackLineState(){
  // this part maybe useful for coordinate
  if (blackLineFL){
    rising_blackLineFL = 1; // 1 = back
  }
  else{
    rising_blackLineFL = 0; // 0 = white
  }
  if (blackLineFR){
    rising_blackLineFR = 1;
  }
  else{
    rising_blackLineFR = 0;
  }
  if (rising_blackLineFL == 1 && prev_blackLineFL == 0){   // from 0 to 1 -> from white to black
    rising_blackLineFL_level = 1; // left side moving in the blackline
  }
  else if (rising_blackLineFL == 0 && prev_blackLineFL == 1){  // from 1 to 0 -> from black to white
    rising_blackLineFL_level = -1; // left side moving out the blackline
  }
  if (rising_blackLineFR == 1 && prev_blackLineFR == 0){  // from 0 to 1 -> from white to black
    rising_blackLineFR_level = 1; // right side moving in the blackline
  }
  else if (rising_blackLineFR == 0 && prev_blackLineFR == 1){  // from 1 to 0 -> from black to white
    rising_blackLineFR_level = -1; // right side moving out the blackline
  }
  if (rising_blackLineFL == prev_blackLineFL){
    rising_blackLineFL_level = 0;
  }
  if (rising_blackLineFR == prev_blackLineFR){
    rising_blackLineFR_level = 0;
  }
  prev_blackLineFL = rising_blackLineFL;
  prev_blackLineFR = rising_blackLineFR;
}

//////////////////////////////////////////////////////
//Input routine
void InputCapture() {
  //black
  blackLineTR = digitalRead(blackLinePinTR);
  blackLineTL = digitalRead(blackLinePinTL);
  blackLineFR = digitalRead(blackLinePinFR);
  blackLineFL = digitalRead(blackLinePinFL);
  blackLineState();
  //Power Sensor
  voltageValue = analogRead(voltageValuePin);
  voltageValue = 25 * voltageValue / 1024;
}
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
//UltraSonic subroutine
void UltraSonic() {
  if (UltraSonicDone) {
    UltraSonicDone = 0;
    UltraSonicStartTime = millis();
    digitalWrite(trigPin, LOW);  // Added this line
  }
  if (millis() > UltraSonicStartTime + 2) {
    digitalWrite(trigPin, HIGH);
  }
  if (millis() > UltraSonicStartTime + 12) {
    digitalWrite(trigPin, LOW);
    //    duration = pulseIn(echoPin, HIGH, 3000); //set 3000ns as timout
    duration = pulseIn(echoPin, HIGH); //set 3000ns as timout
    distance = (duration / 2) / 29.1;
    UltraSonicDone = 1;
  }
}
////////////////////////////////////////////////////
void ColorInput() {
  //Set Clear filter
  digitalWrite(colorpin_s2, 1);
  digitalWrite(colorpin_s3, 0);
  delay(100);
  //read Clear value
  colorValueC = pulseIn(colorpin_out, LOW);

  //Set Red filter
  digitalWrite(colorpin_s2, 0);
  digitalWrite(colorpin_s3, 0);
  delay(100);
  //read Red value
  colorValueR = pulseIn(colorpin_out, LOW);

  //Set Blue filter
  digitalWrite(colorpin_s2, 0);
  digitalWrite(colorpin_s3, 1);
  delay(100);
  //read Blue value
  colorValueB = pulseIn(colorpin_out, LOW);

  //Set Green filter
  digitalWrite(colorpin_s2, 1);
  digitalWrite(colorpin_s3, 1);
  delay(100);
  //read Green value
  colorValueG = pulseIn(colorpin_out, LOW);

  Serial.print(" colorCRBG= ");
  Serial.print(colorValueC);
  Serial.print(" ");
  Serial.print(colorValueR);
  Serial.print(" ");
  Serial.print(colorValueB);
  Serial.print(" ");
  Serial.print(colorValueG);
  Serial.print(" ");
}
void ColorCheck() {
  //Check Green Color  value G small
  if ((2 <= colorValueC && colorValueC <= 9) &&
      (9 <= colorValueR && colorValueR <= 17) &&
      (8 <= colorValueB && colorValueB <= 15) &&
      (8 <= colorValueG && colorValueG <= 14)) {
    detectG = 1;
    Serial.println(" Green is detected. ");
  }
  //Check Red Color value R small
  else if ((3 <= colorValueC && colorValueC <= 13) &&
           (0 <= colorValueR && colorValueR <= 15) &&
           (3 <= colorValueB && colorValueB <= 18) &&
           (11 <= colorValueG && colorValueG <= 23)) {
    detectR = 1;
    Serial.println(" Red is detected. ");
  }
}
//////////////////////////////////////////
void tftPrintTest() {
  display.fillScreen(BLACK);
  display.setCursor(15, 5);
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.println("Senor");
  display.setCursor(0, 25);
  display.setTextColor(CYAN);
  display.setTextSize(2);
  display.println("test");
  display.setCursor(15, 50);
  display.setTextColor(YELLOW);
  display.setTextSize(2);
  display.println("Start");
  delay(1500);
  display.setTextColor(WHITE);
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
//////////////////////////////////////////////////////
