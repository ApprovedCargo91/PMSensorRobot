/** MUST FIRST UPLOAD sensorCode TO SENSOR MODULE!!! **/

#include <SoftwareSerial.h>
#include <Wire.h>
#include <I2CEncoder.h>

I2CEncoder LE;
I2CEncoder RE;

// circumference of wheel in feet
double C = 0.36;

const int RX = 2;
const int EN1 = 11;
const int IN1 = 12;
const int IN2 = 9;
const int IN3 = 8;
const int IN4 = 7;
const int EN2 = 6;
const int TX = 5;
double spdL = 100.0;
const double spdRinit = 80.0;
double spdR = spdRinit;
double spdT = 40.0;
double spdTR = 30.0;
double kp = 20.0;
double E = 0.0;
double maxReading = 0.0;
int mRI = -1;
String tempStr;
int tempInt;
char a;
boolean TR = true;
boolean toX = false;
double turnDist = 0.90; // checked on 7/12/18, rechecked on 7/16/18 (0.90)
double tempDist = 0.0;
double x = 4.0;
double y = 4.0;
int d = 1;

SoftwareSerial BT(TX,RX);

void setup() {
  BT.begin(9600);
  Serial.begin(9600);
  Wire.begin();
  LE.init(C*MOTOR_393_TORQUE_ROTATIONS, MOTOR_393_TIME_DELTA);
  RE.init(C*MOTOR_393_TORQUE_ROTATIONS, MOTOR_393_TIME_DELTA);
  RE.setReversed(true);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(EN1,spdR);
  analogWrite(EN2,spdL);
  BT.println("Waiting for start command...");
  while (a != 'A') { // wait for start command from Android device
    if (BT.available()) {
      a = (BT.read());
    }
  }
  pause();
  a = 'B';
}

void loop() {
  BT.println("Command received! Starting loop.");
  for(int i=0; i<4; i++){
    BT.println("Taking reading...");
    takeReading();
    while(Serial.available()<=0);
    tempStr = Serial.readStringUntil('n');
    tempInt = tempStr.toInt();
    BT.print("Reading: ");
    BT.println(tempInt);
    if(tempInt>maxReading){
      maxReading=tempInt;
      mRI = i;
    }
    if(i<3){
      dist();
      forward();
      turn();
    }
  }
  if(mRI==0){
    BT.println("mRI = 0");
    dist();
    forward();
    turn();
  }
  else if(mRI==1){
    BT.println("mRI = 1");
    dist();
    forward();
    turn();
    dist();
    forward();
    turn();
  }
  else if(mRI==2){
    BT.println("mRI = 2");
    turn();
    forward();
    TR=!TR;
    turn();
  }
  else if(mRI==3){
    BT.println("mRI = 3");
    // already in the right location - don't need to move
  }
  else {
    BT.println("mRI not set - exiting");
    pause();
    exit(0);
  }
  maxReading=0.0;
  mRI=-1;
  d=2*d;
  if(x/d<1.00||y/d<1.00){
    BT.println("Distance less than threshold - exiting");
    pause();
    exit(0);
  }
}

void dist(){
  if(toX){
    tempDist = x/d;
  }
  else{
    tempDist = y/d;
  }
  toX = !toX;
}
void pause(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN2, 20.0);  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN1, 20.0);
  delay(500);
}
void resetO(){
  if(LE.getPosition()<RE.getPosition()){
    while(LE.getPosition()<RE.getPosition()){
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(EN2, spdT);
    }
  }
  else{
    while(RE.getPosition()<LE.getPosition()){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(EN1, (3*spdT)/4);
    }
  }
  pause();
}
void left(){
  while(RE.getPosition()<turnDist){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN1, spdTR);
  }
  pause();
}
void right(){
  while(LE.getPosition()<turnDist){
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(EN2, spdT);
  }
  pause();
}
void takeReading() {
  Serial.write('r');
}
void turn(){
  zeroes();
  if(TR){
    right();
  }
  else{
    left();
  }
}
void zeroes(){
  LE.zero();
  RE.zero();
  spdR=spdRinit;
}
double error(){
  return LE.getPosition() - RE.getPosition();
}
void forward(){
  zeroes();
  while(LE.getPosition()<tempDist&&RE.getPosition()<tempDist){
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(EN2, spdL);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN1, spdR);
    E = error();
    spdR += E*kp;
    if(spdR>255){
      spdR=255;
    }
    delay(50);
  }
  delay(100);
  pause();
  resetO();
}
