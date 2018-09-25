// MUST FIRST UPLOAD demoSensor FILE TO SENSOR ARDUINO

#include <SoftwareSerial.h>
#include <Wire.h>
#include <I2CEncoder.h>

I2CEncoder LE;
I2CEncoder RE;

// circumference of wheel in feet
double C = 0.4;

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
double spdT = 50.0;
double kp = 20.0;
double E = 0.0;
double tempDist = 0.0;
char a;
char t;
int j = 0;

SoftwareSerial BT (TX,RX);

void setup() {
  pinMode(13, OUTPUT);
  BT.begin(9600);
  BT.println("Reset");
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
}

void loop() {
  while (a != 'A') { // wait for start command from Android device
    if (BT.available()) {
      a = (BT.read());
    }
  }
  pause();
  takeReading();
  a = 'B';
  for(int i=1; i<11; i++){ // take 10 readings
    tempDist+=4.1;
    forward();
    takeReading();
  }
  pause();
  kp=2.0;
  for(int i=0; i<5; i++){
    tempDist-=8.2;
    backward();
  }
  pause();
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
void bpause(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN2, 20.0);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
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
      analogWrite(EN1, spdT);
    }
  }
  pause();
}
void takeReading() {
  Serial.write('r');
  while(Serial.available()<=0);
  while(Serial.read()!='n');
}
double error(){
  return LE.getPosition() - RE.getPosition();
}
void forward(){
  spdR=spdRinit;
  while(LE.getPosition()<tempDist&&RE.getPosition()<tempDist){
    if(LE.getPosition()<0||LE.getPosition()<0){
      digitalWrite(13, HIGH);
    }
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
void backward(){
  spdR=spdRinit;
  while(LE.getPosition()>tempDist&&RE.getPosition()>tempDist){
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(EN2, spdL);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN1, spdR);
    E = error();
    spdR -= E*kp;
    if(spdR>255){
      spdR=255;
    }
    delay(50);
  }
  delay(100);
  bpause();
  resetO();
}
