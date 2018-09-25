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
double kp = 20.0;
double E = 0.0;
char a;

SoftwareSerial BT(TX,RX);

void setup() {
  BT.begin(9600);
  BT.println("Haylo");
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
  spdR=80.0;
  if (BT.available()) {
    a = (BT.read());
    if(a == 'E') { // forward
      zeroes();
      while(a != 'e') {
        a = (BT.read());
        forward();
      }
      pause();
    }
    if(a == 'D') { // backward
      zeroes();
      while(a!='d'){
        a = (BT.read());
        backward();
      }
      pause();
    }
    if(a == 'B') { // right
      while(a!='b'){
        a = (BT.read());
        right();
      }
      pause();
    }
    if(a == 'C') { // left
      while(a!='c'){
        a = (BT.read());
        left();
      }
      pause();
    }
    if(a == 'A') { // commands PM sensor to send a reading to the BT terminal
      takeReading();
    }
  }
}

void forward(){
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
void backward(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, spdR); 
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN2, spdL);
  E = error();
  spdR -= E*kp;
  if(spdR>255){
    spdR=255;
  }
  delay(50);
}
void left(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, spdT);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN2, spdT);
}
void right(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, spdT);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN2, spdT);
}
void pause(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
double error(){
  return LE.getPosition() - RE.getPosition();
}
void takeReading() {
  Serial.write('r');
}
void zeroes(){
  LE.zero();
  RE.zero();
  spdR=spdRinit;
}

