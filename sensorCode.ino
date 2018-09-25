

#include <dht.h>
#include <SoftwareSerial.h>

dht DHT;
//pin and sensor parameter definitions
#define humiGr 5 //humiture ground digital 5
#define humiVCC 6 //humiture power digital 6
#define DHT11_PIN 7 //humiture digital 7
#define tempGr A0 //thermistor ground analog 0
#define tempVCC A1 //thermistor power analog 1
#define analogPin A2 //thermistor analog 2
#define beta 4090 //beta of thermistor
#define resistance 10 //pull-down resistor value

//PM Setup
int measurePin = 6; //wire 5 to analog 6
int ledPower = 12; //wire 2 to digital 12
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
double voMeasured = 0;
double calcVoltage = 0;
double dustDensity = 0;
SoftwareSerial BT(3,11);
int count = 0;
char r;
double dustDensitySum = 0;
int numReadings = 100;

//Smoothing Temp Setup
const int thnumReadings = 10;
double readings1[thnumReadings];
int index1 = 0;
double total1 = 0;
double average1 = 0;

//Smoothing Hum Setup
double readings3[thnumReadings];
int index3 = 0;
double total3 = 0;
double average3 = 0;

void setup() {
  count = 0;
  Serial.begin(9600);
  BT.begin(9600);
  pinMode(ledPower,OUTPUT);
  for (int thisReading1 = 0; thisReading1 < thnumReadings; thisReading1++) {
  readings1[thisReading1] = 0;
  }
  for (int thisReading3 = 0; thisReading3 < thnumReadings; thisReading3++) {
  readings3[thisReading3] = 0;
  }
  analogWrite(humiGr,0); // effectively makes humiGr a ground pin
  analogWrite(humiVCC,255); // effectively makes humiVCC a 5V pin
  analogWrite(tempGr,0); // effectively makes tempGr a ground pin
  analogWrite(tempVCC,255); // effectively makes tempVCC a 5V pin
}

void loop() {     
  // read thermistor data
  long a = 1024 - analogRead(analogPin);

  // Read humiture sensor data
  int chk = DHT.read11(DHT11_PIN);

  //Smoothing Loops
  //Temp
  total1 = total1 - readings1[index1];
  readings1[index1] = a;
  total1 = total1 + readings1[index1];
  index1 = index1 + 1;
  if (index1 >= thnumReadings) {
    index1 = 0;
  }
  average1 = total1 / thnumReadings;
  
  //Hum
  total3 = total3 - readings3[index3];
  readings3[index3] = DHT.humidity;
  total3 = total3 + readings3[index3];
  index3 = index3 + 1;
  if (index3 >= thnumReadings) {
    index3 = 0;
  } 
  average3 = total3 / thnumReadings;
     
  //temperature formula
  float tempC = beta /(log((((1025.0 * 10) / average1) - 10) / 10) + (beta / 298.0)) - 273.0;
  float tempF = (tempC * 9 / 5) + 32.0; 
  
  if (Serial.available()) { // change "Serial" here to "BT" to wait for a command from the Android device
    r = (Serial.read()); // do the same here
    if (r == 'r') { // if the above was changed, change the 'r' to 'A'
      dustDensitySum = 0;
      for (int i = 0; i < numReadings; i++) {
        PMCon();
      }
      double dustDensityAvg = dustDensitySum/numReadings;
      BT.print(dustDensityAvg*1000);
      BT.print(" ");
      BT.print(tempC);
      BT.print(" ");
      BT.print(tempF);
      BT.print(" ");  
      BT.println(average3,1);
      double dDA100 = dustDensityAvg*100000;
      int dDAint = (int) dDA100;
      Serial.print(dDAint);
      Serial.print('n');
    }
  }
  delay(600);
}
void PMCon() {
  digitalWrite(ledPower,LOW); // power on the LED
  delayMicroseconds(samplingTime);
  long voMeasured = analogRead(measurePin); // read the pin value
  if (voMeasured <= (0.1 * 1024) / (5.0 * 0.17)) {
    delayMicroseconds(deltaTime);
    digitalWrite(ledPower,HIGH); // turn the LED off
    delayMicroseconds(sleepTime);
    dustDensity = 0;
  }
  else {
    delayMicroseconds(deltaTime);
    digitalWrite(ledPower,HIGH); // turn the LED off
    delayMicroseconds(sleepTime);
    calcVoltage = voMeasured * (5.0 / 1024);
    dustDensity = 0.17 * calcVoltage - 0.1;
  }
  dustDensitySum = dustDensity + dustDensitySum;
//  delay(4000/numReadings);
}
