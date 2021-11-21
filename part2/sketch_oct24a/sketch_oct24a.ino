//#include <HCSR04.h>
#include <string.h>
#include "DHT.h"

#define DHTTYPE DHT11


const int IA1 = 5;
const int IA2 = 4;
const int IB1 = 2;
const int IB2 = 15;

const int distanceEcho = 18;
const int distanceTrig = 19;
const int tempInput = 21;

DHT dht(tempInput, DHTTYPE);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  //HCSR04.begin(distanceTrig, distanceEcho);
  pinMode(distanceTrig,OUTPUT);
  pinMode(distanceEcho,INPUT);

  dht.begin();
  
}

char mess[64];

char* getDistanceMess(){
  //double* distances = HCSR04.measureDistanceCm();

  digitalWrite(distanceTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(distanceTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(distanceTrig, LOW);

  long resultIn = 0;
  resultIn = pulseIn(distanceEcho, HIGH);
  resultIn /= 58;
  sprintf(mess, "dist: %d cm", resultIn);
  //sprintf(mess, "dist: %d cm", distances[0]);

  return mess;
}

char tempMess[64];

char* getTempMess(){
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  sprintf(tempMess, "Humidity: %f , °C %f", h,t);

  return tempMess;
}

void loop() {
  // put your main code here, to run repeatedly:
  //double* distances = HCSR04.measureDistanceCm();

  char* m = getDistanceMess();
  Serial.println(m);

  Serial.println(getTempMess());
  
  delay(1000);
}
