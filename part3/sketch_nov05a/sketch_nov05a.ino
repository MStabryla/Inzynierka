#include "Arduino.h"
#include "analogWrite.h"
#include <string.h>
#include "DHT.h"

#define DHTTYPE DHT11


const int IA1 = 5;
const int IA2 = 4;
const int IB1 = 2;
const int IB2 = 15;

const int distanceEcho = 19;
const int distanceTrig = 22;
const int tempInput = 23;

DHT dht(tempInput, DHTTYPE);


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  pinMode(distanceTrig,OUTPUT);
  pinMode(distanceEcho,INPUT);

  dht.begin();

}

long getDistanceCm(){
  //double* distances = HCSR04.measureDistanceCm();

  digitalWrite(distanceTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(distanceTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(distanceTrig, LOW);

  long resultIn = 0;
  resultIn = pulseIn(distanceEcho, HIGH);
  resultIn /= 58;
  //sprintf(mess, "dist: %d cm", distances[0]);

  return resultIn;
}
void Forward(int speed){
  analogWrite(IA1,speed, 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,speed, 100, 10, 0);
  analogWrite(IB2,0, 100, 10, 0);
}

void Backward(int speed){
  analogWrite(IA1,0, 100, 10, 0);
  analogWrite(IA2,speed, 100, 10, 0);
  analogWrite(IB1,0, 100, 10, 0);
  analogWrite(IB2,speed, 100, 10, 0);
}
void TurnLeft(int speed){
  analogWrite(IA1,speed, 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,0, 100, 10, 0);
  analogWrite(IB2,0, 100, 10, 0);
}

void TurnRight(int speed){
  analogWrite(IA1,0, 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,0, 100, 10, 0);
  analogWrite(IB2,speed, 100, 10, 0);
}
void TurnLeftBackward(int speed){
  analogWrite(IA1,0, 100, 10, 0);
  analogWrite(IA2,speed, 100, 10, 0);
  analogWrite(IB1,0, 100, 10, 0);
  analogWrite(IB2,0, 100, 10, 0);
}

void TurnRightBackward(int speed){
  analogWrite(IA1,0, 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,speed, 100, 10, 0);
  analogWrite(IB2,0, 100, 10, 0);
}
void TurnLeftAdv(int speed){
  analogWrite(IA1,speed, 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,speed, 100, 10, 0);
  analogWrite(IB2,0, 100, 10, 0);
}

void TurnRightAdv(int speed){
  analogWrite(IA1,speed, 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,0, 100, 10, 0);
  analogWrite(IB2,speed, 100, 10, 0);
}

void Stop(){
  analogWrite(IA1,0, 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,0, 100, 10, 0);
  analogWrite(IB2,0, 100, 10, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  long distance = getDistanceCm();

  
  //TurnLeft(1024);
  //delay(3000);
  //Stop();
  //delay(2000);
  //TurnRight(1024);
  //delay(3000);
  //Stop();
  //delay(2000);
  //TurnLeftBackward(1024);
  //delay(3000);
  //Stop();
  //delay(2000);
  //TurnRightBackward(1024);
  //delay(3000);
  //Stop();
  //delay(2000);
  TurnRightAdv(128);
  delay(3000);
  Stop();
  delay(2000);
  //Forward(128);
  //delay(3000);
  //Stop();
  //delay(2000);
  //Backward(1024);
  //delay(3000);
  //Stop();
  //delay(2000);
}
