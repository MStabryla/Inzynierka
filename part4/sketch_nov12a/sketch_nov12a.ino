//#include <HCSR04.h>
#include "Arduino.h"
#include "analogWrite.h"
#include <string.h>
#include "DHT.h"
#include <Wire.h>
#include <LPS.h>
#include <MQUnifiedsensor.h>

#define DHTTYPE DHT11


const int IA1 = 13;
const int IA2 = 12;
const int IB1 = 27;
const int IB2 = 14;

const int distanceEcho1 = 19;
const int distanceTrig1 = 21;
const int distanceEcho2 = 25;
const int distanceTrig2 = 26;
const int tempInput = 18;
const int gasSensor = 4;
const int SDA0_Pin = 22;
const int SCL0_Pin = 23;

DHT dht(tempInput, DHTTYPE);

LPS ps;

/************************Hardware Related Macros************************************/
#define         Board                   ("Arduino UNO")
#define         Pin                     (4)  //Analog input 4 of your arduino
/***********************Software Related Macros************************************/
#define         Type                    ("MQ-9") //MQ9
#define         Voltage_Resolution      (5)
#define         ADC_Bit_Resolution      (10) // For arduino UNO/MEGA/NANO
#define         RatioMQ9CleanAir        (9.6) //RS / R0 = 60 ppm 
MQUnifiedsensor MQ9(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println("help");
  Wire.begin(SDA0_Pin, SCL0_Pin);

  //HCSR04.begin(distanceTrig, distanceEcho);
  pinMode(distanceTrig1,OUTPUT);
  pinMode(distanceEcho1,INPUT);
  pinMode(distanceTrig2,OUTPUT);
  pinMode(distanceEcho2,INPUT);
  pinMode(4,INPUT);

  if(!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
  }
  else
  {
    Serial.println("test");
  }
  
  dht.begin();


  MQ9.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ9.setA(1000.5); MQ9.setB(-2.186); // Configurate the ecuation values to get LPG concentration
  MQ9.init(); 
  
  //if (!ps.init())
  //{
    //Serial.println("Failed to autodetect pressure sensor!");
    //while (1);
  //}

  //ps.enableDefault();

  
  
}

char mess[64];

char* getDistanceMess(int distanceEcho, int distanceTrig){
  //double* distances = HCSR04.measureDistanceCm();

  digitalWrite(distanceTrig, LOW);
  delayMicroseconds(5);
  digitalWrite(distanceTrig, HIGH);
  delayMicroseconds(15);
  digitalWrite(distanceTrig, LOW);

  long resultIn = 0;
  resultIn = pulseIn(distanceEcho, HIGH);
  resultIn /= 58;
  sprintf(mess, "dist: %d cm", resultIn);
  //sprintf(mess, "dist: %d cm", distances[0]);

  return mess;
}
float getDistance(int distanceEcho, int distanceTrig){
  //double* distances = HCSR04.measureDistanceCm();

  digitalWrite(distanceTrig, LOW);
  delayMicroseconds(5);
  digitalWrite(distanceTrig, HIGH);
  delayMicroseconds(15);
  digitalWrite(distanceTrig, LOW);

  float resultIn = 0;
  resultIn = pulseIn(distanceEcho, HIGH);
  resultIn /= 58;
  //sprintf(mess, "dist: %d cm", resultIn);
  //sprintf(mess, "dist: %d cm", distances[0]);

  return resultIn;
}

char tempMess[64];

char* getTempMess(){
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  sprintf(tempMess, "Humidity: %f , °C %f", h,t);

  return tempMess;
}

char pressureMess[64];
char* getPressureMess(){
  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);
  float temperature = ps.readTemperatureC();

  sprintf(pressureMess, "p: %f , inhPa\ta: %f m\t %f C", pressure, altitude, temperature);

  return pressureMess;
}

void getGasSensorData(){
  MQ9.update(); // Update data, the arduino will be read the voltage on the analog pin
  MQ9.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
  MQ9.serialDebug(); // Will print the table on the serial port
}

void Forward(int speed){
  analogWrite(IA1,speed, 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,speed, 100, 10, 0);
  analogWrite(IB2,0, 100, 10, 0);
}
void ForwardWithTurning(int baseSpeed,float turnParameter){
  analogWrite(IA1,baseSpeed *(0.5 - turnParameter), 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,0, 100, 10, 0);
  analogWrite(IB2,baseSpeed *(0.5 + turnParameter), 100, 10, 0);
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

float turningParameter = 0.0;

void loop() {
  // put your main code here, to run repeatedly:
  //double* distances = HCSR04.measureDistanceCm();

  float left = getDistance(distanceEcho1,distanceTrig1);
  float front = getDistance(distanceEcho2,distanceTrig2);
  
  Serial.print("left ");
  Serial.println(left);
  Serial.print("front ");
  Serial.println(front);

  Serial.println(getTempMess());

  Serial.println(getPressureMess());

  getGasSensorData();


   ForwardWithTurning(1024,turningParameter);
  //Forward(512);
  //delay(3000);
  //Stop();

  if(front > 20.0){
    //skręt w prawo
    if(left < 10.0){
      float diff = 10.0 - left;
      if(diff > 5){
        turningParameter = 0.5;
      }
      else{
        turningParameter = 0.5 * (5.0 - diff)/5.0;
      }
    }
    else if(left > 10.0){
      float diff = left - 10.0;
      if(diff > 10){
        turningParameter = -0.5;
      }
      else{
        turningParameter = -0.5 * (10.0 - diff)/10.0;
      }
    }
  }
  else{
    float diff =  20.0 - front;
      if(diff > 10.0){
        turningParameter = 0.5;
      }
      else{
        turningParameter = 0.5 * (5.0 - diff)/5.0;
      }
  }
  
  
  //delay(1000);
}
