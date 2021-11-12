//#include <HCSR04.h>
#include <string.h>
#include "DHT.h"
#include <Wire.h>
#include <LPS.h>
#include <MQUnifiedsensor.h>

#define DHTTYPE DHT11


const int IA1 = 27;
const int IA2 = 14;
const int IB1 = 12;
const int IB2 = 13;

const int distanceEcho = 18;
const int distanceTrig = 19;
const int tempInput = 5;

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
  Wire.begin();

  //HCSR04.begin(distanceTrig, distanceEcho);
  pinMode(distanceTrig,OUTPUT);
  pinMode(distanceEcho,INPUT);
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

char pressureMess[64];
char* getPressureMess(){
  float pressure = ps.readPressureInchesHg();
  float altitude = ps.pressureToAltitudeFeet(pressure);
  float temperature = ps.readTemperatureF();

  sprintf(pressureMess, "p: %f , inHg\ta: %f ft", pressure, altitude);

  return pressureMess;
}

void getGasSensorData(){
  MQ9.update(); // Update data, the arduino will be read the voltage on the analog pin
  MQ9.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
  MQ9.serialDebug(); // Will print the table on the serial port
}

void loop() {
  // put your main code here, to run repeatedly:
  //double* distances = HCSR04.measureDistanceCm();

  //Serial.println(getDistanceMess());

  //Serial.println(getTempMess());

  //Serial.println(getPressureMess());

  getGasSensorData();
  
  delay(1000);
}
