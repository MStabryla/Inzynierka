//#include <HCSR04.h>
#include "Arduino.h"
#include "analogWrite.h"
#include <string.h>
#include "DHT.h"
#include <Wire.h>
#include <LPS.h>
#include <MQUnifiedsensor.h>
#include <WiFi.h>
#include "HTTPClient.h"

#define DHTTYPE DHT11


const int IA1 = 13;
const int IA2 = 12;
const int IB2 = 14;
const int IB1 = 27;


const int distanceEcho1 = 19;
const int distanceTrig1 = 21;
const int distanceEcho2 = 25;
const int distanceTrig2 = 26;
const int distanceEcho3 = 32;
const int distanceTrig3 = 33;
const int tempInput = 18;
const int gasSensor = 4;
const int SDA0_Pin = 22;
const int SCL0_Pin = 23;

DHT dht(tempInput, DHTTYPE);

LPS ps;

//Dane początkowe dla mikrokontrolera ESP-32
/************************Hardware Related Macros************************************/
#define         Board                   ("ESP-32")
#define         Pin                     (34)  //Analog input 4 of your arduino
/***********************Software Related Macros************************************/
#define         Type                    ("MQ-9") //MQ9
#define         Voltage_Resolution      (3.3)
#define         ADC_Bit_Resolution      (12) // For arduino UNO/MEGA/NANO
#define         RatioMQ9CleanAir        (9.6) //RS / R0 = 60 ppm 
MQUnifiedsensor MQ9(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

char* ssid = "MAXX_LAN"; //"Atlantis";
char* password = "debina23"; //"zaq1@WSX";

void setup() {
  // put your setup code here, to run once:

  analogWrite(IA1,0, 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,0, 100, 10, 0);
  analogWrite(IB2,0, 100, 10, 0);
  
  Serial.begin(115200);
  //ustaweinie pinów do komunikacji za pomocą interfejsu I2C
  Wire.begin(SDA0_Pin, SCL0_Pin);
  
  pinMode(distanceTrig1,OUTPUT);
  pinMode(distanceEcho1,INPUT);
  pinMode(distanceTrig2,OUTPUT);
  pinMode(distanceEcho2,INPUT);
  pinMode(distanceTrig3,OUTPUT);
  pinMode(distanceEcho3,INPUT);
  pinMode(Pin,INPUT);

  if(!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
  }
  ps.enableDefault();
  
  dht.begin();

  MQ9.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ9.init(); 
  
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ9.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ9.calibrate(RatioMQ9CleanAir);
  }
  MQ9.setR0(calcR0/10);
  
  WiFi.begin(ssid, password);
  IPAddress ip = WiFi.localIP();
  digitalWrite(IA1,LOW);
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
  return mess;
}
// distanceEcho oraz distanceTrig to numery pinów przypisane do danego czujnika
float getDistance(int distanceEcho, int distanceTrig){
  // aktywowanie czujnika ultradźwiękowego
  digitalWrite(distanceTrig, LOW);
  delayMicroseconds(5);
  digitalWrite(distanceTrig, HIGH);
  delayMicroseconds(15);
  digitalWrite(distanceTrig, LOW);
  float resultIn = 0;
  //pobranie czasu, przez jaki utrzymuje sie stan wysoki
  resultIn = pulseIn(distanceEcho, HIGH);
  // podzielenie przez 58 da wartość w cm
  resultIn /= 58;
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
  // czujnik pozwala też na ustalenie wysokości w metrach n.p.m.
  float altitude = ps.pressureToAltitudeMeters(pressure);
  sprintf(pressureMess, "p:%f,inhPa\ta:%fm\t", pressure, altitude);
  return pressureMess;
}

float getGasSensorData(float a, float b){
  MQ9.update(); // Update data, the arduino will be read the voltage on the analog pin
  MQ9.setA(a);MQ9.setB(b);
  return MQ9.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
}

void Forward(int speed){
  analogWrite(IA1,speed, 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,speed, 100, 10, 0);
  analogWrite(IB2,0, 100, 10, 0);
}
//Jazda do przodu z ustalonym skrętem  na podstawiem turnParameter
void ForwardWithTurning(int baseSpeed,float turnParameter){
  analogWrite(IA1,baseSpeed *(0.5 - turnParameter), 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,baseSpeed *(0.5 + turnParameter), 100, 10, 0);
  analogWrite(IB2,0, 100, 10, 0);
}
//Jazda do tyłu z ustalonym skrętem na podstawiem turnParameter
void BackwardWithTurning(int baseSpeed,float turnParameter){
  analogWrite(IA1,0, 100, 10, 0);
  analogWrite(IA2,baseSpeed *(0.5 + turnParameter), 100, 10, 0);
  analogWrite(IB1,0, 100, 10, 0);
  analogWrite(IB2,baseSpeed *(0.5 - turnParameter), 100, 10, 0);
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

String wifiUrl = "http://192.168.0.99:3000/";
String prodUrl = "http://93.180.174.50:180/api/devices/122/add-values-by-property-id";

void WiFiGet(char mess[]){
  if ((WiFi.status() == WL_CONNECTED))
  {
    HTTPClient http;
    wifiUrl = wifiUrl + mess;
    http.begin(wifiUrl);
    int httpCode = http.GET();
    http.end();
  }
}
void WiFiGet(String mess){
  if ((WiFi.status() == WL_CONNECTED))
  {
    HTTPClient http;
    wifiUrl = wifiUrl + mess;
    http.begin(wifiUrl);
    int httpCode = http.GET();
    http.end();
  }
}
char POSTObject[64];
void WiFiPost(String property,float val){
  if ((WiFi.status() == WL_CONNECTED))
  {
    //Utworzenie struktury http
    HTTPClient http;
    //zapisanie danych w postaci JSON, przy pomocy metod obsługi ciągu znaków
    sprintf(POSTObject, "{ \"propertyId\":\"%s\" , \"val\":\"%f\" }", property, val);
    http.begin(wifiUrl);
    //oznaczenie, zę dane są w postaci JSON
    http.addHeader("Content-Type","application/json");
    int httpCode = http.POST(POSTObject);
    http.end();
    if( httpCode > 200){
      //wykonanie w przypadku poprawnej odpowiedzi, dla potrzeb tego robota nie było potrzeby upewniania się, czy dane zapisały się poprawnie
    }
  }
}
void WiFiPostProd(String property,float val){
  if ((WiFi.status() == WL_CONNECTED))
  {
    HTTPClient http;
    
    sprintf(POSTObject, "{ \"propertyId\":\"%s\" , \"val\":\"%f\" }", property, val);
    http.begin(prodUrl);
    http.addHeader("Content-Type","application/json");
    int httpCode = http.POST(POSTObject);
    http.end();
    if( httpCode > 0){

    }
  }
}
//wyznaczanie średniej z zakresu
float medium(float* arr, int count){
  float sum = 0.0;
  for(int i=0;i< count;i++){
    sum += arr[i];
  }
  return sum / count;
}
//dodawanie nowej informacji do tablicy
void writeRecord(float* arr, int count, float value){
  for(int i=0;i< count-1;i++){
    arr[i] = arr[i+1];
  }
  arr[count-1] = value;
}

float turningParameter = 0.0;

//zdefiniowane granice zachowań w algorytmie wall-following
float leftBorder = 5.0;
float leftStandardDistance = 20.0;
float rightBorder = 40.0;
float frontMax = 10.0;
float frontBorder = 20.0;
float frontStandardDistance = 30.0;

//zapis ostatnio zmierzonych odległości 
float lastRecordsFront[] = {0.0, 0.0, 0.0, 0.0, 0.0 };
float lastRecordsLeft[] = {0.0, 0.0, 0.0, 0.0, 0.0 };

void loop() {
  float left = getDistance(distanceEcho1,distanceTrig1);
  float front = getDistance(distanceEcho2,distanceTrig2);
  float right = getDistance(distanceEcho3,distanceTrig3);

  WiFiPostProd("128",dht.readTemperature());
  WiFiPostProd("129",dht.readHumidity());
  WiFiPostProd("130",ps.readPressureMillibars());
  
  float co = getGasSensorData(599.65,-2.244); // tlenek węgla
  float lpg = getGasSensorData(1000.5,-2.186); // gaz petrochemiczny, butan
  float ch4 = getGasSensorData(4269.6,-2.648); // Metan
  Serial.print(co); Serial.print(" | "); Serial.print(lpg); Serial.print(" | "); Serial.println(ch4);

  writeRecord(lastRecordsFront,5,front);
  writeRecord(lastRecordsLeft,5,left);
  front = medium(lastRecordsFront,5);
  left = medium(lastRecordsLeft,5);
  //wall-following
  bool back = false;
  if(front > frontStandardDistance){
    //skręt w prawo
    if(left < leftStandardDistance){
      float diff = leftStandardDistance - left;
      if(diff > leftStandardDistance - leftBorder){
        turningParameter = 0.5;
      }
      else{
        turningParameter = 0.5 * (leftStandardDistance - left)/(leftStandardDistance - leftBorder);
      }
    }
    //skręt w lewo
    else if(left > leftStandardDistance){
      float diff = left - leftStandardDistance;
      if(left > rightBorder){
        turningParameter = -0.5;
      }
      else{
        turningParameter = -0.5 * (rightBorder - leftStandardDistance + (diff - leftStandardDistance))/(rightBorder - leftStandardDistance);
      }
    }
  }
  //cofanie się z zachowaniem skrętu
  else{
    float diff =  frontStandardDistance - front;
      if(front < frontMax){
        back = true;
      }
      else if(front < frontBorder){
        turningParameter = 0.5;
      }
      else{
        turningParameter = 0.5 * (frontStandardDistance - front)/(frontStandardDistance - frontBorder);
      }
  }
  //cofanie musi trwać przynajmniej sekundę, by robot wydostał się z zaklinowanego miejsca
  if(back){
    BackwardWithTurning(1024,turningParameter);
    delay(1000);
    back = false;
  }
  ForwardWithTurning(1024,turningParameter);

}
