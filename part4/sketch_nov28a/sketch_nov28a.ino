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
#include <Fuzzy.h>

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

/************************Hardware Related Macros************************************/
#define         Board                   ("Arduino UNO")
#define         Pin                     (4)  //Analog input 4 of your arduino
/***********************Software Related Macros************************************/
#define         Type                    ("MQ-9") //MQ9
#define         Voltage_Resolution      (5)
#define         ADC_Bit_Resolution      (10) // For arduino UNO/MEGA/NANO
#define         RatioMQ9CleanAir        (9.6) //RS / R0 = 60 ppm 
MQUnifiedsensor MQ9(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

char* ssid = "MAXX_LAN"; //"Atlantis";
char* password = "debina23"; //"zaq1@WSX";

Fuzzy *fuzzy = new Fuzzy();

void setFuzzy(){
  
  FuzzyInput *left = new FuzzyInput(1);
  FuzzySet *near = new FuzzySet(0, 20, 20, 40);
  distance->addFuzzySet(small);
  FuzzySet *mnear = new FuzzySet(30, 50, 50, 70);
  distance->addFuzzySet(safe);
  FuzzySet *far = new FuzzySet(60, 80, 80, 80);
  distance->addFuzzySet(big);
  fuzzy->addFuzzyInput(distance);
  
}

void setup() {
  // put your setup code here, to run once:

  analogWrite(IA1,0, 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,0, 100, 10, 0);
  analogWrite(IB2,0, 100, 10, 0);
  
  Serial.begin(115200);
  Wire.begin(SDA0_Pin, SCL0_Pin);
  
  //HCSR04.begin(distanceTrig, distanceEcho);
  pinMode(distanceTrig1,OUTPUT);
  pinMode(distanceEcho1,INPUT);
  pinMode(distanceTrig2,OUTPUT);
  pinMode(distanceEcho2,INPUT);
  pinMode(distanceTrig3,OUTPUT);
  pinMode(distanceEcho3,INPUT);
  pinMode(4,INPUT);

  if(!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
  }
  
  dht.begin();


  MQ9.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ9.setA(1000.5); MQ9.setB(-2.186); // Configurate the ecuation values to get LPG concentration
  MQ9.init(); 
  
  WiFi.begin(ssid, password);

  //while (WiFi.status() != WL_CONNECTED) {
    //Serial.println("Connecting to Atlantis");
  //}
  
  //Serial.println("Connected to the Atlantis");
  IPAddress ip = WiFi.localIP();
  //Serial.print("IP Address: ");
  //Serial.println(ip);

  Serial.println("turningParameter");

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

  sprintf(pressureMess, "p:%f,inhPa\ta:%fm\t%fC", pressure, altitude, temperature);

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
  analogWrite(IB1,baseSpeed *(0.5 + turnParameter), 100, 10, 0);
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



void WiFiGet(char mess[]){
  if ((WiFi.status() == WL_CONNECTED))
  {
    HTTPClient http;
    //http.begin("http://51.158.163.165/api/devices");
    String wifiUrl = "http://192.168.0.99:3000/?message=";
    wifiUrl = wifiUrl + mess;
    //delay(100);
    //Serial.println(wifiUrl);
    http.begin(wifiUrl);
    int httpCode = http.GET();
    http.end();
  }
}
void WiFiGet(String mess){
  if ((WiFi.status() == WL_CONNECTED))
  {
    HTTPClient http;
    //http.begin("http://51.158.163.165/api/devices");
    String wifiUrl = "http://192.168.0.99:3000/?message=";
    wifiUrl = wifiUrl + mess;
    //delay(100);
    //Serial.println(wifiUrl);
    http.begin(wifiUrl);
    int httpCode = http.GET();
    http.end();
  }
}

float medium(float* arr, int count){
  float sum = 0.0;
  for(int i=0;i< count;i++){
    sum += arr[i];
  }
  return sum / count;
}

void writeRecord(float* arr, int count, float value){
  for(int i=0;i< count-1;i++){
    arr[i] = arr[i+1];
  }
  arr[count-1] = value;
}

float turningParameter = 0.0;

float leftBorder = 2.0;
float leftStandardDistance = 10.0;
float rightBorder = 20.0;
float frontBorder = 5.0;
float frontStandardDistance = 10.0;

float lastRecordsFront[] = {0.0, 0.0, 0.0, 0.0, 0.0 };
float lastRecordsLeft[] = {0.0, 0.0, 0.0, 0.0, 0.0 };

void loop() {
  // put your main code here, to run repeatedly:
  //double* distances = HCSR04.measureDistanceCm();

  float left = getDistance(distanceEcho1,distanceTrig1);
  float front = getDistance(distanceEcho2,distanceTrig2);
  float right = getDistance(distanceEcho3,distanceTrig3);
  
  //Serial.print("left ");
  //Serial.println(left);
  //Serial.print("front ");
  //Serial.println(front);
  //Serial.print("right ");
  //Serial.println(right);

  //Serial.println(getTempMess());

  //Serial.println(getPressureMess());

  //getGasSensorData();

  //char distanceMess[] = "left: %f , front: %f , right: %f ";
  //sprintf(distanceMess, "left:%5.2f,front:%5.2f,right:%5.2f", left, front, right);
  
  //WiFiGet(distanceMess);
  //WiFiGet(tempMess);
  //String pressMess = getPressureMess();
  //WiFiGet(pressureMess);

  //Forward(512);
  //delay(3000);
  //Stop();
    
  /*writeRecord(lastRecordsFront,5,front);
  writeRecord(lastRecordsLeft,5,left);
  front = medium(lastRecordsFront,5);
  left = medium(lastRecordsLeft,5);
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
  else{
    float diff =  frontStandardDistance - front;
      if(front < frontBorder){
        back = true;
        //turningParameter = -1.0;
      }
      else{
        turningParameter = 0.5 * (front - frontBorder)/(frontStandardDistance - frontBorder);
      }
  }
  char turningParameterMess[] = "";
  //sprintf(turningParameterMess, "turningParameter: %f", turningParameter);
  //WiFiGet(turningParameterMess);
  
  if(back){
    Backward(768);
    delay(1000);
    back = false;
  }
  else{
    ForwardWithTurning(768,turningParameter);
  }*/


  
  //Serial.print(front);
  //Serial.print(" ");
  //Serial.print(left);
  //Serial.print(" ");
  //Serial.println(turningParameter);
  
  //delay(10);
}
