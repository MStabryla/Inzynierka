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

#include "fis_header.h"

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

// Fuzzy:
// Number of inputs to the fuzzy inference system
const int fis_gcI = 3;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 2;
// Number of rules to the fuzzy inference system
const int fis_gcR = 10;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

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

void FuzzyEval(float left, float front, float right){
    // Read Input: left
    g_fisInput[0] = left;
    // Read Input: front
    g_fisInput[1] = front;
    // Read Input: right
    g_fisInput[2] = right;

    g_fisOutput[0] = 0;
    g_fisOutput[1] = 0;

    fis_evaluate();

    //output in g_fisOutput[0] & g_fisOutput[1]
}

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



//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Gaussian Member Function
FIS_TYPE fis_gauss2mf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE c1 = p[1], c2 = p[3];
    FIS_TYPE t1 = ((x >= c1) ? 1.0 : fis_gaussmf(x, p));
    FIS_TYPE t2 = ((x <= c2) ? 1.0 : fis_gaussmf(x, p + 2));
    return (t1*t2);
}

// Gaussian Member Function
FIS_TYPE fis_gaussmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE s = p[0], c = p[1];
    FIS_TYPE t = (x - c) / s;
    return exp(-(t * t) / 2);
}

FIS_TYPE fis_prod(FIS_TYPE a, FIS_TYPE b)
{
    return (a * b);
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    return max(a, b);
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    return min(a, b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] =
{
    fis_gauss2mf, fis_gaussmf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 3, 3, 3 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 5, 2 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { 5, 0, 2.5, 5 };
FIS_TYPE fis_gMFI0Coeff2[] = { 3, 10 };
FIS_TYPE fis_gMFI0Coeff3[] = { 3, 20, 20, 20 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3 };
FIS_TYPE fis_gMFI1Coeff1[] = { 5, 0, 5, 15 };
FIS_TYPE fis_gMFI1Coeff2[] = { 7.5, 30 };
FIS_TYPE fis_gMFI1Coeff3[] = { 7.5, 49.75, 50, 49.75 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3 };
FIS_TYPE fis_gMFI2Coeff1[] = { 5, 0, 2.5, 5 };
FIS_TYPE fis_gMFI2Coeff2[] = { 3, 10 };
FIS_TYPE fis_gMFI2Coeff3[] = { 5, 20, 20, 20 };
FIS_TYPE* fis_gMFI2Coeff[] = { fis_gMFI2Coeff1, fis_gMFI2Coeff2, fis_gMFI2Coeff3 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff, fis_gMFI2Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 0.2, -0.5 };
FIS_TYPE fis_gMFO0Coeff2[] = { 0.2, -0.25 };
FIS_TYPE fis_gMFO0Coeff3[] = { 0.2, 0 };
FIS_TYPE fis_gMFO0Coeff4[] = { 0.2, 0.197354497354497 };
FIS_TYPE fis_gMFO0Coeff5[] = { 0.2, 0.5 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3, fis_gMFO0Coeff4, fis_gMFO0Coeff5 };
FIS_TYPE fis_gMFO1Coeff1[] = { 0.5, -1 };
FIS_TYPE fis_gMFO1Coeff2[] = { 0.5, 1 };
FIS_TYPE* fis_gMFO1Coeff[] = { fis_gMFO1Coeff1, fis_gMFO1Coeff2 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff, fis_gMFO1Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 1, 0 };
int fis_gMFI1[] = { 0, 1, 0 };
int fis_gMFI2[] = { 0, 1, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1, fis_gMFI2};

// Output membership function set
int fis_gMFO0[] = { 1, 1, 1, 1, 1 };
int fis_gMFO1[] = { 1, 1 };
int* fis_gMFO[] = { fis_gMFO0, fis_gMFO1};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 0, 3, 0 };
int fis_gRI1[] = { 0, 2, 0 };
int fis_gRI2[] = { 0, 1, 0 };
int fis_gRI3[] = { 0, 3, 0 };
int fis_gRI4[] = { 2, 2, -1 };
int fis_gRI5[] = { -1, 2, 2 };
int fis_gRI6[] = { 2, 1, -1 };
int fis_gRI7[] = { -1, 1, 2 };
int fis_gRI8[] = { 1, 1, -1 };
int fis_gRI9[] = { -1, 1, 1 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9 };

// Rule Outputs
int fis_gRO0[] = { 0, 2 };
int fis_gRO1[] = { 0, 2 };
int fis_gRO2[] = { 0, 1 };
int fis_gRO3[] = { 3, 0 };
int fis_gRO4[] = { 4, 0 };
int fis_gRO5[] = { 2, 0 };
int fis_gRO6[] = { 5, 0 };
int fis_gRO7[] = { 1, 0 };
int fis_gRO8[] = { 5, 0 };
int fis_gRO9[] = { 1, 0 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0, 0, 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 20, 50, 20 };

// Output range Min
FIS_TYPE fis_gOMin[] = { -0.5, -1 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 0.5, 1 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o)
{
    FIS_TYPE mfOut;
    int r;

    for (r = 0; r < fis_gcR; ++r)
    {
        int index = fis_gRO[r][o];
        if (index > 0)
        {
            index = index - 1;
            mfOut = (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else if (index < 0)
        {
            index = -index - 1;
            mfOut = 1 - (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else
        {
            mfOut = 0;
        }

        fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
    }
    return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}

FIS_TYPE fis_defuzz_bisector(FIS_TYPE** fuzzyRuleSet, int o)
{
    FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
    FIS_TYPE dist, area = 0;
    int i;

    /* find the total area */
    area = 0;
    for (i = 0; i < FIS_RESOLUSION; ++i)
    {
        dist = fis_gOMin[o] + (step * i);
        area += step * fis_MF_out(fuzzyRuleSet, dist, o);
    }
    if (area == 0)
    {
        return ((fis_gOMax[o] + fis_gOMin[o]) / 2);
    }

    area = area / 2;
    FIS_TYPE area2 = 0;
    for (i = 0; area2<area; ++i)
    {
        dist = fis_gOMin[o] + (step * i);
        area2 += step * fis_MF_out(fuzzyRuleSet, dist, o);
    }
    return (dist);
}

//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput2[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, fuzzyInput2, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0, 0, 0 };
    FIS_TYPE fuzzyOutput1[] = { 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, fuzzyOutput1, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = 1;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_prod(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = FIS_MIN;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = fis_defuzz_bisector(fuzzyRuleSet, o);
        }
    }
}
