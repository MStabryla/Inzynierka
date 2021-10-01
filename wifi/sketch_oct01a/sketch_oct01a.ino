#include <WiFi.h>
#include "HTTPClient.h"
 
char* ssid = "Atlantis";
char* password = "zaq1@WSX";

const int IA1 = 5;
const int IA2 = 4;
const int IB1 = 2;
const int IB2 = 15;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  delay(10000);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Atlantis");
  }

  Serial.println("Connected to the Atlantis");
  
  pinMode(IA1,OUTPUT);
  pinMode(IA2,OUTPUT);
  pinMode(IB1,OUTPUT);
  pinMode(IB2,OUTPUT);
  
  digitalWrite(IA1,LOW);
  digitalWrite(IA2,LOW);
  digitalWrite(IB1,LOW);
  digitalWrite(IB2,LOW);
}

String mess;

void loop() {
  if ((WiFi.status() == WL_CONNECTED))
  {
    HTTPClient http;
    http.begin("http://51.158.163.165/api/devices");
    int httpCode = http.GET();
    if( httpCode > 0){
      String response = http.getString();
      Serial.println(httpCode);
      Serial.println(response);
    }
    else {
      Serial.println("Error on HTTP request");
    }
 
    http.end(); 
  }
  delay(5000);
}
