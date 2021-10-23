#include "Arduino.h"
#include "analogWrite.h"

void change(bool *array, bool *array2, int length)
{
    if (array == NULL || array2 == NULL)
        return;
    for (int i = 0 ; i < length ; i++)
        array[i] = array2[i];
}

const int IA1 = 5;
const int IA2 = 4;
const int IB1 = 2;
const int IB2 = 15;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  
  //pinMode(IA1,OUTPUT);
  //pinMode(IA2,OUTPUT);
  //pinMode(IB1,OUTPUT);
  //pinMode(IB2,OUTPUT);
  
  //digitalWrite(IA1,HIGH);
  //digitalWrite(IA2,HIGH);
  //digitalWrite(IB1,HIGH);
  //digitalWrite(IB2,HIGH);
}

String mess;

void setForward(int seconds,int speed,bool* pinStatus){
  int i=0,toSecond = 0;
  while(i < seconds*1000){
    while(toSecond < 1000){
      digitalWrite(IA2,pinStatus[1]);
      digitalWrite(IB2,pinStatus[3]);
      int tempSpeed = 0;
      while(tempSpeed < speed){
        digitalWrite(IA2,HIGH);
        digitalWrite(IB2,HIGH);
        tempSpeed++;
      }
      digitalWrite(IA2,LOW);
      digitalWrite(IB2,LOW);
      delay(1);
      toSecond++;
      i++;
    }
    toSecond = 0;
  }
}

void loop() {
  /*if(Serial.available() > 0) {
    mess = Serial.readStringUntil('\n');
    if(mess == "11"){
        pinStatus[0] = !pinStatus[0];
        digitalWrite(11,pinStatus[0]);
        Serial.println(" 11 set to " +  pinStatus[0]);
    }
    else if(mess == "10"){
        pinStatus[1] = !pinStatus[1];
        digitalWrite(10,pinStatus[1]);
        Serial.println(" 10 set to " +  pinStatus[1]);
    }
    else if(mess == "9"){
        pinStatus[2] = !pinStatus[2];
        digitalWrite(9,pinStatus[2]);
        Serial.println(" 9 set to " +  pinStatus[2]);
    }
    else if(mess == "6"){
        pinStatus[3] = !pinStatus[3];
        digitalWrite(6,pinStatus[3]);
        Serial.println(" 6 set to " +  pinStatus[3]);
    }
  }*/

  //delay(1000);

  bool pinStatus[4] = { true, false, true, false };

  analogWrite(IA1,1024, 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,1024, 100, 10, 0);
  analogWrite(IB2,0, 100, 10, 0);

  //setForward(3,500,pinStatus);
  //Serial.println("Zmiana 1");
  delay(3000);
  printPinsStatus();
  
  bool pinStatus2[4] = { true, false, true, false };
  change(pinStatus,pinStatus2,4);

  analogWrite(IA1,512, 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,512, 100, 10, 0);
  analogWrite(IB2,0, 100, 10, 0);
  //setForward(3,64,pinStatus);
  //Serial.println("Zmiana 2");
  
  delay(3000);
  printPinsStatus();
  
  /*bool pinStatus2[4] = { true, true, true, true };
  change(pinStatus,pinStatus2,4);

  digitalWrite(11,pinStatus[0]);
  analogWrite(10,pinStatus[1] ? 0.5 : 0);
  digitalWrite(9,pinStatus[2]);
  analogWrite(6,pinStatus[3] ? 0.5 : 0);

  delay(3000);*/

  //backward
  
  bool pinStatus3[4] = { false, true, false, true };
  change(pinStatus,pinStatus3,4);

  analogWrite(IA1,0, 100, 10, 0);
  analogWrite(IA2,1024, 100, 10, 0);
  analogWrite(IB1,0, 100, 10, 0);
  analogWrite(IB2,1024, 100, 10, 0);

  //Serial.println("Zmiana 3");
  delay(3000);
  printPinsStatus();

  bool pinStatus4[4] = { false, true, false, true };
  change(pinStatus,pinStatus4,4);

  analogWrite(IA1,0, 100, 10, 0);
  analogWrite(IA2,341, 100, 10, 0);
  analogWrite(IB1,0, 100, 10, 0);
  analogWrite(IB2,341, 100, 10, 0);

  //Serial.println("Zmiana 4");
  delay(3000);
  printPinsStatus();

  /*bool pinStatus4[4] = { true, true, true, true };
  change(pinStatus,pinStatus4,4);

  analogWrite(11,pinStatus[0] ? 0.5 : 0);
  digitalWrite(10,pinStatus[1]);
  analogWrite(9,pinStatus[2]? 0.5 : 0);
  digitalWrite(6,pinStatus[3]);

  delay(3000);*/

}
