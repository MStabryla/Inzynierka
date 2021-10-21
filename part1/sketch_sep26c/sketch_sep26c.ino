#include "Arduino.h"
#include <analogWrite.h>

const int IA1 = 5;
const int IA2 = 4;
const int IB1 = 2;
const int IB2 = 15;

void change(bool *array, bool *array2, int length)
{
    if (array == NULL || array2 == NULL)
        return;
    for (int i = 0 ; i < length ; i++)
        array[i] = array2[i];

    //digitalWrite(IA1,array2[0]);
    //digitalWrite(IA2,array2[1]);
    //digitalWrite(IB1,array2[2]);
    //digitalWrite(IB2,array2[3]);
}



void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  
  pinMode(IA1,OUTPUT);
  pinMode(IA2,OUTPUT);
  pinMode(IB1,OUTPUT);
  pinMode(IB2,OUTPUT);
  
  digitalWrite(IA1,HIGH);
  digitalWrite(IA2,HIGH);
  digitalWrite(IB1,HIGH);
  digitalWrite(IB2,HIGH);
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

  delay(1000);

  bool pinStatus[4] = { true, false, true, false };
  //change(pinStatus,pinStatus,4);
  digitalWrite(IA1,pinStatus[0]);
  digitalWrite(IA2,pinStatus[1]);
  digitalWrite(IB1,pinStatus[2]);
  digitalWrite(IB2,pinStatus[3]);

  Serial.println("Zmiana");
  delay(3000);

  bool pinStatus2[4] = { true, false, true, false };
  change(pinStatus,pinStatus2,4);

  analogWrite(IA1,pinStatus[0] ? 0.5 : 0);
  digitalWrite(IA2,pinStatus[1]);
  analogWrite(IB1,pinStatus[2]? 0.5 : 0);
  digitalWrite(IB2,pinStatus[3]);

  Serial.println("Zmiana 2");
  
  delay(3000);

  /*bool pinStatus2[4] = { true, true, true, true };
  change(pinStatus,pinStatus2,4);

  digitalWrite(11,pinStatus[0]);
  analogWrite(10,pinStatus[1] ? 0.5 : 0);
  digitalWrite(9,pinStatus[2]);
  analogWrite(6,pinStatus[3] ? 0.5 : 0);

  delay(3000);

  //backward
  
  bool pinStatus3[4] = { false, true, false, true };
  change(pinStatus,pinStatus3,4);

  //digitalWrite(IA1,pinStatus[0]);
  //analogWrite(IA2,pinStatus[1] ? 1024 : 0);
  //digitalWrite(IB1,pinStatus[2]);
  //analogWrite(IB2,pinStatus[3] ? 1024 : 0);

  Serial.println("Zmiana 3");
  delay(3000);

  bool pinStatus4[4] = { false, true, false, true };
  change(pinStatus,pinStatus4,4);

  //digitalWrite(IA1,pinStatus[0]);
  //analogWrite(IA2,pinStatus[1] ? 1024 : 0);
  //digitalWrite(IB1,pinStatus[2]);
  //analogWrite(IB2,pinStatus[3] ? 1024 : 0);

  Serial.println("Zmiana 4");
  delay(3000);

  /*bool pinStatus4[4] = { true, true, true, true };
  change(pinStatus,pinStatus4,4);

  analogWrite(11,pinStatus[0] ? 0.5 : 0);
  digitalWrite(10,pinStatus[1]);
  analogWrite(9,pinStatus[2]? 0.5 : 0);
  digitalWrite(6,pinStatus[3]);

  delay(3000);*/

}
