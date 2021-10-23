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

int test = 0;
int step = 1;

void loop() {
  // put your main code here, to run repeatedly:
  //delay(1000);

  //do przodu
  bool pinStatus[4] = { true, false, true, false };
  
  analogWrite(IA1,1024, 100, 10, 0);
  analogWrite(IA2,0, 100, 10, 0);
  analogWrite(IB1,1024, 100, 10, 0);
  analogWrite(IB2,0, 100, 10, 0);

  /*test += step;
  if(test >= 1024 || test<= 0)
    step *= -1;
  delay(10);
  Serial.println(test);*/
  
}
