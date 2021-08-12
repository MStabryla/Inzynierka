void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  
  pinMode(11,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(6,OUTPUT);
  
  digitalWrite(11,LOW);
  digitalWrite(10,HIGH);
  digitalWrite(9,HIGH);
  digitalWrite(6,LOW);
}

String mess;
bool pinStatus[4] = { false, false, false, false };

void loop() {
  if(Serial.available() > 0) {
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
  }

}
