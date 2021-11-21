#include <Wire.h>
#include <LPS.h>

LPS ps;

#define I2C_Freq 100000
 
#define SDA0_Pin 22
#define SCL0_Pin 23

void setup()
{
  //I2C_0.begin(SDA_0 , SCL_0 );
  Serial.begin(115200);
  delay(1000);
  
  Wire.begin(SDA0_Pin, SCL0_Pin);

  if (!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }

  ps.enableDefault();
}

void loop()
{
  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);
  float temperature = ps.readTemperatureC();
  
  Serial.print("p: ");
  Serial.print(pressure);
  Serial.print(" mbar\ta: ");
  Serial.print(altitude);
  Serial.print(" m\tt: ");
  Serial.print(temperature);
  Serial.println(" deg C");

  delay(100);
}
