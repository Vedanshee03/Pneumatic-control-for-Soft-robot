#include "Adafruit_MPRLS.h"
#include<Wire.h>
#define MPRLS_I2C_ADDRESS   0x18
#define RESET_PIN  -1  
#define EOC_PIN    -1  

byte tcaI2CAddress[] = {0x70};
byte numberOfTCAs = 1;
byte numberOfDevicesPerTCA = 4;
const int numberOfDevices = 4;

Adafruit_MPRLS mpr[numberOfDevices];

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(1000);
  
  for(int i=0; i<numberOfDevices; i++){
    mpr[i] =  Adafruit_MPRLS(MPRLS_I2C_ADDRESS);
    mpr[i] = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
    setupmpr(i);
  }
}

void loop() {
  float voltage = 0.0;
  
  for(int i=0; i<numberOfDevices; i++){
    byte tca = setTCAAndChannel(i);
    voltage = mpr[i].readPressure();
    voltage = voltage/ 68.947572932;
    Serial.print("Pressure [  ], MPRLS No ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(voltage);
    disableTCA(tca);
  }
  Serial.println("****************************");  
  delay(1000);
}

byte setTCAAndChannel(byte i){
  byte tca = i/numberOfTCAs;
  byte channel = i%numberOfDevicesPerTCA;
  
  Wire.beginTransmission(tcaI2CAddress[tca]);
  Wire.write(1 << channel);
  Wire.endTransmission();

  return tca;
}

void disableTCA(byte tca){
  Wire.beginTransmission(tcaI2CAddress[tca]);
  Wire.write(0);
  Wire.endTransmission();  
}

void setupmpr(byte i){
  byte tca = setTCAAndChannel(i);
  
  if(!mpr[i].begin()){
    Serial.print("MPRLS No ");
    Serial.print(i);
    Serial.println(" not connected!");
  }
  disableTCA(tca); 
}