#include <Wire.h>
#include "Adafruit_MPRLS.h"
#include <PID_v1.h>

// Motor Driver Pins for 1st chamber ( sensor 0)
#define RPWM1 5
#define LPWM1 6
#define REN1 8
#define LEN1 9

// Motor driver pins for 2nd chamber (using this one-sensor 1-2nd pump) 
#define RPWM2 2
#define LPWM2 3
#define REN2 12
#define LEN2 13

// Motor driver pins for 3rd chamber (sensor 2- 3rd pump)
#define RPWM3 7
#define LPWM3 4
#define REN3 22
#define LEN3 24

// Motor driver pins for 4th chamber (this one)
#define RPWM4 10
#define LPWM4 11
#define REN4 26
#define LEN4 28

// Pressure Sensor and Multiplexer Setup
#define MPRLS_I2C_ADDRESS 0x18
#define RESET_PIN -1
#define EOC_PIN -1
byte tcaI2CAddress[] = {0x70};
byte numberOfTCAs = 1;
byte numberOfDevicesPerTCA = 4;
const int numberOfDevices = 4;

Adafruit_MPRLS mpr[numberOfDevices];

// PID Parameters
double Setpoint[numberOfDevices], Input[numberOfDevices], Output[numberOfDevices];
double Kp=20, Ki=7, Kd=2; // Placeholder values, need tuning
PID myPID[numberOfDevices] = {PID(&Input[0], &Output[0], &Setpoint[0], Kp, Ki, Kd, DIRECT), PID(&Input[1], &Output[1], &Setpoint[1], Kp, Ki, Kd, DIRECT), PID(&Input[2], &Output[2], &Setpoint[2], Kp, Ki, Kd, DIRECT), PID(&Input[3], &Output[3], &Setpoint[3], Kp, Ki, Kd, DIRECT)};
// PID Setup
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // Initialize Serial Communication
  Serial.begin(9600);
  while (!Serial); // Wait for serial port to connect

  // Initialize Motor Driver Pins for all four chambers 
  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM1, OUTPUT);
  pinMode(LEN1, OUTPUT);
  pinMode(REN1, OUTPUT);
  digitalWrite(REN1, HIGH);
  digitalWrite(LEN1, HIGH);

  pinMode(RPWM2, OUTPUT);
  pinMode(LPWM2, OUTPUT);
  pinMode(LEN2, OUTPUT);
  pinMode(REN2, OUTPUT);
  digitalWrite(REN2, HIGH);
  digitalWrite(LEN2, HIGH);

  pinMode(RPWM3, OUTPUT);
  pinMode(LPWM3, OUTPUT);
  pinMode(LEN3, OUTPUT);
  pinMode(REN3, OUTPUT);
  digitalWrite(REN3, HIGH);
  digitalWrite(LEN3, HIGH);

  pinMode(RPWM4, OUTPUT);
  pinMode(LPWM4, OUTPUT);
  pinMode(LEN4, OUTPUT);
  pinMode(REN4, OUTPUT);
  digitalWrite(REN4, HIGH);
  digitalWrite(LEN4, HIGH);

  // Initialize I2C for the multiplexer
  Wire.begin();

  // Initialize MPRLS sensors
  for (int i = 0; i < numberOfDevices; i++) {
    setupmpr(i);
    myPID[i].SetMode(AUTOMATIC);
    myPID[i].SetOutputLimits(0, 255); // PWM limits
  }

  // Get Desired Pressure from User for each chamber 
  for(int i = 0; i<numberOfDevices; i++){
      Serial.println("Enter desired pressure (PSI):");
      Serial.print(i);
      Serial.println(":");

      while (!Serial.available()){
        // wait for input 
      }
      Setpoint[i] = Serial.parseFloat(); // read the first float number 
      while(Serial.available() > 0){
        Serial.read();
      }

      Serial.print(i);
      Serial.print("Set to: ");
      Serial.println(Setpoint[i], 2);
  }
  
}

void loop() {
  for (int i = 0; i < numberOfDevices; i++) {
    byte tca = setTCAAndChannel(i);
    
    // Read the pressure from the sensor
    Input[i] = mpr[i].readPressure() / 68.947572932; // Convert to PSI

    // PID Computation
    myPID[i].Compute();

    // Control the Motor Speed based on PID Output for each chamber 
    switch(i){
      case 0:
        analogWrite(RPWM1, Output[i]);
        analogWrite(LPWM1, 0);
        break;
      case 1:
        analogWrite(RPWM2, Output[i]);
        analogWrite(LPWM2, 0);
        break;
      case 2:
        analogWrite(RPWM3, Output[i]);
        analogWrite(LPWM3, 0);
        break;
      case 3:
        analogWrite(RPWM4, Output[i]);
        analogWrite(LPWM4, 0);
        break;
    }
    disableTCA(tca);

    // Print Pressure Readings
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" Pressure: ");
    Serial.print(Input[i]);
    Serial.println(" PSI");
  }

  Serial.print("PID Output: ");
  for(int i = 0; i < numberOfDevices; i++) {
    Serial.print(Output[i]); // Print each element of the array
    if(i < numberOfDevices - 1) Serial.print(", "); // Separate with commas
  }
  Serial.println();

  delay(50); // Update at regular intervals
}

void setupmpr(byte i) {
  byte tca = setTCAAndChannel(i);
  
  if (!mpr[i].begin()) {
    Serial.print("MPRLS No ");
    Serial.print(i);
    Serial.println(" not found, check wiring!");
  }
  
  disableTCA(tca);
}

byte setTCAAndChannel(byte i) {
  byte tca = i / numberOfTCAs;
  byte channel = i % numberOfDevicesPerTCA;
  
  Wire.beginTransmission(tcaI2CAddress[tca]);
  Wire.write(1 << channel);
  Wire.endTransmission();
  
  return tca;
}

void disableTCA(byte tca) {
  Wire.beginTransmission(tcaI2CAddress[tca]);
  Wire.write(0);
  Wire.endTransmission();
}
