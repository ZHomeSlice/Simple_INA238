#include <Wire.h>
#include "Simple_Wire.h"
#include "Simple_INA238.h"

// Create an INA238 object
Simple_INA238 ina238;

// ESP32 Connection options:
#define SDA_PIN 14 // I am using an ESP32 and chose GPIO pin 14 for the I2C SDA connection
#define SCL_PIN 15 // I am using an ESP32 and chose GPIO pin 15 for the I2C SCL connection
// Arduino Uno Connection options:
//#define SDA_PIN A4 
//#define SCL_PIN A5

#define Shunt_Resistor_Value 0.002f // 0.002 ohms matching the INA-238 30A Breakout Board
#define Maximum_Expected_Current 25.5f // Amps 
#define DelayBetweenDisplayingReadings 1000 //ms
#define ina238Address 0x40

// your function that is called when readings are ready
void ShowReadings(){
  static int ReadingCount = 0;
  float current = ina238.Current();      // in Amps
  float voltage = ina238.VoltageBus();   // in Volts
  float power   = ina238.Power();        // in Watts
  float tempC    = ina238.Temperature();  // in °C (die temp)
  float tempF    = ina238.Temperature(true);  // in °F (die temp)
  Serial.print("\nReading# "); Serial.println(ReadingCount++);
  Serial.print("Current: "); Serial.print(current); Serial.println(" A");
  Serial.print("Voltage: "); Serial.print(voltage); Serial.println(" V");
  Serial.print("Power:   "); Serial.print(power);   Serial.println(" W");
  Serial.print("Temp:    "); Serial.print(tempC);    Serial.println(" °C");
  Serial.print("Temp:    "); Serial.print(tempF);    Serial.println(" °F");
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("\n\nSimple_INA238 Volt Amp Meter"));

  if(!ina238.Simple_Begin(SDA_PIN, SCL_PIN)){ // Connects i2c, finds INA-238 and Verifies Connection. 
    Serial.println("Simple_INA238 failed to find device! \n !!! STOP !!!");
    ina238.I2C_Scanner(); // lets see if anything is on the i2c bus
    while (1);
  }

// Simple_Config() uses the default settings of the INA238 and initializes the
// INA238 ADC MODE to trigger an Alert when Voltage and Current readings are complete
  ina238.Simple_Config(Maximum_Expected_Current,Shunt_Resistor_Value);

// When readings are ready call this function example "ShowReadings"
  ina238.setOnConversionReadyCallback(ShowReadings);
  Serial.println("INA238 initialized!");
}

void loop() {
// place the Alert() functin within your loop() functin to handle all the processes of the INA238 
// this has an optional non blocking delay that will skip checking until the timer has elapsed.
  ina238.Alert(DelayBetweenDisplayingReadings); // Callbacks are triggered withing this alert function
}
