/* ============================================
  Simple_INA238 device library code is placed under the MIT license
  Copyright (c) 2024 Homer Creutz

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/
#include "Arduino.h"
#include "INA238_config.h"
uint16_t DelayBetweenChecks = 1000;
void INA238ConversionReady() {
  yield();
  Serial.println(F("*** New Readings Ready ***"));
  Serial.print(F("Chip Temperature: "));
  Serial.print(VA_Meter.Temperature(false));
  Serial.println(F("°C"));
  Serial.print(VA_Meter.Temperature(true));
  Serial.println(F("°F"));

  Serial.print(F("Shunt Voltage: "));
  Serial.print(VA_Meter.mVoltageShunt()*1000,4);
  Serial.println(F("mV"));

  Serial.print(F("Bus Voltage: "));
  Serial.print(VA_Meter.VoltageBus());
  Serial.println(F("V"));

  Serial.print(F("Current: "));
  Serial.print(VA_Meter.Current());
  Serial.println(F("A"));

  Serial.print(F("Power: "));
  Serial.print(VA_Meter.Power());
  Serial.println(F("W"));
  
  if(ina238_mode < ADC_CONFIG_MODE_SHUTDOWN_ALT) { // We are in one shot mode and must trigger another signal to get more results
    VA_Meter.ADC_CONFIG_MODE(ina238_mode);
  }
}

void INA238OverTemperature(){
  static unsigned long SpamTimer;
  if ((millis() - SpamTimer) >= (DelayBetweenChecks)) {
    SpamTimer = millis();
    Serial.println(F("Temperature Limit Reached"));
  }

}

void INA238OverCurrent(){
  static unsigned long SpamTimer;
  if ((millis() - SpamTimer) >= (DelayBetweenChecks)) {
    SpamTimer = millis();

  Serial.println(F("Over Current Limit Reached"));
  }
}

void INA238UnderCurrent(){
  static unsigned long SpamTimer;
  if ((millis() - SpamTimer) >= (DelayBetweenChecks)) {
    SpamTimer = millis();

  Serial.println(F("Under Current Limit Reached"));
  }
}

void INA238OverVoltage(){
  static unsigned long SpamTimer;
  if ((millis() - SpamTimer) >= (DelayBetweenChecks)) {
    SpamTimer = millis();

  Serial.println(F("Over Voltage Limit Reached"));
  }
}

void INA238UnderVoltage(){
  static unsigned long SpamTimer;
  if ((millis() - SpamTimer) >= (DelayBetweenChecks)) {
    SpamTimer = millis();

  Serial.println(F("Under Voltage Limit Reached"));
  }
}

void INA238OverPower(){
  static unsigned long SpamTimer;
  if ((millis() - SpamTimer) >= (DelayBetweenChecks)) {
    SpamTimer = millis();

  Serial.println(F("Over Power Limit Reached"));
  }
}


void setup() {
  Serial.begin(115200);
  Simple_INA238_Config();
// callback functions to be triggered When Events are discovered This happens during the execution of VA_Meter.Alert();
 // VA_Meter.setOnAlarmCallbackFunction(AlarmFunctionName);
 // VA_Meter.setOnAlarmSoftwareCallbackFunction(SoftwareAlarmFunctionName);
 // VA_Meter.setOnAlarmSensorsCallbackFunction(SensorsAlarmFunctionName);
 // VA_Meter.setOnMathOverflowCallback(MathOverflowAlarmFunctionName);
  VA_Meter.setOnTemperatureOverCallback(INA238OverTemperature);
  VA_Meter.setOnShuntOverVoltageCallback(INA238OverCurrent);
  VA_Meter.setOnShuntUnderVoltageCallback(INA238UnderCurrent);
  VA_Meter.setOnBusOverVoltageCallback(INA238OverVoltage);
  VA_Meter.setOnBusUnderVoltageCallback(INA238UnderVoltage);
  VA_Meter.setOnPowerOverlimitCallback(INA238OverPower);
  //VA_Meter.setOnMemoryChecksumErrorCallback(MemoryChecksumErrorFunctionName);
  VA_Meter.setOnConversionReadyCallback(INA238ConversionReady);
  
}

void loop() {
  // This will trigger if either the Alert pin is not being used or the or if the Conversion ready is not set to trigger when the Alert pin is activated
  if(!ConversionReadyEnable || !AlertInterruptEnabled){ 
      AlertTriggered = true;
  }

  if(AlertTriggered){  // This triggers an alert when the Alert pin is activated.
     VA_Meter.Alert(); // Callbacks are triggered withing this alert
     AlertTriggered = false;
     if(ina238_mode < ADC_CONFIG_MODE_SHUTDOWN_ALT) { // We are in one shot mode and must trigger another signal to get more results
      VA_Meter.ADC_CONFIG_MODE(ina238_mode);
     }
  }

}
