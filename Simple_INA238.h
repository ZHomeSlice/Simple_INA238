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

#ifndef SIMPLE_INA238_H
#define SIMPLE_INA238_H

#include "Arduino.h"
#include "Simple_Wire.h"
#include "INA238_RW_Macros.h"
#include <functional>

#ifdef __AVR__
  #include <avr/pgmspace.h>
  #define printfloatx(Name, Variable, Spaces, Precision, EndTxt) \
    print(Name); { char S[(Spaces + Precision + 3)]; Serial.print(F(" ")); Serial.print(dtostrf((float)Variable, Spaces, Precision, S)); } Serial.print(EndTxt)
#elif defined(ESP32)
  #include <pgmspace.h>
  #include <stdlib_noniso.h>
  #define printfloatx(Name, Variable, Spaces, Precision, EndTxt) \
    print(Name); Serial.print(F(" ")); Serial.print(Variable, Precision); Serial.print(EndTxt)
#else
  #define printfloatx(Name, Variable, Spaces, Precision, EndTxt) \
    print(Name); Serial.print(F(" ")); Serial.print(Variable, Precision); Serial.print(EndTxt)
#endif


class Simple_INA238 : public Simple_Wire {
  public:
    // Callback type definitions:
    using OnAlarmCallback = std::function<void()>;

    // Public member variables:
    uint16_t deviceID = DEVICE_ID;
    uint16_t adcRange = 0xff; // Not set by default.
    float current_lsb = 0.0;
    float power_lsb = 0.0;
    float ShuntResistance = 0.002;
    float shuntCal = 0.0;
    float _maxExpectedCurrent = 0.0;
    bool ConectionVerified = false;
    bool Verbose = false;
    uint16_t LastAlert = 0;

    // Constructors:
    Simple_INA238();
    Simple_INA238(uint8_t address);

    // Startup / Initialization:
    Simple_INA238 & begin(int sdaPin, int sclPin);
    bool Simple_Begin(int sdaPin, int sclPin){return begin(sdaPin,  sclPin).SetAddressWithinRange().TestConnection();}; 
    Simple_INA238 & SetAddress(uint8_t address = CHIP_ID){Simple_Wire::SetAddress(address); return *this;};// Sets the address
    Simple_INA238 & SetAddressWithinRange(uint8_t StartingAddress = CHIP_ID, uint8_t Length = (CHIP_ID + 0xFU));
    Simple_INA238 & SetVerbose(bool V = true){Simple_Wire::SetVerbose(V); Verbose = V; return *this;};
    bool TestConnection(bool Verbose = true);// Tests the connection and verifies this is the correct chip.
    Simple_INA238 &Delay(uint32_t ms){delay(ms);return *this;};
    // Configuration functions:
    Simple_INA238 & Simple_Config(float MaxCurrent,float SResistor, uint16_t ADCMode =  ADC_CONFIG_MODE_CONTINUOUS_SHUNT_AND_BUS_VOLTAGE){Reset().CalculateShuntCal(SResistor, MaxCurrent).ADC_CONFIG_MODE(ADCMode); return *this;};   
    Simple_INA238 & Reset();
    Simple_INA238 & updateCONFIGField(uint16_t mask, uint16_t setting);
    Simple_INA238 & CONFIG(uint16_t CONVDLY, uint16_t ADCRANGE);
    Simple_INA238 & CONFIG_CONVDLY_Value(uint16_t CONVDLY);
    Simple_INA238 & CONFIG_CONVDLY_Miliseconds(uint16_t MiliSeconds);
    Simple_INA238 & CONFIG_CONVDLY(uint16_t CONVDLY);
    Simple_INA238 & CONFIG_ADCRANGE(uint16_t ADCRANGE);
    Simple_INA238 & updateADCField(uint16_t mask, uint16_t setting);
    Simple_INA238 & ADC_CONFIG_MODE(uint16_t mode);
    Simple_INA238 & ADC_Config_CT(uint16_t VBUSCT, uint16_t VSHCT, uint16_t VTCT);
    Simple_INA238 & ADC_Config_AVG(uint16_t AVG);
    Simple_INA238 & CalculateShuntCal(float rShunt, float maxExpectedCurrent, bool Verbose = true);
    Simple_INA238 & SetMaxExpectedCurrent(float maxExpectedCurrent);

    // Alert Setpoints
    // Shunt Limits
    Simple_INA238 & SetShuntOverVoltageThreshold(float Volts){Set_SOVL(adcRange, Volts);return *this;};
    Simple_INA238 & SetShuntUnderVoltageThreshold(float Volts){Set_SUVL(adcRange, Volts);return *this;};
    // or you can set this voltage using Current value in Amps:
    Simple_INA238 & SetShuntOverCurrentThreshold(float Amps){Set_SOVL(adcRange, Amps * ShuntResistance);return *this;};
    Simple_INA238 & SetShuntUnderCurrentThreshold(float Amps){Set_SUVL(adcRange, Amps * ShuntResistance);return *this;};
    // Buss Voltage Limits
    Simple_INA238 & SetBusOverVoltageThreshold(float Volts){Set_BOVL(Volts);return *this;};
    Simple_INA238 & SetBusUnderVoltageThreshold(float Volts){Set_BUVL(Volts);return *this;};
    // Power Limit
    Simple_INA238 & SetPowerThreshold(float Watts){Set_POL(Watts,_maxExpectedCurrent);return *this;}; 
    // Temperature Limits in Celsius or Fahrenheit
    Simple_INA238 & SetTemperatureThresholdCelsius(float Temp_C){Set_TOL_C(Temp_C);return *this;};
    Simple_INA238 & SetTemperatureThresholdFahrenheit(float Temp_F){Set_TOL_F(Temp_F);return *this;};
       

    // Measurement functions (inline where appropriate):
    // Retrieves the Shunt Voltage
    float mVoltageShunt();
    Simple_INA238 & mVoltageShunt(float &ShuntVoltage);
    // Retrieves the Bus Voltage
    float VoltageBus();
    Simple_INA238 & VoltageBus(float &busVoltage);
    // Retrieves the Temperature
    float Temperature(bool F = false);
    Simple_INA238 & Temperature(float &TemperatureC, bool F = false);
    // Retrieves the Current across the Shunt Resistor
    float Current();
    Simple_INA238 & Current(float &Amps);
    // Retrieves the power in Watts
    float Power();
    Simple_INA238 & Power(float &Wats);

    // Alert Trigger Settings
    Simple_INA238 & updateDIAGField(uint16_t mask, uint16_t setting);
    Simple_INA238 & DIAG_Alert_Latch(bool State);
    Simple_INA238 & DIAG_Alert_Conversion_Ready_Flag(bool State);
    Simple_INA238 & DIAG_Alert_Use_Averaged_Value(bool State);
    Simple_INA238 & DIAG_Alert_Pin_Polarity(bool State);

    // Alarm / Alert function:
    uint16_t Alert(uint32_t DelayBetweenChecks); //Insert in loop with non blocking delay
    uint16_t Alert();  // note that bit 0 is inverted to indicate an alarm rather than normal
    bool AlertConversionReady(){return LastAlert & DIAG_ALRT_CNVRF_READY;};
    bool AlertTemperatre(){return LastAlert & DIAG_ALRT_TMPOL_OVER_TEMP;};
    bool AlertHighCurrent(){return LastAlert & DIAG_ALRT_SHNTOL_OVER_SHUNT;};
    bool AlertLowCurrent(){return LastAlert & DIAG_ALRT_SHNTUL_UNDER_SHUNT;};
    bool AlertHighBussVoltage(){return LastAlert & DIAG_ALRT_BUSOL_OVER_BUS;};
    bool AlertLowBussVoltage(){return LastAlert & DIAG_ALRT_BUSUL_UNDER_BUS;};
    bool AlertHighPower(){return LastAlert & DIAG_ALRT_POL_OVER_POWER;};
    
    
     
    
    
    

    // Callback setter functions:
    Simple_INA238 & setOnAlarmCallbackFunction(OnAlarmCallback callback);
    Simple_INA238 & setOnAlarmSoftwareCallbackFunction(OnAlarmCallback callback);
    Simple_INA238 & setOnAlarmSensorsCallbackFunction(OnAlarmCallback callback);
    Simple_INA238 & setOnMathOverflowCallback(OnAlarmCallback callback);
    Simple_INA238 & setOnTemperatureOverCallback(OnAlarmCallback callback);
    Simple_INA238 & setOnShuntOvervoltageCallback(OnAlarmCallback callback);
    Simple_INA238 & setOnShuntUndervoltageCallback(OnAlarmCallback callback);
    Simple_INA238 & setOnBusOvervoltageCallback(OnAlarmCallback callback);
    Simple_INA238 & setOnBusUndervoltageCallback(OnAlarmCallback callback);
    Simple_INA238 & setOnPowerOverlimitCallback(OnAlarmCallback callback);
    Simple_INA238 & setOnConversionReadyCallback(OnAlarmCallback callback);
    Simple_INA238 & setOnMemoryChecksumErrorCallback(OnAlarmCallback callback);

  private:
    // Callback members:
    OnAlarmCallback onAlarmCallback;
    OnAlarmCallback onAlarmSoftwareCallback;
    OnAlarmCallback onAlarmSensorsCallback;
    OnAlarmCallback onMathOverflowCallback;
    OnAlarmCallback onTemperatureOverCallback;
    OnAlarmCallback onShuntOvervoltageCallback;
    OnAlarmCallback onShuntUndervoltageCallback;
    OnAlarmCallback onBusOvervoltageCallback;
    OnAlarmCallback onBusUndervoltageCallback;
    OnAlarmCallback onPowerOverlimitCallback;
    OnAlarmCallback onConversionReadyCallback;
    OnAlarmCallback onMemoryChecksumErrorCallback;
};

#endif // SIMPLE_INA238_H
