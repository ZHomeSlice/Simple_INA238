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
#include <Wire.h>
#include "Simple_Wire.h"
#include "Simple_INA238.h"
#include "INA238_RW_Macros.h"

// Blink Without Delay Serial Port Spam Timer Macro
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis()) // A Way to do a Blink without Delay timer

/**
@brief      Initialization functions
*/
Simple_INA238::Simple_INA238()
{
    SetIntMSBPos(false);
}

Simple_INA238::Simple_INA238(uint8_t address)
{
    SetAddress(address);
}

Simple_INA238 &Simple_INA238::begin(int sdaPin, int sclPin)
{
    delay(650); // hold on for boot
    Simple_Wire::begin(sdaPin, sclPin);
    return *this;
}

Simple_INA238 &Simple_INA238::SetAddressWithinRange(uint8_t StartingAddress, uint8_t Length)
{
    if (StartingAddress >= Length)
    {
        SetAddress(StartingAddress);
        return *this;
    }
    uint16_t deviceIDCheck;
    do
    {
        if (StartingAddress == 0x1)
        {
            SetAddress(CHIP_ID); // fail to default address
            return *this;
        }
        StartingAddress = Find_Address(StartingAddress, Length);
        if (StartingAddress == 0)
        {
            SetAddress(CHIP_ID); // fail to default address
            return *this;
        }
        SetAddress(StartingAddress);
        if ((R_DEVICE_ID(&deviceIDCheck).Success()) && (DEVICE_ID == deviceIDCheck))
            return *this; // Device ID Matches We have Found the correct address
        StartingAddress++;
    } while (StartingAddress <= Length);
    SetAddress(CHIP_ID); //  fail to default address
    return *this;
}

/**
@brief      Test to be sure we have communication to the MPU
Verbose = true - output Results to serial port
returns 1 on success
stops or returns 0 on fail
*/
bool Simple_INA238::TestConnection(bool V)
{
    Verbose = V;
    Simple_Wire::SetVerbose(V);
    uint16_t deviceIDCheck;
    if (Check_Address())
    {
        if (R_DEVICE_ID(&deviceIDCheck).Success())
        {
            ConectionVerified = (deviceID == deviceIDCheck) ? true : false;
            if (Verbose)
            {
                if(ConectionVerified){
                    Serial.print("Found INA238 at: 0x");
                    Serial.println(GetAddress(), HEX);
                    Serial.print("INA238 Device ID: 0x");
                    Serial.println(deviceIDCheck, HEX);
                } else {
                    Serial.print("Found a device at: 0x");
                    Serial.println(GetAddress(), HEX);
                    Serial.print("The Device ID didn't match : 0x2381U <> 0x");
                    Serial.println(deviceIDCheck, HEX);   
                }
            }
            return ConectionVerified;
        }
    }
    return false;
}

Simple_INA238 &Simple_INA238::Reset()
{
    W_CONFIG(CONFIG_RST);
    return *this;
}

// Private helper to update specific bit fields in the ADC_CONFIG register.
Simple_INA238 &Simple_INA238::updateCONFIGField(uint16_t mask, uint16_t setting)
{
    uint16_t reg = 0;
    R_CONFIG(&reg);          // Retrieve the current CONFIG register value
    reg &= ~mask;            // Clear the bits specified by mask
    reg |= (setting & mask); // Set the new bits (only within the mask)
    W_CONFIG(reg);           // Write the updated value back
    return *this;
}

// Sets both values in one command See CONFIG_CONVDLY and CONFIG_ADCRANGE for detailed explinations of each value
// Example Usage:
// CONFIG(CONFIG_CONVDLY_MS(100),CONFIG_ADCRANGE_0H_163_84mV);
Simple_INA238 &Simple_INA238::CONFIG(uint16_t CONVDLY, uint16_t ADCRANGE)
{
    adcRange = ADCRANGE;
    // Define the combined mask for all three fields:
    uint16_t mask = (0xFF << 6) | (0x1 << 4);
    // Combine the individual settings (they should already be shifted correctly)
    uint16_t setting = (CONVDLY | ADCRANGE) & mask;
    return updateCONFIGField(mask, setting);
}

// The following functions allow you to set the CONFIG bits separatly
//  The conversion delay is set to 0 by default
//  Conversion delay can assist in measurement synchronization
//  when multiple external devices are used for voltage or current monitoring purposes.
//  use on of ese to calculate the rpoper Bit placement
//  CONFIG_CONVDLY(val)      // Sets the Delay for initial ADC conversion in steps of 2 ms 0h = 0 s 1h = 2 ms ... FFh = 510 ms
//  CONFIG_CONVDLY_MS(ms)    // Sets the Delay for initial ADC conversion in steps of 2 ms rounded down 0 ms to 510 ms
//  usage example 1 CONFIG_CONVDLY(CONFIG_CONVDLY(50)) sets the Conversion delay to 100ms
//  usage example 2 CONFIG_CONVDLY(CONFIG_CONVDLY_MS(100)) sets the Conversion delay to 100ms
Simple_INA238 &Simple_INA238::CONFIG_CONVDLY_Value(uint16_t CONVDLY)
{
    return CONFIG_CONVDLY(CONFIG_CONVDLY_Val(CONVDLY));
}
Simple_INA238 &Simple_INA238::CONFIG_CONVDLY_Miliseconds(uint16_t MiliSeconds)
{
    return CONFIG_CONVDLY(CONFIG_CONVDLY_MS(MiliSeconds));
}
Simple_INA238 &Simple_INA238::CONFIG_CONVDLY(uint16_t CONVDLY)
{
    // CONVDLY is assumed to be pre-shifted (or will be masked) by (0xF << 6)
    return updateCONFIGField((0xFF << 6), CONVDLY);
}

// ADCRANGE Analog to digital Conversion Range
// CONFIG_ADCRANGE_0H_163_84mV = ±163.84 mV ADC Full Scale Range or a Resolution of 5 µV/LSB (Default)
// ONFIG_ADCRANGE_40_96mV = ± 40.96 mV ADC Full Scale Range or a Resolution of 1.25 µV/LSB
// Shunt full scale range selection across IN+ and IN–.
// Notes:
// this affects the:
//  Shunt Voltage Measurement (VSHUNT)
//  Shunt Overvoltage Threshold (SOVL)
//  Shunt Undervoltage Threshold (SUVL)
Simple_INA238 &Simple_INA238::CONFIG_ADCRANGE(uint16_t ADCRANGE)
{
    adcRange = ADCRANGE;
    return updateCONFIGField((0x1 << 4), adcRange);
}

// Private helper to update specific bit fields in the ADC_CONFIG register.
Simple_INA238 &Simple_INA238::updateADCField(uint16_t mask, uint16_t setting)
{
    uint16_t reg = 0;
    R_ADC_CONFIG(&reg);      // Retrieve the current ADC_CONFIG register value
    reg &= ~mask;            // Clear the bits specified by mask
    reg |= (setting & mask); // Set the new bits (only within the mask)
    W_ADC_CONFIG(reg);       // Write the updated value back
    return *this;
}

// ADC_MODE field (bits 15-12)
// Options for MODE (values in hex, shifted left by 12 bits):
// See INA238_RW_Macros.h for ADC_CONFIG_MODE_... for Settings and use the Macro Name to properly set the value
// ADC_CONFIG_MODE(ADC_CONFIG_MODE_CONTINUOUS_SHUNT_AND_BUS_VOLTAGE);
Simple_INA238 &Simple_INA238::ADC_CONFIG_MODE(uint16_t MODE)
{
    // mode is assumed to be pre-shifted (or will be masked) by (0xF << 12)
    return updateADCField((0xF << 12), MODE);
}

// VBUSCT = Conversion time for bus voltage measurement.
// VSHCT  = Conversion time for shunt voltage measurement.
// VTCT   = Conversion time for temperature measurement.
// See INA238_RW_Macros.h for ADC_CONFIG_:
//    VBUSCT_... for Settings Default is 1052 µs (default) (use the Macro Name to properly set the value)
//    VSHCT_...  for Settings Default is 1052 µs (default) (use the Macro Name to properly set the value)
//    VTCT_...   for Settings Default is 1052 µs (default) (use the Macro Name to properly set the value)
// ADC_Config_CT(ADC_CONFIG_VBUSCT_1052us,ADC_CONFIG_VSHCT_1052us,ADC_CONFIG_VTCT_1052us);
Simple_INA238 &Simple_INA238::ADC_Config_CT(uint16_t VBUSCT, uint16_t VSHCT, uint16_t VTCT)
{
    // Define the combined mask for all three fields:
    uint16_t mask = (0x7 << 9) | (0x7 << 6) | (0x7 << 3);
    // Combine the individual settings (they should already be shifted correctly)
    uint16_t setting = (VBUSCT | VSHCT | VTCT) & mask;
    return updateADCField(mask, setting);
}

// AVG = Analog to Digital Converter sample averaging count.
// See INA238_RW_Macros.h for ADC_CONFIG_AVG. for Settings Default is 1 Sample (use the Macro Name to properly set the value)
// Sets the ADC sample averaging count
// ADC_Config_AVG(ADC_CONFIG_AVG_1024);
Simple_INA238 &Simple_INA238::ADC_Config_AVG(uint16_t AVG)
{
    return updateADCField(0x7, AVG);
}

// rShunt is the resistance Value of the Shunt Resistor
// Max Expected Current is the the max current you desire the device to be able to read
// set the ADC
// Calculates and stores the SHUNT_CAL value in the SHUNT_CAL register
// CalculateShuntCal(0.002,20);

/**
 * @brief Calculate the SHUNT_CAL register value.
 *
 * Equation (1) in the INA238 datasheet:
 *   SHUNT_CAL = 819.2e6 × CURRENT_LSB × R_SHUNT
 *   (If ADCRANGE=1, multiply by 4.)
 *
 * @param rShunt         The shunt resistor value in ohms.
 * @param maxExpectedCurrent     The MaxExpectedCurrent
 * @param adcrange       Set true if ADCRANGE=1, false if ADCRANGE=0.
 * @return The integer value to write into the SHUNT_CAL register.
 */

Simple_INA238 &Simple_INA238::CalculateShuntCal(float rShunt, float maxExpectedCurrent, bool Verbose)
{
    ShuntResistance = rShunt;
    if (Verbose)
    {
        Serial.print("SHUNT Resistance: ");
        Serial.println(ShuntResistance, 5);
        Serial.print("Max Expected Current: ");
        Serial.println(maxExpectedCurrent, 5);
    }
    _maxExpectedCurrent = maxExpectedCurrent;
    // Compute CURRENT_LSB correctly: using 2^15 = 32768, not 2e15
    if (adcRange == 0xff)
    {
        float SVD = 0.0f;
        SVD = SHUNT_VOLTAGE_DROP(maxExpectedCurrent, ShuntResistance);
        adcRange = (SVD > ADC_RANGE_1H_Max) ? CONFIG_ADCRANGE_0H_163_84mV : CONFIG_ADCRANGE_1H_40_96mV;
        CONFIG_ADCRANGE(adcRange); // Set the Value
        if (Verbose)
        {
            Serial.print("ADC Calculated Shunt Max Voltgae: ");
            Serial.println(SVD, 5);
            Serial.print("ADC Shunt Max Voltgae: ");
            Serial.println((((adcRange) == CONFIG_ADCRANGE_0H_163_84mV) ? ADC_RANGE_0H_Max : ADC_RANGE_1H_Max), 5);
            Serial.println(adcRange, HEX);
        }
    }
    current_lsb = maxExpectedCurrent / 32768.0F; // 32,768 == 2^15
    power_lsb = 0.20 * current_lsb;

    shuntCal = ((adcRange != 0) ? 1.0f : 4.0f) * 819.2e6F * current_lsb * ShuntResistance; // 819.2e6 == 819,200,000
    uint16_t shuntCalValue = static_cast<uint16_t>(shuntCal);
    W_SHUNT_CAL(shuntCalValue);
    if (Verbose)
    {
        Serial.print("current lsb: ");
        Serial.println(current_lsb, 10);
        Serial.print("power lsb: ");
        Serial.println(power_lsb, 10);
        Serial.print("shunt Cal: ");
        Serial.println(shuntCal, 10);
        Serial.print("shunt Cal Value: ");
        Serial.println(shuntCalValue);
    }
    if (CHECK_SHUNT_ADC_RANGE(adcRange, _maxExpectedCurrent, ShuntResistance) != true)
    {
        Serial.println("*** Your Max Current and resistor values will overflow the ADC Max reading value ***");
        Serial.print("The Calculated Max Voltage Drop across the Shunt Resistor based on your settings is +-");
        Serial.println(SHUNT_VOLTAGE_DROP(_maxExpectedCurrent, ShuntResistance));
        Serial.print("The Max Voltage drop that is readable by the ADC is +-");
        // CONFIG_ADCRANGE_0H_163_84mV = ±163.84 mV Full Scale (5 µV/LSB)
        // CONFIG_ADCRANGE_1H_40_96mV  = ±40.96 mV Full Scale (1.25 µV/LSB)
        Serial.println((((adcRange) == 0) ? ADC_RANGE_0H_Max : ADC_RANGE_1H_Max), 5);
    }
    return *this;
}

Simple_INA238 &Simple_INA238::SetMaxExpectedCurrent(float maxExpectedCurrent)
{
    _maxExpectedCurrent = maxExpectedCurrent;
    Serial.print("max Expected Current: ");
    Serial.println(_maxExpectedCurrent, 4);
    current_lsb = maxExpectedCurrent / 2e15;
    Serial.print("current lsb: ");
    Serial.println(current_lsb, 10);
    return *this;
}

float Simple_INA238::mVoltageShunt()
{
    int16_t data;
    R_VSHUNT(&data);
    return SHUNT_mVOLTS(data, adcRange);
}

Simple_INA238 &Simple_INA238::mVoltageShunt(float &ShuntVoltage)
{
    ShuntVoltage = mVoltageShunt();
    return *this;
}

float Simple_INA238::VoltageBus()
{
    int16_t data;
    R_VBUS(&data);

    return VOLTS(data);
}

Simple_INA238 &Simple_INA238::VoltageBus(float &busVoltage)
{
    busVoltage = VoltageBus();
    return *this;
}

float Simple_INA238::Temperature(bool isFahrenheit)
{
    int16_t value;
    float Temperature;
    R_DIETEMP(&value);
    Temperature = DIETEMP(value);
    if (!isFahrenheit)
        return Temperature;   // For Celsius Value
    return CtoF(Temperature); // Convert to F
}

Simple_INA238 &Simple_INA238::Temperature(float &Temp, bool IsF)
{
    Temp = Temperature(IsF);
    return *this;
}

float Simple_INA238::Current()
{
    int16_t data;
    R_CURRENT(&data);
    return AMPS(data, current_lsb);
}
Simple_INA238 &Simple_INA238::Current(float &Amps)
{
    Amps = Current();
    return *this;
}

float Simple_INA238::Power()
{
    uint32_t data = 0;
    R_POWER(&data);
    return WATTS(data, current_lsb);
}

Simple_INA238 &Simple_INA238::Power(float &Wats)
{
    Wats = Power();
    return *this;
}

// Private helper function in to store the values in W_DIAG_ALR()
Simple_INA238 &Simple_INA238::updateDIAGField(uint16_t mask, uint16_t setting)
{
    uint16_t reg = 0;
    R_DIAG_ALRT(&reg); // Retrieve the current DIAG_ALRT register value
    reg &= ~mask;      // Clear the bits defined by mask
    reg |= setting;    // Set the new bits
    W_DIAG_ALRT(reg);  // Write the updated value back
    return *this;
}

// When the Alert Latch Enable is set to Transparent mode, the Alert pin and Flag bit reset
// to the idle state when the fault has been cleared. When set to Latch mode, they remain active.
Simple_INA238 &Simple_INA238::DIAG_Alert_Latch(bool State)
{
    uint16_t alertSetting = (State) ? DIAG_ALRT_ALATCH_LATCHED : DIAG_ALRT_ALATCH_TRANSPARENT;
    return updateDIAGField(DIAG_ALRT_ALATCH_LATCHED, alertSetting);
}

// Setting the Conversion Ready flag on ALERT pin.
Simple_INA238 &Simple_INA238::DIAG_Alert_Conversion_Ready_Flag(bool State)
{
    uint16_t alertSetting = (State) ? DIAG_ALRT_CNVR_ENABLE : DIAG_ALRT_CNVR_DISABLE;
    return updateDIAGField(DIAG_ALRT_CNVR_ENABLE, alertSetting);
}

// When using averaged value for ALERT comparison.
Simple_INA238 &Simple_INA238::DIAG_Alert_Use_Averaged_Value(bool State)
{
    uint16_t alertSetting = (State) ? DIAG_ALRT_SLOWALERT_AVERAGED : DIAG_ALRT_SLOWALERT_ADC;
    return updateDIAGField(DIAG_ALRT_SLOWALERT_AVERAGED, alertSetting);
}

// Set the Alert Pin polarity.
Simple_INA238 &Simple_INA238::DIAG_Alert_Pin_Polarity(bool State)
{
    uint16_t alertSetting = (State) ? DIAG_ALRT_APOL_INVERTED : DIAG_ALRT_APOL_NORMAL;
    return updateDIAGField(DIAG_ALRT_APOL_INVERTED, alertSetting);
}

uint16_t Simple_INA238::Alert(uint32_t DelayBetweenChecks)
{
    static unsigned long SpamTimer;
    if ((millis() - SpamTimer) >= (DelayBetweenChecks))
    {
        SpamTimer = millis();
        return Alert();
    }
    return 0;
}

uint16_t Simple_INA238::Alert()
{
    R_DIAG_ALRT(&LastAlert);
    uint32_t mask = DIAG_ALRT_MATHOF_OVERFLOW | DIAG_ALRT_TMPOL_OVER_TEMP | DIAG_ALRT_SHNTOL_OVER_SHUNT | DIAG_ALRT_BUSOL_OVER_BUS | DIAG_ALRT_BUSUL_UNDER_BUS | DIAG_ALRT_POL_OVER_POWER | DIAG_ALRT_CNVRF_READY | DIAG_ALRT_MEMSTAT_CHECKSUM_OK;
    uint32_t alarmMask = DIAG_ALRT_MATHOF_OVERFLOW | DIAG_ALRT_TMPOL_OVER_TEMP | DIAG_ALRT_SHNTOL_OVER_SHUNT | DIAG_ALRT_BUSOL_OVER_BUS | DIAG_ALRT_BUSUL_UNDER_BUS | DIAG_ALRT_POL_OVER_POWER | DIAG_ALRT_MEMSTAT_CHECKSUM_OK;
    uint32_t alarmMaskSoftware = DIAG_ALRT_MATHOF_OVERFLOW | DIAG_ALRT_MEMSTAT_CHECKSUM_OK;
    uint32_t alarmMaskSensors = DIAG_ALRT_TMPOL_OVER_TEMP | DIAG_ALRT_SHNTOL_OVER_SHUNT | DIAG_ALRT_SHNTUL_UNDER_SHUNT | DIAG_ALRT_BUSOL_OVER_BUS | DIAG_ALRT_BUSUL_UNDER_BUS | DIAG_ALRT_POL_OVER_POWER;
    LastAlert &= ((mask)); // Clears all other bits
    LastAlert ^= (1 << 0); // Invert Bit zero as it is 1 when it is in a normal or ok state (see Table 7-13. DIAG_ALRT Register Field Descriptions in Datasheet)

    // Optionally, if any alarm triggered, call a general alarm callback.
    if ((LastAlert & alarmMask) && (onAlarmCallback))
    {
        yield();
        //  Serial.DPRINTBINX("Diag Alert^1 :  ",alarm,true);
        onAlarmCallback();
    }

    if ((LastAlert & alarmMaskSoftware) && (onAlarmSoftwareCallback))
    {
        yield();
        //  Serial.DPRINTBINX("Diag Alert^1 :  ",alarm,true);
        onAlarmSoftwareCallback();
    }

    if ((LastAlert & alarmMaskSensors) && (onAlarmSensorsCallback))
    {
        yield();
        //  Serial.DPRINTBINX("Diag Alert^1 :  ",alarm,true);
        onAlarmSensorsCallback();
    }
    // Check Math Overflow (Bit 9)
    if ((LastAlert & DIAG_ALRT_MATHOF_OVERFLOW) && (onMathOverflowCallback))
    {
        yield();
        onMathOverflowCallback();
    }

    // Check Temperature Over-Limit (Bit 7)
    if ((LastAlert & DIAG_ALRT_TMPOL_OVER_TEMP) && (onTemperatureOverCallback))
    {
        yield();
        onTemperatureOverCallback();
    }

    // Check Shunt Overvoltage (Bit 6)
    if ((LastAlert & DIAG_ALRT_SHNTOL_OVER_SHUNT) && (onShuntOvervoltageCallback))
    {
        yield();
        onShuntOvervoltageCallback();
    }

    // Check Shunt Undervoltage (Bit 5)
    if ((LastAlert & DIAG_ALRT_SHNTUL_UNDER_SHUNT) && (onShuntUndervoltageCallback))
    {
        yield();
        onShuntUndervoltageCallback();
    }

    // Check Bus Overvoltage (Bit 4)
    if ((LastAlert & DIAG_ALRT_BUSOL_OVER_BUS) && (onBusOvervoltageCallback))
    {
        yield();
        onBusOvervoltageCallback();
    }

    // Check Bus Undervoltage (Bit 3)
    if ((LastAlert & DIAG_ALRT_BUSUL_UNDER_BUS) && (onBusUndervoltageCallback))
    {
        yield();
        onBusUndervoltageCallback();
    }

    // Check Power Over-Limit (Bit 2)
    if ((LastAlert & DIAG_ALRT_POL_OVER_POWER) && (onPowerOverlimitCallback))
    {
        yield();
        onPowerOverlimitCallback();
    }

    // Check Conversion Ready Flag (Bit 1)
    // (Not necessarily an alarm, but you can trigger a callback if desired.)
    if ((LastAlert & DIAG_ALRT_CNVRF_READY) && (onConversionReadyCallback))
    {
        yield();
        onConversionReadyCallback();
    }

    // Check Memory Status (Bit 0)
    // Normally this bit is 1. If it's 0, it indicates a checksum error.
    if ((LastAlert & DIAG_ALRT_MEMSTAT_CHECKSUM_OK) && (onMemoryChecksumErrorCallback))
    { // This bit was inverted so it is Not ok
        yield();
        onMemoryChecksumErrorCallback();
    }
    return LastAlert; // Return the modified data to only show alarms and all alarm values are 1
}

// General alarm callback
Simple_INA238 &Simple_INA238::setOnAlarmCallbackFunction(OnAlarmCallback callback)
{
    onAlarmCallback = callback;
    return *this;
}

Simple_INA238 &Simple_INA238::setOnAlarmSoftwareCallbackFunction(OnAlarmCallback callback)
{
    onAlarmSoftwareCallback = callback;
    return *this;
}

Simple_INA238 &Simple_INA238::setOnAlarmSensorsCallbackFunction(OnAlarmCallback callback)
{
    onAlarmSensorsCallback = callback;
    return *this;
}

// Math Overflow
Simple_INA238 &Simple_INA238::setOnMathOverflowCallback(OnAlarmCallback callback)
{
    onMathOverflowCallback = callback;
    return *this;
}
// Temperature Callbacks
Simple_INA238 &Simple_INA238::setOnTemperatureOverCallback(OnAlarmCallback callback)
{
    onTemperatureOverCallback = callback;
    return *this;
}

// Shumt Voltage Callbacks
Simple_INA238 &Simple_INA238::setOnShuntOvervoltageCallback(OnAlarmCallback callback)
{
    onShuntOvervoltageCallback = callback;
    return *this;
}

Simple_INA238 &Simple_INA238::setOnShuntUndervoltageCallback(OnAlarmCallback callback)
{
    onShuntUndervoltageCallback = callback;
    return *this;
}

// Bus Voltage Callbacks
Simple_INA238 &Simple_INA238::setOnBusOvervoltageCallback(OnAlarmCallback callback)
{
    onBusOvervoltageCallback = callback;
    return *this;
}

Simple_INA238 &Simple_INA238::setOnBusUndervoltageCallback(OnAlarmCallback callback)
{
    onBusUndervoltageCallback = callback;
    return *this;
}

// Power Callbacks
Simple_INA238 &Simple_INA238::setOnPowerOverlimitCallback(OnAlarmCallback callback)
{
    onPowerOverlimitCallback = callback;
    return *this;
}

// Conversion Ready Callback
Simple_INA238 &Simple_INA238::setOnConversionReadyCallback(OnAlarmCallback callback)
{
    onConversionReadyCallback = callback;
    return *this;
}
// Memory Checksum Error Callback
Simple_INA238 &Simple_INA238::setOnMemoryChecksumErrorCallback(OnAlarmCallback callback)
{
    onMemoryChecksumErrorCallback = callback;
    return *this;
}
