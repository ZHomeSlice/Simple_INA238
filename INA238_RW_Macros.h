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


#ifndef INA238_RW_Macros_h
#define INA238_RW_Macros_h

// The following macros are based on https://www.ti.com/lit/gpn/ina238 Datasheet
// The naming and descriptions are based on the registry table found in the datasheet.
// 

// -------------------------
// Default Values
// -------------------------
#define CONFIG_RST         (0x1U << 15)    // 0h: Reset ALL settings when bit 15 is set to 1 in the CONFIG register(This bit self-clears)
#define CONFIG_DEFAULT     (0x0000U)       // 0xFB68: Default Value
#define ADC_CONFIG_DEFAULT (0xFB68U)       // 0xFB68: Default Value
#define SHUNT_CAL_DEFAULT  (0x1000U)       // 0x1000: Default Value
#define DIAG_ALRT_DEFAULT  (0x0001U)       // 0x0001: Default Value
#define SOVL_DEFAULT       (0x7FFFU)       // 0x7FFF: Default Value
#define SUVL_DEFAULT       (0x8000U)       // 0x8000: Default Value
#define BOVL_DEFAULT       (0x7FFFU)       // 0x7FFF: Default Value
#define BUVL_DEFAULT       (0x0000U)       // 0x0: Default Value
#define TEMP_LIMIT_DEFAULT (0x7FF0U)       // 0x7FF: Default Value  = 255.875°C (I believe there is a documentation error  [reset = 7FFFh]in the headding vs 7FF0h found in the descriptions)
#define PWR_LIMIT_DEFAULT  (0xFFFFU)       // 0xFFFF: Default Value


// -------------------------
// Paramiters and Calculations
// -------------------------
#define CONFIG_ADCRANGE_0H_163_84mV 0X00U             // 0h = ±163.84 mV ADC Full Scale Range or a Resolution of 5 µV/LSB
#define CONFIG_ADCRANGE_1H_40_96mV  (0x1U << 4)     // 1h = ± 40.96 mV ADC Full Scale Range or a Resolution of 1.25 µV/LSB
#define ADCC_RANGE_0H_LSB 0.000005f
#define ADCC_RANGE_1H_LSB 0.00000125f
#define ADC_RANGE_0H_Max 0.16384f //0h = ±163.84 mV ADC Full Scale Range of the Shunt Voltage Drop
#define ADC_RANGE_1H_Max 0.04096f // 1h = ± 40.96 mV ADC Full Scale Range of the Shunt Voltage Drop


#define VSHUNT_CONVERSION_FACTOR(ADC_RANGE) (((ADC_RANGE) == 0) ? ADCC_RANGE_0H_LSB : ADCC_RANGE_1H_LSB) 
#define VBUS_CONVERSION_FACTOR  0.003125f //  3.125 mV/LSB
#define TEMP_CONVERSION_FACTOR  0.125f     // 125 m°C/LSB

#define SCALING_FACTOR (819.2e6)  // Fixed scaling factor
#define SHUNT_CAL_FIXED_VALUE(ADC_Range)  (SCALING_FACTOR * (((ADC_Range) == 0) ? 1 : 4))
#define current_lsb(Maximum_Expected_Current) ((Maximum_Expected_Current) / 2e15)
#define POWER_CONVERSION_FACTOR(Maximum_Expected_Current) ((Maximum_Expected_Current) / 2e15)

// Calibration
// ADC_RANGE Set in CONFIG
// Maximum_Expected_Current: Maximum Expected Current therough the Shunt
// R_SHUNT Resistance in ohms of the Shunt example 0.002 ohms (Vishay Dale part number WSHP28182L000FEA)
// Using the 0.002 ohm resistor the ADC_RANGE of 0h will measure a current range of +- 81.92Amps and with the AC_Range of 1h it will measure a current range of +- 20.48Amps
// The Vishay Dale part number WSHP28182L000FEA is rates at 10W which equates to about 70Amps continuous if PCB and other conditions are adiquate. (Use your own due dillabance to verify this before committing)

#define Max_Current_0H(R_SHUNT) ADC_RANGE_0H_Max/(R_SHUNT)
#define Max_Current_1H(R_SHUNT) ADC_RANGE_1H_Max/(R_SHUNT)
#define Max_Current_Readable(ADC_RANGE, R_SHUNT) (((ADC_RANGE) == 0) ? Max_Current_0H(R_SHUNT) : Max_Current_1H(R_SHUNT)) 
#define VSHUNT_Range(ADC_RANGE) (((ADC_RANGE) == 0) ? ADC_RANGE_0H_Max : ADC_RANGE_1H_Max) 


#define SHUNT_CAL(ADC_RANGE, Maximum_Expected_Current, R_SHUNT) (SHUNT_CAL_FIXED_VALUE(ADC_RANGE) * (CURRENT_CONVERSION_FACTOR(Maximum_Expected_Current)) * (R_SHUNT))

// Value Calculations:
#define SHUNT_mVOLTS(VSHUNT_REGISTER_VALUE, ADC_RANGE) (static_cast<float>(abs(VSHUNT_REGISTER_VALUE)) * (VSHUNT_CONVERSION_FACTOR(ADC_RANGE)))
#define VOLTS(VBUS_REGISTER_VALUE) (static_cast<float>(abs(VBUS_REGISTER_VALUE)) * (VBUS_CONVERSION_FACTOR))
#define DIETEMP(DIETEMP_REGISTER_VALUE) (static_cast<float>(DIETEMP_REGISTER_VALUE)) * (TEMP_CONVERSION_FACTOR)
#define CtoF(CelsiusTemp) ((CelsiusTemp * 1.8) + 32) //CelsiusTemp to Fahrenheit math
#define AMPS(CURRENT_REGISTER_VALUE, CURRENT_CONVERSION_FACTOR_VALUE) (static_cast<float>(CURRENT_REGISTER_VALUE) * (CURRENT_CONVERSION_FACTOR_VALUE))
#define WATTS(POWER_REGISTER_VALUE, CURRENT_CONVERSION_FACTOR_VALUE) (0.2 * static_cast<float>(POWER_REGISTER_VALUE) * (CURRENT_CONVERSION_FACTOR_VALUE))

// R_SHUNT Resistance in ohms of the Shunt example 0.002f ohms (Vishay Dale part number WSHP28182L000FEA)
// The following equation will keep the sensing range wifthin the max current expected based on the Resistance Value
#define SHUNT_VOLTAGE_DROP(Maximum_Expected_Current, R_SHUNT) ((Maximum_Expected_Current) * (R_SHUNT))

// CHECK_SHUNT_RANGE tests to see if your Maximum Expected Current will work with the ADC_RANGE setting and the Resistance you are using.
// True = good to go
// False = Your maximum current will create a higher voltage than the Analog to Digital Converter (ADC) can read!
#define CHECK_SHUNT_ADC_RANGE(ADC_RANGE, Maximum_Expected_Current, R_SHUNT) (((ADC_RANGE) == 0) ? SHUNT_VOLTAGE_DROP(Maximum_Expected_Current, R_SHUNT) < ADC_RANGE_0H_Max : SHUNT_VOLTAGE_DROP(Maximum_Expected_Current, R_SHUNT) < ADC_RANGE_1H_Max) 

// ADC_RANGE_Selector selects the proper ADC_RANGE for the highest current readings based on you Maximum Expected Current and Resistance value of the shunt resistor
// returns the ADC_RANGE Registery Value that best matches the Current and Resistance values you selected 
// you should also check the CHECK_SHUNT_ADC_RANG() to verify that you are not exceeding the limits of the ADC
#define ADC_RANGE_Selector(Maximum_Expected_Current, R_SHUNT) (SHUNT_VOLTAGE_DROP(Maximum_Expected_Current, R_SHUNT) > ADC_RANGE_1H_Max) ? CONFIG_ADCRANGE_0H_163_84mV : CONFIG_ADCRANGE_1H__40_96mV


// Config Settings:

// -------------------------
// CONFIG Register Fields
// -------------------------

// RST (Reset) :Reset Bit. Setting this bit to '1' generates a system reset 
// that is the same as power-on reset. Resets all registers to default values.
// This bit self-clears.
// *** System Reset ***
#define CONFIG_RESET W_CONFIG(CONFIG_RST)

// The conversion delay is set to 0 by default
// Conversion delay can assist in measurement synchronization 
// when multiple external devices are used for voltage or current monitoring purposes.
#define CONFIG_CONVDLY_Val(val)     (val << 6)                      // Sets the Delay for initial ADC conversion in steps of 2 ms 0h = 0 s 1h = 2 ms ... FFh = 510 ms
#define CONFIG_CONVDLY_MS(ms)   ((uint16_t)(((ms) / 2) << 6))   // Sets the Delay for initial ADC conversion in steps of 2 ms rounded down

// Shunt full scale range selection across IN+ and IN–.
// Notes:
// this affects the:
//  Shunt Voltage Measurement (VSHUNT) F
//  Shunt Overvoltage Threshold (SOVL) F
//  Shunt Undervoltage Threshold (SUVL)F
//  F
//      5 µV/LSB when ADCRANGE = 0  
//      1.25 µV/LSB when ADCRANGE = 1
// (values in hex, shifted left by 4 bits):
//#define CONFIG_ADCRANGE_0H_163_84mV 0X00U             // 0h = ±163.84 mV ADC Full Scale Range or a Resolution of 5 µV/LSB
//#define CONFIG_ADCRANGE_1H__40_96mV  (0x1U << 4)     // 1h = ± 40.96 mV ADC Full Scale Range or a Resolution of 1.25 µV/LSB



// CONFIG (0x00) - Configuration (16-bit, R/W)
#define W_CONFIG(Data)      WriteUInt(0x00U, (uint16_t)Data)      // Write to Configuration register
#define R_CONFIG(Data)      ReadUInt(0x00U, (uint16_t *)Data)     // Read from Configuration register


// -------------------------
// ADC_CONFIG Register Fields
// -------------------------

// The user can set the MODE bits for continuous or triggered mode 
// on bus voltage, shunt voltage or temperature measurement.

// MODE field (bits 15-12)
// Options for MODE (values in hex, shifted left by 12 bits):
#define ADC_CONFIG_MODE_SHUTDOWN                                               0X00U            // 0h: Shutdown
#define ADC_CONFIG_MODE_TRIGGERED_BUS_VOLTAGE_SINGLE_SHOT                      (0x01U << 12)  // 1h: Triggered bus voltage, single shot
#define ADC_CONFIG_MODE_TRIGGERED_SHUNT_VOLTAGE_SINGLE_SHOT                    (0x02U << 12)  // 2h: Triggered shunt voltage, single shot
#define ADC_CONFIG_MODE_TRIGGERED_SHUNT_AND_BUS_VOLTAGE_SINGLE_SHOT            (0x03U << 12)  // 3h: Triggered shunt voltage and bus voltage, single shot
#define ADC_CONFIG_MODE_TRIGGERED_TEMPERATURE_SINGLE_SHOT                      (0x04U << 12)  // 4h: Triggered temperature, single shot
#define ADC_CONFIG_MODE_TRIGGERED_TEMPERATURE_AND_BUS_VOLTAGE_SINGLE_SHOT      (0x05U << 12)  // 5h: Triggered temperature and bus voltage, single shot
#define ADC_CONFIG_MODE_TRIGGERED_TEMPERATURE_AND_SHUNT_VOLTAGE_SINGLE_SHOT    (0x06U << 12)  // 6h: Triggered temperature and shunt voltage, single shot
#define ADC_CONFIG_MODE_TRIGGERED_ALL_SINGLE_SHOT                              (0x07U << 12)  // 7h: Triggered bus voltage, shunt voltage and temperature, single shot
#define ADC_CONFIG_MODE_SHUTDOWN_ALT                                           (0x08U << 12)  // 8h: Shutdown (alternate value)
#define ADC_CONFIG_MODE_CONTINUOUS_BUS_VOLTAGE_ONLY                            (0x09U << 12)  // 9h: Continuous bus voltage only
#define ADC_CONFIG_MODE_CONTINUOUS_SHUNT_VOLTAGE_ONLY                          (0x0AU << 12)  // Ah: Continuous shunt voltage only
#define ADC_CONFIG_MODE_CONTINUOUS_SHUNT_AND_BUS_VOLTAGE                       (0x0BU << 12)  // Bh: Continuous shunt and bus voltage
#define ADC_CONFIG_MODE_CONTINUOUS_TEMPERATURE_ONLY                            (0x0CU << 12)  // Ch: Continuous temperature only
#define ADC_CONFIG_MODE_CONTINUOUS_BUS_VOLTAGE_AND_TEMPERATURE                 (0x0DU << 12)  // Dh: Continuous bus voltage and temperature
#define ADC_CONFIG_MODE_CONTINUOUS_TEMPERATURE_AND_SHUNT_VOLTAGE               (0x0EU << 12)  // Eh: Continuous temperature and shunt voltage
#define ADC_CONFIG_MODE_CONTINUOUS_ALL                                         (0x0FU << 12)  // Fh: Continuous bus voltage, shunt voltage and temperature

// VBUSCT field (bits 11-9)
// Conversion time for bus voltage measurement.
#define ADC_CONFIG_VBUSCT_50us    0U            // 0h = 50 µs
#define ADC_CONFIG_VBUSCT_84us    (0x01U << 9)   // 1h = 84 µs
#define ADC_CONFIG_VBUSCT_150us   (0x02U << 9)   // 2h = 150 µs
#define ADC_CONFIG_VBUSCT_280us   (0x03U << 9)   // 3h = 280 µs
#define ADC_CONFIG_VBUSCT_540us   (0x04U << 9)   // 4h = 540 µs
#define ADC_CONFIG_VBUSCT_1052us  (0x05U << 9)   // 5h = 1052 µs (default)
#define ADC_CONFIG_VBUSCT_2074us  (0x06U << 9)   // 6h = 2074 µs
#define ADC_CONFIG_VBUSCT_4120us  (0x07U << 9)   // 7h = 4120 µs

// VSHCT field (bits 8-6)
// Conversion time for shunt voltage measurement.
#define ADC_CONFIG_VSHCT_50us     0U            // 0h = 50 µs
#define ADC_CONFIG_VSHCT_84us     (0x01U << 6)   // 1h = 84 µs
#define ADC_CONFIG_VSHCT_150us    (0x02U << 6)   // 2h = 150 µs
#define ADC_CONFIG_VSHCT_280us    (0x03U << 6)   // 3h = 280 µs
#define ADC_CONFIG_VSHCT_540us    (0x04U << 6)   // 4h = 540 µs
#define ADC_CONFIG_VSHCT_1052us   (0x05U << 6)   // 5h = 1052 µs (default)
#define ADC_CONFIG_VSHCT_2074us   (0x06U << 6)   // 6h = 2074 µs
#define ADC_CONFIG_VSHCT_4120us   (0x07U << 6)   // 7h = 4120 µs

// VTCT field (bits 5-3)
// Conversion time for temperature measurement.
#define ADC_CONFIG_VTCT_50us      0U            // 0h = 50 µs
#define ADC_CONFIG_VTCT_84us      (0x01U << 3)   // 1h = 84 µs
#define ADC_CONFIG_VTCT_150us     (0x02U << 3)   // 2h = 150 µs
#define ADC_CONFIG_VTCT_280us     (0x03U << 3)   // 3h = 280 µs
#define ADC_CONFIG_VTCT_540us     (0x04U << 3)   // 4h = 540 µs
#define ADC_CONFIG_VTCT_1052us    (0x05U << 3)   // 5h = 1052 µs (default)
#define ADC_CONFIG_VTCT_2074us    (0x06U << 3)   // 6h = 2074 µs
#define ADC_CONFIG_VTCT_4120us    (0x07U << 3)   // 7h = 4120 µs

// AVG field (bits 2-0)
// ADC sample averaging count.
#define ADC_CONFIG_AVG_1          0U            // 0h = 1 sample (default)
#define ADC_CONFIG_AVG_4          (0x01U)   // 1h = 4 samples
#define ADC_CONFIG_AVG_16         (0x02U)   // 2h = 16 samples
#define ADC_CONFIG_AVG_64         (0x03U)   // 3h = 64 samples
#define ADC_CONFIG_AVG_128        (0x04U)   // 4h = 128 samples
#define ADC_CONFIG_AVG_256        (0x05U)   // 5h = 256 samples
#define ADC_CONFIG_AVG_512        (0x06U)   // 6h = 512 samples
#define ADC_CONFIG_AVG_1024       (0x07U)   // 7h = 1024 samples

// ADC_CONFIG (0x01) - ADC Configuration (16-bit, R/W)
#define W_ADC_CONFIG(Data)  WriteUInt(0x01U, (uint16_t)Data)      // Write to ADC Configuration register
#define R_ADC_CONFIG(Data)  ReadUInt(0x01U, (uint16_t *)Data)     // Read from ADC Configuration register

// SHUNT_CAL (0x02) - Shunt Calibration (16-bit, R/W)

// use function CalculateShuntCal(double rShunt, double maxExpectedCurrent) to record this value
#define W_SHUNT_CAL(Data)   WriteUInt(0x02U, (uint16_t)Data)      // Write to Shunt Calibration register
#define R_SHUNT_CAL(Data)   ReadUInt(0x02U, (uint16_t *)Data)     // Read from Shunt Calibration register

// VSHUNT (0x04) - Shunt Voltage Measurement (16-bit, Read-only)
// Shunt voltage:
// ±163.84 mV (ADCRANGE = 0) with a resolution of 5 µV/LSB 
// ±40.96 mV  (ADCRANGE = 1) with a resolution of 1.25 µV/LSB
#define R_VSHUNT(Data)      ReadInt(0x04U, (int16_t *)Data)     // Read Shunt Voltage

// VBUS (0x05) - Bus Voltage Measurement (16-bit, Read-only)
// Bus voltage: 
// 0 V to 85 V with a resolution of 3.125 mV/LSB
#define R_VBUS(Data)        ReadInt(0x05U, (int16_t *)Data)     // Read Bus Voltage

// DIETEMP (0x06) - Temperature Measurement (16-bit, Read-only)
// Temperature:
// –40 °C to +125 °C with a resolution of 125 m°C/LSB
#define Fahrenheit true
#define Celsius false
#define DegF true
#define DegC false
//#define R_DIETEMP(Data)     ReadInt(0x06U, (int16_t *)Data)     // Read Internal Die Temperature
#define R_DIETEMP(Data)     ReadInt(0x06U, (int16_t *)Data);  *(Data) = (abs(*(Data)))>>4    //
//#define R_UDIETEMP(Data)     ReadUInt(0x06U, (uint16_t *)Data)     // Read Internal Die Temperature

// CURRENT (0x07) - Current Result (16-bit, Read-only)
#define R_CURRENT(Data)     ReadInt(0x07U, (int16_t *)Data)     // Read Calculated Current

// POWER (0x08) - Power Result (24-bit, Read-only)
// Assumes a function "Read24" exists that handles 24-bit register reads.
#define R_POWER(Data)       ReadU24(0x08U, (uint32_t *)Data)       // Read Calculated Power (24-bit)


// -------------------------
// DIAG_ALRT Register Fields
// -------------------------

// Read and Write bits

// Bit 15: ALATCH (Alert Latch Enable)
// 0h = Transparent, 1h = Latched
#define DIAG_ALRT_ALATCH_TRANSPARENT    (0x00U)
#define DIAG_ALRT_ALATCH_LATCHED        (0x01U << 15)

// Bit 14: CNVR (Conversion Ready Flag on ALERT Pin)
// 0h = Disable, 1h = Enable conversion ready flag
#define DIAG_ALRT_CNVR_DISABLE          (0x00U)
#define DIAG_ALRT_CNVR_ENABLE           (0x01U << 14)

// Bit 13: SLOWALERT (Averaged vs Non-averaged Comparison)
// 0h = ADC (non-averaged) value, 1h = Averaged value
#define DIAG_ALRT_SLOWALERT_ADC         (0x00U)
#define DIAG_ALRT_SLOWALERT_AVERAGED    (0x01U << 13)

// Bit 12: APOL (Alert Polarity)
// 0h = Normal (Active-low, open-drain), 1h = Inverted (Active-high, open-drain)
#define DIAG_ALRT_APOL_NORMAL           (0x00U)
#define DIAG_ALRT_APOL_INVERTED         (0x01U << 12)

// Bits 11-10: RESERVED (Always 0) -- No macros necessary

// Read only Bits These Bits will reset to the event state so setting them to 0x0h is appropriate when writting the register

// The following Bits will be true when the event is triggered. 
// The ALERT pin will be triggered when the event is True 
// The DIAG_ALRT_CNVRF_READY bit is linked to the ALERT pin with DIAG_ALRT_CNVR_ENABLE bit set to 0x1
// use 
// Bit 9: MATHOF (Arithmetic Overflow Flag)
// 0h = Normal, 1h = Overflow
#define DIAG_ALRT_MATHOF_OVERFLOW       (0x01U << 9)

// Bit 8: RESERVED (Always 0) -- No macros necessary

// Bit 7: TMPOL (Temperature Over-Limit)
// 0h = Normal, 1h = Over Temperature Event
#define DIAG_ALRT_TMPOL_OVER_TEMP       (0x01U << 7)

// Bit 6: SHNTOL (Shunt Overvoltage)
// 0h = Normal, 1h = Over Shunt Voltage Event
#define DIAG_ALRT_SHNTOL_OVER_SHUNT     (0x01U << 6)

// Bit 5: SHNTUL (Shunt Undervoltage)
// 0h = Normal, 1h = Under Shunt Voltage Event
#define DIAG_ALRT_SHNTUL_UNDER_SHUNT    (0x01U << 5)

// Bit 4: BUSOL (Bus Overvoltage)
// 0h = Normal, 1h = Bus Over-Limit Event
#define DIAG_ALRT_BUSOL_OVER_BUS        (0x01U << 4)

// Bit 3: BUSUL (Bus Undervoltage)
// 0h = Normal, 1h = Bus Under-Limit Event
#define DIAG_ALRT_BUSUL_UNDER_BUS       (0x01U << 3)

// Bit 2: POL (Power Over-Limit)
// 0h = Normal, 1h = Power Over-Limit Event
#define DIAG_ALRT_POL_OVER_POWER        (0x01U << 2)

// Bit 1: CNVRF (Conversion Ready Flag)
// 0h = Normal, 1h = Conversion Complete
#define DIAG_ALRT_CNVRF_READY           (0x01U << 1)

// Bit 0: MEMSTAT (Memory Status)
// 0h = Memory Checksum Error, 1h = Normal Operation
#define DIAG_ALRT_MEMSTAT_CHECKSUM_ERROR (0x00U)
#define DIAG_ALRT_MEMSTAT_CHECKSUM_OK    (0x01U)

// DIAG_ALRT (0x0B) - Diagnostic Flags and Alert (16-bit, R/W)

#define W_DIAG_ALRT(Data)   WriteUInt(0x0BU, (uint16_t)Data)      // Write Diagnostic Flags and Alert register
#define R_DIAG_ALRT(Data)   ReadUInt(0x0BU, (uint16_t *)Data)     // Read Diagnostic Flags and Alert register

// Limit setpoints for alerting:

// SOVL (0x0C) - Shunt Overvoltage Threshold (16-bit, R/W)
#define W_SOVL(Data)        WriteUInt(0x0CU, (uint16_t)Data)      // Write Shunt Overvoltage Threshold
#define R_SOVL(Data)        ReadUInt(0x0CU, (uint16_t *)Data)     // Read Shunt Overvoltage Threshold
// Set_SOVL calculates and sets the Overvoltage Threshold
#define Set_SOVL(ADC_RANGE, OVER_VOLTAGE_VALUE) W_SOVL((uint16_t)(static_cast<float>(OVER_VOLTAGE_VALUE) / VSHUNT_CONVERSION_FACTOR(ADC_RANGE)))


// SUVL (0x0D) - Shunt Undervoltage Threshold (16-bit, R/W)
#define W_SUVL(Data)        WriteUInt(0x0DU, (uint16_t)Data)      // Write Shunt Undervoltage Threshold
#define R_SUVL(Data)        ReadUInt(0x0DU, (uint16_t *)Data)     // Read Shunt Undervoltage Threshold
// Set_SUVL calculates and sets the Shunt Undervoltage Threshold
#define Set_SUVL(ADC_RANGE, UNDER_VOLTAGE_VALUE) W_SUVL((uint16_t)(static_cast<float>(UNDER_VOLTAGE_VALUE) / VSHUNT_CONVERSION_FACTOR(ADC_RANGE)))
#define Set_SUVL_Using_Amps(ADC_RANGE,OVER_CURRENT_VALUE,SHUNT_RESISTANCE) W_SUVL(static_cast<uint16_t>((static_cast<float>(OVER_CURRENT_VALUE) * static_cast<float>(SHUNT_RESISTANCE)) / VSHUNT_CONVERSION_FACTOR(ADC_RANGE)))

// BOVL (0x0E) - Bus Overvoltage Threshold (16-bit, R/W)
#define W_BOVL(Data)        WriteUInt(0x0EU, (uint16_t)Data)      // Write Bus Overvoltage Threshold
#define R_BOVL(Data)        ReadUInt(0x0EU, (uint16_t *)Data)     // Read Bus Overvoltage Threshold
// Set_BOVL calculates and sets the Bus Overvoltage Threshold
#define Set_BOVL(OVER_VOLTAGE_VALUE) W_BOVL((uint16_t)(static_cast<float>(OVER_VOLTAGE_VALUE) / VBUS_CONVERSION_FACTOR))

// BUVL (0x0F) - Bus Undervoltage Threshold (16-bit, R/W)
#define W_BUVL(Data)        WriteUInt(0x0FU, (uint16_t)Data)      // Write Bus Undervoltage Threshold
#define R_BUVL(Data)        ReadUInt(0x0FU, (uint16_t *)Data)     // Read Bus Undervoltage Threshold
// Set_BUVL calculates and sets the Bus Undervoltage Threshold
#define Set_BUVL(UNDER_VOLTAGE_VALUE) W_BUVL((uint16_t)(static_cast<float>(UNDER_VOLTAGE_VALUE) / VBUS_CONVERSION_FACTOR))

// TEMP_LIMIT (0x10) - Temperature Over-Limit Threshold (16-bit, R/W)
#define W_TEMP_LIMIT(Data)  WriteUInt(0x10U, (uint16_t)Data)      // Write Temperature Over-Limit Threshold
#define R_TEMP_LIMIT(Data)  ReadUInt(0x10U, (uint16_t *)Data)     // Read Temperature Over-Limit Threshold
// Set_TOL calculates and sets the Bus Over Temperature Threshold
#define Set_TOL_C(OVER_TEMPERATURE_VALUE_C) W_TEMP_LIMIT((uint16_t)(static_cast<float>(OVER_TEMPERATURE_VALUE_C) / TEMP_CONVERSION_FACTOR))
#define Set_TOL_F(OVER_TEMPERATURE_VALUE_F) W_TEMP_LIMIT((uint16_t)(((static_cast<float>(OVER_TEMPERATURE_VALUE_F)-32)/1.8) / TEMP_CONVERSION_FACTOR))

// PWR_LIMIT (0x11) - Power Over-Limit Threshold (16-bit, R/W)
#define W_PWR_LIMIT(Data)   WriteUInt(0x11U, (uint16_t)Data)      // Write Power Over-Limit Threshold
#define R_PWR_LIMIT(Data)   ReadUInt(0x11U, (uint16_t *)Data)     // Read Power Over-Limit Threshold
// Set_POL calculates and sets the Power Threshold
#define Set_POL(OVER_POWER_VALUE_WATTS, Maximum_Expected_Current) W_PWR_LIMIT((uint16_t)(static_cast<float>(OVER_POWER_VALUE_WATTS) / (256 * POWER_CONVERSION_FACTOR(Maximum_Expected_Current))))


// MANUFACTURER_ID (0x3E) - Manufacturer ID (16-bit, Read-only)
// Expected value: 0x5449 ("TI" in ASCII)
#define R_MANUFACTURER_ID(Data) ReadUInt(0x3EU, (uint16_t *)Data)  // Read Manufacturer ID

// DEVICE_ID (0x3F) - Device ID (16-bit, Read-only)
// Expected default value: 0x2381
#define R_DEVICE_ID(Data)   ReadUInt(0x3FU, (uint16_t *)Data)     // Read Device ID
#define DEVICE_ID           0x2381U                               // INA238 DEVICE ID 
#define CHIP_ID             0x40U                                 // i2c address with A0 and A1 Grounded (Can Ranges from 0x80 ~ 0x8F)
#define ChIP_ID_END         0x4FU

#endif