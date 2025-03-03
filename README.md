# Simple_INA238

**Easily Access the INA238’s Registers and Measured Values on Arduino**

The **INA238** is an ultra-precise digital power monitor with a 16-bit delta-sigma ADC, designed for high-accuracy current-sensing applications. With this Arduino library, you can quickly configure the INA238, read measurements (current, bus voltage, temperature, and power), and harness the device’s ultra-low offset, wide common-mode range, and built-in temperature sensor—all with minimal code.

## Key Features

- **High-Resolution 16-bit Delta-Sigma ADC**  
- **Current Monitoring Accuracy**  
  - Offset Voltage: ±5 µV (max)  
  - Offset Drift: ±0.02 µV/°C (max)  
  - Gain Error: ±0.1% (max)  
  - Gain Error Drift: ±25 ppm/°C (max)  
  - Common Mode Rejection: 140 dB (min)  
- **Power Monitoring Accuracy**  
  - 0.7% full scale (–40°C to +125°C)  
- **Fast Alert Response**: 75 µs  
- **Wide Common-Mode Range**: –0.3 V to +85 V  
- **Bus Voltage Sense Input**: 0 V to 85 V  
- **Shunt Full-Scale Differential Range**: ±163.84 mV / ±40.96 mV  
- **Extremely Low Input Bias Current**: 2.5 nA (max)  
- **Integrated Temperature Sensor**: ±1°C accuracy (at 25°C)  
- **Programmable Conversion Time & Averaging**  
- **High-Speed I2C Interface** (2.94 MHz) with 16 Pin-Selectable Addresses  
- **Low-Power Operation**  
  - Supply Voltage: 2.7 V to 5.5 V  
  - Operational Current: 640 µA (typ)  
  - Shutdown Current: 5 µA (max)

## Why Use the INA238?

1. **Ultra-Low Offset & Drift**  
   Enables accurate measurements from micro-amps to kilo-amps without significant shunt power loss.
2. **Wide Dynamic Range**  
   ±163.84 mV or ±40.96 mV full-scale input allows you to sense a broad range of current levels.
3. **Built-In Power Calculations**  
   Automatically calculates current, voltage, temperature, and power—reducing microcontroller overhead.
4. **High-Speed & Programmable**  
   Configure conversion times (50 µs to 4.12 ms) and averaging (1x to 1024x) for optimized noise vs. speed.

## Getting Started

### Installation

1. **Using the Arduino IDE Library Manager**  
   - In the Arduino IDE, go to **Sketch → Include Library → Manage Libraries...**  
   - Search for “Simple_INA238” and install the latest version.
   
2. **Manual Installation**  
   - Download or clone this repository.  
   - Move the `Simple_INA238` folder into your `Documents/Arduino/libraries` directory.  
   - Restart the Arduino IDE if it’s open.

### Basic Usage

Here’s a quick example of how to initialize and read values from the INA238 using this library:

```cpp
#include <Wire.h>
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

  if(!ina238.begin(SDA_PIN, SCL_PIN).SetAddress(ina238Address).Check_Address()){
    Serial.println(F("Simple_INA238 failed to find device! \n !!! STOP !!!"));
    while (1);
  }

  ina238.Simple_Config(Maximum_Expected_Current,Shunt_Resistor_Value);
  ina238.setOnConversionReadyCallback(ShowReadings);
  Serial.println("INA238 initialized!");
}

void loop() {
  ina238.Alert(DelayBetweenDisplayingReadings); // Callbacks are triggered withing this alert function
}

```

- **`ina238.begin(SDA_PIN, SCL_PIN).SetAddress(ina238Address).Check_Address()`**: Initializes the sensor at the specified I2C address and performs a connection test.  
- **`Current()`**, **`VoltageBus()`**, **`Power()`**, **`Temperature()`**: High-level functions to retrieve sensor data.

## Documentation

- **Library Reference**: Check out the inline comments in the source files for detailed usage and advanced configuration.  
- **INA238 Datasheet**: For a deep dive into register definitions and electrical characteristics, see the [official datasheet](https://www.ti.com/product/INA238).

## Contributing

1. **Fork** the repo on GitHub.  
2. **Create** a feature branch for your changes (`git checkout -b my-new-feature`).  
3. **Commit** your changes.  
4. **Open** a Pull Request, describing what you’ve changed or added.

All contributions—bug reports, new features, and documentation improvements—are welcome!

## License

This project is licensed under the [MIT License](LICENSE). See the `LICENSE` file for details.

---

### Happy Measuring!

Enjoy using **Simple_INA238** to accurately measure current, voltage, power, and temperature in your Arduino projects. If you have questions or run into issues, please open a GitHub issue, and we’ll be glad to help!
