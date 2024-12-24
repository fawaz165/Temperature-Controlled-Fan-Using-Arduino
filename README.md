# Temperature Controlled Fan Using Arduino

This project demonstrates the development of an energy-efficient system to regulate a fan's speed automatically based on room temperature. By integrating an Arduino Uno microcontroller, a DHT11 temperature and humidity sensor, and PWM (Pulse Width Modulation), the system dynamically adjusts fan speed and provides real-time feedback through an LCD display.

---

##  Overview

The **Temperature Controlled Fan Using Arduino** project showcases the application of embedded systems in energy efficiency and automation. It is designed to:
- Dynamically adjust fan speed based on real-time temperature readings.
- Enhance user experience with a 16x2 LCD for temperature and fan status display.
- Reduce energy consumption by running the fan only when necessary.

![System Architecture](https://github.com/fawaz165/Temperature-Controlled-Fan-Using-Arduino/blob/main/images/architecture.png)

---

##  Key Features

1. **Real-Time Temperature Monitoring**
   - Measures room temperature using a cost-effective DHT11 sensor.

2. **Dynamic Fan Speed Control**
   - Adjusts fan speed based on temperature using PWM signals.

3. **User Interface**
   - Displays temperature and fan speed on a 16x2 LCD.

4. **Energy Optimization**
   - Reduces power usage by regulating fan activity.

---


##  Components Used

### Hardware
- **Arduino Uno**: Microcontroller for processing and control.
- **DHT11 Sensor**: Temperature and humidity sensor.
- **DC Fan**: The actuator whose speed is controlled.
- **16x2 LCD**: Displays real-time temperature and fan status.
- **Potentiometer**: Adjusts LCD contrast.
- Resistors, wires, and a breadboard for connections.

### Software
- Embedded C/C++ code for the Arduino platform.
- Libraries:
  - `DHT.h` for DHT11 sensor interfacing.
  - `LiquidCrystal.h` for LCD control.

![Components](https://github.com/fawaz165/Temperature-Controlled-Fan-Using-Arduino/blob/main/images/components.png)

---

##  Implementation Details

1. **Phase 1: Basic Fan Control**
   - The DHT11 sensor reads the room temperature.
   - The Arduino uses PWM to control fan speed based on temperature thresholds.

![Phase 1](https://github.com/fawaz165/Temperature-Controlled-Fan-Using-Arduino/blob/main/images/phase%201%20hardware%20architecture.png)

2. **Phase 2: LCD Integration**
   - A 16x2 LCD displays real-time temperature and fan speed, enhancing user feedback.

![phase 2](https://github.com/fawaz165/Temperature-Controlled-Fan-Using-Arduino/blob/main/images/phase%202%20hardware%20architecture.png)

---

##  Applications

- **Server Rooms**: Prevents overheating of critical components.
- **Households**: Enhances comfort and reduces energy usage.
- **Industries**: Maintains optimal environmental conditions.

---

##  How to Use

1. **Hardware Setup**
   - Connect the DHT11 sensor, fan, and LCD display to the Arduino Uno as per the provided schematic.
   - Use a potentiometer to adjust LCD contrast.

2. **Software Setup**
   - Install the necessary libraries (`DHT.h` and `LiquidCrystal.h`) in the Arduino IDE.
   - Upload the provided code to the Arduino.

3. **Run the System**
   - Power the Arduino board.
   - Observe the real-time temperature readings and fan adjustments on the LCD.

---

##  Code Snippet

#include "DHT.h"
#include <LiquidCrystal.h>

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

#define DHTPIN 12     // Pin for DHT sensor
#define DHTTYPE DHT11 // Type of DHT sensor
#define PWM_PIN 9     // Pin for PWM control

byte degree[8] = 
{
    0b00011,
    0b00011,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000
};

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

void setup() {
    lcd.begin(16, 2);
    lcd.createChar(1, degree);
    lcd.clear();
    lcd.print("   Fan Speed  ");
    lcd.setCursor(0, 1);
    lcd.print("  Controlling ");
    delay(2000);
    analogWrite(PWM_PIN, 255);
    lcd.clear();
    lcd.print("Robu ");
    delay(2000);
    Serial.begin(9600);
    dht.begin();
}

void loop() {
    delay(2000);

    float humidity = dht.readHumidity();
    float tempC = dht.readTemperature();
    float tempF = dht.readTemperature(true);

    if (isnan(humidity) || isnan(tempC) || isnan(tempF)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }

    float heatIndex = dht.computeHeatIndex(tempF, humidity);

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print(" *C ");
    Serial.print(tempF);
    Serial.print(" *F\t");
    Serial.print("Heat Index: ");
    Serial.print(heatIndex);
    Serial.println(" *F");

    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(tempC);
    lcd.print(" C");
    lcd.setCursor(0, 1);

    if (tempC < 20) {
        analogWrite(PWM_PIN, 0);
        lcd.print("Fan OFF            ");
    } else if (tempC == 20) {
        analogWrite(PWM_PIN, 102);
        lcd.print("Fan Speed: 40%    ");
    } else if (tempC == 26) {
        analogWrite(PWM_PIN, 51);
        lcd.print("Fan Speed: 20%    ");
    } else if (tempC == 28) {
        analogWrite(PWM_PIN, 153);
        lcd.print("Fan Speed: 60%    ");
    } else if (tempC > 25) {
        analogWrite(PWM_PIN, 204);
        lcd.print("Fan Speed: 10%    ");
    } else if (tempC < 30) {
        analogWrite(PWM_PIN, 51);
        lcd.print("Fan Speed: 5%    ");
    }
    
    delay(3000);
}

# Output:
![Output](https://github.com/fawaz165/Temperature-Controlled-Fan-Using-Arduino/blob/main/images/phase%201%20output.png)

##  Outcome

- Provides an automated, energy-efficient fan control system.
- Prevents overheating of sensitive components.
- Ensures user comfort by regulating room temperature dynamically.

---

##  Conclusion

The project showcases the use of embedded systems in energy efficiency and automation, adjusting fan speed based on real-time temperature readings. The modular design allows for additional sensors and advanced control algorithms, showcasing Arduino's versatility in addressing modern challenges.

---
