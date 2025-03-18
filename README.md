# ESP32-based IoT Sensor System with MQTT and LCD Display

## Overview
This project showcases an ESP32-based IoT system that reads temperature and humidity data from an **HTU21D** sensor and transmits the data to an MQTT broker. Additionally, the ESP32 communicates with a **Topway HMT028ATB-C** touchscreen LCD using the **Topway SG Tool** for user interface design. The system enables real-time monitoring of environmental parameters with display and cloud integration.

## Features
- Reads **temperature and humidity** from the **HTU21D** sensor.
- Publishes sensor data to an **MQTT broker** (Mosquitto) for remote monitoring.
- Displays sensor readings on a **HMT028ATB-C touchscreen LCD**.
- Uses **Topway SG Tool** for designing touchscreen UI.
- Implements **UART communication** between ESP32 and the LCD.
- Supports **Wi-Fi connectivity** for IoT-based applications.

## Hardware Components
1. **ESP32** - Microcontroller with Wi-Fi and Bluetooth capabilities.
2. **HTU21D** - Temperature and Humidity sensor (I2C-based).
3. **HMT028ATB-C LCD** - Topway 2.8-inch touchscreen display.
4. **Jumper Wires** - For interconnections.
5. **Power Supply** - 5V/3.3V power source.

## Software Tools
- **Arduino IDE** - For programming ESP32.
- **Topway SG Tool** - For designing touchscreen LCD UI.
- **Mosquitto MQTT Broker** - For real-time data publishing.
- **Wire Library** - For I2C communication with HTU21D.
- **PubSubClient Library** - For MQTT communication.

## Wiring Diagram
| Component       | ESP32 Pin |
|---------------|----------|
| HTU21D SDA    | GPIO 13  |
| HTU21D SCL    | GPIO 15  |
| LCD TX (HMT028ATB-C) | GPIO 21  |
| LCD RX (HMT028ATB-C) | GPIO 22  |

## Code Breakdown
### 1. **Including Required Libraries**
```cpp
#include <Wire.h>
#include <Adafruit_HTU21DF.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>
```
- `Wire.h`: Enables I2C communication for the sensor.
- `Adafruit_HTU21DF.h`: Library for the HTU21D sensor.
- `HardwareSerial.h`: Allows UART communication with LCD.
- `WiFi.h` and `PubSubClient.h`: Manage Wi-Fi and MQTT communication.

### 2. **Wi-Fi and MQTT Configuration**
```cpp
const char* ssid = "ssid";
const char* password = "Password";
const char* mqtt_broker = "test.mosquitto.org";
const int   mqtt_port   = 1883;
const char* mqtt_sensor_topic = "esp32/sensor";
```
- Defines Wi-Fi credentials.
- Specifies MQTT broker details for data publishing.

### 3. **Setting Up UART for LCD Communication**
```cpp
#define TX_PIN 22
#define RX_PIN 21
HardwareSerial mySerial(1);
```
- Configures **UART1** for ESP32-LCD communication.

### 4. **Initializing HTU21D Sensor**
```cpp
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
```
- Creates an instance of the HTU21D sensor.

### 5. **Connecting to Wi-Fi**
```cpp
WiFi.begin(ssid, password);
while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
}
Serial.println("WiFi Connected");
```
- Establishes a Wi-Fi connection before proceeding.

### 6. **Reconnecting to MQTT Broker**
```cpp
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    String clientId = "ESP32_Client-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println(" connected");
    } else {
      delay(5000);
    }
  }
}
```
- Ensures ESP32 maintains a connection to the MQTT broker.

### 7. **Sending Data to LCD Display**
```cpp
void sendText(uint32_t address, String text) {
  uint8_t command[10 + text.length()];
  command[0] = 0xAA;
  command[1] = 0x44;
  command[2] = (address >> 24) & 0xFF;
  command[3] = (address >> 16) & 0xFF;
  command[4] = (address >> 8) & 0xFF;
  command[5] = address & 0xFF;
  for (size_t i = 0; i < text.length(); i++) {
    command[6 + i] = text[i];
  }
  command[6 + text.length()] = 0xCC;
  command[7 + text.length()] = 0x33;
  command[8 + text.length()] = 0xC3;
  command[9 + text.length()] = 0x3C;
  mySerial.write(command, sizeof(command));
}
```
- Formats and sends data to the LCD via UART.

### 8. **Reading and Publishing Sensor Data**
```cpp
float temperature = htu.readTemperature();
float humidity = htu.readHumidity();
String sensorPayload = "{\"temperature\":" + String(temperature, 2) + ",\"humidity\":" + String(humidity, 2) + "}";
mqttClient.publish(mqtt_sensor_topic, sensorPayload.c_str());
```
- Reads temperature and humidity data.
- Formats the data in JSON.
- Publishes it to the MQTT broker.

## Using Topway SG Tool for UI Design
**Topway SG Tool** is used to design the user interface on the **HMT028ATB-C LCD**. It enables:
- **Button creation** for user interactions.
- **Graphical elements** for displaying sensor values.
- **Serial commands** to update displayed text dynamically.

### Steps to Design UI:
1. Install **Topway SG Tool**.
2. Create a new project and select **HMT028ATB-C LCD**.
3. Add text elements to display temperature and humidity.
4. Assign serial communication addresses to text fields.
5. Export and upload the UI to the LCD module.

## Expected Output
- Temperature and humidity values will be displayed on the LCD.
- Data will be published to the MQTT broker.
- The LCD UI will update dynamically with new sensor readings.

## Future Enhancements
- Integrate **Cloud Dashboard** for real-time monitoring.
- Add **Touchscreen Buttons** for interactive control.
- Implement **Data Logging** and **Alerts** for critical thresholds.

## Conclusion
This project demonstrates how ESP32 can be used to read sensor data, send it to an MQTT broker, and display it on a touchscreen LCD. The integration of **Topway SG Tool** makes UI design simple and efficient.

