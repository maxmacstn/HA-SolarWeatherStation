# Solar Weather Station
DIY ESP32 based weather station, fully supported Home Assistant via MQTT Discovery integration. It can measure Particulate matter (PM2.5, PM10, PM1), temperature, and humidity. 

This project utilise ESP32's deep sleep feature to minimise battery usage, and the unit will wake up and fetch sensors data at every 5 minutes interval.


![](https://github.com/maxmacstn/HA-SolarWeatherStation/blob/main/images/IMG_8655.jpg?raw=true)

## Features
- Low power: Able to operate without sunlight for at least 1 week, in case of rainy season.
- Fully Waterproof: Isolated compartment between sensors part and MCU part.
- Works with Home Assistant: the weather station will automagically added into Home Assistant via MQTT discovery.
- Adaptive sleeping: sleep 5 minutes in normal condition, 30 mins when battery is less than 30%, and completely stopped operating if the battery reach critical level.

## Components
- TTGO T18 ESP32 Development board
- CN3791 MPPT Charging Module
- 6V 6W Solar panel
- SHT3X Temperature & Humidity sensor
- PMS3003 Paticulate matter sensor
- PMS Sensor switching circuit
  - BC337 Transistor
  - 330Ω Resistor 
- 1x 18650 Li-ion battery
- 18650 battery holder
- 2x NANO-201CW 2x4x2" junction box
- PCB, Connectors, Wires, etc.

## Connection Diagram
![](https://raw.githubusercontent.com/maxmacstn/HA-SolarWeatherStation/main/images/connection_diagram.jpg)

## Platform and dependencies
The source code was written on PlatformIO IDE with Arduino framework.

Library used:
- [PubSubClient](https://github.com/knolleary/pubsubclient)
- [ArduinoHA](https://github.com/dawidchyrzynski/arduino-home-assistant)
- [PMS](https://github.com/fu-hsi/PMS)
- [Arduino SHT](https://github.com/Sensirion/arduino-sht)

## Changelog
- **2023.4 : Change temp&humid sensor to SHT sensor**
	- Use SHT3x sensor module  instead of DHT22. This significantly improves stability and accuracy.
	- Re-wire the sensor component box using I2C lines (instead of DHT22's one-wire protocol).
	- Use common ground for PMS sensor and SHT sensor.
	- Use single 18650 battery instead of two. (one battery can lasts more than one week without charging.) 
	
## Screenshots

### Home Assistant integration page
![](https://github.com/maxmacstn/HA-SolarWeatherStation/blob/main/images/ha-ss-1.png?raw=true)

### Grafana
![](https://github.com/maxmacstn/HA-SolarWeatherStation/blob/main/images/grafana-ss-1.png?raw=true)
From the data, the minimum battery SoC was at around 95% although it was cloudy for many days. In hindsight, using only single 18650 battery is probably sufficient. 
