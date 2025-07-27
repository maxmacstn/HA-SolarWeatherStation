// #include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoHA.h>
#include <PMS.h>
#include <HardwareSerial.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include "SHTSensor.h"
#include "BLEDevice.h"



#define WIFI_STA_NAME "UniFi"
#define WIFI_STA_PASS "0891031231"

#define MQTT_SERVER   "192.168.1.15"
#define MQTT_PORT     1883
#define MQTT_USERNAME ""
#define MQTT_PASSWORD ""
#define MQTT_NAME     "Solar_weather_station"
#define DEEP_SLEEP_NORMAL 5     //Deep sleep for 5 mins if battery sufficient
#define DEEP_SLEEP_LOW 30       //Deep sleep for 30 mins if battery is running low
#define DEEP_SLEEP_EMPTY 120    //Deep sleep for 2 hr if battery ran out.
#define WIFI_TIMEOUT  30        //Do not connect Wi-Fi more than 30 seconds
// #define WDT_TIMEOUT 120         //All process should be finished within 2 minutes.
#define WDT_TIMEOUT  240        //If enable MiFlora, All process should be finished within 4 minutes.
#define STA_ID "WeatherStation_1"

#define LED_BUILTIN 5
#define DHT22_PIN 33
#define BATT_PIN 35
#define SENSOR_EN_PIN 13
#define TX2_PIN 25
#define RX2_PIN 26
#define CHARGE_EN 27

#define MAX_POSSIBLE_TEMP 45
#define MIN_POSSIBLE_TEMP 15
#define MAX_POSSIBLE_HUMID 100
#define MIN_POSSIBLE_HUMID  20

#define MI_FLORA_ENA 1
#define MI_FLORA_MAC "C4:7C:8D:6D:98:2A"
#define MI_FLORA_SCAN_DURATION 60
#define MI_FLORA_SCAN_ATTEMPT 2
// the remote service we wish to connect to
static BLEUUID serviceUUID("00001204-0000-1000-8000-00805f9b34fb");
// the characteristic of the remote service we are interested in
static BLEUUID uuid_version_battery("00001a02-0000-1000-8000-00805f9b34fb");
static BLEUUID uuid_sensor_data("00001a01-0000-1000-8000-00805f9b34fb");
static BLEUUID uuid_write_mode("00001a00-0000-1000-8000-00805f9b34fb");



WiFiClient client;
// PubSubClient mqtt(client);

//HA Lib
HAMqtt *haMQTT;
HADevice *haWeatherStationDevice;
HASensor *haSensor_vBatt;
HASensor *haSensor_SoC;
HASensor *haSensor_PM1_0;
HASensor *haSensor_PM2_5;
HASensor *haSensor_PM10_0;
HASensor *haSensor_temp;
HASensor *haSensor_humid;

HASensor *haSensor_MiFloraBatt;
HASensor *haSensor_MiFloraTemp;
HASensor *haSensor_MiFloraMoisture;
HASensor *haSensor_MiFloraLight;
HASensor *haSensor_MiFloraConductivity;



//Sensors
PMS pms(Serial2);
PMS::DATA data;
SHTSensor sht(SHTSensor::SHT3X);

//Sensor values
float vBatt = 0;
int SoC = 100;

bool pmsOK = false;
int PM1_0 = 0;
int PM2_5 = 0;
int PM10_0 = 0;

bool shtOK = false;
float temp = 0;
float humid = 0;

bool miFloraOK = false;
float miFlora_temp = 0;
int miFlora_moisture = 0;
int miFlora_light = 0;
int miFlora_battery = 0;
int miFlora_conductivity = 0;

enum BatteryCondition {BATT_NORMAL, BATT_LOW, BATT_EMPTY};

void deepSleep(BatteryCondition BatteryCondition){
  digitalWrite(LED_BUILTIN,LOW);
  
  switch (BatteryCondition)
  {
  case BATT_NORMAL:
    Serial.println("Start deep sleep (normal)");
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_NORMAL * 60  * 1000000);
    break;
  case BATT_LOW:
    Serial.println("Start deep sleep (batt low)");
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_LOW * 60  * 1000000);
    break;
  case BATT_EMPTY:
    Serial.println("Start deep sleep (batt empty)");
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_EMPTY * 60  * 1000000);
    break;
  default:
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_NORMAL * 60  * 1000000);
    break;
  }

  esp_deep_sleep_start();
}

int calculateSoC(float voltage){
    float BATT_MIN = 3.15;
    float BATT_MAX = 4.2;
    int soc = ((voltage - BATT_MIN) / (BATT_MAX - BATT_MIN)) * 100;
    if (soc > 100)
      soc = 100;
    if (soc < 0)
      soc = 0;

    return soc;
}

float getBattVoltage(){
  //read battery voltage per %
    long sum = 0;                  // sum of samples taken
    float voltage = 0.0;           // calculated voltage
    int soc ;            //SoC
    const float battery_max = 4.2; //maximum voltage of battery
    const float battery_min = 3.1; //minimum voltage of battery before shutdown

    float R1 = 100000.0; // resistance of R1 (100K)
    float R2 = 100000.0;  // resistance of R2 (100K)

    for (int i = 0; i < 500; i++)
    {
        sum += analogRead(BATT_PIN);
        delayMicroseconds(1000);
    }
    // calculate the voltage
    voltage = sum / (float)500;
    voltage = (voltage * 3.6) / 4096.0; //for internal 1.35v reference
   
    // Compensate Divider Circuit
    voltage = voltage / (R2/(R1+R2));

    if (voltage > 10 || voltage < 0)
      voltage = 0;

    return voltage;

}

bool fetchTempHumid(){

  if (sht.init()) {
      Serial.print("SHT success\n");
  } else {
      Serial.print("SHT failed\n");
      shtOK = false;
  }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);

  if(sht.readSample()){
    temp = sht.getTemperature();
    humid = sht.getHumidity();

    if( temp >= MAX_POSSIBLE_TEMP || temp <= MIN_POSSIBLE_TEMP || humid >= MAX_POSSIBLE_HUMID || humid <= MIN_POSSIBLE_HUMID){
      shtOK = false;
    }else{
      shtOK = true;
    }

  }else{
    shtOK = false;
  }


  return shtOK;

}

bool fetchPMS(){

  Serial.print("Warming up PMS Sensor");
  pms.passiveMode();    // Switch to passive mode
  for (int i = 0 ; i < 30; i ++)
  {
    Serial.print(".");
    if (pms.readUntil(data))
    {
      // Serial.print("PM 1.0 (ug/m3): ");
      // Serial.println(data.PM_AE_UG_1_0);

      // Serial.print("PM 2.5 (ug/m3): ");
      // Serial.println(data.PM_AE_UG_2_5);

      // Serial.print("PM 10.0 (ug/m3): ");
      // Serial.println(data.PM_AE_UG_10_0);

      PM1_0 = data.PM_AE_UG_1_0;
      PM2_5 = data.PM_AE_UG_2_5;
      PM10_0 = data.PM_AE_UG_10_0;
      pmsOK = true;
    }
    else
    {
      // Serial.println("No data.");
    }
    delay(1000);
  }

  Serial.println();
  return pmsOK;
}








BLEClient* getFloraClient(BLEAddress floraAddress) {
  BLEClient* floraClient = BLEDevice::createClient();

  if (!floraClient->connect(floraAddress)) {
    Serial.println("- Mi Flora Connection failed, skipping");
    return nullptr;
  }

  Serial.println("- Mi Flora Connection successful");
  return floraClient;
}

BLERemoteService* getFloraService(BLEClient* floraClient) {
  BLERemoteService* floraService = nullptr;

  try {
    floraService = floraClient->getService(serviceUUID);
  }
  catch (...) {
    // something went wrong
  }
  if (floraService == nullptr) {
    Serial.println("- Mi Flora Failed to find data service");
  }
  else {
    Serial.println("- Mi Flora Found data service");
  }

  return floraService;
}

bool forceFloraServiceDataMode(BLERemoteService* floraService) {
  BLERemoteCharacteristic* floraCharacteristic;
  
  // get device mode characteristic, needs to be changed to read data
  Serial.println("- Force device in data mode");
  floraCharacteristic = nullptr;
  try {
    floraCharacteristic = floraService->getCharacteristic(uuid_write_mode);
  }
  catch (...) {
    // something went wrong
  }
  if (floraCharacteristic == nullptr) {
    Serial.println("-- Failed, skipping device");
    return false;
  }

  // write the magic data
  uint8_t buf[2] = {0xA0, 0x1F};
  floraCharacteristic->writeValue(buf, 2, true);

  delay(500);
  return true;
}

bool readFloraDataCharacteristic(BLERemoteService* floraService) {
  BLERemoteCharacteristic* floraCharacteristic = nullptr;

  // get the main device data characteristic
  Serial.println("- Access characteristic from device");
  try {
    floraCharacteristic = floraService->getCharacteristic(uuid_sensor_data);
  }
  catch (...) {
    // something went wrong
  }
  if (floraCharacteristic == nullptr) {
    Serial.println("-- Failed, skipping device");
    return false;
  }

  // read characteristic value
  Serial.println("- Read value from characteristic");
  std::string value;
  try{
    value = floraCharacteristic->readValue();
  }
  catch (...) {
    // something went wrong
    Serial.println("-- Failed, skipping device");
    return false;
  }
  const char *val = value.c_str();

  Serial.print("Hex: ");
  for (int i = 0; i < 16; i++) {
    Serial.print((int)val[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");

  int16_t* temp_raw = (int16_t*)val;
  miFlora_temp = (*temp_raw) / ((float)10.0);
  Serial.print("-- Temperature: ");
  Serial.println(miFlora_temp);

  miFlora_moisture = val[7];
  Serial.print("-- Moisture: ");
  Serial.println(miFlora_moisture);

  miFlora_light = val[3] + val[4] * 256;
  Serial.print("-- Light: ");
  Serial.println(miFlora_light);
 
  miFlora_conductivity = val[8] + val[9] * 256;
  Serial.print("-- Conductivity: ");
  Serial.println(miFlora_conductivity);

  if (miFlora_temp > 60) {
    //Unresonable value
    return false;
  }

  miFloraOK = true;
  return true;
}

bool readFloraBatteryCharacteristic(BLERemoteService* floraService) {
  BLERemoteCharacteristic* floraCharacteristic = nullptr;

  // get the device battery characteristic
  Serial.println("- Access battery characteristic from device");
  try {
    floraCharacteristic = floraService->getCharacteristic(uuid_version_battery);
  }
  catch (...) {
    // something went wrong
  }
  if (floraCharacteristic == nullptr) {
    Serial.println("-- Failed, skipping battery level");
    return false;
  }

  // read characteristic value
  Serial.println("- Read value from characteristic");
  std::string value;
  try{
    value = floraCharacteristic->readValue();
  }
  catch (...) {
    // something went wrong
    Serial.println("-- Failed, skipping battery level");
    return false;
  }
  const char *val2 = value.c_str();
  miFlora_battery = val2[0];

  char buffer[64];

  Serial.print("-- MiFlora Battery: ");
  Serial.println(miFlora_battery);


  return true;
}

bool processFloraService(BLERemoteService* floraService, char* deviceMacAddress) {
  // set device in data mode
  if (!forceFloraServiceDataMode(floraService)) {
    return false;
  }

  bool dataSuccess = readFloraDataCharacteristic(floraService);
  bool batterySuccess = readFloraBatteryCharacteristic(floraService);

  return dataSuccess && batterySuccess;
}




bool processFloraDevice(BLEAddress floraAddress, char* deviceMacAddress, int tryCount) {
  Serial.print("Processing Flora device at ");
  Serial.print(floraAddress.toString().c_str());
  Serial.print(" (try ");
  Serial.print(tryCount);
  Serial.println(")");

  // connect to flora ble server
  BLEClient* floraClient = getFloraClient(floraAddress);
  if (floraClient == nullptr) {
    return false;
  }

  // connect data service
  BLERemoteService* floraService = getFloraService(floraClient);
  if (floraService == nullptr) {
    floraClient->disconnect();
    return false;
  }

  // process devices data
  bool success = processFloraService(floraService, deviceMacAddress);
  // disconnect from device
  floraClient->disconnect();

  return success;
}

bool fetchMiFlora(){
    //https://github.com/sidddy/flora/blob/master/flora/flora.ino


    BLEDevice::init("");
    BLEDevice::setPower(ESP_PWR_LVL_P7);

    char* deviceMacAddress = MI_FLORA_MAC;
    BLEAddress floraAddress(deviceMacAddress);
    uint8_t tryCount = 0;

    while (tryCount < MI_FLORA_SCAN_ATTEMPT) {
      tryCount++;
      if (processFloraDevice(floraAddress, deviceMacAddress, tryCount)) {
        miFloraOK = true;
        break;
      }
      delay(1000);
    }

    BLEDevice::deinit();

    return miFloraOK;

}

void fetchSensorValues(){

  Wire.begin();

  //Turn on PMS sensor and SHT sensor.
  digitalWrite(SENSOR_EN_PIN,1);

  if(!fetchPMS())
    Serial.println("PMS Sensor error");

  if(!fetchTempHumid())
    Serial.println("SHT Sensor error");

  if(MI_FLORA_ENA && fetchMiFlora()){
    Serial.println("Mi Flora Failed");
  }

  //Turn off PMS sensor and SHT sensor.
  digitalWrite(SENSOR_EN_PIN,LOW);
  
  Serial.println();
  Serial.println("---Weather Station Data---");
  Serial.printf("vBatt\t= %.2f\n", vBatt);
  Serial.printf("SoC\t= %-3d%\n", SoC);
  Serial.printf("PM1\t= %-3d\n", data.PM_AE_UG_1_0);
  Serial.printf("PM2.5\t= %-3d\n", data.PM_AE_UG_2_5);
  Serial.printf("PM10\t= %-3d\n", data.PM_AE_UG_10_0);
  Serial.printf("Temp\t= %.2fc\n", temp);
  Serial.printf("Humid\t= %2.f%\n", humid);

  if (MI_FLORA_ENA){
    Serial.println("---Mi Flora Data---");
    Serial.printf("Battery\t=  %-3d%\n", miFlora_battery);
    Serial.printf("Soil moisture\t=  %-3d%\n", miFlora_moisture);
    Serial.printf("Soil conductivity\t=  %dus/cm\n", miFlora_conductivity);
    Serial.printf("Temperature\t=  %.2fc\n", miFlora_temp);
    Serial.printf("Light\t=  %dlux\n", miFlora_light);

  }

 
}

bool connectAndSend(){

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_STA_NAME);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_STA_NAME, WIFI_STA_PASS);

  int connectCount = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    if(connectCount / 10 > WIFI_TIMEOUT){
      digitalWrite(LED_BUILTIN, connectCount%2);
      return false;
    }
    connectCount++;
  }

  //initilise Wi-Fi
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  delay(100);
  bool mqttRes = haMQTT->begin(MQTT_SERVER,MQTT_PORT, MQTT_USERNAME,MQTT_PASSWORD);
  if(!mqttRes){
    Serial.println("MQTT Connection Failed");
    return false;
  }
  delay(100);

  //Try to publish at least 10 times (in order to make haMQTT loop works)
  for(int i = 0; i < 10; i++){
    haSensor_vBatt->setValue(vBatt);
    haSensor_SoC->setValue(SoC);

    if (pmsOK){ //Sensor OK
      haSensor_PM1_0->setValue(PM1_0);
      haSensor_PM2_5->setValue(PM2_5);
      haSensor_PM10_0->setValue(PM10_0);
    }else{  //Sensor Failure
      haSensor_PM1_0->setAvailability(false);
      haSensor_PM2_5->setAvailability(false);
      haSensor_PM10_0->setAvailability(false);
    }

    if(shtOK){  //Sensor OK
      haSensor_temp->setValue(temp);
      haSensor_humid->setValue(humid);
    }else{  //Sensor Failure
      haSensor_temp->setAvailability(false);
      haSensor_humid->setAvailability(false);
    }

    if (MI_FLORA_ENA){
        if(miFloraOK){
          haSensor_MiFloraBatt->setValue(miFlora_battery);
          haSensor_MiFloraTemp->setValue(miFlora_temp);
          haSensor_MiFloraConductivity->setValue(miFlora_conductivity);
          haSensor_MiFloraLight->setValue(miFlora_light);
          haSensor_MiFloraMoisture->setValue(miFlora_moisture);
        }else{
          haSensor_MiFloraBatt->setAvailability(true);
          haSensor_MiFloraTemp->setAvailability(true);
          haSensor_MiFloraConductivity->setAvailability(true);
          haSensor_MiFloraMoisture->setAvailability(true);
          haSensor_MiFloraLight->setAvailability(true);
        }


    }

    haMQTT->loop();
    delay(10);
  }

  return true;

}


void setup() {

  //Config WDT
  esp_task_wdt_init(WDT_TIMEOUT, true); 
  esp_task_wdt_add(NULL); 

  Serial.begin(115200);
  Serial2.begin(9600,SERIAL_8N1, RX2_PIN, TX2_PIN);  // PMS Serial

  Serial.println();
  Serial.println("Starting Solar Weather Station");

  //Setup IO   
  analogSetAttenuation(ADC_11db);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BATT_PIN, INPUT);
  pinMode(SENSOR_EN_PIN,OUTPUT);
  // pinMode(CHARGE_EN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // digitalWrite(CHARGE_EN,LOW);

  //Check minimum battery level before turning on wi-fi
  delay(2000);
  vBatt = getBattVoltage();
  SoC = calculateSoC(vBatt);
  digitalWrite(CHARGE_EN,HIGH);

  if (SoC == 0){
    //Battery too low, can't do anything further.
    deepSleep(BATT_EMPTY);
  }

  
  //initialise HA MQTT Library
  char id[17] = STA_ID;

  haWeatherStationDevice = new HADevice(id);
  haMQTT = new HAMqtt(client, *haWeatherStationDevice);
  haSensor_vBatt = new HASensor("vBatt");
  haSensor_SoC = new HASensor("SoC");
  haSensor_PM1_0 = new HASensor("PM1_0");
  haSensor_PM2_5 = new HASensor("PM2_5");
  haSensor_PM10_0 = new HASensor("PM10_0");
  haSensor_temp = new HASensor("TEMP");
  haSensor_humid = new HASensor("HUMID");


  haWeatherStationDevice->setName("SolarWeatherStation");
  haWeatherStationDevice->setManufacturer("Magi");
  haWeatherStationDevice->setModel("ESP32 WROVER B");

  haSensor_vBatt->setUnitOfMeasurement("V");
  haSensor_vBatt->setDeviceClass("voltage");
  haSensor_vBatt->setIcon("mdi:alpha-v-circle-outline");
  haSensor_vBatt->setName("Battery voltage");

  haSensor_SoC->setUnitOfMeasurement("%");
  haSensor_SoC->setDeviceClass("battery");
  haSensor_SoC->setIcon("mdi:battery");
  haSensor_SoC->setName("Battery SoC");

  haSensor_PM1_0->setUnitOfMeasurement("µg/m³");
  haSensor_PM1_0->setDeviceClass("pm1");
  haSensor_PM1_0->setIcon("mdi:weather-fog");
  haSensor_PM1_0->setName("PM1");

  haSensor_PM2_5->setUnitOfMeasurement("µg/m³");
  haSensor_PM2_5->setDeviceClass("pm25");
  haSensor_PM2_5->setIcon("mdi:weather-fog");
  haSensor_PM2_5->setName("PM2.5");

  haSensor_PM10_0->setUnitOfMeasurement("µg/m³");
  haSensor_PM10_0->setDeviceClass("pm10");
  haSensor_PM10_0->setIcon("mdi:weather-fog");
  haSensor_PM10_0->setName("PM10");

  haSensor_temp->setUnitOfMeasurement("°C");
  haSensor_temp->setDeviceClass("temperature");
  haSensor_temp->setIcon("mdi:thermometer");
  haSensor_temp->setName("Temperature");

  haSensor_humid->setUnitOfMeasurement("%");
  haSensor_humid->setDeviceClass("humidity");
  haSensor_humid->setIcon("mdi:water-percent");
  haSensor_humid->setName("Humidity");


  if (MI_FLORA_ENA){
      haSensor_MiFloraBatt = new HASensor("miflora_vBatt");
      haSensor_MiFloraTemp = new HASensor("miflora_temp");
      haSensor_MiFloraMoisture = new HASensor("miflora_moisture");
      haSensor_MiFloraLight = new HASensor("miflora_light");
      haSensor_MiFloraConductivity = new HASensor("miflora_conductivity");


      haSensor_MiFloraBatt->setUnitOfMeasurement("%");
      haSensor_MiFloraBatt->setDeviceClass("battery");
      haSensor_MiFloraBatt->setIcon("mdi:battery");
      haSensor_MiFloraBatt->setName("Mi Flora Battery");

      haSensor_MiFloraTemp->setUnitOfMeasurement("°C");
      haSensor_MiFloraTemp->setDeviceClass("temperature");
      haSensor_MiFloraTemp->setIcon("mdi:thermometer");
      haSensor_MiFloraTemp->setName("Mi Flora temperature");

      haSensor_MiFloraMoisture->setUnitOfMeasurement("%");
      haSensor_MiFloraMoisture->setDeviceClass("moisture");
      haSensor_MiFloraMoisture->setIcon("mdi:water-percent");
      haSensor_MiFloraMoisture->setName("Mi Flora Moisture");

      haSensor_MiFloraLight->setUnitOfMeasurement("lx");
      haSensor_MiFloraLight->setDeviceClass("illuminance");
      haSensor_MiFloraLight->setIcon("mdi:weather-sunny");
      haSensor_MiFloraLight->setName("Mi Flora Light");

      haSensor_MiFloraConductivity->setUnitOfMeasurement("µS/cm");
      haSensor_MiFloraConductivity->setIcon("mdi:flower");
      haSensor_MiFloraConductivity->setName("Mi Flora Conductivity");

  }



  
  //Get values from sensors.
  fetchSensorValues();

  //Connect to wi-fi and send data.
  connectAndSend();

  //Deep Sleep
  if (SoC > 30){
    deepSleep(BATT_NORMAL);
  }
  else{
    deepSleep(BATT_LOW);
  }
}



void loop() {

}