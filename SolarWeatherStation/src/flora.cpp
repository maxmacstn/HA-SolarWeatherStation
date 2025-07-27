

// BLEClient* getFloraClient(BLEAddress floraAddress) {
//   BLEClient* floraClient = BLEDevice::createClient();

//   if (!floraClient->connect(floraAddress)) {
//     Serial.println("- Connection failed, skipping");
//     return nullptr;
//   }

//   Serial.println("- Connection successful");
//   return floraClient;
// }

// BLERemoteService* getFloraService(BLEClient* floraClient) {
//   BLERemoteService* floraService = nullptr;

//   try {
//     floraService = floraClient->getService(serviceUUID);
//   }
//   catch (...) {
//     // something went wrong
//   }
//   if (floraService == nullptr) {
//     Serial.println("- Failed to find data service");
//   }
//   else {
//     Serial.println("- Found data service");
//   }

//   return floraService;
// }

// bool forceFloraServiceDataMode(BLERemoteService* floraService) {
//   BLERemoteCharacteristic* floraCharacteristic;
  
//   // get device mode characteristic, needs to be changed to read data
//   Serial.println("- Force device in data mode");
//   floraCharacteristic = nullptr;
//   try {
//     floraCharacteristic = floraService->getCharacteristic(uuid_write_mode);
//   }
//   catch (...) {
//     // something went wrong
//   }
//   if (floraCharacteristic == nullptr) {
//     Serial.println("-- Failed, skipping device");
//     return false;
//   }

//   // write the magic data
//   uint8_t buf[2] = {0xA0, 0x1F};
//   floraCharacteristic->writeValue(buf, 2, true);

//   delay(500);
//   return true;
// }

// bool readFloraDataCharacteristic(BLERemoteService* floraService, String baseTopic) {
//   BLERemoteCharacteristic* floraCharacteristic = nullptr;

//   // get the main device data characteristic
//   Serial.println("- Access characteristic from device");
//   try {
//     floraCharacteristic = floraService->getCharacteristic(uuid_sensor_data);
//   }
//   catch (...) {
//     // something went wrong
//   }
//   if (floraCharacteristic == nullptr) {
//     Serial.println("-- Failed, skipping device");
//     return false;
//   }

//   // read characteristic value
//   Serial.println("- Read value from characteristic");
//   std::string value;
//   try{
//     value = floraCharacteristic->readValue();
//   }
//   catch (...) {
//     // something went wrong
//     Serial.println("-- Failed, skipping device");
//     return false;
//   }
//   const char *val = value.c_str();

//   Serial.print("Hex: ");
//   for (int i = 0; i < 16; i++) {
//     Serial.print((int)val[i], HEX);
//     Serial.print(" ");
//   }
//   Serial.println(" ");

//   int16_t* temp_raw = (int16_t*)val;
//   float temperature = (*temp_raw) / ((float)10.0);
//   Serial.print("-- Temperature: ");
//   Serial.println(temperature);

//   int moisture = val[7];
//   Serial.print("-- Moisture: ");
//   Serial.println(moisture);

//   int light = val[3] + val[4] * 256;
//   Serial.print("-- Light: ");
//   Serial.println(light);
 
//   int conductivity = val[8] + val[9] * 256;
//   Serial.print("-- Conductivity: ");
//   Serial.println(conductivity);

//   if (temperature > 200) {
//     Serial.println("-- Unreasonable values received, skip publish");
//     return false;
//   }

//   char buffer[64];

//   snprintf(buffer, 64, "%f", temperature);
//   client.publish((baseTopic + "temperature").c_str(), buffer); 
//   snprintf(buffer, 64, "%d", moisture); 
//   client.publish((baseTopic + "moisture").c_str(), buffer);
//   snprintf(buffer, 64, "%d", light);
//   client.publish((baseTopic + "light").c_str(), buffer);
//   snprintf(buffer, 64, "%d", conductivity);
//   client.publish((baseTopic + "conductivity").c_str(), buffer);

//   return true;
// }

// bool readFloraBatteryCharacteristic(BLERemoteService* floraService, String baseTopic) {
//   BLERemoteCharacteristic* floraCharacteristic = nullptr;

//   // get the device battery characteristic
//   Serial.println("- Access battery characteristic from device");
//   try {
//     floraCharacteristic = floraService->getCharacteristic(uuid_version_battery);
//   }
//   catch (...) {
//     // something went wrong
//   }
//   if (floraCharacteristic == nullptr) {
//     Serial.println("-- Failed, skipping battery level");
//     return false;
//   }

//   // read characteristic value
//   Serial.println("- Read value from characteristic");
//   std::string value;
//   try{
//     value = floraCharacteristic->readValue();
//   }
//   catch (...) {
//     // something went wrong
//     Serial.println("-- Failed, skipping battery level");
//     return false;
//   }
//   const char *val2 = value.c_str();
//   int battery = val2[0];

//   char buffer[64];

//   Serial.print("-- Battery: ");
//   Serial.println(battery);
//   snprintf(buffer, 64, "%d", battery);
//   client.publish((baseTopic + "battery").c_str(), buffer);

//   return true;
// }

// bool processFloraService(BLERemoteService* floraService, char* deviceMacAddress, bool readBattery) {
//   // set device in data mode
//   if (!forceFloraServiceDataMode(floraService)) {
//     return false;
//   }

//   String baseTopic = MQTT_BASE_TOPIC + "/" + deviceMacAddress + "/";
//   bool dataSuccess = readFloraDataCharacteristic(floraService, baseTopic);

//   bool batterySuccess = true;
//   if (readBattery) {
//     batterySuccess = readFloraBatteryCharacteristic(floraService, baseTopic);
//   }

//   return dataSuccess && batterySuccess;
// }
