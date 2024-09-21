//------------------------------------
// BLE Client Application for Sensor Data
//
// This application connects to a BLE server (another ESP32) to retrieve
// sensor data using BLE notifications. It reads data from multiple sensors
// and combines it with local sensor data (anemometer readings) for transmission
// over a serial interface.
//
// Author: Joaquín Sopena (joasop@gmail.com)
//------------------------------------

#include "BLEDevice.h"

// Global Variables
char sensorData[51];      // Buffer to store combined sensor data
char adcVal[5], sensor1[18], sensor2[18], sensor3[18];  // Buffers for individual sensors

// BLE Server Configuration
//----------------------------------------------------------------------------------------
// BLE Server name (name of the other ESP32 running the server sketch)
#define BLE_SERVER_NAME "ESP32-JSA"

/* UUIDs of the service and characteristics we want to read */
// BLE Service UUID
#define BME_SERVICE_UUID BLEUUID((uint16_t)0x181A)  // Environmental Sensing

// BLE Characteristics UUIDs
static BLEUUID bmeSensor1CharUUID("8843c2c8-5b64-11ec-bf63-0242ac130002");
static BLEUUID bmeSensor2CharUUID("f45902ea-60c1-11ec-8607-0242ac130002");
static BLEUUID bmeSensor3CharUUID("0512bf0e-60c2-11ec-8607-0242ac130002");

// BLE connection flags
static boolean doConnect = false;
static boolean connected = false;

// Address of the peripheral device found during scanning
static BLEAddress *pServerAddress = nullptr;

// Characteristics we want to read from the BLE server
static BLERemoteCharacteristic* bmeSensor1Characteristic;
static BLERemoteCharacteristic* bmeSensor2Characteristic;
static BLERemoteCharacteristic* bmeSensor3Characteristic;

// Notification activation settings
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

// Variables to store sensor data received via BLE
char* sensor1Char;
char* sensor2Char;
char* sensor3Char;

// Flags to check whether new sensor readings are available
boolean newSensor1 = false, newSensor2 = false, newSensor3 = false;

// Function to connect to the BLE server and set up characteristics
bool connectToServer(BLEAddress pAddress) {
  BLEClient* pClient = BLEDevice::createClient();

  // Connect to the remote BLE server
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");

  // Get a reference to the desired service on the remote BLE server
  BLERemoteService* pRemoteService = pClient->getService(BME_SERVICE_UUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(BME_SERVICE_UUID.toString().c_str());
    return false;
  }

  // Get a reference to the characteristics in the service
  bmeSensor1Characteristic = pRemoteService->getCharacteristic(bmeSensor1CharUUID);
  bmeSensor2Characteristic = pRemoteService->getCharacteristic(bmeSensor2CharUUID);
  bmeSensor3Characteristic = pRemoteService->getCharacteristic(bmeSensor3CharUUID);

  if (!bmeSensor1Characteristic || !bmeSensor2Characteristic || !bmeSensor3Characteristic) {
    Serial.println("Failed to find one or more characteristic UUIDs.");
    return false;
  }
  Serial.println(" - Found our characteristics");

  // Register callbacks for notifications
  bmeSensor1Characteristic->registerForNotify(sensor1NotifyCallback);
  bmeSensor2Characteristic->registerForNotify(sensor2NotifyCallback);
  bmeSensor3Characteristic->registerForNotify(sensor3NotifyCallback);
  
  return true;
}

// BLE device callback to handle advertisements and connection
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      if (advertisedDevice.getName() == BLE_SERVER_NAME) {  // Check for the correct server
        advertisedDevice.getScan()->stop();  // Stop scanning
        pServerAddress = new BLEAddress(advertisedDevice.getAddress());  // Store the server address
        doConnect = true;  // Set flag to initiate connection
        Serial.println("Device found. Connecting!");
      }
    }
};

// Notification callbacks for each characteristic
//----------------------------------------------------------------------------------------

// Sensor 1 notification callback
static void sensor1NotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  sensor1Char = (char*)pData;
  newSensor1 = true;  // Set flag indicating new data is available
}

// Sensor 2 notification callback
static void sensor2NotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  sensor2Char = (char*)pData;
  newSensor2 = true;
}

// Sensor 3 notification callback
static void sensor3NotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  sensor3Char = (char*)pData;
  newSensor3 = true;
}

// Task to receive BLE data
void receiveDataBLE(void *pvParameters) {
  const TickType_t xDelay1s = pdMS_TO_TICKS(1000);
  TickType_t xLastWakeTime;

  // Initialize BLE device
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(60);  // Scan for 60 seconds

  while (1) {
    // If the flag "doConnect" is true, attempt to connect to the server
    if (doConnect) {
      if (connectToServer(*pServerAddress)) {
        Serial.println("We are now connected to the BLE Server.");
        
        // Enable notifications for each characteristic
        bmeSensor1Characteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue(notificationOn, 2, true);
        bmeSensor2Characteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue(notificationOn, 2, true);
        bmeSensor3Characteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue(notificationOn, 2, true);
        
        connected = true;
      } else {
        Serial.println("Failed to connect to the server. Restart to scan again.");
      }
      doConnect = false;
    }

    // Handle new sensor data
    if (newSensor1) {
      newSensor1 = false;
      snprintf(sensor1, sizeof(sensor1), "%s", sensor1Char);
    }
    if (newSensor2) {
      newSensor2 = false;
      snprintf(sensor2, sizeof(sensor2), "%s", sensor2Char);
    }
    if (newSensor3) {
      newSensor3 = false;
      snprintf(sensor3, sizeof(sensor3), "%s", sensor3Char);
    }

    vTaskDelayUntil(&xLastWakeTime, xDelay1s);
  }
}

// Task to capture anemometer data and send it over serial
void captureSendData(void *pvParameters) {
  const TickType_t xDelay1s = pdMS_TO_TICKS(1000);
  TickType_t xLastWakeTime;
  const uint16_t anemometerPin = 4;  // ADC2_CH0 pin
  uint16_t anemometerValue = 0;

  while (1) {
    // Capture data from anemometer (dummy reading for now)
    anemometerValue = analogRead(anemometerPin);
    
    // Format and send data over serial
    snprintf(adcVal, sizeof(adcVal), "%04x", anemometerValue);
    snprintf(sensorData, sizeof(sensorData), "%s%s%s%s", adcVal, sensor1, sensor2, sensor3);
    Serial.println(sensorData);
    
    vTaskDelayUntil(&xLastWakeTime, xDelay1s);
  }
}

// Setup function
void setup() {
  // Start serial communication
  Serial.begin(9600);
  Serial.println("Starting Arduino BLE Client application...");
  
  // Create tasks for receiving BLE data and capturing anemometer data
  xTaskCreate(receiveDataBLE, "receiveDataBLE", 3000, NULL, 2, NULL);
  xTaskCreate(captureSendData, "captureSendData", 2000, NULL, 1, NULL);
}

void loop() {
  // Main loop intentionally left empty
}

