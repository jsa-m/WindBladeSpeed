//------------------------------------
// Wind Speed Estimation Application
//
// This application estimates wind speed using an ESP32 and a 9-axis sensor
// (accelerometer, magnetometer, and gyroscope). The device must be attached
// to a rotational object (e.g., windmill blades) to gather 9-axis data,
// which, after a prior calibration, is used to calculate wind speed.
// Once the wind speed is estimated, it is sent via a BLE notification.
//
// Author: Joaquín Sopena (joasop@gmail.com)
//------------------------------------

#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <string.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Global Variables
// Sensor data structure to store accelerometer and gyroscope values
struct ThreeAxisSensorData {
    float aX, gY;
};

QueueHandle_t sensorQueue;

// BLE-related variables
bool deviceConnected = false;

// BLE server and service configuration
#define BLE_SERVER_NAME "ESP32-JSA"
#define SERVICE_UUID BLEUUID((uint16_t)0x181A)   // Environmental Sensing Service
BLECharacteristic accelXCharacteristic(BLEUUID((uint16_t)0x2A70), BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor accelXDescriptor(BLEUUID((uint16_t)0x2902));
BLECharacteristic gyroYCharacteristic(BLEUUID((uint16_t)0x2A72), BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor gyroYDescriptor(BLEUUID((uint16_t)0x2902));

// BLE server callbacks to manage device connections
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

// Sensor data transmission variables
char accelBuffer[10], speedBufferAX[18], speedBufferGY[18];
uint16_t gyroY = 0, accelX = 0;

// Task to capture accelerometer and gyroscope data
void captureSensorData(void *pvParameters) {
    struct ThreeAxisSensorData sensorData;
    const TickType_t delay = pdMS_TO_TICKS(250);  // 250ms delay
    uint8_t sampleCount = 0;

    // Initialize sensors
    Adafruit_BMP280 pressureSensor;
    MPU9250_asukiaaa imuSensor;  // 9-axis sensor: accelerometer, gyroscope, magnetometer

    Wire.begin(21, 22);
    imuSensor.setWire(&Wire);
    pressureSensor.begin();
    imuSensor.beginAccel();
    imuSensor.beginGyro();

    // Main loop: collect sensor data
    while (1) {
        if (sampleCount < 4) {
            imuSensor.accelUpdate();
            imuSensor.gyroUpdate();
            sensorData.aX += imuSensor.accelX();
            sensorData.gY += imuSensor.gyroY();
            sampleCount++;
        } else {
            // Average and send data after collecting 4 samples
            sensorData.aX /= 4;
            sensorData.gY /= 4;
            xQueueSendToBack(sensorQueue, (void *)&sensorData, 0);
            memset(&sensorData, 0, sizeof(sensorData));  // Reset sensor data
            sampleCount = 0;
        }
        vTaskDelay(delay);
    }
}

// Task to send sensor data over BLE
void sendSensorDataBLE(void *pvParameters) {
    struct ThreeAxisSensorData receivedData;
    BaseType_t queueStatus;
    const TickType_t delay = pdMS_TO_TICKS(1000);  // 1-second delay

    float accelReading, gyroReading;
    float windSpeedAX = 0, windSpeedGY = 0;

    // Initialize BLE
    BLEDevice::init(BLE_SERVER_NAME);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create BLE service and characteristics
    BLEService *sensorService = pServer->createService(SERVICE_UUID);
    sensorService->addCharacteristic(&accelXCharacteristic);
    accelXCharacteristic.addDescriptor(&accelXDescriptor);
    sensorService->addCharacteristic(&gyroYCharacteristic);
    gyroYCharacteristic.addDescriptor(&gyroYDescriptor);

    // Start the BLE service
    sensorService->start();

    // Start BLE advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pServer->getAdvertising()->start();
    Serial.println("Waiting for client connection...");

    // Main loop: process and send sensor data over BLE
    while (1) {
        if (uxQueueSpacesAvailable != 0) {
            while ((xQueueReceive(sensorQueue, (void *)&receivedData, 0) == pdPASS)) {
                // Process data from queue
            }
        }

        // Calculate wind speed based on accelerometer and gyroscope readings
        gyroReading = (receivedData.gY + 2000);
        accelReading = (receivedData.aX + 16);
        windSpeedAX = -0.0623 * accelReading * accelReading + 3.0295 * accelReading - 32.592;
        windSpeedGY = -0.0157 * gyroReading + 32.914;

        // Debugging output
        Serial.print(windSpeedAX);
        Serial.print(" ");
        Serial.println(windSpeedGY);

        // Prepare data for BLE transmission
        snprintf(speedBufferAX, sizeof(speedBufferAX), "%.3f", windSpeedAX);
        snprintf(speedBufferGY, sizeof(speedBufferGY), "%.3f", windSpeedGY);

        // Send data via BLE if connected
        if (deviceConnected) {
            accelXCharacteristic.setValue(speedBufferAX);
            accelXCharacteristic.notify();
            gyroYCharacteristic.setValue(speedBufferGY);
            gyroYCharacteristic.notify();
        }

        vTaskDelay(delay);
    }
}

void setup() {
    Serial.begin(9600);

    // Create queue for sensor data
    sensorQueue = xQueueCreate(1, sizeof(struct ThreeAxisSensorData));
    if (sensorQueue != NULL) {
        // Create tasks if the queue is successfully created
        xTaskCreate(captureSensorData, "CaptureSensorData", 2000, NULL, 1, NULL);
        xTaskCreate(sendSensorDataBLE, "SendSensorDataBLE", 3000, NULL, 1, NULL);
    }
}

void loop() {
    // Main loop left intentionally empty
}

