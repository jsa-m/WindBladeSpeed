//------------------------------------
// Wind Speed Estimation Application - Extended Version
//
// This section is responsible for reading and processing data from the
// ESP32, 9-axis sensor (accelerometer, magnetometer, gyroscope), and a 
// pressure sensor to estimate environmental parameters like wind speed. 
// The data is transmitted over BLE (Bluetooth Low Energy).
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
// Sensor data structure for all three-axis and environmental data
struct ThreeAxisSensorData {
    float aX, aY, aZ, gX, gY, gZ, mDirection, mX, mY, mZ, temp, pressure;
};

QueueHandle_t sensorQueue;

// BLE-related variables
bool deviceConnected = false;

// BLE server and service configuration
#define BLE_SERVER_NAME "ESP32-JSA"
#define SERVICE_UUID BLEUUID((uint16_t)0x181A)   // Environmental Sensing Service
BLECharacteristic accelCharacteristic1("8843c2c8-5b64-11ec-bf63-0242ac130002", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor accelDescriptor1(BLEUUID((uint16_t)0x2902));

BLECharacteristic accelCharacteristic2("f45902ea-60c1-11ec-8607-0242ac130002", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor accelDescriptor2(BLEUUID((uint16_t)0x2902));

BLECharacteristic accelCharacteristic3("0512bf0e-60c2-11ec-8607-0242ac130002", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor accelDescriptor3(BLEUUID((uint16_t)0x2902));

// BLE server callbacks to manage device connections
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

// Sensor data variables for BLE transmission
char accelXBuffer[10], dataBuffer1[18], dataBuffer2[18], dataBuffer3[18], fullDataBuffer[46];
char axisBuffer[20], tempBuffer[20];
char comma[2] = ",";
int16_t ax, ay, az;
uint32_t gx, gy, gz, mx, my, mz;
float tempFloat = 0;

// Task to capture data from the sensors
void captureThreeAxis(void *pvParameters) {
    // Local structure to store sensor data
    struct ThreeAxisSensorData sensorData;
    
    const TickType_t delay250ms = pdMS_TO_TICKS(250);
    uint8_t sampleCount = 0;

    // Sensor objects
    Adafruit_BMP280 pressureSensor;   // Pressure sensor
    MPU9250_asukiaaa imuSensor;       // 9-axis sensor (accelerometer, gyroscope, magnetometer)

    // Initialize sensors
#ifdef _ESP32_HAL_I2C_H_   // For ESP32
    Wire.begin(21, 22);
    imuSensor.setWire(&Wire);
#endif
    pressureSensor.begin();
    imuSensor.beginAccel();
    imuSensor.beginGyro();
    imuSensor.beginMag();

    // Main loop: gather sensor data
    while (1) {
        if (sampleCount < 4) {
            imuSensor.accelUpdate();
            imuSensor.gyroUpdate();
            imuSensor.magUpdate();

            // Accumulate readings over 4 cycles
            sensorData.aX += imuSensor.accelX();
            sensorData.aY += imuSensor.accelY();
            sensorData.aZ += imuSensor.accelZ();
            sensorData.gX += imuSensor.gyroX();
            sensorData.gY += imuSensor.gyroY();
            sensorData.gZ += imuSensor.gyroZ();
            sensorData.mDirection += imuSensor.magHorizDirection();
            sensorData.mX += imuSensor.magX();
            sensorData.mY += imuSensor.magY();
            sensorData.mZ += imuSensor.magZ();
            sensorData.temp += pressureSensor.readTemperature();
            sensorData.pressure += pressureSensor.readPressure();
            
            sampleCount++;
        } else {
            // Calculate average values after collecting 4 samples
            sensorData.aX /= 4;
            sensorData.aY /= 4;
            sensorData.aZ /= 4;
            sensorData.gX /= 4;
            sensorData.gY /= 4;
            sensorData.gZ /= 4;
            sensorData.mDirection /= 4;
            sensorData.mX /= 4;
            sensorData.mY /= 4;
            sensorData.mZ /= 4;
            sensorData.temp /= 4;
            sensorData.pressure /= 4;

            // Send averaged data to queue
            xQueueSendToBack(sensorQueue, (void*)&sensorData, 0);
            
            // Reset sensor data and counter
            memset(&sensorData, 0, sizeof(sensorData));
            sampleCount = 0;
        }

        vTaskDelay(delay250ms);
    }
}

// Task to send sensor data via BLE
void sendSensorDataBLE(void *pvParameters) {
    struct ThreeAxisSensorData receivedData;
    BaseType_t queueStatus;
    const TickType_t delay1s = pdMS_TO_TICKS(1000);

    // Initialize BLE device
    BLEDevice::init(BLE_SERVER_NAME);

    // Create BLE server and set callbacks
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create BLE service and characteristics
    BLEService *sensorService = pServer->createService(SERVICE_UUID);

    sensorService->addCharacteristic(&accelCharacteristic1);
    accelDescriptor1.setValue("Accelerometer Data");
    accelCharacteristic1.addDescriptor(&accelDescriptor1);

    sensorService->addCharacteristic(&accelCharacteristic2);
    accelDescriptor2.setValue("Gyroscope Data");
    accelCharacteristic2.addDescriptor(&accelDescriptor2);

    sensorService->addCharacteristic(&accelCharacteristic3);
    accelDescriptor3.setValue("Magnetometer Data");
    accelCharacteristic3.addDescriptor(&accelDescriptor3);

    // Start the BLE service and advertising
    sensorService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pServer->getAdvertising()->start();
    Serial.println("Waiting for client connection...");

    // Main loop: process and send sensor data via BLE
    while (1) {
        if (uxQueueSpacesAvailable != 0) {
            while ((xQueueReceive(sensorQueue, (void*)&receivedData, 0) == pdPASS)) {
                // Process sensor data here
            }
        }

        // Convert sensor data to integers and format as BLE payload
        tempFloat = (receivedData.aX + 16) * 1000;
        ax = (int16_t)tempFloat;
        tempFloat = (receivedData.aY + 16) * 1000;
        ay = (int16_t)tempFloat;
        tempFloat = (receivedData.aZ + 16) * 1000;
        az = (int16_t)tempFloat;
        tempFloat = (receivedData.gX + 2000) * 100;
        gx = (int32_t)tempFloat;
        tempFloat = (receivedData.gY + 2000) * 100;
        gy = (int32_t)tempFloat;
        tempFloat = (receivedData.gZ + 2000) * 100;
        gz = (int32_t)tempFloat;
        tempFloat = (receivedData.mX + 16384) * 100;
        mx = (int32_t)tempFloat;
        tempFloat = (receivedData.mY + 16384) * 100;
        my = (int32_t)tempFloat;
        tempFloat = (receivedData.mZ + 16384) * 100;
        mz = (int32_t)tempFloat;

        // Format sensor data into hexadecimal strings
        snprintf(dataBuffer1, sizeof(dataBuffer1), "%04x%04x%04x%05x", ax, ay, az, gx);
        snprintf(dataBuffer2, sizeof(dataBuffer2), "%05x%05x%06x", gy, gz, mx);
        snprintf(dataBuffer3, sizeof(dataBuffer3), "%06x%06x", my, mz);
        snprintf(fullDataBuffer, sizeof(fullDataBuffer), "%s%s%s", dataBuffer1, dataBuffer2, dataBuffer3);

        // Send sensor data via BLE if connected
        if (deviceConnected) {
            accelCharacteristic1.setValue(dataBuffer1);
            accelCharacteristic1.notify();

            accelCharacteristic2.setValue(dataBuffer2);
            accelCharacteristic2.notify();

            accelCharacteristic3.setValue(dataBuffer3);
            accelCharacteristic3.notify();
        }

        vTaskDelay(delay1s);
    }
}

void setup() {
    Serial.begin(9600);

    // Create queue for sensor data
    sensorQueue = xQueueCreate(1, sizeof(struct ThreeAxisSensorData));
    if (sensorQueue != NULL) {
        // Create tasks if the queue is successfully created
        xTaskCreate(captureThreeAxis, "CaptureThreeAxis", 2000, NULL, 1, NULL);
        xTaskCreate(sendSensorDataBLE, "SendSensorDataBLE", 3000, NULL, 1, NULL);
    }
}

void loop() {
    // Main loop intentionally left empty
}

