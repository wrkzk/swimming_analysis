// Includes
#include "LSM6DS3.h"
#include "Wire.h"
#include "Adafruit_TinyUSB.h"
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <FreeRTOS.h>
#include <cstdio>

using namespace Adafruit_LittleFS_Namespace;
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Struct to hold IMU data
struct IMUData {
    unsigned long timestamp;
    float ax, ay, az;
    float gx, gy, gz;
};

// IMU data queue
QueueHandle_t imuDataQueue;

// Data logging constants
const int DATA_LOG_FREQUENCY_HZ = 50; // Frequency to record data (Hz)
const int BUFFER_SIZE = 50; // Buffer Size
const int LOG_INTERVAL_MS = 1000 / DATA_LOG_FREQUENCY_HZ;

// Tasks
void readIMUTask(void *pvParameters);
void writeDataTask(void *pvParameters);

// Runs once on initialization
void setup() {

    // Setup pins for output
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);

    // Initialize LEDs to an OFF state initially (they are active-LOW)
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);

    // Wait for 2.5 seconds to see if device is connected to usb
    unsigned long startTime = millis();
    bool usbMounted = false;
    
    while (millis() - startTime < 2500) {
        if (TinyUSBDevice.mounted()) {
            usbMounted = true;
            break;
        }
        delay(50);
    }

    // Start the filesystem
    if (!InternalFS.begin()){
        InternalFS.format();
        InternalFS.begin();
    }

    // If the device is connected to a computer: set LED to red, and wait for a serial
    // connection to begin printing out data. Use serial-reader script to save data to a CSV file.
    if (usbMounted) {
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_BLUE, HIGH);

        File data = InternalFS.open("/data.bin", FILE_O_READ);
        IMUData entry;

        if (data) {
            Serial.begin(115200);
            while (!Serial) {
                Serial.println("Waiting for serial");
                vTaskDelay(10);
            }
            Serial.println("Serial found");

            Serial.println("Reading from /data.bin:");
            Serial.println("---FILE START---");
           
            while (data.read((uint8_t*)&entry, sizeof(IMUData)) == sizeof(IMUData)) {
                if (entry.timestamp == 0xFFFFFFFF) {
                    Serial.println("time_s,ax,ay,az,gx,gy,gz");
                } else {
                    Serial.printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                        entry.timestamp / 1000.0f,
                        entry.ax, entry.ay, entry.az,
                        entry.gx, entry.gy, entry.gz);
                }
            }

            Serial.println("---FILE END---");
            Serial.println("Done");

            Serial.println("Removing /data.bin...");
            InternalFS.remove("/data.bin");
            InternalFS.format();
            Serial.println("Removed /data.bin successfully.");
        }

    // If the device is not connected to a computer: set LED to blue
    // While the IMU is starting up, switch LED to green, and start recording data.
    } else {
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_RED, HIGH);

        // Initialize the IMU and I2C connection
        Wire.begin();
        Wire.setClock(400000);
        delay(100);
        myIMU.begin();

        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_GREEN, LOW);

        File data = InternalFS.open("/data.bin", FILE_O_WRITE);
        data.seek(SEEK_END);
        
        // Write a divider line to separate separate data sessions
        IMUData divider;
        divider.timestamp = 0xFFFFFFFF;
        divider.ax = NAN;
        divider.ay = NAN;
        divider.az = NAN;
        divider.gx = NAN;
        divider.gy = NAN;
        divider.gz = NAN;

        data.write((uint8_t*)&divider, sizeof(IMUData));
        data.close();

        // Create the queue
        imuDataQueue = xQueueCreate(50, sizeof(IMUData));

        // Create the tasks
        xTaskCreate(readIMUTask, "Read IMU", 2048, NULL, 2, NULL);
        xTaskCreate(writeDataTask, "Write Data", 4096, NULL, 1, NULL);
        return;
    }
}

void loop() {}

// Task to continuously read data from the IMU at a specified interval
void readIMUTask(void *pvParameters) {
    IMUData data;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(LOG_INTERVAL_MS);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        data.timestamp = millis();
        data.ax = myIMU.readFloatAccelX();
        data.ay = myIMU.readFloatAccelY();
        data.az = myIMU.readFloatAccelZ();
        data.gx = myIMU.readFloatGyroX();
        data.gy = myIMU.readFloatGyroY();
        data.gz = myIMU.readFloatGyroZ();

        xQueueSend(imuDataQueue, &data, 0);
    }
}

// Task to continuously write data to a buffer, and write to the log
// file once the buffer has filled up
void writeDataTask(void *pvParameters) {
    IMUData data;
    IMUData buffer[BUFFER_SIZE];
    int bufferIndex = 0;

    File dataFile = InternalFS.open("/data.bin", FILE_O_WRITE);
    dataFile.seek(SEEK_END);

    for (;;) {
        if (xQueueReceive(imuDataQueue, &data, portMAX_DELAY) == pdTRUE) {

            buffer[bufferIndex] = data;
            bufferIndex++;

            if (bufferIndex >= BUFFER_SIZE) {

                for (int i = 0; i < bufferIndex; i++) {
                    dataFile.write((uint8_t*)&buffer[i], sizeof(IMUData));
                }

                dataFile.flush();
                bufferIndex = 0;
            }
        }
    }
    dataFile.close();
}