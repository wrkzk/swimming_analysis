// Includes
#include "qspi.h"
#include "MPU9250.h"
#include "Wire.h"
#include <FreeRTOS.h>
#include <Adafruit_TinyUSB.h>
#include <cstdio>

// MPU9250 I2C Settings
#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0

// Instantiate IMU Class
MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

// IMU data queue
QueueHandle_t imuDataQueue;

// Data logging constants
const int DATA_LOG_FREQUENCY_HZ = 50; // Frequency to record data (Hz)
const int BUFFER_SIZE = 50;           // Buffer Size
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

    QSPI_Initialize();
    QSPI_LoadWriteAddr();

    // If the device is connected to a computer: set LED to red, and wait for a serial
    // connection to begin printing out data. Use serial-reader script to save data to a CSV file.
    if (usbMounted) {
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_BLUE, HIGH);

        Serial.begin(115200);
        while (!Serial) {
            Serial.println("Waiting for serial...");
            vTaskDelay(10);
        }
        Serial.println("Serial found.");
        
        // Output data
        dumpFlashAsCSV();

        Serial.println("\nErasing QSPI flash...");
        QSPI_ResetQSPI();
        Serial.println("Erase successful. Ready to log data.");


    // If the device is not connected to a computer: set LED to blue
    // While the IMU is starting up, switch to green, and start recording data.
    } else {
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_RED, HIGH);

        // Initialize the IMU and I2C connection
        Wire.begin();
        Wire.setClock(400000);
        delay(100);

        // Initialize and calibrate the MPU9250
        myIMU.MPU9250SelfTest(myIMU.selfTest);
        myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
        myIMU.initMPU9250();
        myIMU.initAK8963(myIMU.factoryMagCalibration);
        myIMU.getAres();
        myIMU.getGres();
        myIMU.getMres();

        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_GREEN, LOW);

        // Write header struct to memory
        IMUData header;
        header.timestamp = 0xFFFFFFFFu;
        header.ax = 0.0;
        header.ay = 0.0;
        header.az = 0.0;
        header.gx = 0.0;
        header.gy = 0.0;
        header.gz = 0.0;
        header.mx = 0.0;
        header.my = 0.0;
        header.mz = 0.0;

        QSPI_WriteData(header);
        QSPI_SaveWriteAddr();

        // Create the queue
        imuDataQueue = xQueueCreate(50, sizeof(IMUData));

        // Create the tasks
        xTaskCreate(readIMUTask, "Read IMU", 2048, NULL, 2, NULL);
        xTaskCreate(writeDataTask, "Write Data", 8192, NULL, 1, NULL);
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

        // Read IMU data
        myIMU.readAccelData(myIMU.accelCount);
        myIMU.readGyroData(myIMU.gyroCount);
        myIMU.readMagData(myIMU.magCount);

        // Write to struct
        data.timestamp = millis();
        data.ax = (float) myIMU.accelCount[0] * myIMU.aRes;
        data.ay = (float) myIMU.accelCount[1] * myIMU.aRes;
        data.az = (float) myIMU.accelCount[2] * myIMU.aRes;

        data.gx = (float) myIMU.gyroCount[0] * myIMU.gRes;
        data.gy = (float) myIMU.gyroCount[1] * myIMU.gRes;
        data.gz = (float) myIMU.gyroCount[2] * myIMU.gRes;

        data.mx = (float) myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
        data.my = (float) myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
        data.mz = (float) myIMU.magCount[2] * myIMU.mRes * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];

        xQueueSend(imuDataQueue, &data, 0);
    }
}

// Task to continuously write data to a buffer, and write to the log
// file once the buffer has filled up
void writeDataTask(void *pvParameters) {
    IMUData data;

    int bufferIndex = 0;
    IMUData dataBuffer[BUFFER_SIZE];

    for (;;) {
        if (xQueueReceive(imuDataQueue, &data, 0) == pdTRUE) {

            dataBuffer[bufferIndex++] = data;
            if (bufferIndex >= BUFFER_SIZE) {

                for (int i = 0; i < bufferIndex; i++) {
                    QSPI_WriteData(dataBuffer[i]);
                }
                
                QSPI_SaveWriteAddr();
                bufferIndex = 0;
            }
        }
    }
}