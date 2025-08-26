// Includes
#include "qspi.h"
#include "Wire.h"
#include <FreeRTOS.h>
#include <Adafruit_TinyUSB.h>
#include <Adafruit_BNO08x.h>
#include <cstdio>

// MPU9250 I2C Settings
#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0

// Instantiate IMU Class
Adafruit_BNO08x bno;
static const uint8_t BNO_ADDR = 0x4A;

// IMU data queue
QueueHandle_t imuDataQueue;

// Data logging constants
const int DATA_LOG_FREQUENCY_HZ = 40; // Frequency to record data (Hz)
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

        // Initialize the BNO08x IMU
        bno.begin_I2C(BNO_ADDR, &Wire);
        uint32_t interval_us = 1000000UL / DATA_LOG_FREQUENCY_HZ;
        uint32_t interval_us_slow = 1000000UL / 20;
        bno.enableReport(SH2_ACCELEROMETER, interval_us);
        bno.enableReport(SH2_GYROSCOPE_CALIBRATED, interval_us);
        bno.enableReport(SH2_ROTATION_VECTOR, interval_us_slow);

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
        header.qx = 0.0;
        header.qy = 0.0;
        header.qz = 0.0;
        header.qw = 0.0;

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
    sh2_SensorValue_t sensorValue;

    float latest_qx = 0.0, latest_qy = 0.0, latest_qz = 0.0, latest_qw = 0.0;

    // These flags are persistent. They are only reset after a full packet is sent.
    bool hasAccel = false;
    bool hasGyro = false;

    for (;;) {
        // This task will now run continuously, cooperatively checking for data.
        
        if (bno.getSensorEvent(&sensorValue)) {
            // A new report has arrived from the sensor.
            switch (sensorValue.sensorId) {
                case SH2_ACCELEROMETER:
                    data.ax = sensorValue.un.accelerometer.x;
                    data.ay = sensorValue.un.accelerometer.y;
                    data.az = sensorValue.un.accelerometer.z;
                    hasAccel = true;
                    break;
                case SH2_GYROSCOPE_CALIBRATED:
                    data.gx = sensorValue.un.gyroscope.x;
                    data.gy = sensorValue.un.gyroscope.y;
                    data.gz = sensorValue.un.gyroscope.z;
                    hasGyro = true;
                    break;
                case SH2_ROTATION_VECTOR:
                    latest_qx = sensorValue.un.rotationVector.i;
                    latest_qy = sensorValue.un.rotationVector.j;
                    latest_qz = sensorValue.un.rotationVector.k;
                    latest_qw = sensorValue.un.rotationVector.real;
                    break;
            }

            // Check if we have collected a complete set of data.
            if (hasAccel && hasGyro) {
                data.qx = latest_qx;
                data.qy = latest_qy;
                data.qz = latest_qz;
                data.qw = latest_qw;

                data.timestamp = xTaskGetTickCount();
                xQueueSend(imuDataQueue, &data, 0);

                // Reset the flags to begin assembling the next packet.
                hasAccel = false;
                hasGyro = false;
            }
        }
        
        // A small delay is crucial to prevent this task from starving all others.
        // This gives the I2C driver and other system tasks time to run.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Task to continuously write data to a buffer, and write to the log
// file once the buffer has filled up
void writeDataTask(void *pvParameters) {
    IMUData data;

    int bufferIndex = 0;
    IMUData dataBuffer[BUFFER_SIZE];

    for (;;) {
        if (xQueueReceive(imuDataQueue, &data, portMAX_DELAY) == pdTRUE) {

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