// Data recording steps:
// Attach with USB-c port facing inward toward the swimmer

#ifndef NRFX_QSPI_DEFAULT_CONFIG_IRQ_PRIORITY
#define NRFX_QSPI_DEFAULT_CONFIG_IRQ_PRIORITY 6
#endif

// Includes
#include "LSM6DS3.h"
#include "Wire.h"
#include "nrfx_qspi.h"
#include <FreeRTOS.h>
#include <Adafruit_TinyUSB.h>
#include <cstdio>

LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Struct to hold IMU data
#pragma pack(push, 1)
struct IMUData {
    unsigned long timestamp;
    float ax, ay, az;
    float gx, gy, gz;
};
#pragma pack(pop)

// IMU data queue
QueueHandle_t imuDataQueue;

// Data logging constants
const int DATA_LOG_FREQUENCY_HZ = 50; // Frequency to record data (Hz)
const int BUFFER_SIZE = 25; // Buffer Size
const int LINE_BUFFER_SIZE = 128; // Number if characters in a single buffer entry
const int LOG_INTERVAL_MS = 1000 / DATA_LOG_FREQUENCY_HZ;

// Data write address
uint32_t flash_write_addr = 0x1000;
const uint32_t data_sector_start_addr = 0x1000;
const uint32_t metadata_addr = 0x0;

// Tasks
void readIMUTask(void *pvParameters);
void writeDataTask(void *pvParameters);

void QSPI_WaitForReady() {
    while (nrfx_qspi_mem_busy_check() != NRFX_SUCCESS) {
        __WFI();
    }
}

void QSPI_Initialize() {
    nrfx_qspi_config_t config = NRFX_QSPI_DEFAULT_CONFIG(21, 25, 20, 24, 22, 23);
    nrfx_err_t status = nrfx_qspi_init(&config, nullptr, nullptr);
    if (status == NRFX_SUCCESS) {
        Serial.println("(QSPI) Successfully Initialized");
    } else if (status == NRFX_ERROR_TIMEOUT) {
        Serial.println("(QSPI) Timed Out");
    } else if (status == NRFX_ERROR_INVALID_STATE) {
        Serial.println("(QSPI) Driver Already Initialized");
    } else if (status == NRFX_ERROR_INVALID_PARAM) {
        Serial.println("(QSPI) Pin Configuration Incorrect");
    } else {
        Serial.println("(QSPI) Unknown Error");
    }
    QSPI_WaitForReady();

    nrf_qspi_cinstr_conf_t cinstr_cfg = {
        .opcode = 0x06,
        .length = NRF_QSPI_CINSTR_LEN_1B,
        .io2_level = true,
        .io3_level = true,
        .wipwait = false,
        .wren = true
    };
    status = nrfx_qspi_cinstr_xfer(&cinstr_cfg, nullptr, nullptr);
    if (status == NRFX_SUCCESS) {
        Serial.println("(QSPI) Configuration Transferred");
    } else if (status == NRFX_ERROR_TIMEOUT) {
        Serial.println("(QSPI) Memory Busy / Connection Issue");
    } else if (status == NRFX_ERROR_INVALID_STATE) {
        Serial.println("(QSPI) Driver Handling Other Operation");
    }
    QSPI_WaitForReady();
}

void QSPI_EraseSector(uint32_t address) {
    nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, address);
    QSPI_WaitForReady();
}

void QSPI_EraseChip() {
    nrfx_qspi_chip_erase();
    QSPI_WaitForReady();
}

void QSPI_SaveWriteAddr() {
    nrfx_qspi_write((uint8_t*)&flash_write_addr, sizeof(flash_write_addr), metadata_addr);
    QSPI_WaitForReady();
}

void QSPI_WriteData(const IMUData& data) {
    if (flash_write_addr % 4096 == 0) {
        QSPI_EraseSector(flash_write_addr);
    }

    nrfx_qspi_write((uint8_t*)&data, sizeof(IMUData), flash_write_addr);
    QSPI_WaitForReady();
    flash_write_addr += sizeof(IMUData);
    QSPI_SaveWriteAddr();
}

void QSPI_ResetQSPI() {
    QSPI_EraseChip();
    flash_write_addr = data_sector_start_addr;
    nrfx_qspi_write((uint8_t*)&flash_write_addr, sizeof(flash_write_addr), metadata_addr);
    QSPI_WaitForReady();
}

void QSPI_LoadWriteAddr() {
    nrfx_qspi_read((uint8_t*)&flash_write_addr, sizeof(flash_write_addr), metadata_addr);
    QSPI_WaitForReady();
}

void dumpFlashAsCSV() {
    IMUData data;
    uint32_t addr = data_sector_start_addr;

    Serial.print("Starting read at: 0x");
    Serial.println(addr, HEX);
    Serial.print("Stopping read at: 0x");
    Serial.println(flash_write_addr, HEX);

    while (addr < flash_write_addr) {
        nrfx_qspi_read((uint8_t*)&data, sizeof(IMUData), addr);
        QSPI_WaitForReady();

        Serial.printf("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            data.timestamp / 1000.0f,
            data.ax, data.ay, data.az,
            data.gx, data.gy, data.gz);

        addr += sizeof(IMUData);
    }
}

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
            Serial.println("Waiting for serial");
            vTaskDelay(10);
        }
        Serial.println("Serial found");

        Serial.println("Dumping flash memory as CSV");
        Serial.println("---FILE START---");
        dumpFlashAsCSV();
        Serial.println("---FILE END---");
        Serial.println("Done");

        Serial.println("Reformatting flash memory, resetting memory write addr to 0");
        QSPI_ResetQSPI();
        Serial.println("Reformat successful");


    // If the device is not connected to a computer: set LED to blue
    // While the IMU is starting up, switch to green, and start recording data.
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

        data.timestamp = millis();
        //data.ax = myIMU.readFloatAccelX();
        data.ax = 5.0;
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
    //int bufferIndex = 0;
    //char buffer[BUFFER_SIZE][LINE_BUFFER_SIZE];

    data.timestamp = millis();
    data.ax = 3.0;
    data.ay = 1.0;
    data.az = 1.0;
    data.gx = 1.0;
    data.gy = 2.0;
    data.gz = 1.0;

    QSPI_WriteData(data);

    for (;;) {
    //    if (xQueueReceive(imuDataQueue, &data, 0) == pdTRUE) {

    //        QSPI_WriteData(data);

            //snprintf(buffer[bufferIndex], LINE_BUFFER_SIZE, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
            //         data.timestamp / 1000.0f,
            //         data.ax, data.ay, data.az,
            //         data.gx, data.gy, data.gz);
            //bufferIndex++;

            data.timestamp = millis();
            data.ax = 3.0;
            data.ay = 1.0;
            data.az = 1.0;
            data.gx = 1.0;
            data.gy = 2.0;
            data.gz = 1.0;

            QSPI_WriteData(data);
            vTaskDelay(pdMS_TO_TICKS(20));

            //if (bufferIndex >= BUFFER_SIZE) {

            //    for (int i = 0; i < bufferIndex; i++) {
            //        dataFile.println(buffer[i]);
            //    }
                
            //    bufferIndex = 0;
            //}
        //}
    }
}