#ifndef QSPI_H
#define QSPI_H

// Not entirely sure why this is necessary
#ifndef NRFX_QSPI_DEFAULT_CONFIG_IRQ_PRIORITY
#define NRFX_QSPI_DEFAULT_CONFIG_IRQ_PRIORITY 6
#endif

// Struct to hold IMU data
#pragma pack(push, 1)
struct IMUData {
    unsigned long timestamp;
    float ax, ay, az;
    float gx, gy, gz;
};
#pragma pack(pop)

// Wait for QSPI memory to not be busy
void QSPI_WaitForReady();

// Initialize QSPI flash memory
void QSPI_Initialize();

// Erase entire QSPI flash memory
void QSPI_EraseChip();

// Save the active memory address to the metadata sector
void QSPI_SaveWriteAddr();

// Write a single data entry to flash memory
// Update active memory address, and save it
void QSPI_WriteData(const IMUData& data);

// After data has been printed to Serial, reset to prepare for next logging session
void QSPI_ResetQSPI();

// Load the active memory address from the metadata sector to resume logging
void QSPI_LoadWriteAddr();

// Output data as CSV to Serial
// Use included python script to read Serial output and write to CSV file
void dumpFlashAsCSV();

#endif