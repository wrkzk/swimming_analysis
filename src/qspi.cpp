#include "qspi.h"
#include "nrfx_qspi.h"
#include "LSM6DS3.h" // Fix this - this allows the use of Serial, but not sure if its right library to use

// Memory addresses
uint32_t flash_write_addr = 0x0;                         // Active QSPI write address
const uint32_t metadata_addr = 0x0;             // Beginning of the metadata sector
const uint32_t data_sector_start_addr = 0x1000; // Beginning of the data sector

void QSPI_WaitForReady() {
    while (nrfx_qspi_mem_busy_check() != NRFX_SUCCESS) {
        __WFI();
    }
}

void QSPI_Initialize() {
    nrfx_qspi_config_t config = NRFX_QSPI_DEFAULT_CONFIG(21, 25, 20, 24, 22, 23);
    nrfx_qspi_init(&config, nullptr, nullptr);
    QSPI_WaitForReady();

    nrf_qspi_cinstr_conf_t cinstr_cfg = {
        .opcode = 0x06,
        .length = NRF_QSPI_CINSTR_LEN_1B,
        .io2_level = true,
        .io3_level = true,
        .wipwait = false,
        .wren = true
    };

    nrfx_qspi_cinstr_xfer(&cinstr_cfg, nullptr, nullptr);
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

    Serial.println("Beginning QSPI read...");
    Serial.print("Starting read at: 0x");
    Serial.println(addr, HEX);
    Serial.print("Stopping read at: 0x");
    Serial.println(flash_write_addr, HEX);

    Serial.println("---FILE START---");

    while (addr < flash_write_addr) {
        nrfx_qspi_read((uint8_t*)&data, sizeof(IMUData), addr);
        QSPI_WaitForReady();

        if (data.timestamp == 0xFFFFFF) {
            Serial.println("time_s,ax,ay,az,gx,gy,gz");
        } else {
            Serial.printf("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                data.timestamp / 1000.0f,
                data.ax, data.ay, data.az,
                data.gx, data.gy, data.gz);
        }

        addr += sizeof(IMUData);
    }

    Serial.println("---FILE END---");
}