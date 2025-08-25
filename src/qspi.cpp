#include "qspi.h"
#include "nrfx_qspi.h"
#include "LSM6DS3.h" // Fix this - this allows the use of Serial, but not sure if its right library to use

// Memory addresses
uint32_t flash_write_addr = 0x1000;             // Active QSPI write address
const uint32_t metadata_addr = 0x0;             // Beginning of the metadata sector
const uint32_t data_sector_start_addr = 0x1000; // Beginning of the data sector
const uint32_t METADATA_SECTOR_SIZE = 4096;     // 4KB sector size

const uint32_t BLANK_WORD = 0xFFFFFFFFu;
const size_t META_ENTRY_SIZE = sizeof(MetaEntry);
const size_t MAX_META_ENTRIES = METADATA_SECTOR_SIZE / META_ENTRY_SIZE;

static inline bool is_blank(const MetaEntry &e) {
    return e.addr == BLANK_WORD && e.addr_inv == BLANK_WORD;
}

static inline bool is_valid(const MetaEntry &e) {
    return (e.addr ^ e.addr_inv) == 0xFFFFFFFFu;
}

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

void QSPI_EraseMetadataSector() {
    nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, metadata_addr);
    QSPI_WaitForReady();
}

void QSPI_SaveWriteAddr() {
    MetaEntry entry;
    uint32_t meta_ptr = metadata_addr;
    size_t index = 0;

    // Find the first blank metadata entry
    for (; index < MAX_META_ENTRIES; ++index, meta_ptr += META_ENTRY_SIZE) {
        nrfx_qspi_read(reinterpret_cast<uint8_t*>(&entry), sizeof(entry), meta_ptr);
        QSPI_WaitForReady();
        if (is_blank(entry)) break;
    }

    // If the metadata sector is full, then erase it
    if (index >= MAX_META_ENTRIES) {
        QSPI_EraseMetadataSector();
        meta_ptr = metadata_addr;
    }

    // Write a new metadata entry to save logging positiion
    MetaEntry new_entry;
    new_entry.addr = flash_write_addr;
    new_entry.addr_inv = ~flash_write_addr;

    nrfx_qspi_write(reinterpret_cast<uint8_t*>(&new_entry), sizeof(new_entry), meta_ptr);
    QSPI_WaitForReady();
}

void QSPI_WriteData(const IMUData& data) {
    nrfx_qspi_write((uint8_t*)&data, sizeof(IMUData), flash_write_addr);
    QSPI_WaitForReady();
    flash_write_addr += sizeof(IMUData);
}

void QSPI_ResetQSPI() {
    QSPI_EraseChip();
    flash_write_addr = data_sector_start_addr;
    QSPI_SaveWriteAddr();
}

void QSPI_LoadWriteAddr() {
    flash_write_addr = data_sector_start_addr;
    MetaEntry entry;
    MetaEntry last_valid = {BLANK_WORD, BLANK_WORD};

    // Load the last valid entry to find where to resume data logging
    uint32_t meta_ptr = metadata_addr;
    for (size_t i = 0; i < MAX_META_ENTRIES; ++i, meta_ptr += META_ENTRY_SIZE) {
        nrfx_qspi_read(reinterpret_cast<uint8_t*>(&entry), sizeof(entry), meta_ptr);
        QSPI_WaitForReady();
        if (is_blank(entry)) break;
        if (is_valid(entry)) last_valid = entry;
    }

    if (is_valid(last_valid)) {
        flash_write_addr = last_valid.addr;
    }
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

        if (data.timestamp == 0xFFFFFFFFu) {
            Serial.println("time_s,ax,ay,az,gx,gy,gz,qx,qy,qz,qw");
        } else {
            Serial.printf("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                data.timestamp / 1000.0f,
                data.ax, data.ay, data.az,
                data.gx, data.gy, data.gz,
                data.qx, data.qy, data.qz, data.qw);
        }

        addr += sizeof(IMUData);
    }

    Serial.println("---FILE END---");
}