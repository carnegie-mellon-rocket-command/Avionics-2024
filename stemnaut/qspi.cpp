/**
 * This code adapts MIT-license code, which was originally written by Ha Thach for Adafruit Industries, and
 * adapted by Matthieu Charbonnier. Source: https://gitlab.com/arduino5184213/seeed_xiao_nrf52840/flash_speedtest/-/blob/master/flash_speedtest.ino?ref_type=heads
 */
#include "qspi.h"

// Built from the P25Q16H datasheet.
static const SPIFlash_Device_t P25Q16H {
  .total_size = (1UL << 21), // 2MiB
  .start_up_time_us = 10000, // Don't know where to find that value
  
  .manufacturer_id = 0x85,
  .memory_type = 0x60,
  .capacity = 0x15,

  .max_clock_speed_mhz = 55,
  .quad_enable_bit_mask = 0x02, // Datasheet p. 27
  .has_sector_protection = 1,   // Datasheet p. 27
  .supports_fast_read = 1,      // Datasheet p. 29
  .supports_qspi = 1,           // Obviously
  .supports_qspi_writes = 1,    // Datasheet p. 41
  .write_status_register_split = 1, // Datasheet p. 28
  .single_status_byte = 0,      // 2 bytes
  .is_fram = 0,                 // Flash Memory
};

static Adafruit_FlashTransport_QSPI flashTransport;
static Adafruit_SPIFlash flash(&flashTransport);

static const uint32_t QSPI_FLASH_SIZE = (1UL << 21); // Total size of the chip: 2MiB.
static bool read_mode;                               // True = read mode, false = write mode.
static uint32_t flash_address;                       // Current address to read from / write to.
static uint32_t flash_max_address = 0x0;             // Maximum address to read up until, when in read mode.

void qspi_init_flash(bool read_mode_in) {
    // Initialize hardware if it hasn't been done already. Only needs to be done once.
    static bool initialized = false;
    if (!initialized) {
        flash.begin(&P25Q16H, 1);
        initialized = true;
    }
    
    // Reset address to beginning
    flash_address = 0x4;
    read_mode = read_mode_in;

    if (read_mode) {
        // Read in maximum address from first 4 bytes of memory.
        uint8_t max_address_bytes[4];
        if (!flash.readBuffer(0, max_address_bytes, 4)) {
            Serial.println("Error reading max address from QSPI flash in read mode startup (flash.readBuffer() returned 0).");
            return;
        }

        // Address is stored as little-endian
        flash_max_address = (((uint32_t)max_address_bytes[3]) << 24) |
                            (((uint32_t)max_address_bytes[2]) << 16) |
                            (((uint32_t)max_address_bytes[1]) << 8)  |
                            ((uint32_t)max_address_bytes[3]);
    } else {
        // Zero out maximum address
        uint8_t zero_address[4] = {0, 0, 0, 0};
        uint32_t num_written = flash.writeBuffer(0, zero_address, 4);
        if (num_written != 4) {
            char buf[256];
            snprintf(buf, 256, "Error zeroing out max address in QSPI flash in write mode startup (flash.writeBuffer() returned %d, 4 expected).", num_written);
            Serial.println(buf);
            return;
        }
    }
}

uint32_t qspi_data_length() {
    if (!read_mode) return 0;
    return flash_max_address - 0x04; // First 4 bytes are max address.
}

bool qspi_read(uint8_t *buffer, uint32_t len) {
    if (!read_mode) {
        Serial.println("Error: qspi_read called in write mode.");
        return false;
    } else if (flash_address + len > flash_max_address) {
        char buf[256];
        snprintf(buf, 256, "Error: qspi_read called with length greater than remaining data (current address: 0x%x, max address: 0x%x, length requested: 0x%x).", 
            flash_address, flash_max_address, len);
        Serial.print(buf);
        return false;
    }

    uint32_t num_read = flash.readBuffer(flash_address, buffer, len);
    if (num_read == 0) {
        Serial.println("Error reading from QSPI flash (flash.readBuffer() returned 0).");
        return false;
    } else {
        // From reading the source code, read always returns either 0 or len. So we can assume num_read == len.
        flash_address += len;
        return true;
    }
}

bool qspi_write(uint8_t *buffer, uint32_t len) {
    if (read_mode) {
        Serial.println("Error: qspi_write called in read mode.");
        return false;
    } else if (flash_address + len > QSPI_FLASH_SIZE) {
        char buf[256];
        snprintf(buf, 256, "Error: qspi_write called with length greater than remaining data (current address: 0x%x, flash size: 0x%x, length requested: 0x%x).", 
            flash_address, QSPI_FLASH_SIZE, len);
        Serial.print(buf);
        return false;
    }
    
    uint32_t num_written = flash.writeBuffer(flash_address, buffer, len);
    flash_address += num_written;

    if (num_written != len) {
        char buf[256];
        snprintf(buf, 256, "Error writing to QSPI flash (flash.writeBuffer() returned 0x%x, 0x%x expected).", num_written, len);
        Serial.println(buf);
        return false;
    } else {
        // Update max address in first 4 bytes. Address is stored as little-endian.
        uint8_t address_bytes[4];
        address_bytes[0] = flash_address & 0xFF;
        address_bytes[1] = (flash_address >> 8) & 0xFF;
        address_bytes[2] = (flash_address >> 16) & 0xFF;
        address_bytes[3] = (flash_address >> 24) & 0xFF;

        uint32_t num_written_address = flash.writeBuffer(0, address_bytes, 4);
        if (num_written_address != 4) {
            char buf[256];
            snprintf(buf, 256, "Error writing new max address (flash.writeBuffer() returned %d, 4 expected).", num_written_address);
            Serial.println(buf);
            return false;
        }

        return true;
    }
}
