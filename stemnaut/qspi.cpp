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
        if (!flash.readBuffer(0, max_address_bytes, 4)) {} // todo: log error

        // Address is stored as little-endian
        flash_max_address = (((uint32_t)max_address_bytes[3]) << 24) |
                            (((uint32_t)max_address_bytes[2]) << 16) |
                            (((uint32_t)max_address_bytes[1]) << 8)  |
                            ((uint32_t)max_address_bytes[3]);
    } else {
        // Zero out maximum address
        uint8_t zero_address[4] = {0, 0, 0, 0};
        if (!flash.writeBuffer(0, zero_address, 4)) {} // todo: log error
    }
}

bool qspi_read(uint8_t *buffer, uint32_t len) {
    if (!read_mode) return false; // wrong mode. todo: log
    if (flash_address + len > flash_max_address) return false; // out of bounds. todo: log

    uint32_t num_read = flash.readBuffer(flash_address, buffer, len);
    if (num_read == 0) {
        return false;
    } else {
        // From reading the source code, read always returns either 0 or len. So we can assume num_read == len.
        flash_address += len;
        return true;
    }
}

bool qspi_write(uint8_t *buffer, uint32_t len) {
    if (read_mode) return false; // wrong mode. todo: log
    if (flash_address + len > QSPI_FLASH_SIZE) return false; // out of bounds. todo: log
    
    uint32_t num_written = flash.writeBuffer(flash_address, buffer, len);
    flash_address += num_written;

    if (num_written != len) {
        // Number of bytes written was not what we requested. Likely an unrecoverable hardware failure. Todo: log.
        return false;
    } else {
        // Update max address in first 4 bytes. Address is stored as little-endian.
        uint8_t address_bytes[4];
        address_bytes[0] = flash_address & 0xFF;
        address_bytes[1] = (flash_address >> 8) & 0xFF;
        address_bytes[2] = (flash_address >> 16) & 0xFF;
        address_bytes[3] = (flash_address >> 24) & 0xFF;

        if (!flash.writeBuffer(0, address_bytes, 4)) {
            // Error updating max address in first 4 bytes. Likely hardware failure.
            return false;
        } 

        return true;
    }
}
