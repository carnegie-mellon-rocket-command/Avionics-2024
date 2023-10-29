/**
 * Functionality related to the Seeed XIAO nRF52840 Sense's inbuild 2MB of QSPI flash (specifically,
 * 2 MiB). This header provides a file descriptor-like interface to the QSPI flash.
 * 
 * This file allows the flash to operate in two modes: read mode, and write mode. Both offer a
 * sequential one-time pass through the flash memory, with either reading or writing supported.
 * Which mode is active is supplied when qspi_init_flash() is called. 
 * 
 * Note that these modes are not limitations of the QSPI flash, but rather limitations imposed
 * by this interface. The motivation for this is code correctness - for our particular use case,
 * it does not make sense to both read and write, or to read/write non-sequentially.
 * 
 * In write mode, sequential writing is supported to the entire of flash memory, starting at
 * address 0x04. Addresses 0x00 - 0x03 store the end of what has currently been written to flash,
 * as a little-endian unsigned 32 bit integer. This is written so that subsequent uses of the flash
 * in read mode know where to stop reading. Write mode overwrites all previous data on the flash.
 * 
 * In read mode, the end address is first loaded from addresses 0x00 - 0x03. Sequential reading
 * is supported up until this address (non-inclusive). 
 */
#include "Adafruit_SPIFlash.h"

/**
 * @brief Initializes the QSPI flash in a particular mode. This function MUST be called before other
 * functions in this header are called.
 * 
 * The location to begin reading/writing from is set to address 0x4, as the first 4 bytes of flash
 * are used to store the length of the data on the flash.
 * 
 * Subsequent calls to this function will re-initialize in the new mode. For example, the following 
 * code is legitimate:
 * 
 *   qspi_init_flash(false)
 *   <write operations>
 *   qspi_init_flash(true)
 *   <read operations>
 * 
 * @param read_mode Whether the flash is in read mode. True = read mode, false = write mode.
 */
void qspi_init_flash(bool read_mode);

/**
 * @brief Reads a number of bytes from the current location in flash into a buffer. The current location in
 * flash in advanced by that many bytes.
 * 
 * NOTE: Calls to this function when the QSPI is in write mode will fail. 
 * NOTE: Calls to this function without calling qspi_init_flash() has unspecified results. But it certainly won't work.
 * 
 * @param buffer The buffer to read into.
 * @param len The number of bytes to read into that buffer.
 * @return Whether the read was successful. If the read was unsuccessful, this is likely due to an unrecoverable
 * harware fault, and program execution should terminate and indicate an error.
 */
bool qspi_read(uint8_t *buffer, uint32_t len);

/**
 * @brief Writes a number of bytes to the current location in flash from a buffer. The current location in
 * flash in advanced by that many bytes.
 * 
 * NOTE: Calls to this function when the QSPI is in read mode will fail. 
 * NOTE: Calls to this function without calling qspi_init_flash() has unspecified results. But it certainly won't work.
 * 
 * @param buffer The buffer to write into flash.
 * @param len The number of bytes to write from that buffer.
 * @return Whether the write was successful. If the write was unsuccessful, this is likely due to an unrecoverable
 * harware fault, and program execution should terminate and indicate an error.
 */
bool qspi_write(uint8_t *buffer, uint32_t len);