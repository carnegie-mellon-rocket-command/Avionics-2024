#include "accelerometers.h"
#include "qspi.h"
#include <LSM6DS3.h>

//=============================================================================
//             FORWARD DECLARATIONS
//=============================================================================

static void sample_timer_callback();

//=============================================================================
//             TIMER SETUP
//=============================================================================

#define TIMER_INTERRUPT_DEBUG 0     // Library says don't define this as > 0.
#define _TIMERINTERRUPT_LOGLEVEL_ 3 // Library says don't define this as > 0 for production. // TODO: change to 0 when done

// This header CANNOT be included in more than one source file, per the library documentation. 
// Attemping to do so will cause a multiple definition error. If timers are needed in multiple source files,
// this code needs to be refactored.
#include "NRF52TimerInterrupt.h"

static NRF52Timer sample_timer(NRF_TIMER_1); // Hardware timer for logging data.

//=============================================================================
//             ACCELEROMETER SETUP
//=============================================================================

static LSM6DS3 low_g_imu(I2C_MODE, 0x6A); // 0x6A = I2C address

/**
 * @brief Initializes the low-g accelerometer (the inbuilt LSM6DS3 in the Seeed XIAO nRF52840 Sense).
 * 
 * @return Whether the accelerometer was successfully initialized.
 */
static bool low_g_begin() {
    low_g_imu.settings.tempEnabled = 0;
    low_g_imu.settings.accelSampleRate = CMRC_LOW_G_REFRESH_RATE;
    low_g_imu.settings.gyroSampleRate = CMRC_LOW_G_REFRESH_RATE;
    
    return low_g_imu.begin() == IMU_SUCCESS; // TODO: log error
}

bool cmrc_accelerometers_begin() {
    if (!low_g_begin()) {
        return false; 
    } 

    // TODO: high g accelerometer

    // Set timer interrupt to handle logging data.
    if (!sample_timer.attachInterruptInterval(1000000 / CMRC_SAMPLE_RATE, sample_timer_callback)) {
        // creating timer failed. todo: log error
        return false;
    }

    return true;
}

//=============================================================================
//             LOGGING DATA
//=============================================================================

/**
 * Data is logged by a timer interrupt, which goes off CMRC_SAMPLE_RATE times per second. The interrupt handler
 * reads all data from the IMUs, and logs it to one of two buffers: sample_buffer_0 and sample_buffer_1. Which 
 * buffer is being used currently is determined by using_buffer_0. 
 * 
 * When a buffer is full, the interrupt signals this by setting buffer_0_ready or buffer_1_ready true, based on
 * which buffer it just filled up. It also sets buffer_ready_len to the amount of data in that buffer (which
 * may be less than the size of the buffer). It then switches buffers, so that the next time the interrupt
 * fires, data is logged to the other buffer.
 * 
 * These two buffers are used to avoid race conditions on the data. When one buffer is full, it can be written
 * out to QSPI flash by the main program, and if interrupts fire during this time, they write to the other buffer
 * (which therefore doesn't affect the QSPI write). As long as the main program flushes the data between when a
 * buffer is filled up and when the next buffer is filled up (which should be about every second), data is 
 * correctly written out to flash.
 * 
 * Note that if both buffers are ready at the same time, this means that the interrupts have filled up both buffers.
 * Hence data is being lost, and the main program also does not know which buffer should be written to flash first. 
 * If the program gets to this state, this is an unrecoverable error.
 */

// X, Y, Z acceleration: 2 bytes each
// X, Y, Z rotation: 2 bytes each
#define CMRC_SAMPLE_SIZE 12  
// Buffers are large enough to hold 1 second of data
#define CMRC_SAMPLE_BUFFER_LEN (CMRC_SAMPLE_RATE * CMRC_SAMPLE_SIZE)

static uint32_t sample_buffer_index = 0; // Index in the buffer currently being used for logging.
static bool using_buffer_0 = true;       // Whether the interrupt is currently logging to buffer0 or buffer1.

static bool buffer_0_ready = false;   // Whether buffer0 is ready to be written out to QSPI flash.
static bool buffer_1_ready = false;   // Whether buffer1 is ready to be written out to QSPI flash.
static uint32_t buffer_ready_len = 0; // How much data is ready to be written to flash, in whichever buffer is ready.

static uint8_t sample_buffer_0[CMRC_SAMPLE_BUFFER_LEN]; // First buffer to log data in.
static uint8_t sample_buffer_1[CMRC_SAMPLE_BUFFER_LEN]; // Second buffer to log data in.

/**
 * @brief Samples X, Y and Z acceleration from the low-g IMU (LSM6DS3) into the buffer. Values are
 * written in little-endian order, and in X, Y, Z order. Values are stored as raw bits, as this is
 * more space-efficient. They must be appropriately processed when read back.
 * 
 * @param buffer Which buffer to sample into.
 */
static inline void sample_low_g_acc(uint8_t *buffer) {
    int16_t xAccelerationRaw = low_g_imu.readRawAccelX();
    int16_t yAccelerationRaw = low_g_imu.readRawAccelY();
    int16_t zAccelerationRaw = low_g_imu.readRawAccelZ();

    uint8_t xAccelerationLow = xAccelerationRaw & 0xFF;
    uint8_t xAccelerationHigh = (xAccelerationRaw >> 8) & 0xFF;
    uint8_t yAccelerationLow = yAccelerationRaw & 0xFF;
    uint8_t yAccelerationHigh = (yAccelerationRaw >> 8) & 0xFF;
    uint8_t zAccelerationLow = zAccelerationRaw & 0xFF;
    uint8_t zAccelerationHigh = (zAccelerationRaw >> 8) & 0xFF;

    buffer[sample_buffer_index] = xAccelerationLow;
    buffer[sample_buffer_index+1] = xAccelerationHigh;
    buffer[sample_buffer_index+2] = yAccelerationLow;
    buffer[sample_buffer_index+3] = yAccelerationHigh;
    buffer[sample_buffer_index+4] = zAccelerationLow;
    buffer[sample_buffer_index+5] = zAccelerationHigh;

    sample_buffer_index += 6;
}

/**
 * @brief Samples X, Y and Z rotation from the low-g IMU (LSM6DS3) into the buffer. Values are
 * written in little-endian order, and in X, Y, Z order. Values are stored as raw bits, as this is
 * more space-efficient. They must be appropriately processed when read back.
 * 
 * @param buffer Which buffer to sample into.
 */
static inline void sample_low_g_gyro(uint8_t *buffer) {
    int16_t xGyroRaw = low_g_imu.readRawGyroX();
    int16_t yGyroRaw = low_g_imu.readRawGyroY();
    int16_t zGyroRaw = low_g_imu.readRawGyroZ();

    uint8_t xGyroLow = xGyroRaw & 0xFF;
    uint8_t xGyroHigh = (xGyroRaw >> 8) & 0xFF;
    uint8_t yGyroLow = yGyroRaw & 0xFF;
    uint8_t yGyroHigh = (yGyroRaw >> 8) & 0xFF;
    uint8_t zGyroLow = zGyroRaw & 0xFF;
    uint8_t zGyroHigh = (zGyroRaw >> 8) & 0xFF;

    buffer[sample_buffer_index] = xGyroLow;
    buffer[sample_buffer_index+1] = xGyroHigh;
    buffer[sample_buffer_index+2] = yGyroLow;
    buffer[sample_buffer_index+3] = yGyroHigh;
    buffer[sample_buffer_index+4] = zGyroLow;
    buffer[sample_buffer_index+5] = zGyroHigh;

    sample_buffer_index += 6;
}

/**
 * Callback function for the hardware timer, to log data. 
 */
static void sample_timer_callback() {
    uint8_t *buffer = using_buffer_0 ? sample_buffer_0 : sample_buffer_1;

    // Sample data
    sample_low_g_acc(buffer);
    sample_low_g_gyro(buffer);

    if (sample_buffer_index + CMRC_SAMPLE_SIZE > CMRC_SAMPLE_BUFFER_LEN) {
        // Current buffer is full: switch buffers and signal
        buffer_ready_len = sample_buffer_index;

        if (using_buffer_0) buffer_0_ready;
        else buffer_1_ready;

        using_buffer_0 = !using_buffer_0;
        sample_buffer_index = 0;
    }
}

bool cmrc_flush_data() {
    if (buffer_0_ready) {
        if (!qspi_write(sample_buffer_0, buffer_ready_len)) return false; // todo: log error
        buffer_0_ready = false;
        buffer_ready_len = 0;
    } else if (buffer_1_ready) {
        if (!qspi_write(sample_buffer_1, buffer_ready_len)) return false; // todo: log error
        buffer_1_ready = false;
        buffer_ready_len = 0;
    }
    return true;
}