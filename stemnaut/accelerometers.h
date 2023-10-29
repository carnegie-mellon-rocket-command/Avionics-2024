/**
 * Manages the accelerometers, and logging data. Samples are currently 12 bytes, consisting of the following:
 * 
 *   - X acceleration (2 bytes, little endian)
 *   - Y acceleration (2 bytes, little endian)
 *   - Z acceleration (2 bytes, little endian)
 *   - X rotation (2 bytes, little endian)
 *   - Y rotation (2 bytes, little endian)
 *   - Z rotation (2 bytes, little endian)
 */

#define CMRC_SAMPLE_RATE 100        // Sample rate to read from sensors.
#define CMRC_LOW_G_REFRESH_RATE 416 // The underlying sample rate of the LSM6DS3 accelerometer. This must be one of a 
                                    // few specific values, and should be set to higher than CMRC_SAMPLE_RATE. The
                                    // LSM6DS3 samples acceleration at this rate, and we sample it at CMRC_SAMPLE_RATE.

/**
 * @brief Initailzies the accelerometers, and begins logging data.
 * 
 * Logging data is done via timer interrupts to ensure that spacing between samples remains consistent. As a result,
 * data is only written to internal buffers, and not to QSPI flash, in order for the interrupt routines that handle
 * logging to be as quick as possible. cmrc_flush_data() must be called regularly from the main program to flush
 * data to QSPI flash. 
 * 
 * NOTE: cmrc_flush_data() must be called at least once per second, or data may be lost. This is a hard lower bound,
 * and calling it at only once per second still runs the risk of losing data. Ideally, should be called at least twice
 * per second.
 * 
 * @return Whether intialization succeeded. If initialization failed, this is likely due to an unrecoverable hardware
 * error, and the program should terminate with an error. 
 */
bool cmrc_accelerometers_begin();

/**
 * Flushes buffered data to QSPI flash.
 * 
 * NOTE: cmrc_flush_data() must be called at least once per second, or data may be lost. This is a hard lower bound,
 * and calling it at only once per second still runs the risk of losing data. Ideally, should be called at least twice
 * per second.
 */
bool cmrc_flush_data();