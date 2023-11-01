/**
 * Manages the accelerometers, and logging data. Samples are currently 16 bytes, consisting of the following:
 * 
 *   - timestamp (4 bytes)
 *   - X acceleration (2 bytes, little endian)
 *   - Y acceleration (2 bytes, little endian)
 *   - Z acceleration (2 bytes, little endian)
 *   - X rotation (2 bytes, little endian)
 *   - Y rotation (2 bytes, little endian)
 *   - Z rotation (2 bytes, little endian)
 */
#include <Arduino.h>

#define CMRC_SAMPLE_RATE 100        // Sample rate to read from sensors.
#define CMRC_LOW_G_REFRESH_RATE 416 // The underlying sample rate of the LSM6DS3 accelerometer. This must be one of a 
                                    // few specific values, and should be set to higher than CMRC_SAMPLE_RATE. The
                                    // LSM6DS3 samples acceleration at this rate, and we sample it at CMRC_SAMPLE_RATE.

// Timestamp: 4 bytes
// X, Y, Z acceleration: 2 bytes each
// X, Y, Z rotation: 2 bytes each
// X, Y, X high-g-acceleration: 2 bytes each
#define CMRC_SAMPLE_SIZE 22

#define CMRC_LOW_G_ACC_RANGE 16    // Acceleration range for low-g accelerometer: +- this value
#define CMRC_LOW_G_GYRO_RANGE 2000 // Gyro range for low-g accelerometer: +- this value

/**
 * Struct for holding all information in a sample. Note: this struct is NOT what is saved to flash, and in general,
 * sizeof(cmrc_sample_t) != CMRC_SAMPLE_SIZE. This is because samples are stored as raw bits, but are converted
 * to floats in this structure. This struct is only for passing a sample between functions.
 */
typedef struct {
    uint32_t timestamp;

    float xAcceleration;
    float yAcceleration;
    float zAcceleration;

    float xGyro;
    float yGyro;
    float zGyro;

    double xHighAccel;
    double yHighAccel;
    double zHighAccel;
} cmrc_sample_t;

/**
 * @brief Initailzies the accelerometers. Must be called prior to recording samples.
 * 
 * @return Whether intialization succeeded. If initialization failed, this is likely due to an unrecoverable hardware
 * error, and the program should terminate with an error. 
 */
bool cmrc_accelerometers_begin();

/**
 * @brief Takes a sample of all measurements, and saves them to QSPI flash.
 * 
 * NOTE: QSPI flash must have been initailized with qspi_init_flash(), and must be in WRITE mode.
 * 
 * @return Whether recording the sample to QSPI flash succeeded. This can only fail if QSPI flash returns an
 * error when writing to flash - if this occurs, it is likely due to an unrecoverable hardware error.
 */
bool cmrc_record_sample();

/**
 * @brief Reads a sample from QSPI flash. Stores the output in `sample_out`. 
 * 
 * NOTE: QSPI flash must have been initailized with qspi_init_flash(), and must be in READ mode.
 * 
 * @param sample_out Struct to write output sample into.
 * @return Whether reading the sample from QSPI flash succeeded. This can only fail if QSPI flash returns an
 * error when reading from flash - if this occurs, it is likely due to an unrecoverable hardware error. Note
 * that if the return value is false, `sample_out` has not been modified.
 */
bool cmrc_read_qspi_sample(cmrc_sample_t *sample_out);