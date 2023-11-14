#include "accelerometers.h"
#include "qspi.h"
#include <LSM6DS3.h>
#include <Wire.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>

//=============================================================================
//             ACCELEROMETER SETUP
//=============================================================================

static LSM6DS3 low_g_imu(I2C_MODE, 0x6A); // 0x6A = I2C address
static Adafruit_H3LIS331 lis = Adafruit_H3LIS331(); //0x18 = I2C adress

/**
 * @brief Initializes the low-g accelerometer (the inbuilt LSM6DS3 in the Seeed XIAO nRF52840 Sense).
 * 
 * @return Whether the accelerometer was successfully initialized.
 */
static bool low_g_begin() {
    low_g_imu.settings.tempEnabled = 0;
    low_g_imu.settings.accelSampleRate = CMRC_LOW_G_REFRESH_RATE;
    low_g_imu.settings.gyroSampleRate = CMRC_LOW_G_REFRESH_RATE;

    low_g_imu.settings.gyroRange = CMRC_LOW_G_GYRO_RANGE;
    low_g_imu.settings.accelRange = CMRC_LOW_G_ACC_RANGE;
    
    return low_g_imu.begin() == IMU_SUCCESS; // TODO: log error
}


static bool high_g_begin(){
    if (!lis.begin_I2C()) {   // change this to 0x19 for alternative i2c address
        Serial.println("Couldnt start");
        while (1) yield();
    }
    lis.setRange(H3LIS331_RANGE_100_G);
    lis.setDataRate(LIS331_DATARATE_100_HZ);
    return true; //find a better way to test if working
}

bool cmrc_accelerometers_begin() {
    if (!low_g_begin() || !high_g_begin()) {
        return false; 
    } 

    // TODO: high g accelerometer

    return true;
}

//=============================================================================
//             LOGGING DATA
//=============================================================================

/**
 * @brief Gets the current timestamp and puts it into the sample buffer. Value is written in
 * little-endian order.
 * 
 * @param buffer Buffer to write into.
 * @param start_index Index into that buffer to start writing at (will write 4 bytes). 
 */
static inline void get_timestamp(uint8_t *buffer, uint32_t start_index) {
    uint32_t time = millis();

    uint8_t byte0 = time & 0xFF;
    uint8_t byte1 = (time >> 8) & 0xFF;
    uint8_t byte2 = (time >> 16) & 0xFF;
    uint8_t byte3 = (time >> 24) & 0xFF;

    buffer[start_index] = byte0;
    buffer[start_index+1] = byte1;
    buffer[start_index+2] = byte2;
    buffer[start_index+3] = byte3;
}

/**
 * @brief Samples X, Y and Z acceleration from the low-g IMU (LSM6DS3) into the buffer. Values are
 * written in little-endian order, and in X, Y, Z order. Values are stored as raw bits, as this is
 * more space-efficient. They must be appropriately processed when read back.
 * 
 * @param buffer Buffer to sample into.
 * @param start_index Index into that buffer to start writing at (will write 6 bytes). 
 */
static inline void sample_low_g_acc(uint8_t *buffer, uint32_t start_index) {
    int16_t xAccelerationRaw = low_g_imu.readRawAccelX();
    int16_t yAccelerationRaw = low_g_imu.readRawAccelY();
    int16_t zAccelerationRaw = low_g_imu.readRawAccelZ();

    uint8_t xAccelerationLow = xAccelerationRaw & 0xFF;
    uint8_t xAccelerationHigh = (xAccelerationRaw >> 8) & 0xFF;
    uint8_t yAccelerationLow = yAccelerationRaw & 0xFF;
    uint8_t yAccelerationHigh = (yAccelerationRaw >> 8) & 0xFF;
    uint8_t zAccelerationLow = zAccelerationRaw & 0xFF;
    uint8_t zAccelerationHigh = (zAccelerationRaw >> 8) & 0xFF;

    buffer[start_index] = xAccelerationLow;
    buffer[start_index+1] = xAccelerationHigh;
    buffer[start_index+2] = yAccelerationLow;
    buffer[start_index+3] = yAccelerationHigh;
    buffer[start_index+4] = zAccelerationLow;
    buffer[start_index+5] = zAccelerationHigh;
}

/**
 * @brief Samples X, Y and Z rotation from the low-g IMU (LSM6DS3) into the buffer. Values are
 * written in little-endian order, and in X, Y, Z order. Values are stored as raw bits, as this is
 * more space-efficient. They must be appropriately processed when read back.
 * 
 * @param buffer Buffer to sample into.
 * @param start_index Index into that buffer to start writing at (will write 6 bytes). 
 */
static inline void sample_low_g_gyro(uint8_t *buffer, uint32_t start_index) {
    int16_t xGyroRaw = low_g_imu.readRawGyroX();
    int16_t yGyroRaw = low_g_imu.readRawGyroY();
    int16_t zGyroRaw = low_g_imu.readRawGyroZ();

    uint8_t xGyroLow = xGyroRaw & 0xFF;
    uint8_t xGyroHigh = (xGyroRaw >> 8) & 0xFF;
    uint8_t yGyroLow = yGyroRaw & 0xFF;
    uint8_t yGyroHigh = (yGyroRaw >> 8) & 0xFF;
    uint8_t zGyroLow = zGyroRaw & 0xFF;
    uint8_t zGyroHigh = (zGyroRaw >> 8) & 0xFF;

    buffer[start_index] = xGyroLow;
    buffer[start_index+1] = xGyroHigh;
    buffer[start_index+2] = yGyroLow;
    buffer[start_index+3] = yGyroHigh;
    buffer[start_index+4] = zGyroLow;
    buffer[start_index+5] = zGyroHigh;
}

static inline void sample_high_g_acc(uint8_t *buffer, uint32_t start_index) {
    sensors_event_t event;
    lis.getEvent(&event);
    //I have no idea if this is correct because of casting (i just made stuff up);
    int16_t xAccRaw = round(event.acceleration.x); //& 0xFFFF0000;
    int16_t yAccRaw = round(event.acceleration.y); //& 0xFFFF0000;
    int16_t zAccRaw = round(event.acceleration.z); //& 0xFFFF0000;

    uint8_t xAccLow = xAccRaw & 0xFF;
    uint8_t xAccHigh = (xAccRaw >> 8) & 0xFF;
    uint8_t yAccLow = yAccRaw & 0xFF;
    uint8_t yAccHigh = (yAccRaw >> 8) & 0xFF; 
    uint8_t zAccLow = zAccRaw & 0xFF;
    uint8_t zAccHigh = (zAccRaw >> 8) & 0xFF;

    buffer[start_index] = xAccLow;
    buffer[start_index+1] = xAccHigh;
    buffer[start_index+2] = yAccLow;
    buffer[start_index+3] = yAccHigh;
    buffer[start_index+4] = zAccLow;
    buffer[start_index+5] = zAccHigh;
}

bool cmrc_record_sample() {
    uint8_t sample[CMRC_SAMPLE_SIZE];

    get_timestamp(sample, 0);
    sample_low_g_acc(sample, 4);
    sample_low_g_gyro(sample, 10);
    sample_high_g_acc(sample, 16);

    return qspi_write(sample, CMRC_SAMPLE_SIZE); // todo: potentially log failure
}

//=============================================================================
//             READING DATA BACK OFF FLASH
//=============================================================================

// Taken from LSM6DS3.cpp.
static float calculate_acceleration(int16_t input) {
    float output = (float)input * 0.061 * (CMRC_LOW_G_ACC_RANGE >> 1) / 1000;
    return output;
}

// Taken from LSM6DS3.cpp.
static float calculate_gyro(int16_t input) {
    uint8_t gyroRangeDivisor = CMRC_LOW_G_GYRO_RANGE / 125;
    if (CMRC_LOW_G_GYRO_RANGE == 245) {
        gyroRangeDivisor = 2;
    }

    float output = (float)input * 4.375 * (gyroRangeDivisor) / 1000;
    return output;
}

bool cmrc_read_qspi_sample(cmrc_sample_t *out) {
    uint8_t sample[CMRC_SAMPLE_SIZE];

    if (!qspi_read(sample, CMRC_SAMPLE_SIZE)) return false; // todo: log failure

    uint32_t timestamp = ((uint32_t)sample[0])         |
                         (((uint32_t)sample[1]) << 8)  |
                         (((uint32_t)sample[2]) << 16) |
                         (((uint32_t)sample[3]) << 24);

    int16_t xAccelerationRaw = ((int16_t)sample[4]) | 
                                (((int16_t)sample[5]) << 8);
    int16_t yAccelerationRaw = ((int16_t)sample[6]) | 
                                (((int16_t)sample[7]) << 8);
    int16_t zAccelerationRaw = ((int16_t)sample[8]) | 
                                (((int16_t)sample[9]) << 8);
    
    int16_t xGyroRaw = ((int16_t)sample[10]) | 
                       (((int16_t)sample[11]) << 8);
    int16_t yGyroRaw = ((int16_t)sample[12]) | 
                       (((int16_t)sample[13]) << 8);
    int16_t zGyroRaw = ((int16_t)sample[14]) | 
                       (((int16_t)sample[15]) << 8);
          
    int16_t xHighAccelerationRaw = ((int16_t)sample[16]) | 
                       (((int16_t)sample[17]) << 8);
    int16_t yHighAccelerationRaw = ((int16_t)sample[18]) | 
                       (((int16_t)sample[19]) << 8);
    int16_t zHighAccelerationRaw = ((int16_t)sample[20]) | 
                       (((int16_t)sample[21]) << 8);

    out->timestamp = timestamp;

    out->xAcceleration = calculate_acceleration(xAccelerationRaw);
    out->yAcceleration = calculate_acceleration(yAccelerationRaw);
    out->zAcceleration = calculate_acceleration(zAccelerationRaw);

    out->xGyro = calculate_gyro(xGyroRaw);
    out->yGyro = calculate_gyro(yGyroRaw);
    out->zGyro = calculate_gyro(zGyroRaw);

    out->xHighAccel = (int)xHighAccelerationRaw;
    out->yHighAccel = (int)yHighAccelerationRaw;
    out->zHighAccel = (int)zHighAccelerationRaw;

    return true;
}
