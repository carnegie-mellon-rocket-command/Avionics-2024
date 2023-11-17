#include "accelerometers.h"
#include "qspi.h"

#define MODE_PIN 0 // Short this pin to ground on startup to enter alternate (read) mode.

/**
 * @brief Gets whether the STEMnaut should enter normal operation, or enter 'read mode'
 * (read recorded data off of flash and send it over serial to a computer).
 * 
 * @return True = normal operation (recording data). False = reading recorded data
 * off of flash and sending it to a computer.
 */
static bool normal_operation() {
    pinMode(MODE_PIN, INPUT_PULLUP);
    return digitalRead(MODE_PIN) == HIGH;
}

/**
 * @brief Enters read mode. Writes all logged data to Serial, in CSV format. 
 * 
 * NOTE: This function does not return.
 */
static void enter_read_mode() {
    Serial.println("Entering alternate read mode (outputting logged data).");
    qspi_init_flash(true);

    uint32_t data_length = qspi_data_length();
    uint32_t num_samples_remaining = data_length / CMRC_SAMPLE_SIZE;

    Serial.print("# Num samples: ");
    Serial.println(num_samples_remaining);

    while (num_samples_remaining > 0) {
        cmrc_sample_t sample;
        if (!cmrc_read_qspi_sample(&sample)) { 
            Serial.println("Error reading sample (cmrc_read_qspi_sample returned false.)");
            delay(100);
            exit(0);
        } 

        char output[256];
        snprintf(output, 256, "%lu, "
                "%f, %f, %f, "
                "%f, %f, %f, "
                "%d, %d, %d\n", 
            sample.timestamp, 
            sample.xAccelerationLowG, sample.yAccelerationLowG, sample.zAccelerationLowG,
            sample.xGyroLowG, sample.yGyroLowG, sample.zGyroLowG, 
            sample.xAccelerationHighG, sample.yAccelerationHighG, sample.zAccelerationHighG);
        Serial.write(output);
        num_samples_remaining -= 1;
    }

    Serial.write("\n\n\n");
    exit(0);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100); 

    if (!normal_operation()) enter_read_mode(); // Does not return.
    Serial.println("Entering normal operation (reading and logging acceleration).");

    qspi_init_flash(false);
    cmrc_accelerometers_begin();
}

void loop() {
    cmrc_record_sample();
    delay(1000 / CMRC_SAMPLE_RATE);
}