#include "accelerometers.h"
#include "qspi.h"

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100); 
    
    qspi_init_flash(false);
    cmrc_accelerometers_begin();
}

void loop() {
    cmrc_flush_data();
}