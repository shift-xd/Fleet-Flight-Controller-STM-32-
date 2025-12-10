#include "mpu6000.h"
#include <stdlib.h>
void mpu_init(void){ /* SPI init + sensor reset - implement for your board */ }
sensor_data_t mpu_read(void){
    sensor_data_t s = {0};
    // return zeros by default - replace with real SPI readings
    return s;
}
