#include "pwm_out.h"
#include <stdint.h>
void pwm_out_init(void){
    // Configure timers for PWM - implement using HAL timers
}
void pwm_out_write(float motor[4]){
    // motor values expected 0.0..1.0 map to 1000..2000us - implement timer compare updates
    (void)motor;
}
