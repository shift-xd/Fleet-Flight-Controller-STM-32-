#include <stdint.h>
#include "board.h"
#include "hal_init.h"
#include "mpu6000.h"
#include "madgwick.h"
#include "pid.h"
#include "pwm_out.h"
#include "uart.h"

volatile uint32_t systick_ms = 0;
void SysTick_Handler(void){ systick_ms++; }

int main(void){
    HAL_Init();
    SystemClock_Config();
    board_init();
    hal_init();
    uart_init();
    pwm_out_init();
    mpu_init();
    madgwick_init();
    pid_init_defaults();

    uint32_t last = systick_ms;
    while(1){
        if ((systick_ms - last) >= 1){
            last = systick_ms;
            sensor_data_t s = mpu_read();
            madgwick_update(s.gx,s.gy,s.gz,s.ax,s.ay,s.az, 0.001f);
            euler_t e = madgwick_get_euler();
            rc_frame_t rc = {0}; // stub - replace with real RC read
            control_output_t out = attitude_control_step(e, rc);
            float motor[4];
            mixer_mix(out, motor);
            pwm_out_write(motor);
            uart_telemetry_send(e, rc);
        }
    }
    return 0;
}
