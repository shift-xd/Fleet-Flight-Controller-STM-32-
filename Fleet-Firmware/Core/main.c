#include "board.h"
#include "hal_init.h"
#include "mpu6000.h"
#include "madgwick.h"
#include "pid.h"
#include "pwm_out.h"
#include "uart.h"
#include "mixer.h"
#include "params.h"


volatile uint32_t systick_ms = 0;


void SysTick_Handler(void){
systick_ms++;
}


int main(void){
HAL_Init();
SystemClock_Config();
board_init();
hal_init();
pwm_out_init();
uart_init();


mpu_init();
madgwick_init();
pid_init_defaults();


uint32_t last = systick_ms;
while(1){
// Run at 1 kHz main loop (1ms)
if ((systick_ms - last) >= 1){
last = systick_ms;


// read sensors
sensor_data_t s = mpu_read();


// run sensor fusion
madgwick_update(s.gx, s.gy, s.gz, s.ax, s.ay, s.az, 0.001f);


// get euler
euler_t e = madgwick_get_euler();


// compute control
rc_frame_t rc = rc_read_frame();
control_output_t out = attitude_control_step(e, rc);


// mix to motors
float motor[4];
mixer_mix(out, motor);


// output pwm
pwm_out_write(motor);


// telemetry periodically
static uint32_t tcnt = 0; if(++tcnt > 50){uart_telemetry_send(e, rc); tcnt=0;}
}
}
