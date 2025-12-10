#include "uart.h"
#include <stdio.h>
void uart_init(void){ /* init UART/USB CDC - implement for your board */ }
void uart_telemetry_send(euler_t e, rc_frame_t rc){
    (void)rc;
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "ROLL:%.2f PITCH:%.2f YAW:%.2f\n", e.roll, e.pitch, e.yaw);
    // send buf via UART - stub
    (void)n;
}
