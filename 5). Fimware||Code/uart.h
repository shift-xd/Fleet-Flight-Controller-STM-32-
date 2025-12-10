#ifndef UART_H
#define UART_H
#include "madgwick.h"
#include "pid.h"
void uart_init(void);
void uart_telemetry_send(euler_t e, rc_frame_t rc);
#endif
