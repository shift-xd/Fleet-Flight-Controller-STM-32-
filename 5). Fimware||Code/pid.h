#ifndef PID_H
#define PID_H
typedef struct { float kp,ki,kd; float integral, prev; } pid_t;
void pid_init_defaults(void);
float pid_update(pid_t* p, float target, float measured, float dt);

typedef struct { float roll, pitch, yaw; } control_output_t;
typedef struct { int channels[16]; } rc_frame_t;
control_output_t attitude_control_step(euler_t e, rc_frame_t rc);
void mixer_mix(control_output_t out, float motors[4]);
#endif
