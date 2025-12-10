#ifndef MADGWICK_H
#define MADGWICK_H
typedef struct { float roll, pitch, yaw; } euler_t;
void madgwick_init(void);
void madgwick_update(float gx,float gy,float gz,float ax,float ay,float az,float dt);
euler_t madgwick_get_euler(void);
#endif
