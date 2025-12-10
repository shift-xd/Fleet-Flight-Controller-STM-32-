#ifndef MPU6000_H
#define MPU6000_H
typedef struct { float ax, ay, az; float gx, gy, gz; } sensor_data_t;
void mpu_init(void);
sensor_data_t mpu_read(void);
#endif
