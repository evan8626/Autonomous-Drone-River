#ifndef motors_H
#define motors_H

int calc_angle(int32_t distance1, int32_t distance2);
void control_servo(uint32_t distance1, uint32_t distance2);
void motion(void *pvParameters);

#endif
