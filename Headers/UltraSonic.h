#ifndef ultra_sonic_H
#define ultra_sonic_H

void ultraSetup();
void ultraSonic_sensor(uint32_t *distance1, uint32_t *distance2);
void objDetect(void *pvParameters);

#endif
