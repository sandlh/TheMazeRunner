#ifndef ORIENTATION_H
#define ORIENTATION_H
#include <stdbool.h>
#include <stdint.h>

#define MODE_FRONT_LEFT 0
#define MODE_FRONT_RIGHT 1
#define MODE_BACK_RIGHT 2
#define MODE_BACK_LEFT 3

#define ORIENTATION_THREAD_PERIOD	4//[ms]

void orientation_start(void);

int16_t get_error(void);
int16_t get_norme(void);
int8_t get_mode_deplacement(void);
float get_cos_gravity(void);
float get_sin_gravity(void);

#endif
