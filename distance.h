/*
 * distance.h
 *
 *  Created on: Apr 25, 2022
 *      Author: BDEGROOT
 */

#ifndef DISTANCE_H_
#define DISTANCE_H_

#include <stdint.h>
#include <stdbool.h>

#define FRONT_RIGHT 0 //#define to use the values from the sensor in the array sensor_value
#define FRONT_LEFT 1
#define BACK_RIGHT 2
#define BACK_LEFT 3
#define RIGHT 4
#define LEFT 5

void distance_start(void);

int16_t get_error_distance_to_wall_right(void);
int16_t get_error_distance_to_wall_left(void);
int16_t get_error_orientation_right(void);
int16_t get_error_orientation_left(void);

bool is_there_obstacle(void);
uint8_t index_highest_sensor_value(void);



#endif /* DISTANCE_H_ */
