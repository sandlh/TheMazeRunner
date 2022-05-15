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

void distance_start(void);

int16_t get_error_distance_to_wall_right(void);
int16_t get_error_distance_to_wall_left(void);
int16_t get_error_orientation_right(void);
int16_t get_error_orientation_left(void);

bool is_there_obstacle(void);
uint8_t index_highest_sensor_value(void);



#endif /* DISTANCE_H_ */
