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
uint16_t get_distance_front_right(void);
uint16_t get_distance_front_left(void);
uint16_t get_distance_back_right(void);
uint16_t get_distance_back_left(void);
uint16_t get_distance_right(void);
uint16_t get_distance_left(void);
bool is_there_obstacle(void);
void avoid_obstacle(int* speed);
int get_speed_right(void);
int get_speed_left(void);


#endif /* DISTANCE_H_ */