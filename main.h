/*
 * main.h
 *
 *  Created on: Apr 25, 2022
 *      Author: Sandra L'Herminé et Barbara De Groot
 */
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <camera/dcmi_camera.h>
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


#define MAIN_THREAD_TIME 1000

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif/* MAIN_H_ */
