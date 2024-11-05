/*
 * commons.h
 *
 *  Created on: Nov 4, 2024
 *      Author: brazenparadox
 */

#ifndef INC_COMMONS_H_
#define INC_COMMONS_H_

#include <inttypes.h>

typedef struct{
    char device_type; // can either be l or s for LED and Servo respectively
    uint8_t data;
} Payload;

int parse_input(char*, Payload*, int);


#endif /* INC_COMMONS_H_ */
