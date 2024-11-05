/*
 * peripherals.c
 *
 *  Created on: Nov 4, 2024
 *      Author: brazenparadox
 */

#include "commons.h"

int i2c_data_to_payload(uint8_t* data, Payload* payload){
	// convert the data into payload
	payload->device_type = data[0];
	payload->data = data[1];

	return 0;
}
