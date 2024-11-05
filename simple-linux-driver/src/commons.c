#include "commons.h"

int parse_input(char* input_data, Payload* payload, size_t count){

    payload->device_type = input_data[0];
    uint8_t data = 0;   
    for(int i=0; i<3; i++){
        int index = i+2;
	    if(!(input_data[index] >= '0' && input_data[index] <= '9'))
		    break;
    	data = (10*data + (input_data[index] - '0'));	
	//printk(KERN_INFO "%c %u", copied_data[i], data); 
    } 
    payload->data = data; 
    return count;
}
