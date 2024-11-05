#include<linux/types.h>

typedef struct{
    char device_type; // can either be l or s for LED and Servo respectively
    uint8_t data;
} Payload;

int parse_input(char*, Payload*, size_t);
