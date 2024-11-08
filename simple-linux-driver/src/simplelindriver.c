#include<linux/module.h>
#include<linux/init.h>
#include<linux/fs.h>
#include<linux/cdev.h>
#include<linux/i2c.h>

#include "commons.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Roche Christopher");


/* Function Declarations */
//int init_arm_i2c_device(void);


/* I2C details */
#define ARM_I2C_ADDRESS 0x10 // TODO: Should be changed later
#define ARM_I2C_SLAVE_NAME "ARM I2C DEVICE"

/* Define the I2C device details */

struct i2c_adapter *arm_i2c_adapter;
struct i2c_client *arm_i2c_client;
    /* Create an I2C device client */
    struct i2c_board_info arm_i2c_board_info = {
	    I2C_BOARD_INFO(ARM_I2C_SLAVE_NAME, ARM_I2C_ADDRESS)
    };

    struct i2c_driver arm_i2c_driver = {
   	.driver = {
		.owner = THIS_MODULE,
       		.name = ARM_I2C_SLAVE_NAME 
	}	
    };
int arm_i2c_device_registered = 0;

dev_t dev;
int device_count = 1;
char* device_name = "simple_linux_driver";

struct cdev simple_char_device;

/* Custom Functions */


int send_payload(Payload *payload){
    char buff[2];
    buff[0] = payload->device_type;
    buff[1] = payload->data;
    int count = i2c_master_send(arm_i2c_client, buff, 2);
    return count;
}

/* I2C device initialization function */


int init_arm_i2c_device(void){

    arm_i2c_adapter = i2c_get_adapter(1);
    /* Create the linux device */    
    if(arm_i2c_adapter == NULL){
    	printk(KERN_ERR "\nI2C adapter cannot be created. Most likely, no i2c bus is available in the device\n");
	return -1;
    }

    arm_i2c_client = i2c_new_client_device(arm_i2c_adapter, &arm_i2c_board_info); 
    if(arm_i2c_client == NULL){
	    printk(KERN_ERR "A client device for arm i2c communication cannot be created\n");
	    i2c_put_adapter(arm_i2c_adapter); 
	    return -1;
    }
    printk(KERN_INFO "I2C client is %d", arm_i2c_client->addr);
    
    /* Add the i2c driver to the system */
    if(i2c_add_driver(&arm_i2c_driver) != -1){
    	printk(KERN_INFO "I2C driver for ARM has been added to the system\n");
   	arm_i2c_device_registered = 1;  
    }
    else{
    	printk(KERN_ERR "I2C driver for ARM cannot be added to the system");
    }

    return 0;
}

void arm_i2c_device_cleanup(void){
	if(arm_i2c_device_registered){
		printk(KERN_INFO "Going to delete the driver %s\n", ARM_I2C_SLAVE_NAME);
		i2c_del_driver(&arm_i2c_driver);
		i2c_unregister_device(arm_i2c_client);	
	}
}


/* File operations */

/* This method gets called when something is written to this device*/
ssize_t driver_write(struct file *filep, const char __user *buff, size_t count, loff_t *offp){
    //printk("The following was written to the device : ");
    
    // For demo 2, the input would be of format device:char data:char[3]

    const uint8_t max_input_count = 1+1+3+1; // 1 device character + 1 space + 3 digits + 1 null character 
    char copied_data[max_input_count];
    if(count > max_input_count)
	    count = max_input_count; // copy only the required data 
    copy_from_user(copied_data, buff, count); // data from the userspace should not be dereferenced in the kernel space. 
   // uint8_t data2send = frequency2send; 
   //printk(KERN_INFO "The data to be sent to the device is %u", frequency2send); 
   //printk("\n");
   //printk(KERN_INFO "The amount of data to be sent is %d", count);
    // parse the input into Payload struct
    
    Payload payload;
    parse_input(copied_data, &payload, count);
    count = send_payload(&payload);

  return count; 
}

/* This method gets called when the device is opened */
static int driver_open(struct inode *inode, struct file *file_pointer){
    printk("A device using the simple linux driver was opened\n");
    return 0;
}

/* This method gets called when the device is closed */
static int driver_close(struct inode *inode, struct file *file_pointer){
    printk("A device using the simple linux driver was closed\n");
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = driver_open,
    .write = driver_write,
    .release = driver_close
};

static int __init simplelindrvr_init(void){
    // get device driver numbers
    int allocation_retval = alloc_chrdev_region(&dev, 0, device_count, device_name);
    if (allocation_retval == 0){
        printk("Simple Linux Driver Initialized with major number %d and minor number %d\n", MAJOR(dev), MINOR(dev));
    }
    else{
        printk("An error occurred while trying to initialize the simple linux driver");
    }

     /* Create a device file */
    cdev_init(&simple_char_device, &fops);
    simple_char_device.owner = THIS_MODULE;

    /* Add the device file to the system*/
    cdev_add(&simple_char_device, dev, 1);

    /* Initialize the i2c device */
    init_arm_i2c_device();


    return 0;
}

static void __exit simplelindrvr_cleanup(void){

    /* Delete the device file */
    cdev_del(&simple_char_device);
    // unregister the device numbers
    unregister_chrdev_region(dev, device_count);
    
    // Delete i2c device
    arm_i2c_device_cleanup();
    printk("Simple Linux Driver Cleanup done\n");
}


module_init(simplelindrvr_init);
module_exit(simplelindrvr_cleanup);

