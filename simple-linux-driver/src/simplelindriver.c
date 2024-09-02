#include<linux/module.h>
#include<linux/init.h>
#include<linux/fs.h>
#include<linux/cdev.h>

MODULE_LICENSE("GPL2");
MODULE_AUTHOR("Roche Christopher");

dev_t dev;
int device_count = 1;
char* device_name = "simple_linux_driver";

struct cdev simple_char_device;

// /* File operations */
// static int driver_read(){
//     return 0;
// }

// /* This method gets called when something is written to this device*/
// static int driver_write(){
//     return 0;
// }

/* This method gets called when the device is opened */
static int driver_open(struct inode *inode, struct file *file_pointer){
    printk("A device using the simple linux driver was opened");
    return 0;
}

/* This method gets called when the device is closed */
static int driver_close(struct inode *inode, struct file *file_pointer){
    printk("A device using the simple linux driver was closed");
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = driver_open,
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

    // /* Create a device file */
    // cdev_init(&simple_char_device, &fops);
    // simple_char_device.owner = THIS_MODULE;

    // /* Add the device file to the system*/
    // cdev_add(&simple_char_device, 0, 1);

    return 0;
}

static void __exit simplelindrvr_cleanup(void){

    /* Delete the device file */
    cdev_del(&simple_char_device);
    // unregister the device numbers
    unregister_chrdev_region(dev, device_count);
    printk("Simple Linux Driver Cleanup done\n");
}


module_init(simplelindrvr_init);
module_exit(simplelindrvr_cleanup);

