// for kernel version 4.15.0-76-generic

#include <linux/kernel.h>       
#include <linux/module.h>       
#include <linux/fs.h>
#include <linux/uaccess.h>      
#include <linux/time.h>
#include <linux/cdev.h>
#include <linux/random.h>
#include <linux/sched.h>

#include "adcdev.h"
#define SUCCESS 0
#define DEVICE_NAME "adc8"
#define BUF_LEN 2
//#define DEBUG 1

/*
 * Is the device open right now? Used to prevent
 * concurent access into the same device
 */
static int Device_Open = 0;

char *channel;
char *align;

//######################  RANDOM NUMBER GENERATION ####################
static u32 __random32(struct rnd_state *state)
{
#define TAUSWORTHE(s,a,b,c,d) ((s&c)<<d) ^ (((s <<a) ^ s)>>b)

	state->s1 = TAUSWORTHE(state->s1, 13, 19, 4294967294UL, 12);
	state->s2 = TAUSWORTHE(state->s2, 2, 25, 4294967288UL, 4);
	state->s3 = TAUSWORTHE(state->s3, 3, 11, 4294967280UL, 17);
	#ifdef DEBUG
	printk(KERN_INFO "T nos: %d %d %d",state->s1,state->s2,state->s3);
	#endif

	return (state->s1 ^ state->s2 ^ state->s3); //EXOR of states
}

int get_random_number(void)
{
struct timespec ts;
uint32_t rand;
uint16_t num;
ts = current_kernel_time();
struct rnd_state *state;
state->s1=(uint32_t)ts.tv_nsec; //current system time in nanoseconds
state->s2=(uint32_t)task_pid_nr(current); //currend process id
state->s3=(uint32_t)ts.tv_sec; //current system time in seconds

rand = __random32(state);
num=(uint16_t)rand%1024;
#ifdef DEBUG
printk(KERN_INFO "rand: %d",rand);
#endif


return num;
}

//#########################################################################
static dev_t dev_no; // variable for device number
static struct cdev c_dev; // variable for the character device structure
static struct class *cls;



static int adc_open(struct inode *inode, struct file *file)
{
#ifdef DEBUG
        printk(KERN_INFO "ADC device_open(%p)\n", file);
#endif

    /*
     * We don't want to talk to two processes at the same time
     */
    if (Device_Open)
        return -EBUSY;

    Device_Open++;
    try_module_get(THIS_MODULE);
    return SUCCESS;
}

uint16_t value,rand;

static ssize_t adc_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
	rand=get_random_number();
	if(*align=='L')
		value=rand*64;
	else value=rand;
	printk(KERN_INFO "Actual ADC value : read from channel %s is %d  \n",channel,rand);
	if(copy_to_user(buf,&value,sizeof(value))<0)
		printk(KERN_INFO "data missing");
	
	return sizeof(value);
}

long adc_ioctl(struct file *file,             
                  unsigned int ioctl_num,        /* number and param for ioctl */
                  unsigned long ioctl_param)
{      
    /*
     * Switch according to the ioctl called
     */
 switch (ioctl_num) {
    case CHANNEL_SEL:
        /*
         * Receive a pointer to a message (in user space) and set that
         * to be the device's message.  Get the parameter given to
         * ioctl by the process.
         */
        channel = (char *)ioctl_param;
	
	printk(KERN_INFO "channel %s",channel);
         break;

    case ALIGN_SEL:
        /*
         * Receive a pointer to a message (in user space) and set that
         * to be the device's message.  Get the parameter given to
         * ioctl by the process.
         */
        align = (char *)ioctl_param;

         
	 printk(KERN_INFO "align %s",align);
         
         break;
}

return 0;
}


static int adc_close(struct inode *inode, struct file *file)
{
#ifdef DEBUG
    printk(KERN_INFO "device_release(%p,%p)\n", inode, file);
#endif

    /*
     * We're now ready for our next caller
     */
    Device_Open--;

    module_put(THIS_MODULE);
    return SUCCESS;
}


static struct file_operations fops =
{
  .owner 	= THIS_MODULE,
  .open 	= adc_open,
  .release 	= adc_close,
  .read 	= adc_read,
  .unlocked_ioctl = adc_ioctl
};


//#########################################################################
static int __init myadc_init(void) 
{
	printk(KERN_INFO "ADC driver registered");

	if (alloc_chrdev_region(&dev_no, 0, 1, "8 channel ADC") < 0)
	{
		return -1;
	}
	
	if ((cls = class_create(THIS_MODULE, "adc8")) == NULL)
	{
		unregister_chrdev_region(dev_no, 1);
		return -1;
	}
        if (device_create(cls, NULL, dev_no, NULL, "adc8") == NULL)
	{
		class_destroy(cls);
		unregister_chrdev_region(dev_no, 1);
		return -1;
	}
	cdev_init(&c_dev, &fops);
    	if (cdev_add(&c_dev, dev_no, 1) == -1)
	{
		device_destroy(cls, dev_no);
		class_destroy(cls);
		unregister_chrdev_region(dev_no, 1);
		return -1;
	}
	return 0;
}

static void __exit myadc_exit(void) 
{
	cdev_del(&c_dev);
	device_destroy(cls, dev_no);
	class_destroy(cls);
	unregister_chrdev_region(dev_no, 1);
	printk(KERN_INFO "ADC driver unregistered\n\n");
}
 
module_init(myadc_init);
module_exit(myadc_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("SWATHI");
MODULE_DESCRIPTION("Driver for 8 channel ADC");
 

	
