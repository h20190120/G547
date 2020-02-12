
//#ifndef ADCDEV_H
//#define ADCDEV_H

#include <linux/ioctl.h>

#define MAGIC 'S'

#define CHANNEL_SEL _IOW(MAGIC,1,char *)
#define ALIGN_SEL _IOW(MAGIC,2,char *)


