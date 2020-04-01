//kernel version : 4.15.0-91-generic
#include<linux/kernel.h>
#include<linux/module.h>
#include<linux/usb.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/kref.h>
#include <linux/errno.h>

#define HP_VID  0x03f0
#define HP_PID  0xbf07

#define SANDISC_VID 0x0781  
#define SANDISC_PID 0x5567

#define TRANSCEND_VID  0x0457
#define TRANSCEND_PID  0x0151

#define be_to_int32(buf) (((buf)[0]<<24)|((buf)[1]<<16)|((buf)[2]<<8)|(buf)[3])

struct usb_dev{
	struct usb_device *	udev;			/* the usb device for this device */
	struct usb_interface *	interface;		/* the interface for this device */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	struct kref		kref;
};


// Command Block Wrapper (CBW)
struct command_block_wrapper {
	uint8_t dCBWSignature[4];
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t bmCBWFlags;
	uint8_t bCBWLUN;
	uint8_t bCBWCBLength;
	uint8_t CBWCB[16];
};

// Command Status Wrapper (CSW)
struct command_status_wrapper {
	uint8_t dCSWSignature[4];
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t bCSWStatus;
};


static uint8_t cdb_length[256] = {
//	 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  0
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  1
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  2
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  3
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  4
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  5
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  6
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  7
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  8
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  9
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  A
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  B
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  C
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  D
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  E
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  F
};

uint8_t lun;
uint32_t ret_tag;
int i;


static int send_mass_storage_command(struct usb_dev *dev,uint8_t lun,uint8_t *cdb, uint8_t direction, int data_length, uint32_t *ret_tag)
{
	static uint32_t tag = 1;
	uint8_t cdb_len;
	int ret1,actual_len,i=0;

	typedef struct command_block_wrapper command_block_wrapper;
	command_block_wrapper *cbw;
	cbw = (command_block_wrapper *) kmalloc(sizeof(command_block_wrapper),GFP_KERNEL);

	if (cdb == NULL) {
		return -1;
	}
	cdb_len = cdb_length[cdb[0]];
	if ((cdb_len == 0) || (cdb_len > sizeof(cbw->CBWCB))) {
		printk(KERN_INFO "send_mass_storage_command: don't know how to handle this command (%02X, length %d)\n",
			cdb[0], cdb_len);
		return -1;
	}

	cbw->dCBWSignature[0] = 'U';
	cbw->dCBWSignature[1] = 'S';
	cbw->dCBWSignature[2] = 'B';
	cbw->dCBWSignature[3] = 'C';
	*ret_tag = tag;
	cbw->dCBWTag = tag++;
	cbw->dCBWDataTransferLength = data_length;
	cbw->bmCBWFlags = direction;
	cbw->bCBWLUN = lun;
	cbw->bCBWCBLength = cdb_len;

	for(i=0;i<16;i++)
		cbw->CBWCB[i] = *(cdb+i);

        /*Sending command via cbw for 2 times*/
	i = 0;
	do{
ret1 = usb_bulk_msg(dev->udev, usb_sndbulkpipe(dev->udev,dev->bulk_out_endpointAddr),(void *)cbw,31,&actual_len,10*HZ);
	}while((ret1!=0)&&(i<2));

	kfree(cbw);
	return i;
}


static int get_mass_storage_status(struct usb_dev *dev,uint32_t expected_tag)
{
	int i, r, size;
	typedef struct command_status_wrapper command_status_wrapper;
	command_status_wrapper *csw;
	csw = ( command_status_wrapper *) kmalloc(sizeof(struct command_status_wrapper),GFP_KERNEL);

	/*Receive csw*/
	i = 0;
	do{
	r = usb_bulk_msg(dev->udev, usb_rcvbulkpipe(dev->udev,dev->bulk_in_endpointAddr),(void *)csw,13,&size,10*HZ);
		i++;
 	}while ((r!=0)&&(i<2));

	printk(KERN_INFO "   	Mass Storage Status, %02X (%s)\n", csw->bCSWStatus, csw->bCSWStatus?"FAILED":"Success");
	if (r != 0) {
		printk(KERN_INFO "   get_mass_storage_status FAILED \n");
	return -1;
	}
	if (size != 13) {
		printk(KERN_INFO "   get_mass_storage_status: received %d bytes (expected 13)\n", size);
		return -1;
	}
	if (csw->dCSWTag != expected_tag) {
		printk(KERN_INFO "  get_mass_storage_status: mismatched tags (expected %08X, received %08X)\n",
			expected_tag, csw->dCSWTag);
		return -1;
	}
	
	
	if (csw->bCSWStatus) {

		if (csw->bCSWStatus == 1)
			return -2;	// request Get Sense
		else
			return -1;
	}

	return 0;
}



int test_device(struct usb_dev *dev)
{
uint8_t * rcv_lun;
int ret1,ret2;
int actual_len,i;
uint8_t *cdb;
uint8_t *buffer;
unsigned long long size;
uint32_t max_lba,block_size;
int integer;

rcv_lun =(uint8_t *) kmalloc(sizeof(uint8_t),GFP_KERNEL);


  /*
int usb_control_msg(struct usb_device * dev, unsigned int pipe, __u8 request, __u8 requesttype, __u16 value, __u16 index, void * data, __u16 size, int timeout);*/
  
/*****************Sending BOM_RESET control********************/
	ret1 = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),0xFF,0x21,0, 0, NULL, 0,100*HZ);

if(ret1 == 0)
printk(KERN_INFO "BULK STORAGE DEVICE RESET : SUCCESS \n");
else
printk(KERN_INFO "BULK STORAGE DEVICE RESET : FAILED \n");

/******************Sending GET_MAX_LUN control*****************/
printk(KERN_INFO "OBTAINING LUN: \n");
	ret1 = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),0xFE,0xA1,0, 0, (void *)rcv_lun, 1,100*HZ);

lun= *rcv_lun;
if(ret1 < 0) printk(KERN_INFO "ERROR IN CONTROL MSG TRANSFER \n");
else{
 printk(KERN_INFO " 	Max LUN = %d\n", lun);
 //printk(KERN_INFO " Bytes transfered = %d\n", ret1);
}



printk(KERN_INFO "READING CAPACITY : \n");

cdb= kmalloc(16*sizeof(uint8_t ),GFP_KERNEL);

for(i=0;i<16;i++)
		*(cdb+i)=0;

*cdb = 0x25;	// Read Capacity         
buffer=(uint8_t *)kmalloc(8*sizeof(uint8_t),GFP_KERNEL);

	for(i=0;i<8;i++)
			*(buffer+i)=0;

ret1= send_mass_storage_command(dev,lun,cdb, 0x80, 0x08, &ret_tag);
 //printk(KERN_INFO " Status after sending cbw = %d\n", ret1);

if(ret1 != 0) printk(KERN_INFO " CAPACITY REQUEST FAILED \n");
else
{
ret2 = usb_bulk_msg(dev->udev, usb_rcvbulkpipe(dev->udev,dev->bulk_in_endpointAddr),(void *)buffer,8,&actual_len,10*HZ);

	if(ret2 != 0) printk(KERN_INFO " CAPACITY RESPONSE FAILED \n");
	else
	{
		//printk(KERN_INFO " 	Length of data received = %d\n", actual_len);
		
			/*Conversion to GB*/
		
		max_lba= be_to_int32(buffer);
		block_size = be_to_int32(buffer+4);

		printk(KERN_INFO "	Max_lba = %u , Block_size = %u \n",max_lba,block_size);

		
		size = ((unsigned long long)max_lba * (unsigned long long )block_size);
		printk(KERN_INFO "	Capacity = %llu MB \n",(size >> 20));

		
		integer = (size >> 30);
		size = (size >> 27) *1000;
		size = (size >> 3);
		printk(KERN_INFO "	Approximately  = %02d.%03d GB \n",(int)integer,(int)(size-integer*1000));
	}
	}

ret1=get_mass_storage_status(dev,ret_tag);
//printk(KERN_INFO " status after csw = %d\n", ret);



//   Inquiry 
*cdb = 0x12;	
*(cdb+4) = 0x24;
ret1= send_mass_storage_command(dev,lun,cdb, 0x80,0x24, &ret_tag);
//printk(KERN_INFO " status after cbw = %d\n", ret1);
buffer=(uint8_t *)kmalloc(64*sizeof(uint8_t),GFP_KERNEL);

	for(i=0;i<64;i++)
			*(buffer+i)=0;
if(ret1 != 0)
printk(KERN_INFO "INQUIRY REQUEST FAILED \n");
else
{
ret2 = usb_bulk_msg(dev->udev, usb_rcvbulkpipe(dev->udev,dev->bulk_in_endpointAddr),(void *)buffer,64,&actual_len,10*HZ);
kfree(buffer);

printk(KERN_INFO "INQUIRY RESPONSE : \n");
if(ret2 != 0)
printk(KERN_INFO "INQUIRY RESPONSE FAILED \n");
else
{
char vid[9], pid[9], rev[5];
for (i=0; i<8; i++) {
		vid[i] = *(buffer+8+i);
		pid[i] = *(buffer+16+i);
		rev[i/2] =*(buffer+32+i/2);	
	}
	vid[8] = 0;
	pid[8] = 0;
	rev[4] = 0;
	printk(KERN_INFO "   VID:PID:REV \"%8s\":\"%8s\":\"%4s\"\n", vid, pid, rev);
}
}
ret1=get_mass_storage_status(dev,ret_tag);
return 0;
}

static struct usb_device_id usbdev_table [] = {
	{USB_DEVICE(TRANSCEND_VID, TRANSCEND_PID)},
	{USB_DEVICE(SANDISC_VID, SANDISC_PID)},
	{USB_DEVICE(HP_VID, HP_PID)},
	{} /*terminating entry*/	
};



static void usbdev_disconnect(struct usb_interface *interface)
{
	printk(KERN_INFO "USBDEV Device Removed\n");
	return;
}


static int usbdev_probe(struct usb_interface *interface, const struct usb_device_id *id)
{       
	struct usb_dev *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int i;
	
	if((id->idProduct == TRANSCEND_PID)&&(id->idVendor == TRANSCEND_VID))
	{
		printk(KERN_INFO "Transcend JetFlash pendrive Plugged in\n");
		printk(KERN_INFO "Known USB detected\n");
	}
	
	else if((id->idProduct == SANDISC_PID)&&(id->idVendor == SANDISC_VID))
	{
		printk(KERN_INFO "Sandisc CruserBlade Pendrive Plugged in\n");
		printk(KERN_INFO "Known USB detected\n");
	}
	else if((id->idProduct == HP_PID)&&(id->idVendor == HP_VID))
	{
		printk(KERN_INFO "HP Pendrive Plugged in\n");
		printk(KERN_INFO "Known USB detected\n");
	}


	/* allocate memory for our device state and initialize it */
	dev = kmalloc(sizeof(struct usb_dev), GFP_KERNEL);
	if (dev == NULL) {
		printk(KERN_INFO "Out of memory \n");
		goto error;
	}
	memset(dev, 0x00, sizeof (*dev));
	kref_init(&dev->kref);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

        printk(KERN_INFO "No. of Altsettings : %d\n",interface->num_altsetting);

	printk(KERN_INFO "USB VID : %x", dev->udev->descriptor.idVendor);
	printk(KERN_INFO "USB PID : %x", dev->udev->descriptor.idProduct);
	printk(KERN_INFO "USB DEVICE CLASS : %x", interface->cur_altsetting->desc.bInterfaceClass);
	printk(KERN_INFO "USB DEVICE SUB CLASS : %x", interface->cur_altsetting->desc.bInterfaceSubClass);
	printk(KERN_INFO "USB DEVICE Protocol : %x", interface->cur_altsetting->desc.bInterfaceProtocol);
	printk(KERN_INFO "No. of Endpoints : %d\n", interface->cur_altsetting->desc.bNumEndpoints);

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
      	/* update settings only for bulk SCSI*/
	iface_desc = interface->cur_altsetting;
if((iface_desc->desc.bInterfaceClass == 0x08)&& (iface_desc->desc.bInterfaceSubClass == 0x06))
{
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) 
	{
		endpoint = &iface_desc->endpoint[i].desc;
        	if (!dev->bulk_in_endpointAddr &&
		    (endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)== USB_ENDPOINT_XFER_BULK)) {
		dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
		printk(KERN_INFO "Bulk in Endpoint address : %d EP %d \n",dev->bulk_in_endpointAddr,(i+1));			
					
		}

		if (!dev->bulk_out_endpointAddr &&
		    !(endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
					== USB_ENDPOINT_XFER_BULK))  {dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
		printk(KERN_INFO "Bulk out Endpoint address : %d EP %d \n",dev->bulk_out_endpointAddr,(i+1));
		}
	}
}

	if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
		printk(KERN_INFO "Could not find both bulk-in and bulk-out endpoints \n");
		
	}

		
	if((dev->interface->cur_altsetting->desc.bInterfaceClass == 0x08)&& (dev->interface->cur_altsetting->desc.bInterfaceSubClass == 0x06)&&(dev->interface->cur_altsetting->desc.bInterfaceProtocol ==0x50 )){
	printk( KERN_INFO "Device of Mass storage class , SCSI subclass , Bulk-Only protocol detected  \n ");
	test_device(dev);}
error :
	usb_put_dev(dev->udev);
	kfree (dev);
	return -6;
}


/*Operations structure*/
static struct usb_driver usbdev_driver = {
	name: "my_USB_dev",  //name of the device
	probe: usbdev_probe, // Whenever Device is plugged in
	disconnect: usbdev_disconnect, // When we remove a device
	id_table: usbdev_table, //  List of devices served by this driver
};


int device_init(void)
{
	usb_register(&usbdev_driver);
        printk(KERN_INFO "UAS Read capacity driver inserted \n");
	return 0;
}

void device_exit(void)
{
	usb_deregister(&usbdev_driver);
	printk(KERN_NOTICE "Leaving Kernel\n");
	//return 0;
}

module_init(device_init);
module_exit(device_exit);
MODULE_LICENSE("GPL");

