#include <stdio.h> 
#include <string.h> 
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h> 



#include "adcdev.h"

char data[16]; 
char ch,a; 
uint16_t val,adc_val;

char *to_binary(int n)
    {
    int c, d, count;
    char *pointer;
    count = 0;
    pointer = (char*)malloc(16+1);
    if ( pointer == NULL )
    exit(EXIT_FAILURE);
    for ( c = 15 ; c >= 0 ; c-- )
    {
    d = n >> c;
    if ( d & 1 )
    *(pointer+count) = 1 + '0';
    else
    *(pointer+count) = 0 + '0';
    count++;
    }
    *(pointer+count) = '\0';
    return pointer;
    }


	

int ioctl_set_channel(int file_desc, char *message)
{
    int ret_val;

    ret_val = ioctl(file_desc, CHANNEL_SEL, message);

    if (ret_val < 0) {
        printf("ioctl_set_channel failed:%d\n", ret_val);
        exit(-1);
    }
    return 0;
}

int ioctl_set_alignment(int file_desc, char *message)
{
    int ret_val;

    ret_val = ioctl(file_desc, ALIGN_SEL, message);

    if (ret_val < 0) {
        printf("ioctl_set_alignment failed:%d\n", ret_val);
        exit(-1);
    }
    return 0;
}
 
int main( ) 
{ 

    int fd;
    
    fd= open("/dev/adc8", O_RDWR) ;
    do{

    printf("Select the adc channel from 0 to 7 \n");
    scanf("%s",&ch);
	if(ch>=56 || ch<=47) printf("Error in selection, please select again");
    }while(ch>=56 || ch<=47);

    do
    {printf("Select alignment L for left and R for right \n");
    scanf("%s",&a);
    if (!(a=='L' || a=='R')) printf("Error in selection, please select again \n");
    }while (!(a=='L' || a=='R'));

    ioctl_set_channel(fd,&ch);
    ioctl_set_alignment(fd,&a);


    int n=read(fd,&val,100);
	if(n=0) printf("no data read");
	else printf("Raw data from ADC in binary %s, in decimal %d \n",to_binary(val),val);

	if(a=='L')
		adc_val=val/64;
	else    adc_val=val;
	printf("Value from channel %d of ADC in binary %s, in decimal %d \n ",(ch-48),to_binary(adc_val),adc_val);
    close(fd);
	
    return 0;
}
