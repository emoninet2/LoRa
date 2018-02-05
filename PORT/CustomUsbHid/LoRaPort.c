/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 * 
 * 
 * Port is Unstable and needs more work done. 
 * Data Parsing is too slow. 
 * 
 * 
 */

#include "../LoRaPort.h"
#include "../PortConfig.h"


#if (LoRa_Port_usbhid == 1)



#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>


/* Linux */
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>



/*
 * Ugly hack to work around failing compilation on systems that don't
 * yet populate new version of hidraw.h to userspace.
 */
#ifndef HIDIOCSFEATURE
#warning Please have your distro update the userspace kernel headers
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

/* Unix */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
/* C */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdbool.h>

static int fd;

char *bus_str(int bus)
{
	switch (bus) {
	case BUS_USB:
		return "USB";
		break;
	case BUS_HIL:
		return "HIL";
		break;
	case BUS_BLUETOOTH:
		return "Bluetooth";
		break;
	case BUS_VIRTUAL:
		return "Virtual";
		break;
	default:
		return "Other";
		break;
	}
}


int initializeUSBDevice(){
	//fd = _fd;
	int i;
	int desc_size;
	int res;
	char buf[256];
	struct hidraw_report_descriptor rpt_desc;
	struct hidraw_devinfo info;
	char *device = "/dev/hidraw0";

	//if (argc > 1)
	//	device = argv[1];

	/* Open the Device with non-blocking reads. In real life,
	   don't use a hard coded path; use libudev instead. */
	fd = open(device, O_RDWR|O_NONBLOCK);

	if (fd < 0) {
		perror("Unable to open device");
		return 1;
	}

	memset(&rpt_desc, 0x0, sizeof(rpt_desc));
	memset(&info, 0x0, sizeof(info));
	memset(buf, 0x0, sizeof(buf));

	/* Get Report Descriptor Size */
	res = ioctl(fd, HIDIOCGRDESCSIZE, &desc_size);
	if (res < 0)
		perror("HIDIOCGRDESCSIZE");
	else
		printf("Report Descriptor Size: %d\n", desc_size);

	/* Get Report Descriptor */
	rpt_desc.size = desc_size;
	res = ioctl(fd, HIDIOCGRDESC, &rpt_desc);
	if (res < 0) {
		perror("HIDIOCGRDESC");
	} else {
		printf("Report Descriptor:\n");
		for (i = 0; i < rpt_desc.size; i++)
			printf("%hhx ", rpt_desc.value[i]);
		puts("\n");
	}

	/* Get Raw Name */
	res = ioctl(fd, HIDIOCGRAWNAME(256), buf);
	if (res < 0)
		perror("HIDIOCGRAWNAME");
	else
		printf("Raw Name: %s\n", buf);

	/* Get Physical Location */
	res = ioctl(fd, HIDIOCGRAWPHYS(256), buf);
	if (res < 0)
		perror("HIDIOCGRAWPHYS");
	else
		printf("Raw Phys: %s\n", buf);

	/* Get Raw Info */
	res = ioctl(fd, HIDIOCGRAWINFO, &info);
	if (res < 0) {
		perror("HIDIOCGRAWINFO");
	} else {
		printf("Raw Info:\n");
		printf("\tbustype: %d (%s)\n",
			info.bustype, bus_str(info.bustype));
		printf("\tvendor: 0x%04hx\n", info.vendor);
		printf("\tproduct: 0x%04hx\n", info.product);
	}
}


int deinitializeUSBDevice(){
	close(fd);
}

int SendCmd(uint8_t *TxData, unsigned int TxCmdSize, uint8_t *RxCmd, unsigned int RxCmdSizeMax){
	int i;
	int res = write(fd, TxData, TxCmdSize);
	if (res < 0) {
		//printf("Error: %d\n", errno);
		//perror("write");
	} else {
		//printf("write() wrote %d bytes\n", res);
	}

	/* Get a report from the device */

	while(1){
		res = read(fd, RxCmd, RxCmdSizeMax);
		if(res>0) break;
	}


	if (res < 0) {
		perror("read");
	} else {
		//printf("read() read %d bytes:\n\t", res);
		for (i = 0; i < res; i++){
			//printf("%hhx ", RxCmd[i]);
		}
		//puts("\n");
	}

}


void __usbhid_gpio_setval(int pin, bool value){
	/* Send a Report to the Device */
	uint8_t buf[3];
	buf[0] = 0x12; /* Report Number */
	buf[1] = pin;
	buf[2] = value; /* Report Number */

	SendCmd(buf, 3, buf, 8);
}

bool __usbhid_gpio_getval(int pin){
	uint8_t buf[2];
	buf[0] = 0x13; /* Report Number */
	buf[1] = pin;


	SendCmd(buf, 2, buf, 8);

	return buf[1];
}

void __usbhid_gpio_setdir(int pin, int dir){
	uint8_t buf[3];
	buf[0] = 0x11; /* Report Number */
	buf[1] = pin;
	buf[2] = dir; /* Report Number */

	SendCmd(buf, 3, buf, 8);
}

uint8_t __usbhid_spi_byte_Xfer(uint8_t data){
	uint8_t buf[2];
	buf[0] = 0x22; /* Report Number */
	buf[1] = data;
	SendCmd(buf, 2, buf, 8);

	return buf[1];

}



void SX127x_Port_Initialize(){


    SX127x_GPIO_ConfigAndInit(SX127x_GPIO_NSS, SX127x_GPIOMODE_OUTPUT, SX127x_GPIOPULL_NOPULL,SX127x_GPIO_INTERRUPT_NONE,NULL);
    SX127x_GPIO_ConfigAndInit(SX127x_GPIO_RESET, SX127x_GPIOMODE_OUTPUT, SX127x_GPIOPULL_NOPULL,SX127x_GPIO_INTERRUPT_NONE,NULL);
    SX127x_GPIO_ConfigAndInit(SX127x_GPIO_TX, SX127x_GPIOMODE_OUTPUT, SX127x_GPIOPULL_NOPULL,SX127x_GPIO_INTERRUPT_NONE,NULL);
    SX127x_GPIO_ConfigAndInit(SX127x_GPIO_RX, SX127x_GPIOMODE_OUTPUT, SX127x_GPIOPULL_NOPULL,SX127x_GPIO_INTERRUPT_NONE,NULL);
    
    SX127x_SPI_Init();
    
}

void SX127x_GPIO_ConfigAndInit(LoRaGpio_t pin, LoRaGpioPinMode_t Mode, LoRaGpioPinPull_t pull, LoRaGpioInterruptEdge_t InterruptEdge,void (*function)(void) ){

    if((Mode == SX127x_GPIOMODE_INPUT) || (Mode == SX127x_GPIOMODE_OUTPUT)   ){
        int pinModeValue;
        int pinPullUpDown;

        if(Mode == SX127x_GPIOMODE_INPUT){
            pinModeValue = SX127x_GPIOMODE_INPUT;
        }else if (Mode == SX127x_GPIOMODE_OUTPUT){
            pinModeValue = SX127x_GPIOMODE_OUTPUT;
        }

        if(pull == SX127x_GPIOPULL_NOPULL){

        }else if(pull == SX127x_GPIOPULL_PULLUP){

        }else if(pull == SX127x_GPIOPULL_PULLDOWN){

        }

        switch(pin){
            case SX127x_GPIO_NSS 	: {
            	__usbhid_gpio_setdir(0,pinModeValue);
            }break;
            case SX127x_GPIO_RESET 	: {
            	__usbhid_gpio_setdir(3,pinModeValue);
            }break;
            case SX127x_GPIO_TX 	: {
            	__usbhid_gpio_setdir(2,pinModeValue);
            }break;
            case SX127x_GPIO_RX 	: {
            	__usbhid_gpio_setdir(1,pinModeValue);
            }break;
            case SX127x_GPIO_DIO0 	: {

            }break;
            case SX127x_GPIO_DIO1 	: {

            }break;
            case SX127x_GPIO_DIO2 	: {

            }break;
            case SX127x_GPIO_DIO3 	: {

            }break;
            case SX127x_GPIO_DIO4 	: {

            }break;
            case SX127x_GPIO_DIO5 	: {

            }break;
            default		: break;
            }
    }
    else if(Mode == SX127x_GPIOMODE_INTERRUPT_IN){
        
        int edgeType;
        
        if(InterruptEdge == SX127x_GPIO_INTERRUPT_RISING){

        }
        else if (InterruptEdge == SX127x_GPIO_INTERRUPT_FALLING){

        }
        else if(InterruptEdge == SX127x_GPIO_INTERRUPT_BOTH_EDGES){

        }
    

        
        switch(pin){
        case SX127x_GPIO_DIO0 	: {

        }break;
        case SX127x_GPIO_DIO1 	: {

        }break;
        case SX127x_GPIO_DIO2 	: {

        }break;
        case SX127x_GPIO_DIO3 	: {

        }break;
        case SX127x_GPIO_DIO4 	: {

        }break;
        case SX127x_GPIO_DIO5 	: {

        }break;
        default		: break;
        }
        
    }

}
void SX127x_GPIO_Init(LoRaGpio_t pin){
}
void SX127x_GPIO_DeInit(LoRaGpio_t pin){
}
void SX127x_GPIO_SetValue(LoRaGpio_t pin, bool value){
    	switch(pin){
	case SX127x_GPIO_NSS 	: __usbhid_gpio_setval(0,value);break;
	case SX127x_GPIO_RESET 	: __usbhid_gpio_setval(3,value);break;
	case SX127x_GPIO_TX 	: __usbhid_gpio_setval(2,value);break;
	case SX127x_GPIO_RX 	: __usbhid_gpio_setval(1,value);break;
	case SX127x_GPIO_DIO0 	: break;
	case SX127x_GPIO_DIO1 	: break;
	case SX127x_GPIO_DIO2 	: break;
	case SX127x_GPIO_DIO3 	: break;
	case SX127x_GPIO_DIO4 	: break;
	case SX127x_GPIO_DIO5 	: break;
	default		: break;
	}
}
bool SX127x_GPIO_GetValue(LoRaGpio_t pin){
	bool pinValue;
	switch(pin){
	case SX127x_GPIO_NSS 	: pinValue = __usbhid_gpio_getval(0);break;
	case SX127x_GPIO_RESET 	: pinValue = __usbhid_gpio_getval(3);break;
	case SX127x_GPIO_TX 	: pinValue = __usbhid_gpio_getval(2);break;
	case SX127x_GPIO_RX 	: pinValue = __usbhid_gpio_getval(1);break;
	case SX127x_GPIO_DIO0 	: break;
	case SX127x_GPIO_DIO1 	: break;
	case SX127x_GPIO_DIO2 	: break;
	case SX127x_GPIO_DIO3 	: break;
	case SX127x_GPIO_DIO4 	: break;
	case SX127x_GPIO_DIO5 	: break;
	default		: break;
	}

	return pinValue;
}

void SX127x_SPI_Init(void){

}
void SX127x_SPI_TranscieveBuffer( uint8_t *dataInOut, unsigned int size ){

	int i;
	for(i=0;i<size;i++){
		dataInOut[i] = SX127x_SPI_TranscieveByte(dataInOut[i]);
	}

}
uint8_t SX127x_SPI_TranscieveByte( uint8_t dataIn ){
	return __usbhid_spi_byte_Xfer(dataIn);
}

void SX127x_Timeout_init(LoRaTimeout_t timeout){
}
void SX127x_Timeout_SetTime(LoRaTimeout_t timeout, unsigned int time){
    
    
}

void SX127x_delayUs(unsigned int us){
    usleep(us);
}
void SX127x_delayMs(unsigned int ms){
    usleep(1000*ms);
}

void SX127x_debug(const char *format, ...){
    va_list args;
    va_start(args,format);
    vfprintf(stdout,format, args);
    va_end(args);
}
void SX127x_debug_if(bool condition, const char *format, ...){
    if(condition){
        va_list args;
        va_start(args,format);
        vfprintf(stdout,format, args);
        va_end(args);
    }
}


#endif