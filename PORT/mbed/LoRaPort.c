/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "../LoRaPort.h"
#include "../PortConfig.h"


#if (LoRa_Port_mbed == 1)

#include "mbed.h"

#define SX1272_mosi_pin PA_7
#define SX1272_miso_pin PA_6
#define SX1272_sck_pin PA_5

#define SX127x_nss_pin PA_4
#define SX127x_reset_pin PA_0
#define SX127x_rx_pin PA_8
#define SX127x_tx_pin PF_1


#define SX127x_d0_pin 
#define SX127x_d1_pin 
#define SX127x_d2_pin 
#define SX127x_d3_pin 
#define SX127x_d4_pin 
#define SX127x_d5_pin

//#define SX127x_SPI 0
#define SX127x_SPI_Speed 8000000



SPI _port_mbed_spi(SX1272_mosi_pin,SX1272_miso_pin,SX1272_sck_pin);
DigitalOut _port_mbed_nss(SX127x_nss_pin);
DigitalOut _port_mbed_rxSwitch(SX127x_rx_pin);
DigitalOut _port_mbed_txSwitch(SX127x_tx_pin);
DigitalOut _port_mbed_resetPin(SX127x_reset_pin);




void SX127x_Port_Initialize(){
    setenv("WIRINGPI_GPIOMEM", "1", 1);
    wiringPiSetup () ;

    SX127x_GPIO_ConfigAndInit(SX127x_GPIO_NSS, SX127x_GPIOMODE_OUTPUT, SX127x_GPIOPULL_NOPULL,SX127x_GPIO_INTERRUPT_NONE,NULL);
    SX127x_GPIO_ConfigAndInit(SX127x_GPIO_RESET, SX127x_GPIOMODE_OUTPUT, SX127x_GPIOPULL_NOPULL,SX127x_GPIO_INTERRUPT_NONE,NULL);
    SX127x_GPIO_ConfigAndInit(SX127x_GPIO_TX, SX127x_GPIOMODE_OUTPUT, SX127x_GPIOPULL_NOPULL,SX127x_GPIO_INTERRUPT_NONE,NULL);
    SX127x_GPIO_ConfigAndInit(SX127x_GPIO_RX, SX127x_GPIOMODE_OUTPUT, SX127x_GPIOPULL_NOPULL,SX127x_GPIO_INTERRUPT_NONE,NULL);
    
    SX127x_SPI_Init();
    
}

void SX127x_GPIO_ConfigAndInit(LoRaGpio_t pin, LoRaGpioPinMode_t Mode, LoRaGpioPinPull_t pull, LoRaGpioInterruptEdge_t InterruptEdge,void (*function)(void) ){
}
void SX127x_GPIO_Init(LoRaGpio_t pin){
}
void SX127x_GPIO_DeInit(LoRaGpio_t pin){
}
void SX127x_GPIO_SetValue(LoRaGpio_t pin, bool value){
    switch(pin){
	case SX127x_GPIO_NSS 	: _port_mbed_nss = value; break;
	case SX127x_GPIO_RESET 	: _port_mbed_resetPin = value; break;
	case SX127x_GPIO_TX 	: _port_mbed_txSwitch = value; break;
	case SX127x_GPIO_RX 	: _port_mbed_rxSwitch = value; break;
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
	case SX127x_GPIO_NSS 	: pinValue= _port_mbed_nss; break;
	case SX127x_GPIO_RESET 	: pinValue= _port_mbed_resetPin; break;
	case SX127x_GPIO_TX 	: pinValue= _port_mbed_txSwitch; break;
	case SX127x_GPIO_RX 	: pinValue= _port_mbed_rxSwitch; break;
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
    uint8_t temp = *dataInOut;
    temp = _port_mbed_spi.write(temp);
    *dataInOut = temp;
}
uint8_t SX127x_SPI_TranscieveByte( uint8_t dataIn ){
    uint8_t temp = *dataInOut;
    temp = _port_mbed_spi.write(temp);
    *dataInOut = temp;
}

void SX127x_Timeout_init(LoRaTimeout_t timeout){
}
void SX127x_Timeout_SetTime(LoRaTimeout_t timeout, unsigned int time){
    
    
}

void SX127x_delayUs(unsigned int us){
    wait_us(us);
}
void SX127x_delayMs(unsigned int ms){
    wait_ms(ms);
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