/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "../LoRaPort.h"

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdlib.h>
#include <stdio.h>


#define SX127x_nss_pin 11
#define SX127x_reset_pin 21
#define SX127x_rx_pin 22
#define SX127x_tx_pin 23


#define SX127x_d0_pin 24
#define SX127x_d1_pin 25
#define SX127x_d2_pin 29
#define SX127x_d3_pin 28
#define SX127x_d4_pin 27
#define SX127x_d5_pin 26

#define SX127x_SPI 0
#define SX127x_SPI_Speed 8000000


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

    if((Mode == SX127x_GPIOMODE_INPUT) || (Mode == SX127x_GPIOMODE_OUTPUT)   ){
        int pinModeValue;
        int pinPullUpDown;

        if(Mode == SX127x_GPIOMODE_INPUT){
            pinModeValue = SX127x_GPIOMODE_INPUT;
        }else if (Mode == SX127x_GPIOMODE_OUTPUT){
            pinModeValue = SX127x_GPIOMODE_OUTPUT;
        }

        if(pull == SX127x_GPIOPULL_NOPULL){
            pinPullUpDown = PUD_OFF;
        }else if(pull == SX127x_GPIOPULL_PULLUP){
            pinPullUpDown = PUD_UP;
        }else if(pull == SX127x_GPIOPULL_PULLDOWN){
            pinPullUpDown = PUD_DOWN;
        }

        switch(pin){
            case SX127x_GPIO_NSS 	: {
                pinMode(SX127x_nss_pin,pinModeValue); 
                pullUpDnControl(SX127x_nss_pin,pinPullUpDown );
            }break;
            case SX127x_GPIO_RESET 	: {
                pinMode(SX127x_reset_pin,pinModeValue);
                pullUpDnControl(SX127x_reset_pin,pinPullUpDown );
            }break;
            case SX127x_GPIO_TX 	: {
                pinMode(SX127x_tx_pin,pinModeValue); 
                pullUpDnControl(SX127x_tx_pin,pinPullUpDown );
            }break;
            case SX127x_GPIO_RX 	: {
                pinMode(SX127x_rx_pin,pinModeValue); 
                pullUpDnControl(SX127x_rx_pin,pinPullUpDown );
            }break;
            case SX127x_GPIO_DIO0 	: {
                pinMode(SX127x_d0_pin,pinModeValue); 
                pullUpDnControl(SX127x_rx_pin,pinPullUpDown );
            }break;
            case SX127x_GPIO_DIO1 	: {
                pinMode(SX127x_d1_pin,pinModeValue); 
                pullUpDnControl(SX127x_rx_pin,pinPullUpDown );
            }break;
            case SX127x_GPIO_DIO2 	: {
                pinMode(SX127x_d2_pin,pinModeValue); 
                pullUpDnControl(SX127x_rx_pin,pinPullUpDown );
            }break;
            case SX127x_GPIO_DIO3 	: {
                pinMode(SX127x_d3_pin,pinModeValue); 
                pullUpDnControl(SX127x_rx_pin,pinPullUpDown );
            }break;
            case SX127x_GPIO_DIO4 	: {
                pinMode(SX127x_d4_pin,pinModeValue); 
                pullUpDnControl(SX127x_rx_pin,pinPullUpDown );
            }break;
            case SX127x_GPIO_DIO5 	: {
                pinMode(SX127x_d5_pin,pinModeValue); 
                pullUpDnControl(SX127x_rx_pin,pinPullUpDown );
            }break;
            default		: break;
            }
    }
    else if(Mode == SX127x_GPIOMODE_INTERRUPT_IN){
        
        int edgeType;
        
        if(InterruptEdge == SX127x_GPIO_INTERRUPT_RISING){
            edgeType = INT_EDGE_RISING;
        }
        else if (InterruptEdge == SX127x_GPIO_INTERRUPT_FALLING){
            edgeType = INT_EDGE_FALLING;
        }
        else if(InterruptEdge == SX127x_GPIO_INTERRUPT_BOTH_EDGES){
            edgeType = INT_EDGE_BOTH;
        }
    

        
        switch(pin){
        case SX127x_GPIO_DIO0 	: {
            wiringPiISR(SX127x_d0_pin,edgeType, function );
        }break;
        case SX127x_GPIO_DIO1 	: {
            wiringPiISR(SX127x_d1_pin,edgeType, function );
        }break;
        case SX127x_GPIO_DIO2 	: {
            wiringPiISR(SX127x_d2_pin,edgeType, function );
        }break;
        case SX127x_GPIO_DIO3 	: {
            wiringPiISR(SX127x_d3_pin,edgeType, function );
        }break;
        case SX127x_GPIO_DIO4 	: {
            wiringPiISR(SX127x_d4_pin,edgeType, function );
        }break;
        case SX127x_GPIO_DIO5 	: {
            wiringPiISR(SX127x_d5_pin,edgeType, function );
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
	case SX127x_GPIO_NSS 	: digitalWrite(SX127x_nss_pin,value); break;
	case SX127x_GPIO_RESET 	: digitalWrite(SX127x_reset_pin,value); break;
	case SX127x_GPIO_TX 	: digitalWrite(SX127x_tx_pin,value); break;
	case SX127x_GPIO_RX 	: digitalWrite(SX127x_rx_pin,value); break;
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
	case SX127x_GPIO_NSS 	: pinValue= digitalRead(SX127x_nss_pin); break;
	case SX127x_GPIO_RESET 	: pinValue= digitalRead(SX127x_reset_pin); break;
	case SX127x_GPIO_TX 	: pinValue= digitalRead(SX127x_tx_pin); break;
	case SX127x_GPIO_RX 	: pinValue= digitalRead(SX127x_rx_pin); break;
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
    wiringPiSPISetup(SX127x_SPI,SX127x_SPI_Speed);
}
void SX127x_SPI_TranscieveBuffer( uint8_t *dataInOut, unsigned int size ){
     wiringPiSPIDataRW(SX127x_SPI, (unsigned char*)dataInOut,size);
}
uint8_t SX127x_SPI_TranscieveByte( uint8_t dataIn ){
    char temp = dataIn;
    wiringPiSPIDataRW(SX127x_SPI, (unsigned char*)&temp,1);
    return temp;
}

void SX127x_Timeout_init(LoRaTimeout_t timeout){
}
void SX127x_Timeout_SetTime(LoRaTimeout_t timeout, unsigned int time){
    
    
}

void SX127x_delayUs(unsigned int us){
    delayMicroseconds(us);
}
void SX127x_delayMs(unsigned int ms){
    delay(ms);
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