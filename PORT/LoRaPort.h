/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   LoRaPort.h
 * Author: emon
 *
 * Created on September 26, 2017, 6:33 PM
 */

#ifndef LORAPORT_H
#define LORAPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <assert.h>    
#include <stdbool.h>

    
    typedef enum {
            SX127x_GPIO_NSS,
            SX127x_GPIO_RESET,
            SX127x_GPIO_TX,
            SX127x_GPIO_RX,
            SX127x_GPIO_DIO0, 
            SX127x_GPIO_DIO1,
            SX127x_GPIO_DIO2,
            SX127x_GPIO_DIO3,
            SX127x_GPIO_DIO4,
            SX127x_GPIO_DIO5
    }LoRaGpio_t;

    typedef enum {
                    SX127x_GPIOMODE_INPUT, SX127x_GPIOMODE_OUTPUT, SX127x_GPIOMODE_INTERRUPT_IN
    }LoRaGpioPinMode_t;

    typedef enum {
                    SX127x_GPIOPULL_NOPULL, SX127x_GPIOPULL_PULLUP, SX127x_GPIOPULL_PULLDOWN
    }LoRaGpioPinPull_t;

    typedef enum {
                    SX127x_GPIO_INTERRUPT_NONE, SX127x_GPIO_INTERRUPT_RISING, SX127x_GPIO_INTERRUPT_FALLING, SX127x_GPIO_INTERRUPT_BOTH_EDGES
    }LoRaGpioInterruptEdge_t;



    typedef enum {
            txTimeout, rxTimeout, rxTimeoutSyncWord
    }LoRaTimeout_t;


    
    
    
    
    typedef struct{
        void(*SX127x_Port_Initialize)(void);
        void (*GPIO_ConfigAndInit)(
            LoRaGpio_t pin, 
            LoRaGpioPinMode_t Mode, 
            LoRaGpioPinPull_t pull, 
            LoRaGpioInterruptEdge_t 
            InterruptEdge,
            void (*function)(void) );
        void (*GPIO_Init)(LoRaGpio_t pin);
        void (*GPIO_DeInit)(LoRaGpio_t pin);
        void (*GPIO_SetValue)(LoRaGpio_t pin, bool value);
        bool (*GPIO_GetValue)(LoRaGpio_t pin);

        void (*SPI_Init)(void);
        void (*SPI_TranscieveBuffer)( uint8_t *dataInOut, unsigned int size );
        uint8_t (*SPI_TranscieveByte)( uint8_t dataIn );

        void (*Timeout_init)(LoRaTimeout_t timeout);
        void (*Timeout_SetTime)(LoRaTimeout_t timeout, unsigned int time);

        void (*delayUs)(unsigned int us);
        void (*delayMs)(unsigned int ms);
        
        void (*debug)(const char *format, ...);
        void (*debug_if)(bool condition, const char *format, ...);



    }LoRaPort_t;

    void SX127x_Port_Initialize();
    void SX127x_GPIO_ConfigAndInit(LoRaGpio_t pin, LoRaGpioPinMode_t Mode, LoRaGpioPinPull_t pull, LoRaGpioInterruptEdge_t InterruptEdge,void (*function)(void) );
    void SX127x_GPIO_Init(LoRaGpio_t pin);
    void SX127x_GPIO_DeInit(LoRaGpio_t pin);
    void SX127x_GPIO_SetValue(LoRaGpio_t pin, bool value);
    bool SX127x_GPIO_GetValue(LoRaGpio_t pin);

    void SX127x_SPI_Init(void);
    void SX127x_SPI_TranscieveBuffer( uint8_t *dataInOut, unsigned int size );
    uint8_t SX127x_SPI_TranscieveByte( uint8_t dataIn );

    void SX127x_Timeout_init(LoRaTimeout_t timeout);
    void SX127x_Timeout_SetTime(LoRaTimeout_t timeout, unsigned int time);

    void SX127x_delayUs(unsigned int us);
    void SX127x_delayMs(unsigned int ms);

    void SX127x_debug(const char *format, ...);
    void SX127x_debug_if(bool condition, const char *format, ...);
    
    
    
    
    
#ifdef __cplusplus
}
#endif

#endif /* LORAPORT_H */

