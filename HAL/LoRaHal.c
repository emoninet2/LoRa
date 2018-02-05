/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "LoRaHal.h"

void SX127x_HalInit(LoRaPort_t *_port){
    _port->SX127x_Port_Initialize();
}
void SX127x_antInit(LoRaPort_t *_port){
    _port->GPIO_Init(SX127x_GPIO_TX);
    _port->GPIO_Init(SX127x_GPIO_RX);
}
void SX127x_antDeInit(LoRaPort_t *_port){
    _port->GPIO_DeInit(SX127x_GPIO_TX);
    _port->GPIO_DeInit(SX127x_GPIO_RX);
}

void SX127x_antSet(LoRaPort_t *_port,bool txrx){
    if(txrx){
            _port->GPIO_SetValue(SX127x_GPIO_TX,1);
            _port->GPIO_SetValue(SX127x_GPIO_RX,0);
    }
    else{
            _port->GPIO_SetValue(SX127x_GPIO_TX,0);
            _port->GPIO_SetValue(SX127x_GPIO_RX,1);
    } 
}
void SX127x_Reset(LoRaPort_t *_port){
    	_port->GPIO_Init(SX127x_GPIO_RESET);
	_port->GPIO_SetValue(SX127x_GPIO_RESET,1);
	_port->delayMs(1);
	_port->GPIO_SetValue(SX127x_GPIO_RESET,0);
	//GPIO_DeInit(PIN_RESET);
        _port->delayMs(6);
}
void SX127x_Write(LoRaPort_t *_port,uint8_t addr, uint8_t data){
    uint8_t address = addr | (1<<7);
    _port->GPIO_SetValue(SX127x_GPIO_NSS,0);
    _port->SPI_TranscieveBuffer(&address, 1);
    _port->SPI_TranscieveBuffer(&data, 1);
    _port->GPIO_SetValue(SX127x_GPIO_NSS,1);
}
uint8_t SX127x_Read(LoRaPort_t *_port,uint8_t addr){
        uint8_t address = addr& ~(1<<7);
    uint8_t data;
    _port->GPIO_SetValue(SX127x_GPIO_NSS,0);
    _port->SPI_TranscieveBuffer(&address, 1);
    _port->SPI_TranscieveBuffer(&data, 1);
    _port->GPIO_SetValue(SX127x_GPIO_NSS,1);
return data;
}
void SX127x_WriteBuffer(LoRaPort_t *_port, uint8_t addr, uint8_t *buffer, uint8_t size ){
    uint8_t address = addr& ~(1<<7);
    _port->GPIO_SetValue(SX127x_GPIO_NSS,0);
    _port->SPI_TranscieveBuffer(&address, 1);
    _port->SPI_TranscieveBuffer(buffer, size);
    _port->GPIO_SetValue(SX127x_GPIO_NSS,1);
}
void SX127x_ReadBuffer(LoRaPort_t *_port, uint8_t addr, uint8_t *buffer, uint8_t size ){
    uint8_t address = addr | (1<<7);
    _port->GPIO_SetValue(SX127x_GPIO_NSS,0);
    _port->SPI_TranscieveBuffer(&address, 1);
    _port->SPI_TranscieveBuffer(buffer, size);
    _port->GPIO_SetValue(SX127x_GPIO_NSS,1);
}