/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   LoRaHal.h
 * Author: emon
 *
 * Created on September 26, 2017, 6:32 PM
 */

#ifndef LORAHAL_H
#define LORAHAL_H


#include "../PORT/LoRaPort.h"    

#ifdef __cplusplus
extern "C" {
#endif
        void SX127x_HalInit(LoRaPort_t *_port);
        void SX127x_antInit(LoRaPort_t *_port);
        void SX127x_antDeInit(LoRaPort_t *_port);
        void SX127x_antSet(LoRaPort_t *_port,bool txrx);
        void SX127x_Reset(LoRaPort_t *_port);
        void SX127x_Write(LoRaPort_t *_port,uint8_t addr, uint8_t data);
        uint8_t SX127x_Read(LoRaPort_t *_port,uint8_t addr);
        void SX127x_WriteBuffer(LoRaPort_t *_port, uint8_t addr, uint8_t *buffer, uint8_t size );
        void SX127x_ReadBuffer(LoRaPort_t *_port, uint8_t addr, uint8_t *buffer, uint8_t size );

#ifdef __cplusplus
}
#endif

#endif /* LORAHAL_H */

