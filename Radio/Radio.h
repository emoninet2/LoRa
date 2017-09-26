/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Radio.h
 * Author: emon
 *
 * Created on September 26, 2017, 9:42 PM
 */

#ifndef RADIO_H
#define RADIO_H

#include "../SX127x/SX1272/SX1272.h"


#ifdef __cplusplus
extern "C" {
#endif
    
    typedef struct{
        uint8_t *dataPtr;
        unsigned int size;
        int SNR;
        int RSSI;
    }RxPacket_t;
    
    typedef struct{
        uint8_t *dataPtr;
        unsigned int size;
    }TxPacket_t;
    
    
    
    void RadioInit(SX1272_t *Radio);
    void Mode(SX1272_t *Radio, OpMode_t mode);
    int ReadPacket(SX1272_t *Radio, RxPacket_t *packet);
    int SendPacket(SX1272_t *Radio, RxPacket_t *packet);


    static void InterrupDio0(void);
    static void InterrupDio1(void);
    static void InterrupDio2(void);
    static void InterrupDio3(void);
    static void InterrupDio4(void);
    static void InterrupDio5(void);
    
    
#ifdef __cplusplus
}
#endif

#endif /* RADIO_H */

