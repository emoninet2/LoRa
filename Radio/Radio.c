/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "Radio.h"


void RadioInit(SX1272_t *Radio){
    
    SX1272_Init(Radio);
    
}

void Mode(SX1272_t *Radio, OpMode_t mode){
    if(Radio->Settings.Modem  == MODEM_LORA){
        LoRaMode(Radio,(LoRa_OpMode_t) mode);
    }
    else if(Radio->Settings.Modem  == MODEM_FSK){
        if((mode != OpMode_RX_Single) || (mode != OpMode_CAD)){
            FskMode(Radio,(FSK_OpMode_t)mode );
    }
        
} 
}

int ReadPacket(SX1272_t *Radio, RxPacket_t *packet){
    if(Radio->Settings.Modem  == MODEM_LORA){
        Mode(Radio,OpMode_STDBY);
        //LoRaMode(LoRa_OpMode_STDBY);
        packet->size = LoRaRxPayloadBytes(Radio);
        uint8_t temp;
        LoRaWriteFifoAddrPtr(Radio,0x00);
        int i;
        for(i=0;i<packet->size;i++){
                ReadFifo(Radio,&packet->dataPtr[i],1);
        }
        uint8_t snr = LoRaPacketSnr(Radio);
        uint8_t rssi = LoRaPacketRssi(Radio);
        
        
        if(snr &0x80){
            packet->SNR = -( (~snr + 1) &0xFF)>>2 ;
        }
        else{
            packet->SNR = ( snr & 0xFF ) >> 2;
        }
        
        if(packet->SNR  < 0){
           //packet->RSSI = -139 + rssi + snr/4;
           packet->RSSI = -139 + rssi + (rssi >>4) + packet->SNR;
        }
        else{
           //packet->RSSI =  -139 + rssi
           packet->RSSI =  -139 + rssi + (rssi >> 4);
        }
       
        WriteRegister (Radio, REG_LORA, REG_LORA_IRQFLAGS, 0xFF);
    }
}
int SendPacket(SX1272_t *Radio, RxPacket_t *packet){
    if(Radio->Settings.Modem == MODEM_LORA){
        LoRaMode(Radio,LoRa_OpMode_STDBY);
        LoRaWriteFifoAddrPtr(Radio,0x00);
        LoRaWriteFifoTxBaseAddr(Radio,0x00);
        LoRaPayloadLength(Radio,packet->size);
        WriteFifo(Radio,packet->dataPtr, packet->size );
   
        DioMapping(Radio,0,SX1272_DioMapping_1 );
        Mode(Radio,OpMode_TX);
        while(LoRaIrqFlags(Radio,LoRa_IrqFlags_TxDone) == 0);
        
        DioMapping(Radio,0, Radio->Settings.DioMapping.dio0);
        Mode(Radio,OpMode_STDBY);
        //LoRaMode(LoRa_OpMode_STDBY);

        
        WriteRegister (Radio, REG_LORA, REG_LORA_IRQFLAGS, 0xFF);
    }   
}


