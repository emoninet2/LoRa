/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "SX1272.h"



void LoRaRxDoneHandler(SX1272_t *Radio){
    printf("RXD\r\n");
}
void LoRaTxDoneHandler(SX1272_t *Radio){
    printf("TXD\r\n");
}


void LoRaCadDoneHandler(SX1272_t *Radio){
  
}
void LoRaRxTimeoutHandler(SX1272_t *Radio){

}
void LoRaFhssChangeChannelHandler(SX1272_t *Radio){
  
}
void LoRaCadDetectedHandler(SX1272_t *Radio){
   
}
void LoRaValidHeaderHandler(SX1272_t *Radio){

}
void LoRaPyloadCrcErrorHandler(SX1272_t *Radio){

}
void LoRaPllLockHandler(SX1272_t *Radio){

}
void LoRaModeReadyHandler(SX1272_t *Radio){

}





void FskModeReadyHandler(SX1272_t *Radio){}
void FskRxReadyHandler(SX1272_t *Radio){}
void FskTxReadyHandler(SX1272_t *Radio){}
void FskPllLockHandler(SX1272_t *Radio){}
void FskRssiHandler(SX1272_t *Radio){}
void FskTimeoutHandler(SX1272_t *Radio){}
void FskPreambleDetectHandler(SX1272_t *Radio){}
void FskSyncAddressMatchHandler(SX1272_t *Radio){}
void FskFifoFullHandler(SX1272_t *Radio){}
void FskFifoEmptyHandler(SX1272_t *Radio){}
void FskFifoLevelHandler(SX1272_t *Radio){}
void FskFifoOverrun(SX1272_t *Radio){}
void FskDataHandler(SX1272_t *Radio){}///////CHECK IF SAME as FskFifoOverrun

void FskPacketSentHandler(SX1272_t *Radio){}
void FskPayloadReadyHandler(SX1272_t *Radio){}
void FskCrcOkHandler(SX1272_t *Radio){}
void FskTempChangeLowBatHandler(SX1272_t *Radio){}




void dio0IrqHandler(SX1272_t *Radio  ){

    if(Radio->Settings.Modem == MODEM_LORA){
        switch(Radio->Settings.DioMapping.dio0){
            case SX1272_DioMapping_0:{
                LoRaRxDoneHandler(Radio);
            }break;
            case SX1272_DioMapping_1:{
                LoRaTxDoneHandler(Radio);
            }break;
            case SX1272_DioMapping_2:{
                LoRaCadDoneHandler(Radio);
            }break;

        }
    }
}
void dio1IrqHandler(SX1272_t *Radio  ){
    if(Radio->Settings.Modem  == MODEM_LORA){
        switch(Radio->Settings.DioMapping.dio1){
            case SX1272_DioMapping_0:{
                LoRaRxTimeoutHandler(Radio);
            }break;
            case SX1272_DioMapping_1:{
                LoRaFhssChangeChannelHandler(Radio);
            }break;
            case SX1272_DioMapping_2:{
                LoRaCadDetectedHandler(Radio);
            }break;
        }
    }
}
void dio2IrqHandler(SX1272_t *Radio  ){
    if(Radio->Settings.Modem  == MODEM_LORA){
        switch(Radio->Settings.DioMapping.dio2){
            case SX1272_DioMapping_0:{
                LoRaFhssChangeChannelHandler(Radio);
            }break;
            case SX1272_DioMapping_1:{
                LoRaFhssChangeChannelHandler(Radio);
            }break;
            case SX1272_DioMapping_2:{
                LoRaFhssChangeChannelHandler(Radio);
            }break;
        }
    }
}
void dio3IrqHandler(SX1272_t *Radio  ){
    if(Radio->Settings.Modem  == MODEM_LORA){
        switch(Radio->Settings.DioMapping.dio3){
            case SX1272_DioMapping_0:{
                LoRaCadDoneHandler(Radio);
            }break;
            case SX1272_DioMapping_1:{
                LoRaValidHeaderHandler(Radio);
            }break;
            case SX1272_DioMapping_2:{
                LoRaPyloadCrcErrorHandler(Radio);
            }break;
        }
    }    
}
void dio4IrqHandler(SX1272_t *Radio){
    if(Radio->Settings.Modem  == MODEM_LORA){
        switch(Radio->Settings.DioMapping.dio4){
            case SX1272_DioMapping_0:{
                LoRaCadDetectedHandler(Radio);
            }break;
            case SX1272_DioMapping_1:{
                LoRaPllLockHandler(Radio);
            }break;
            case SX1272_DioMapping_2:{
                LoRaPllLockHandler(Radio);
            }break;
        }
    }    
}
void dio5IrqHandler(SX1272_t *Radio  ){
    if(Radio->Settings.Modem  == MODEM_LORA){
        switch(Radio->Settings.DioMapping.dio5){
            case SX1272_DioMapping_0:{
                LoRaModeReadyHandler(Radio);
            }break;
        }
    }    
}

void SX1272_Init(SX1272_t *Radio){
    SX127x_Reset(&Radio->Port);
    SX127x_HalInit(&Radio->Port);
    SetModem(Radio, Radio->Settings.Modem);
    
    
    Frf(Radio,Radio->Settings.Frequency);
    PaConfig(Radio,Radio->Settings.Pa.PaSel, Radio->Settings.Pa.OutputPower);
    PaRamp(Radio,Radio->Settings.Pa.LowPnTxPllOff , Radio->Settings.Pa.Ramp);
    OCP(Radio,Radio->Settings.Ocp.On, Radio->Settings.Ocp.trim);
    LNA(Radio,Radio->Settings.Lna.gain, Radio->Settings.Lna.boostOn)    ;
    
    DioMapping(Radio,0, Radio->Settings.DioMapping.dio0);
    DioMapping(Radio,1, Radio->Settings.DioMapping.dio1);
    DioMapping(Radio,2, Radio->Settings.DioMapping.dio2);
    DioMapping(Radio,3, Radio->Settings.DioMapping.dio3);
    DioMapping(Radio,4, Radio->Settings.DioMapping.dio4);
    DioMapping(Radio,5, Radio->Settings.DioMapping.dio5);
    

    LoRaModemConfig(Radio,
        Radio->Settings.LoRaSettings.ModemBandwidth,
        Radio->Settings.LoRaSettings.ModemCodingRate,
        Radio->Settings.LoRaSettings.implicitHeaderModeOn,
        Radio->Settings.LoRaSettings.RxPayloadCrcOn,
        Radio->Settings.LoRaSettings.LowDataRateOptimize,
        Radio->Settings.LoRaSettings.spreadingfactor,
        Radio->Settings.LoRaSettings.txContinuousMode,
        Radio->Settings.LoRaSettings.AgcAutoOn
    );  
    LoRaSymbTimeout(Radio,Radio->Settings.LoRaSettings.SymbTimeout);
    LoRaPreambleLength(Radio,Radio->Settings.LoRaSettings.PreambleLength);
    LoRaPayloadLength(Radio,Radio->Settings.LoRaSettings.PayloadLength);
    LoRaPayloadMaxLength(Radio,Radio->Settings.LoRaSettings.PayloadMaxLength);
    LoRaFreqHopPeriod(Radio,Radio->Settings.LoRaSettings.FreqHopPeriod);
    LoRaInvertIQSignal(Radio,Radio->Settings.LoRaSettings.InvertIqSignal);
    LoRaSyncWord(Radio,Radio->Settings.LoRaSettings.SyncWord);
    
}

void SetModem(SX1272_t *Radio, RadioModems_t modem){
    //if(Radio->Settings.Modem == modem) return;
    
    if(modem == MODEM_FSK){
        FskMode(Radio,FSK_OpMode_Sleep);
        LongRangeMode(Radio,SX1272_LongRangeMode_FSK);
        printf("will set to FSK modem\r\n");
    }
    else if(modem == MODEM_LORA){
        FskMode(Radio,FSK_OpMode_Sleep);
        LongRangeMode(Radio,SX1272_LongRangeMode_LoRa);
        printf("will set to LoRa modem\r\n");
    }
    
    Radio->Settings.Modem = modem;
    

}
uint8_t ReadRegister(SX1272_t *Radio, Reg_t regType, uint8_t addr){
    uint8_t retVal; 
    if((Radio->Settings.Modem == MODEM_LORA) && (regType == REG_FSK ) ){
        uint8_t RegOpMode_Original = SX127x_Read(&Radio->Port, REG_COMMON_OPMODE );  
        LoRaMode(Radio,LoRa_OpMode_STDBY_FskRegAccess);
        retVal = SX127x_Read(&Radio->Port, addr);
        SX127x_Write(&Radio->Port, REG_COMMON_OPMODE , RegOpMode_Original);
    }else{
        retVal = SX127x_Read(&Radio->Port, addr);
    }

    return retVal;
    
}
void WriteRegister(SX1272_t *Radio, Reg_t regType, uint8_t addr, uint8_t data){
        if((Radio->Settings.Modem == MODEM_LORA) && (regType == REG_FSK ) ){
        uint8_t RegOpMode_Original = SX127x_Read(&Radio->Port,REG_COMMON_OPMODE );
        LoRaMode(Radio,LoRa_OpMode_STDBY_FskRegAccess);
        SX127x_Write(&Radio->Port,addr, data);
        SX127x_Write(&Radio->Port,REG_COMMON_OPMODE , RegOpMode_Original);
    }else{
        SX127x_Write(&Radio->Port,addr, data);
    }
}
void WriteFifo(SX1272_t *Radio,   uint8_t *buffer, uint8_t size ){
    int i;
    for(i=0;i<size;i++){
        SX127x_Write(&Radio->Port,0, buffer[i]);
    }
}
void ReadFifo(SX1272_t *Radio,  uint8_t *buffer, uint8_t size) {
    int i;
    for(i=0;i<size;i++){
        buffer[i] = SX127x_Read(&Radio->Port,0);
    }
}

void LongRangeMode(SX1272_t *Radio, SX1272_LongRangeMode_t mode){
    SX127x_Write(&Radio->Port, REG_COMMON_OPMODE, mode );
}
void Frf(SX1272_t *Radio, uint32_t freq){
    SX127x_Write(&Radio->Port, REG_COMMON_FRFMSB, (freq>>16) );
    SX127x_Write(&Radio->Port, REG_COMMON_FRFMID, (freq>>8)  );
    SX127x_Write(&Radio->Port, REG_COMMON_FRFLSB, (freq>>0) );
}
void PaConfig(SX1272_t *Radio, SX1272_PaSelect_t PAsel, uint8_t outputPower){
    uint8_t temp = (PAsel<<SX1272_PaConfig_PaSelect_bp) |
                    ((outputPower<<SX1272_PaConfig_OutputPower_gp)&SX1272_PaConfig_OutputPower_gm);
    SX127x_Write(&Radio->Port, REG_COMMON_PACONFIG,temp );
}
void PaRamp(SX1272_t *Radio, bool LowPnTxPllOff , SX1272_PaRamp_t PaRamp){
    uint8_t temp = (LowPnTxPllOff<<SX1272_PaRamp_LowPnTxPllOff_bp )|
                    ((PaRamp<<SX1272_PaRamp_PaRamp_gp)&SX1272_PaRamp_PaRamp_gm);
    SX127x_Write(&Radio->Port, REG_COMMON_PARAMP,temp );
}
void OCP(SX1272_t *Radio, bool ocpOn, uint8_t trim){
    uint8_t temp = (ocpOn<<SX1272_Ocp_OcpOn_bp) | (trim<<SX1272_Ocp_OcpTrim_gp);
    SX127x_Write(&Radio->Port, REG_COMMON_OCP,temp );
}
void LNA(SX1272_t *Radio, SX1272_LnaGain_t gain, SX1272_LnaBoost_t boostOn){
    uint8_t temp = gain | boostOn;
    SX127x_Write(&Radio->Port, REG_COMMON_LNA,temp );
}
void DioMapping(SX1272_t *Radio, uint8_t pin, SX1272_DioMapping_t map){
    uint8_t temp;
	switch(pin){
	case 0:
		temp = SX127x_Read(&Radio->Port, REG_COMMON_DIOMAPPING1 );
		temp&= ~SX1272_Dio0Mapping_gm;
		temp |= map<<SX1272_Dio0Mapping_gp;
		SX127x_Write(&Radio->Port, REG_COMMON_DIOMAPPING1,temp );
		break;
	case 1:
		temp = SX127x_Read(&Radio->Port, REG_COMMON_DIOMAPPING1 );
		temp&= ~SX1272_Dio1Mapping_gm;
		temp |= map<<SX1272_Dio1Mapping_gp;
		SX127x_Write(&Radio->Port, REG_COMMON_DIOMAPPING1,temp );
		break;
	case 2:
		temp = SX127x_Read(&Radio->Port, REG_COMMON_DIOMAPPING1 );
		temp&= ~SX1272_Dio2Mapping_gm;
		temp |= map<<SX1272_Dio2Mapping_gp;
		SX127x_Write(&Radio->Port, REG_COMMON_DIOMAPPING1,temp );
		break;
	case 3:
		temp = SX127x_Read(&Radio->Port, REG_COMMON_DIOMAPPING1 );
		temp&= ~SX1272_Dio3Mapping_gm;
		temp |= map<<SX1272_Dio3Mapping_gp;
		SX127x_Write(&Radio->Port, REG_COMMON_DIOMAPPING1,temp );
		break;
	case 4:
		temp = SX127x_Read(&Radio->Port, REG_COMMON_DIOMAPPING2 );
		temp&= ~SX1272_Dio4Mapping_gm;
		temp |= map<<SX1272_Dio4Mapping_gp;
		SX127x_Write(&Radio->Port, REG_COMMON_DIOMAPPING2,temp );
		break;
	case 5:
		temp = SX127x_Read(&Radio->Port, REG_COMMON_DIOMAPPING2 );
		temp&= ~SX1272_Dio5Mapping_gm;
		temp |= map<<SX1272_Dio5Mapping_gp;
		SX127x_Write(&Radio->Port, REG_COMMON_DIOMAPPING2,temp );
		break;
	default:
		break;
    }
}
void DioMapPreambleDetect(SX1272_t *Radio, SX1272_DioMapPreambleDetect_t sel){
    uint8_t temp = SX127x_Read(&Radio->Port, REG_COMMON_DIOMAPPING2 );
    temp&= ~LoRa_ModemStat_SignalDetected_bm;
    temp |= sel<<LoRa_ModemStat_SignalDetected_bp;
    SX127x_Write(&Radio->Port, REG_COMMON_DIOMAPPING2,temp );
}
uint8_t Version(SX1272_t *Radio ){
    return SX127x_Read(&Radio->Port, REG_COMMON_VERSION );
}
void AgcRefLevel(SX1272_t *Radio, uint8_t level){
    SX127x_Write(&Radio->Port, REG_COMMON_AGCREF,level );
}
void AgcStepThreshold(SX1272_t *Radio, SX1272_AgcStep_t step, uint8_t threshold){
    uint8_t temp;
    switch(step){
    case SX1272_AgcStep1:
            temp = SX127x_Read(&Radio->Port, REG_COMMON_AGCTHRESH1 );
            temp &= ~SX1272_ArgStep1Thresl_gm;
            temp |= (threshold<<SX1272_ArgStep1Thresl_gp)&SX1272_ArgStep1Thresl_gm;
            SX127x_Write(&Radio->Port, REG_COMMON_AGCTHRESH1, temp );
            break;
    case SX1272_AgcStep2:
            temp = SX127x_Read(&Radio->Port, REG_COMMON_AGCTHRESH2 );
            temp &= ~SX1272_ArgStep1Thres2_gm;
            temp |= (threshold<<SX1272_ArgStep1Thres2_gp)&SX1272_ArgStep1Thres2_gm;
            SX127x_Write(&Radio->Port, REG_COMMON_AGCTHRESH2, temp );
            break;
    case SX1272_AgcStep3:
            temp = SX127x_Read(&Radio->Port, REG_COMMON_AGCTHRESH2 );
            temp &= ~SX1272_ArgStep1Thres3_gm;
            temp |= (threshold<<SX1272_ArgStep1Thres3_gp)&SX1272_ArgStep1Thres3_gm;
            SX127x_Write(&Radio->Port, REG_COMMON_AGCTHRESH2, temp );
            break;
    case SX1272_AgcStep4:
            temp = SX127x_Read(&Radio->Port, REG_COMMON_AGCTHRESH3 );
            temp &= ~SX1272_ArgStep1Thres4_gm;
            temp |= (threshold<<SX1272_ArgStep1Thres4_gp)&SX1272_ArgStep1Thres4_gm;
            SX127x_Write(&Radio->Port, REG_COMMON_AGCTHRESH3, temp );
            break;
    case SX1272_AgcStep5:
            temp = SX127x_Read(&Radio->Port, REG_COMMON_AGCTHRESH3 );
            temp &= ~SX1272_ArgStep1Thres5_gm;
            temp |= (threshold<<SX1272_ArgStep1Thres5_gp)&SX1272_ArgStep1Thres5_gm;
            SX127x_Write(&Radio->Port, REG_COMMON_AGCTHRESH3, temp );
            break;
    }
}
void FastHopOn(SX1272_t *Radio, bool sel){
    uint8_t temp = sel<<SX1272_PllHop_FastHopOn_bp;
    SX127x_Write(&Radio->Port, REG_COMMON_PLLHOP, temp );
}
void Tcx0InputOn(SX1272_t *Radio, bool sel){
    uint8_t temp = sel<<SX1272_Txco_FastHopOn_bp;
    SX127x_Write(&Radio->Port, REG_COMMON_TCXO, temp );
}
void PaDac(SX1272_t *Radio, SX1272_PaDac_t padac){
    SX127x_Write(&Radio->Port, REG_COMMON_PADAC, padac );
}
void PllBandwidth(SX1272_t *Radio, SX1272_PllBandwidth_t sel){
    uint8_t temp = (sel<<SX1272_PllBandwidth_gp);
    SX127x_Write(&Radio->Port, REG_COMMON_PLL, temp&SX1272_PllBandwidth_gm );
}
void LowNoisePllBandwidth(SX1272_t *Radio, SX1272_PllBandwidth_t sel){
    uint8_t temp = (sel<<SX1272_PllBandwidthLowPhaseNoise_gp);
    SX127x_Write(&Radio->Port, REG_COMMON_PLLLOWPN, temp&SX1272_PllBandwidthLowPhaseNoise_gm );
}
uint8_t FormerTemp(SX1272_t *Radio ){
    return SX127x_Read(&Radio->Port, REG_COMMON_FORMERTEMP );
}
void BitRateFrac(SX1272_t *Radio, uint8_t frac){
    SX127x_Write(&Radio->Port, REG_COMMON_PLLLOWPN, frac&SX1272_BitrateFrac_gm );
}



void FskCommonSetting(SX1272_t *Radio, 
        FSK_ModulationType_t ModulationType,
        FSK_ModulationShaping_FSK_t ModulationShapingFsk,
        FSK_ModulationShaping_OOK_t ModulationShapingOok,
        uint16_t bitrate,
        uint16_t fdev){

   if(ModulationType == FSK_ModulationType_FSK){
        uint8_t temp =  ReadRegister ( Radio,  REG_FSK,REG_COMMON_OPMODE);
        temp &= ~FSK_OpMode_ModulationType_gm;
	temp |= ModulationType << FSK_OpMode_ModulationType_gp;
        temp &= ~FSK_OpMode_ModulationShaping_gm;
	temp |= ModulationShapingFsk<<FSK_OpMode_ModulationShaping_gp;
        WriteRegister ( Radio,  REG_FSK, REG_COMMON_OPMODE, temp );
    }
    else if(ModulationType == FSK_ModulationType_OOK){
        uint8_t temp =  ReadRegister ( Radio,  REG_FSK,REG_COMMON_OPMODE);
        temp &= ~FSK_OpMode_ModulationType_gm;
	temp |= ModulationType << FSK_OpMode_ModulationType_gp;
        temp &= ~FSK_OpMode_ModulationShaping_gm;
	temp |= ModulationShapingOok<<FSK_OpMode_ModulationShaping_gp;
        WriteRegister ( Radio,  REG_FSK, REG_COMMON_OPMODE, temp );    
    }
    
    WriteRegister ( Radio,  REG_FSK, REG_FSK_BITRATEMSB, (bitrate>>8) );
    WriteRegister ( Radio,  REG_FSK, REG_FSK_BITRATELSB, (bitrate>>0) );
    
    WriteRegister ( Radio,  REG_FSK, REG_FSK_FDEVMSB, (fdev>>8) );
    WriteRegister ( Radio,  REG_FSK, REG_FSK_FDEVLSB, (fdev>>0) );

}

void FskMode(SX1272_t *Radio, FSK_OpMode_t sel){
    uint8_t temp =  ReadRegister ( Radio,  REG_FSK,REG_COMMON_OPMODE);
    temp &= ~FSK_OpMode_Mode_gm;
    temp |= sel<<FSK_OpMode_Mode_gp;
    WriteRegister ( Radio,  REG_FSK, REG_COMMON_OPMODE, temp );
}

void FskRestartRxOnCollision(SX1272_t *Radio, bool sel){
    uint8_t temp =  ReadRegister ( Radio,  REG_FSK,REG_FSK_RXCONFIG);
    temp &= ~FSK_RegRxConfig_RestartRxOnCollision_bm;
    temp |= sel<<FSK_RegRxConfig_RestartRxOnCollision_bp;
    WriteRegister ( Radio,  REG_FSK, REG_FSK_RXCONFIG, temp );
}
void FskRestartRxWithougPllLock(SX1272_t *Radio ){
    uint8_t temp =  ReadRegister ( Radio,  REG_FSK,REG_FSK_RXCONFIG);
    temp &= ~FSK_RegRxConfig_RestartRxWithoutPllLock_bm;
    temp |= 1<<FSK_RegRxConfig_RestartRxWithoutPllLock_bp;
    WriteRegister ( Radio,  REG_FSK, REG_FSK_RXCONFIG, temp );
}
void FskRestartRxWithPllLock(SX1272_t *Radio ){
    uint8_t temp =  ReadRegister ( Radio,  REG_FSK,REG_FSK_RXCONFIG);
    temp &= ~FSK_RegRxConfig_RestartRxWithPllLock_bm;
    temp |= 1<<FSK_RegRxConfig_RestartRxWithPllLock_bp;
    WriteRegister ( Radio,  REG_FSK, REG_FSK_RXCONFIG, temp );
}
void FskAfcAutoOn(SX1272_t *Radio, bool sel){
	uint8_t temp =  ReadRegister ( Radio,  REG_FSK,REG_FSK_RXCONFIG);
	temp &= ~FSK_RegRxConfig_AfcAutoOn_bm;
	temp |= sel<<FSK_RegRxConfig_AfcAutoOn_bp;
WriteRegister ( Radio,  REG_FSK, REG_FSK_RXCONFIG, temp );
}
void FskAgcAutoOn(SX1272_t *Radio, bool sel){
	uint8_t temp =  ReadRegister ( Radio,  REG_FSK,REG_FSK_RXCONFIG);
	temp &= ~FSK_RegRxConfig_AgcAutoOn_bm;
	temp |= sel<<FSK_RegRxConfig_AgcAutoOn_bp;
WriteRegister ( Radio,  REG_FSK, REG_FSK_RXCONFIG, temp );
}
void FskRxTrigger(SX1272_t *Radio, FSK_RxTrigger_t rxTrigger){
    uint8_t temp = ReadRegister ( Radio,  REG_FSK,REG_FSK_RXCONFIG);
    temp &= ~FSK_RegRxConfig_RxTrigger_gm;
    temp |= (rxTrigger << FSK_RegRxConfig_RxTrigger_gp);
    WriteRegister ( Radio,  REG_FSK, REG_FSK_RXCONFIG, temp );
}//needs table to be checked (SX1272_t *Radio, Table 23 for description on datasheet)
void FskRssiConfig(SX1272_t *Radio, 
                uint8_t rssiOffset,
                FSK_RssiSmoothing_t rssiSmoothing,
                uint8_t rssiCollision,
                uint8_t rssiThres,
                uint8_t rssiVal){
    uint8_t temp = (rssiOffset << FSK_RegRssiConfig_RssiOffset_gp) | (rssiSmoothing << FSK_RegRssiConfig_RssiSmoothing_gp) ;
    WriteRegister ( Radio,  REG_FSK, REG_FSK_RSSICONFIG, temp );
    WriteRegister ( Radio,  REG_FSK, REG_FSK_RSSICOLLISION, rssiCollision );
    WriteRegister ( Radio,  REG_FSK, REG_FSK_RSSITHRESH, rssiThres );
    WriteRegister ( Radio,  REG_FSK, REG_FSK_RSSIVALUE, rssiVal );
}
void FskRxBw(SX1272_t *Radio, FSK_RxBwMant_t RxBwMant, uint8_t RxBwExp){
    uint8_t temp = (RxBwMant << FSK_RegRxBw_RxBwMant_gp) | (RxBwExp << FSK_RegRxBw_RxBwExp_gp );
    WriteRegister ( Radio,  REG_FSK, REG_FSK_RXBW, temp );
}
void FskAfcBw(SX1272_t *Radio, uint8_t RxBwMantAfc, uint8_t RxBwExpAfc){
    uint8_t temp = (RxBwMantAfc << FSK_RegAfcBw_RxBwMantAfc_gp) | (RxBwExpAfc << FSK_RegAfcBw_RxBwExpAfc_gp );
    WriteRegister ( Radio,  REG_FSK, REG_FSK_AFCBW, temp );
}
void FskOok(SX1272_t *Radio, 
                bool bitSyncOn,
                FSK_OokThresType_t OokThresType,
                FSK_OokPeakThresStep_t OokPeakTheshStep,
                uint8_t OokFixedThreshold,
                FSK_OokPeakThresDec_t OokPeakThreshDec,
                FSK_OokAverageOffset_t OokAverageOffset,
                FSK_OokAverageThreshFilt_t  OokAverageThreshFilt
                ){
uint8_t temp = (bitSyncOn << FSK_RegOokPeak_BitSyncOn_bp)
			| (OokThresType << FSK_RegOokPeak_OokThreshType_gp)
			| (OokPeakTheshStep << FSK_RegOokPeak_OokPeakTheshStep_gp) ;
	WriteRegister ( Radio,  REG_FSK, REG_FSK_OOKPEAK, temp );
	WriteRegister ( Radio,  REG_FSK, REG_FSK_OOKFIX, OokFixedThreshold );

	temp = (OokPeakThreshDec << FSK_RegOokAvg_OokPeakThreshDec_gp)
			| (OokAverageOffset<<FSK_RegOokAvg_OokAverageOffset_gp)
			| (OokAverageThreshFilt<<FSK_RegOokAvg_OokAverageThreshFilt_gp);
        WriteRegister ( Radio,  REG_FSK, REG_FSK_OOKAVG, temp );
}
void FskAgcStart(SX1272_t *Radio ){
    uint8_t temp =  ReadRegister ( Radio,  REG_FSK,REG_FSK_AFCFEI);
    temp |= 1<<FSK_RegAfcFei_AgcStart_bp;
    WriteRegister ( Radio,  REG_FSK, REG_FSK_AFCFEI, temp );
}
void FskAfcClear(SX1272_t *Radio ){
    uint8_t temp =  ReadRegister ( Radio,  REG_FSK,REG_FSK_AFCFEI);
    temp |= 1<<FSK_RegAfcFei_AfcClear_bp;
    WriteRegister ( Radio,  REG_FSK, REG_FSK_AFCFEI, temp );
}
void FskAfcAutoClearOn(SX1272_t *Radio, bool sel){
uint8_t temp =  ReadRegister ( Radio,  REG_FSK,REG_FSK_AFCFEI);
	temp &= ~FSK_RegAfcFei_AfcAutoClearOn_bm;
	temp |= 1<<FSK_RegAfcFei_AfcAutoClearOn_bp;
WriteRegister ( Radio,  REG_FSK, REG_FSK_AFCFEI, temp );
}
void FskAfcValue(SX1272_t *Radio, uint16_t val){
WriteRegister ( Radio,  REG_FSK, REG_FSK_AFCMSB, (val>>8) );
WriteRegister ( Radio,  REG_FSK, REG_FSK_AFCLSB, (val>>0) );
}
void FskFeiValue(SX1272_t *Radio, uint16_t val){
WriteRegister ( Radio,  REG_FSK, REG_FSK_FEIMSB, (val>>8)  );
WriteRegister ( Radio,  REG_FSK, REG_FSK_FEILSB, (val>>0) );
}
void FskPreambleDetector(SX1272_t *Radio, 
                bool PreambleDetectorOn,
                FSK_PreambleDetectorSize_t PreambleDetectorSize,
                uint8_t PreambleDetectorTol
                ){
    uint8_t temp = (PreambleDetectorOn<<FSK_RegPreambleDetect_PreambleDetectorOn_bp)
                            | (PreambleDetectorSize <<FSK_RegPreambleDetect_PreambleDetectorSize_gp)
                            | (PreambleDetectorTol<<FSK_RegPreambleDetect_PreambleDetectorTol_gp);
    WriteRegister ( Radio,  REG_FSK, REG_FSK_PREAMBLEDETECT, temp );
}
void FskTimeout(SX1272_t *Radio, uint8_t TimeoutRxRssi, uint8_t TimeoutRxPreamble, uint8_t TimeoutSignalSync, uint8_t InterPacketRxDelay){
    WriteRegister ( Radio,  REG_FSK, REG_FSK_RXTIMEOUT1, TimeoutRxRssi );
    WriteRegister ( Radio,  REG_FSK, REG_FSK_RXTIMEOUT2, TimeoutRxPreamble );
    WriteRegister ( Radio,  REG_FSK, REG_FSK_RXTIMEOUT3, TimeoutSignalSync );
    WriteRegister ( Radio,  REG_FSK, REG_FSK_RXDELAY, InterPacketRxDelay );
}

void FskRcCalStart(SX1272_t *Radio ){
	uint8_t temp = ReadRegister ( Radio,  REG_FSK,REG_FSK_OSC);
	temp |= FSK_RegOsc_RcCalStart_bm;
WriteRegister ( Radio,  REG_FSK, REG_FSK_OSC, temp );
}
void FskClkOut(SX1272_t *Radio, FSK_ClkOut_t sel){
WriteRegister ( Radio,  REG_FSK, REG_FSK_OSC, sel );
}

void FskPacketHandlingConfig(SX1272_t *Radio, 
        uint16_t PreambleSize,
        FSK_AutoRestartRxMode_t AutoRestartRxMode,
        FSK_PreamblePolarity_t PreamblePolarity,
        bool SyncOn,
        bool FifoFillCondition,
        uint8_t SyncSize,
        uint64_t SyncVal,
        FSK_PacketFormat_t PacketFormat,
        FSK_DCFree_t DcFree,
        bool CrcOn,
        bool CrcAutoClearOff,
        FSK_AddressFiltering_t AddressFiltering,
        FSK_CrcWhitening_t CrcWhiteningType,
        FSK_DataMode_t DataMode,
        bool IoHomeOn,
        bool IoHomePowerFrame,
        bool BeaconOn,
        uint16_t payloadLen,
        uint8_t nodeAddr,
        uint8_t broadcastAddr,
        FSK_TxStartCondition_t startCond,
        uint8_t fifoThreshold ){

WriteRegister ( Radio,  REG_FSK, REG_FSK_PREAMBLEMSB, (PreambleSize>>8) );
    WriteRegister ( Radio,  REG_FSK, REG_FSK_PREAMBLELSB, (PreambleSize>>0) );
    
    uint8_t temp = (AutoRestartRxMode << FSK_RegSyncConfig_AutoRestartRxMode_gp)
			| (PreamblePolarity<<FSK_RegSyncConfig_PreamblePolarity_bp)
			| (SyncOn<<FSK_RegSyncConfig_SyncOn_bp)
			| (FifoFillCondition << FSK_RegSyncConfig_FifoFillCondition_bp)
			| (SyncSize<<FSK_RegSyncConfig_SyncSize_gp);
    WriteRegister ( Radio,  REG_FSK, REG_FSK_SYNCCONFIG, temp);

    WriteRegister ( Radio,  REG_FSK, REG_FSK_SYNCVALUE1, (SyncVal>>56));
    WriteRegister ( Radio,  REG_FSK, REG_FSK_SYNCVALUE2, (SyncVal>>48));
    WriteRegister ( Radio,  REG_FSK, REG_FSK_SYNCVALUE3, (SyncVal>>40));
    WriteRegister ( Radio,  REG_FSK, REG_FSK_SYNCVALUE4, (SyncVal>>32));
    WriteRegister ( Radio,  REG_FSK, REG_FSK_SYNCVALUE5, (SyncVal>>24));
    WriteRegister ( Radio,  REG_FSK, REG_FSK_SYNCVALUE6, (SyncVal>>16));
    WriteRegister ( Radio,  REG_FSK, REG_FSK_SYNCVALUE7, (SyncVal>>8));
    WriteRegister ( Radio,  REG_FSK, REG_FSK_SYNCVALUE8, (SyncVal>>0));
    
    
    uint8_t config1 = (PacketFormat << FSK_RegPacketConfig1_PacketFormat_bp)
			| (DcFree<<FSK_RegPacketConfig1_DcFree_gp)
			| (CrcOn<<FSK_RegPacketConfig1_CrcOn_bp)
			| (CrcAutoClearOff << FSK_RegPacketConfig1_CrcAutoClearOff_bp)
			| (AddressFiltering<<FSK_RegPacketConfig1_AddressFiltering_gp)
			| (AddressFiltering<<FSK_RegPacketConfig1_CrcWhiteningType_bp);

    uint8_t config2  = (DataMode << FSK_RegPacketConfig2_DataMode_bp)
                    | (IoHomeOn<<FSK_RegPacketConfig2_IoHomeOn_bp)
                    | (IoHomePowerFrame<<FSK_RegPacketConfig2_IoHomePowerFrame_bp)
                    | (BeaconOn<<FSK_RegPacketConfig2_BeaconOn_bp);

    WriteRegister ( Radio,  REG_FSK, REG_FSK_PACKETCONFIG1, config1);
    WriteRegister ( Radio,  REG_FSK, REG_FSK_PACKETCONFIG2, config2);
    
    
    temp = ReadRegister ( Radio,  REG_FSK,REG_FSK_PACKETCONFIG2);
    temp &= ~0x07;
    temp |= (payloadLen>>8)&0x07;
    WriteRegister ( Radio,  REG_FSK, REG_FSK_PACKETCONFIG2, temp);
    WriteRegister ( Radio,  REG_FSK, REG_FSK_PAYLOADLENGTH, (payloadLen>>0));
    

    WriteRegister ( Radio,  REG_FSK,REG_FSK_NODEADRS,nodeAddr);
    WriteRegister ( Radio,  REG_FSK,REG_FSK_BROADCASTADRS,broadcastAddr);
    
    temp = (startCond << FSK_RegFifoThresh_TxStartCondition_bp) | (fifoThreshold << FSK_RegFifoThresh_FifoThreshold_gp );
    WriteRegister ( Radio,  REG_FSK,REG_FSK_FIFOTHRESH, temp);
}


void FskSequenceStart(SX1272_t *Radio ){
    	uint8_t temp =  ReadRegister ( Radio,  REG_FSK,REG_FSK_SEQCONFIG1);
	temp &= ~FSK_RegSeqConfig1_SequencerStart_bm;
	temp |= 1<<FSK_RegSeqConfig1_SequencerStart_bp;
WriteRegister ( Radio,  REG_FSK, REG_FSK_SEQCONFIG1, temp );
}
void FskSequenceStop(SX1272_t *Radio ){
	uint8_t temp =  ReadRegister ( Radio,  REG_FSK,REG_FSK_SEQCONFIG1);
	temp &= ~FSK_RegSeqConfig1_SequencerStop_bm;
	temp |= 1<<FSK_RegSeqConfig1_SequencerStop_bp;
WriteRegister ( Radio,  REG_FSK, REG_FSK_SEQCONFIG1, temp );
}
void FskSequencerConfig(SX1272_t *Radio, 
                FSK_IdleMode_t IdleMode,
                FSK_TransitionFromStart_t FromStart,
                FSK_LowPowerSelection_t LowPowerSelection,
                FSK_TransitionFromIdle_t FromIdle,
                FSK_TransitionFromTransmit_t FromTransmit,
                FSK_TransitionFromReceive_t FromReceive,
                FSK_TransitionFromRxTimeout_t FromRxTimeout,
                FSK_TransitionFromPacketReceived_t FromPacketReceived
                ){
uint8_t config1 = (IdleMode << FSK_RegSeqConfig1_IdleMode_bp)
			| (FromStart << FSK_RegSeqConfig1_FromStart_gp)
			| (LowPowerSelection << FSK_RegSeqConfig1_LowPowerSelection_bp)
			| (FromIdle << FSK_RegSeqConfig1_FromIdle_bp)
			| (FromTransmit << FSK_RegSeqConfig1_FromTransmit_bp);

	uint8_t config2 = (FromReceive << FSK_RegSeqConfig2_FromReceive_gp)
			| (FromRxTimeout << FSK_RegSeqConfig2_FromRxTimeout_gp)
			| (FromPacketReceived << FSK_RegSeqConfig2_FromPacketReceived_gp);

	WriteRegister ( Radio,  REG_FSK,REG_FSK_SEQCONFIG1, config1);
WriteRegister ( Radio,  REG_FSK,REG_FSK_SEQCONFIG2, config2);
}
void FskTimer(SX1272_t *Radio, 
                FSK_TimerResolution_t Timer1,
                FSK_TimerResolution_t Timer2,
                uint8_t Timer1Coeff,
                uint8_t Timer2Coeff
                ){
	uint8_t timerResol = (Timer1 << FSK_RegTimerResol_Timer1Resolution_gp) | (Timer2 << FSK_RegTimerResol_Timer2Resolution_gp);
	WriteRegister ( Radio,  REG_FSK,REG_FSK_TIMERRESOL, timerResol);

	WriteRegister ( Radio,  REG_FSK,REG_FSK_TIMER1COEF, Timer1Coeff);
WriteRegister ( Radio,  REG_FSK,REG_FSK_TIMER2COEF, Timer2Coeff);
}
void FskAutoImageCAlOn(SX1272_t *Radio, bool sel){
   uint8_t temp = ReadRegister ( Radio,  REG_FSK,REG_FSK_IMAGECAL);
    temp &= ~FSK_RegImageCal_AutoImageCalOn_bm;
    temp |= (sel << FSK_RegImageCal_AutoImageCalOn_bp);
WriteRegister ( Radio,  REG_FSK,REG_FSK_IMAGECAL, temp);
}
void FskImageCalStart(SX1272_t *Radio ){
   uint8_t temp = ReadRegister ( Radio,  REG_FSK,REG_FSK_IMAGECAL);
    temp |= (1 << FSK_RegImageCal_ImageCalStart_bp);
WriteRegister ( Radio,  REG_FSK,REG_FSK_IMAGECAL, temp);
}
bool FskImageCalRunning(SX1272_t *Radio ){
    uint8_t temp = ReadRegister ( Radio,  REG_FSK,REG_FSK_IMAGECAL);
    if( (temp & FSK_RegImageCal_ImageCalRunning_bm) == 0){
            return 0;
    }
    else{
            return 1;
    }
}
bool FskTempChange(SX1272_t *Radio ){
    uint8_t temp = ReadRegister ( Radio,  REG_FSK,REG_FSK_IMAGECAL);
    if( (temp & FSK_RegImageCal_TempChange_bm) == 0){
            return 0;
    }
    else{
            return 1;
    }
}
void FskTempThreshold(SX1272_t *Radio, FSK_TempThreshold_t sel){
    uint8_t temp = ReadRegister ( Radio,  REG_FSK,REG_FSK_IMAGECAL);
    temp &= ~FSK_RegImageCal_TempThreshold_gm;
    temp |= (sel << FSK_RegImageCal_TempThreshold_gp);
WriteRegister ( Radio,  REG_FSK,REG_FSK_IMAGECAL, temp);
}
void FskTempMonitorOff(SX1272_t *Radio, bool sel){
    uint8_t temp = ReadRegister ( Radio,  REG_FSK,REG_FSK_IMAGECAL);
    temp &= ~FSK_RegImageCal_TempMonitorOff_bm;
    temp |= (sel << FSK_RegImageCal_TempMonitorOff_bp);
WriteRegister ( Radio,  REG_FSK,REG_FSK_IMAGECAL, temp);
}
uint8_t FskTempValue(SX1272_t *Radio ){
    return ReadRegister ( Radio,  REG_FSK,REG_FSK_TEMP);
}
void FskLowBat(SX1272_t *Radio, bool LowBatOn,FSK_LowBatTrim_t LowBatTrim){
    uint8_t temp = (LowBatOn<<FSK_RegLowBat_LowBatOn_bp) | (LowBatTrim << FSK_RegLowBat_LowBatTrim_gp);
WriteRegister ( Radio,  REG_FSK,REG_FSK_LOWBAT, temp);
}
bool FskIrqFlags(SX1272_t *Radio, FSK_IrqFlags_t flag){
    if((flag >=0) && (flag<=7)){
		uint8_t temp = ReadRegister ( Radio,  REG_FSK,REG_FSK_IRQFLAGS1);
		return (temp >> flag)&0x01;
	}
	else if((flag >=8) && (flag<=15)){
		uint8_t temp = ReadRegister ( Radio,  REG_FSK,REG_FSK_IRQFLAGS2);
		return (temp >> (flag - 8))&0x01;
    }
}
void FskClearIrqFlag(SX1272_t *Radio, FSK_IrqFlags_t flag){

	if( (flag ==   FSK_IrqFlags_SyncAddressMatch)
			||  (flag ==   FSK_IrqFlags_PreambleDetect)
			|| (flag ==   FSK_IrqFlags_Rssi)
			|| (flag ==   FSK_IrqFlags_LowBat)
			|| (flag ==   FSK_IrqFlags_FIfoOverrun)
			){
		if((flag >=0) && (flag<=7)){
			uint8_t temp = ReadRegister ( Radio,  REG_FSK,REG_FSK_IRQFLAGS1);
			temp |= (1<<flag);
			WriteRegister ( Radio,  REG_FSK,REG_FSK_IRQFLAGS1, temp);
		}
		else if((flag >=8) && (flag<=15)){
			uint8_t temp = ReadRegister ( Radio,  REG_FSK,REG_FSK_IRQFLAGS2);
			temp |= (1<< (flag - 8));
			WriteRegister ( Radio,  REG_FSK,REG_FSK_IRQFLAGS2, temp);
		}
    }
}



void LoRaMode(SX1272_t *Radio, LoRa_OpMode_t mode){
    WriteRegister ( Radio, REG_LORA,REG_COMMON_OPMODE, mode);
}
void LoRaWriteFifoAddrPtr(SX1272_t *Radio, uint8_t addr){
    WriteRegister ( Radio, REG_LORA,REG_LORA_FIFOADDRPTR, addr);
}
uint8_t LoRaReadFifoAddrPtr(SX1272_t *Radio ){
    return ReadRegister ( Radio, REG_LORA,REG_LORA_FIFOADDRPTR);
}
void LoRaWriteFifoTxBaseAddr(SX1272_t *Radio, uint8_t addr){
    WriteRegister ( Radio, REG_LORA,REG_LORA_FIFOTXBASEADDR, addr);
}
uint8_t LoRaReadFifoTxBaseAddr(SX1272_t *Radio ){
    return ReadRegister ( Radio, REG_LORA,REG_LORA_FIFOTXBASEADDR);
}
void LoRaWriteFifoRxBaseAddr(SX1272_t *Radio, uint8_t addr){
    WriteRegister ( Radio, REG_LORA,REG_LORA_FIFORXBASEADDR, addr);
}
uint8_t LoRaReadFifoRxBaseAddr(SX1272_t *Radio){
    return ReadRegister ( Radio, REG_LORA,REG_LORA_FIFORXBASEADDR);
}
uint8_t LoRaReadFifoRxCurrentAddr(SX1272_t *Radio ){
    return ReadRegister ( Radio, REG_LORA,REG_LORA_FIFORXCURRENTADDR);
}
void LoRaIrqFlagsMask(SX1272_t *Radio, uint8_t mask){
    WriteRegister ( Radio, REG_LORA,REG_LORA_IRQFLAGSMASK, mask);
}
bool LoRaIrqFlags(SX1272_t *Radio, LoRa_IrqFlags_t flag){
    uint8_t temp = ReadRegister ( Radio, REG_LORA,REG_LORA_IRQFLAGS);
    if ((temp & flag)!= 0) return 1;
    else return 0;
}   
void LoRaClearIrqFlags(SX1272_t *Radio, LoRa_IrqFlags_t flag){
    WriteRegister ( Radio, REG_LORA, REG_LORA_IRQFLAGS, flag);
}
uint8_t LoRaRxPayloadBytes(SX1272_t *Radio ){
    return ReadRegister ( Radio, REG_LORA, REG_LORA_RXNBBYTES);
}
uint16_t LoRaValidHeaderCount(SX1272_t *Radio ){
    return (ReadRegister ( Radio, REG_LORA,REG_LORA_RXHEADERCNTVALUEMSB)<<8) |
        (ReadRegister ( Radio, REG_LORA,REG_LORA_RXHEADERCNTVALUELSB)<<0) ;
}
uint16_t LoRaValidPacketCount(SX1272_t *Radio ){
    return (ReadRegister ( Radio, REG_LORA,REG_LORA_RXPACKETCNTVALUEMSB)<<8) |
        (ReadRegister ( Radio, REG_LORA,REG_LORA_RXPACKETCNTVALUELSB)<<0) ;
}
uint8_t LoRaModemStatus(SX1272_t *Radio, LoRa_ModemStatus_t status){
    uint8_t temp =  ReadRegister ( Radio, REG_LORA,REG_LORA_MODEMSTAT);
    switch(status){
    case LoRa_ModemStatus_RxCodingRate:
            return (temp&LoRa_ModemStat_RxCodingRate_gm)>>LoRa_ModemStat_RxCodingRate_gp;
            break;
    case LoRa_ModemStatus_ModemClear:
            return (temp&LoRa_ModemStat_ModemClear_bm)>>LoRa_ModemStat_ModemClear_bp;
            break;
    case LoRa_ModemStatus_HeaderInfoValid:
            return (temp&LoRa_ModemStat_HeaderInfoValid_bm)>>LoRa_ModemStat_HeaderInfoValid_bp;
            break;
    case LoRa_ModemStatus_RxOnGoing:
            return (temp&LoRa_ModemStat_RxOnGoing_bm)>>LoRa_ModemStat_RxOnGoing_bp;
            break;
    case LoRa_ModemStatus_SignalSynchronized:
            return (temp&LoRa_ModemStat_SignalSynchronized_bm)>>LoRa_ModemStat_SignalSynchronized_bp;
            break;
    case LoRa_ModemStatus_SignalDetected:
            return (temp&LoRa_ModemStat_SignalDetected_bm)>>LoRa_ModemStat_SignalDetected_bp;
            break;
    default:
            return temp;
            break;
    }
}
uint8_t LoRaPacketSnr(SX1272_t *Radio ){
    return ReadRegister ( Radio, REG_LORA,REG_LORA_PKTSNRVALUE);
}
uint8_t LoRaPacketRssi(SX1272_t *Radio ){
    return ReadRegister ( Radio, REG_LORA,REG_LORA_PKTRSSIVALUE);
}
uint8_t LoRaRssi(SX1272_t *Radio ){
    return ReadRegister ( Radio, REG_LORA,REG_LORA_RSSIVALUE);
}   
bool LoRaPllTimeout(SX1272_t *Radio ){
    uint8_t temp =  ReadRegister ( Radio, REG_LORA,REG_LORA_HOPCHANNEL);
    if( (temp & LoRa_HopChannel_PllTimeout_bm) != 0) return 1;
    else return 0;
}
bool LoRaCrcOnPayload(SX1272_t *Radio ){
    uint8_t temp =  ReadRegister ( Radio, REG_LORA,REG_LORA_HOPCHANNEL);
    if( (temp & LoRa_HopChannel_CrcOnPayload_bm) != 0) return 1;
    else return 0;
}
uint8_t LoRaFhssPresentChannel(SX1272_t *Radio ){
    uint8_t temp =  ReadRegister ( Radio, REG_LORA,REG_LORA_HOPCHANNEL);
    return temp & LoRa_HopChannel_FhssPresentChannel_gm;
}
void LoRaModemConfig(SX1272_t *Radio, 
        LoRa_ModemBw_t bandwidth,
        LoRa_ModemCodingRate_t codingRate,
        bool implicitHeaderModeOn,
        bool RxPayloadCrcOn,
        bool LowDataRateOptimize,
        LoRa_ModemSpreadingFactor_t spreadingfactor,
        bool txContinuousMode,
        bool AgcAutoOn
        ){

    if(spreadingfactor == LoRa_ModemSpreadingFactor_6){
        implicitHeaderModeOn = 1;
    }


    uint8_t config1 = bandwidth |
                    codingRate |
                    (implicitHeaderModeOn<<LoRa_ModemConfig1_ImplicitHeaderModeOn_bp) |
                    (RxPayloadCrcOn<<LoRa_ModemConfig1_RxPayloadCrcOn_bp) |
                    (LowDataRateOptimize<<LoRa_ModemConfig1_LowDataRateOptimize_bp);

    uint8_t config2 = spreadingfactor |
                    (txContinuousMode<<LoRa_ModemConfig2_TxContMode_bp) |
                    (AgcAutoOn<<LoRa_ModemConfig2_ArcAutoOn_bp);

    WriteRegister ( Radio, REG_LORA,REG_LORA_MODEMCONFIG1, config1);
    WriteRegister ( Radio, REG_LORA,REG_LORA_MODEMCONFIG2, config2);


    if(spreadingfactor == LoRa_ModemSpreadingFactor_6){
        LoRaDetectOptimize(Radio,DetectOptimize_SF6 );
        LoRaDetectThreshold(Radio,DetectThreshold_SF6);
    }
    else{
        LoRaDetectOptimize(Radio,DetectOptimize_SF7_SF12 );
        LoRaDetectThreshold(Radio,DetectThreshold_SF7_SF12);
    }

}
void LoRaSymbTimeout(SX1272_t *Radio, uint16_t timeout){
    uint8_t tempMSB = ReadRegister ( Radio, REG_LORA,REG_LORA_MODEMCONFIG2);
    tempMSB &= ~LoRa_ModemConfig2_SymbTimeoutMSB_gm;
    tempMSB |= (timeout>>8) & LoRa_ModemConfig2_SymbTimeoutMSB_gm ;
    WriteRegister ( Radio, REG_LORA,REG_LORA_MODEMCONFIG2, tempMSB);

    uint8_t tempLSB = timeout&0xFF;
    WriteRegister ( Radio, REG_LORA,REG_LORA_SYMBTIMEOUTLSB, tempLSB);
}
void LoRaPreambleLength(SX1272_t *Radio, uint16_t len){
    WriteRegister ( Radio, REG_LORA,REG_LORA_PREAMBLEMSB, (len>>8)&0xFF);
    WriteRegister ( Radio, REG_LORA,REG_LORA_PREAMBLELSB, (len>>0)&0xFF);
}
void LoRaPayloadLength(SX1272_t *Radio, uint8_t len){
    WriteRegister ( Radio, REG_LORA,REG_LORA_PAYLOADLENGTH, len);
}
void LoRaPayloadMaxLength(SX1272_t *Radio, uint8_t len){
    WriteRegister ( Radio, REG_LORA,REG_LORA_PAYLOADMAXLENGTH, len);
}
void LoRaFreqHopPeriod(SX1272_t *Radio, uint8_t period){
    WriteRegister ( Radio, REG_LORA,REG_LORA_HOPPERIOD, period);
}
uint8_t LoRaFifoRxByteAddrPtr(SX1272_t *Radio ){
    return ReadRegister ( Radio, REG_LORA,REG_LORA_FIFORXBYTEADDR);
}
uint32_t LoRaFreqError(SX1272_t *Radio ){
    uint8_t FEIMSB =  ReadRegister ( Radio, REG_LORA,REG_LORA_FEIMSB);
    uint8_t FEIMID =  ReadRegister ( Radio, REG_LORA,REG_LORA_FEIMID);
    uint8_t FEILSB =  ReadRegister ( Radio, REG_LORA,REG_LORA_FEILSB);

    return (FEIMSB<<16) | (FEIMID<<8) | (FEILSB);
}
uint8_t LoRaRssiWideband(SX1272_t *Radio ){
    return ReadRegister ( Radio, REG_LORA,REG_LORA_RSSIWIDEBAND);
}
void LoRaDetectOptimize(SX1272_t *Radio, LoRa_DetectOptimize_t sel){
    WriteRegister ( Radio, REG_LORA,REG_LORA_DETECTOPTIMIZE,0xC0 | sel);
}
void LoRaInvertIQSignal(SX1272_t *Radio, bool sel){
    //0x27 is reserved for bits [5:0]. Removing this will cause error
    WriteRegister ( Radio, REG_LORA,REG_LORA_INVERTIQ,(sel<<LoRa_InvertIQ_bp) | 0x27 );
}
void LoRaDetectThreshold(SX1272_t *Radio, LoRa_DetectThreshold_t sel){
    WriteRegister ( Radio, REG_LORA,REG_LORA_DETECTIONTHRESHOLD,sel);
}
void LoRaSyncWord(SX1272_t *Radio, uint8_t SyncWord){
    WriteRegister ( Radio, REG_LORA,REG_LORA_SYNCWORD,SyncWord);
}