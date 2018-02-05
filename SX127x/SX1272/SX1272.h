/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SX1272.h
 * Author: emon
 *
 * Created on September 26, 2017, 7:53 PM
 */

#ifndef SX1272_H
#define SX1272_H

#include "../../PORT/LoRaPort.h"
#include "../../HAL/LoRaHal.h"

#include "SX1272_Reg.h"


#ifdef __cplusplus
extern "C" {
#endif

    

    typedef enum
    {
        REG_FSK = 0,
        REG_LORA,
    }Reg_t;
    
    typedef enum
    {
        MODEM_FSK = 0,
        MODEM_LORA,
    }RadioModems_t;
 
    typedef struct{
        SX1272_PaSelect_t PaSel;
        uint8_t OutputPower;
        bool LowPnTxPllOff;
        SX1272_PaRamp_t Ramp;
    }Settings_Pa_t;
   
    
    typedef struct{
        bool On;
        uint8_t trim;
    }Settings_Ocp_t;
    
    typedef struct{
        SX1272_LnaGain_t gain;
        SX1272_LnaBoost_t boostOn;
    }Settings_Lna_t;
    
    typedef struct{
        SX1272_DioMapping_t dio0;
        SX1272_DioMapping_t dio1;
        SX1272_DioMapping_t dio2;
        SX1272_DioMapping_t dio3;
        SX1272_DioMapping_t dio4;
        SX1272_DioMapping_t dio5;
    }Settings_DioMapping_t;
    
    
    typedef struct{
        uint8_t level;
        uint8_t Step1Threshold;
        uint8_t Step2Threshold;
        uint8_t Step3Threshold;
        uint8_t Step4Threshold;
        uint8_t Step5Threshold;
        uint8_t Step6Threshold;
    }Settings_Agc;
    
    
    
    typedef struct{
        LoRa_ModemBw_t ModemBandwidth;
        LoRa_ModemCodingRate_t ModemCodingRate;
        bool implicitHeaderModeOn;
        bool RxPayloadCrcOn;
        bool LowDataRateOptimize;
        LoRa_ModemSpreadingFactor_t spreadingfactor;
        bool txContinuousMode;
        bool AgcAutoOn;
        uint16_t SymbTimeout;
        uint16_t PreambleLength;
        uint8_t PayloadLength;
        uint8_t PayloadMaxLength;
        uint8_t FreqHopPeriod;
        bool InvertIqSignal;
        uint8_t SyncWord;
        
    }LoRaSettings_t;
    
    
    typedef struct{
        FSK_ModulationType_t ModulationType;
        FSK_ModulationShaping_FSK_t ModulationShapingFsk;
        FSK_ModulationShaping_OOK_t ModulationShapingOok;
        uint16_t bitrate;
        uint16_t fdev;
        FSK_OpMode_t mode;
    }FskCommonSettings_t;
    
    typedef struct{
        bool RestartRxOnCollision;
        bool AfcAutoOn;
        bool AgcAutoOn;
        FSK_RxTrigger_t RxTrigger;
        uint8_t RssiOffset;
        FSK_RssiSmoothing_t RssiSmoothing;
        uint8_t RssiCollision;
        uint8_t RssiThreshold;
        uint8_t RssiValue;
        FSK_RxBwMant_t RxBwMant;
        uint8_t RxBwExp;
        uint8_t RxBwMantAfc;
        uint8_t RxBwExpAfc;
        bool bitSyncOn;
        FSK_OokThresType_t OokThresType;
        FSK_OokPeakThresStep_t OokPeakTheshStep;
        uint8_t OokFixedThreshold;
        FSK_OokPeakThresDec_t OokPeakThreshDec;
        FSK_OokAverageOffset_t OokAverageOffset;
        FSK_OokAverageThreshFilt_t  OokAverageThreshFilt;
        bool AfcAutoClearOn;
        uint16_t AfcValue;
        uint16_t FeiValue;
        bool PreambleDetectorOn;
        FSK_PreambleDetectorSize_t PreambleDetectorSize;
        uint8_t PreambleDetectroTol;
        uint8_t TimeoutRxRssi; 
        uint8_t TimeoutRxPreamble; 
        uint8_t TimeoutSignalSync; 
        uint8_t InterPacketRxDelay;
    }FskReceiverSettings_t;
    
    typedef struct{
        uint16_t PreambleSize;
        FSK_AutoRestartRxMode_t AutoRestartRxMode;
        FSK_PreamblePolarity_t PreamblePolarity;
        bool SyncOn;
        bool FifoFillCondition;
        uint8_t SyncSize;
        uint64_t SyncVal;
        FSK_PacketFormat_t PacketFormat;
        FSK_DCFree_t DcFree;
        bool CrcOn;
        bool CrcAutoClearOff;
        FSK_AddressFiltering_t AddressFiltering;
        FSK_CrcWhitening_t CrcWhiteningType;
        FSK_DataMode_t DataMode;
        bool IoHomeOn;
        bool IoHomePowerFrame;
        bool BeaconOn;
        uint16_t payloadLen;
        uint8_t nodeAddr;
        uint8_t broadcastAddr;
        FSK_TxStartCondition_t TxStartCond;
        uint8_t fifoThreshold;
    }FskPacketHandlingSettings_t;
    
    typedef struct{
        FSK_IdleMode_t IdleMode;
        FSK_TransitionFromStart_t TransitionFromStart;
        FSK_LowPowerSelection_t LowPowerSelection;
        FSK_TransitionFromIdle_t TransitionFromIdle;
        FSK_TransitionFromTransmit_t TransitionFromTransmit;
        FSK_TransitionFromReceive_t TransitionFromReceive;
        FSK_TransitionFromRxTimeout_t TransitionFromRxTimeout;
        FSK_TransitionFromPacketReceived_t TransitionFromPacketReceived;
        FSK_TimerResolution_t Timer1;
        FSK_TimerResolution_t Timer2;
        uint8_t Timer1Coeff;
        uint8_t Timer2Coeff;
    }FskSequencerSettings_t;
    
    
    typedef struct{
        bool AutoImageCalOn;
        FSK_TempThreshold_t TempThreshold;
        bool TempMonitorOn;
        bool LowBatOn;
        FSK_LowBatTrim_t LowBatTrim;
    }FskServiceSettings_t;
    
    typedef struct{
        FskCommonSettings_t Common;
        FskReceiverSettings_t Receiver;
        FSK_ClkOut_t ClkOut;
        FskPacketHandlingSettings_t PacketHandling;
        FskSequencerSettings_t Sequencer;
        FskServiceSettings_t Service;
    }FskSettings_t;
    
    typedef struct{
        RadioModems_t Modem;
        uint32_t Frequency;
        Settings_Pa_t Pa;
        Settings_Ocp_t Ocp;
        Settings_Lna_t Lna;
        Settings_DioMapping_t DioMapping;
        SX1272_DioMapPreambleDetect_t DioMapPreambleDetect;
        Settings_Agc Agc;
        bool FastHopOn;
        bool TcxoInputOn;
        SX1272_PaDac_t PaDac;
        SX1272_PllBandwidth_t LowNoisePllBandwidth;
        uint8_t BitRateFrac;
        
        LoRaSettings_t LoRaSettings;
        FskSettings_t FskSettings;
    }Settings_t;
    
    
    typedef struct{
        LoRaPort_t Port;
        Settings_t Settings;
    }SX1272_t;
    

    
    void LoRaRxTimeoutHandler(SX1272_t *Radio);
    void LoRaRxDoneHandler(SX1272_t *Radio);
    void LoRaPyloadCrcErrorHandler(SX1272_t *Radio);
    void LoRaValidHeaderHandler(SX1272_t *Radio);
    void LoRaTxDoneHandler(SX1272_t *Radio);
    void LoRaCadDoneHandler(SX1272_t *Radio); 
    void LoRaFhssChangeChannelHandler(SX1272_t *Radio);
    void LoRaCadDetectedHandler(SX1272_t *Radio);
    void LoRaPllLockHandler(SX1272_t *Radio);
    void LoRaModeReadyHandler(SX1272_t *Radio);

    
    void FskModeReadyHandler(SX1272_t *Radio);
    void FskRxReadyHandler(SX1272_t *Radio);
    void FskTxReadyHandler(SX1272_t *Radio);
    void FskPllLockHandler(SX1272_t *Radio);
    void FskRssiHandler(SX1272_t *Radio);
    void FskTimeoutHandler(SX1272_t *Radio);
    void FskPreambleDetectHandler(SX1272_t *Radio);
    void FskSyncAddressMatchHandler(SX1272_t *Radio);
    void FskFifoFullHandler(SX1272_t *Radio);
    void FskFifoEmptyHandler(SX1272_t *Radio);
    void FskFifoLevelHandler(SX1272_t *Radio);
    void FskFifoOverrun(SX1272_t *Radio);
    void FskDataHandler(SX1272_t *Radio);///////CHECK IF SAME as FskFifoOverrun

    void FskPacketSentHandler(SX1272_t *Radio);
    void FskPayloadReadyHandler(SX1272_t *Radio);
    void FskCrcOkHandler(SX1272_t *Radio);
    void FskTempChangeLowBatHandler(SX1272_t *Radio);
    

   
    void dio0IrqHandler(SX1272_t *Radio );
    void dio1IrqHandler(SX1272_t *Radio );
    void dio2IrqHandler(SX1272_t *Radio );
    void dio3IrqHandler(SX1272_t *Radio );
    void dio4IrqHandler(SX1272_t *Radio );
    void dio5IrqHandler(SX1272_t *Radio );
    
    
    
    
    


    void SX1272_Init(SX1272_t *Radio);

    void SetModem(SX1272_t *Radio, RadioModems_t modem);
    uint8_t ReadRegister(SX1272_t *Radio, Reg_t regType, uint8_t addr);
    void WriteRegister(SX1272_t *Radio, Reg_t regType, uint8_t addr, uint8_t data);
    void WriteFifo(SX1272_t *Radio,   uint8_t *buffer, uint8_t size );
    void ReadFifo(SX1272_t *Radio,  uint8_t *buffer, uint8_t size) ;

    void LongRangeMode(SX1272_t *Radio, SX1272_LongRangeMode_t mode);
    void Frf(SX1272_t *Radio, uint32_t freq);
    void PaConfig(SX1272_t *Radio, SX1272_PaSelect_t PAsel, uint8_t outputPower);
    void PaRamp(SX1272_t *Radio, bool LowPnTxPllOff , SX1272_PaRamp_t PaRamp);
    void OCP(SX1272_t *Radio, bool ocpOn, uint8_t trim);
    void LNA(SX1272_t *Radio, SX1272_LnaGain_t gain, SX1272_LnaBoost_t boostOn);
    void DioMapping(SX1272_t *Radio, uint8_t pin, SX1272_DioMapping_t map);
    void DioMapPreambleDetect(SX1272_t *Radio, SX1272_DioMapPreambleDetect_t sel);
    uint8_t Version(SX1272_t *Radio );
    void AgcRefLevel(SX1272_t *Radio, uint8_t level);
    void AgcStepThreshold(SX1272_t *Radio, SX1272_AgcStep_t step, uint8_t threshold);
    void FastHopOn(SX1272_t *Radio, bool sel);
    void Tcx0InputOn(SX1272_t *Radio, bool sel);
    void PaDac(SX1272_t *Radio, SX1272_PaDac_t sel);
    void PllBandwidth(SX1272_t *Radio, SX1272_PllBandwidth_t sel);
    void LowNoisePllBandwidth(SX1272_t *Radio, SX1272_PllBandwidth_t sel);
    uint8_t FormerTemp(SX1272_t *Radio );
    void BitRateFrac(SX1272_t *Radio, uint8_t frac);



    void FskCommonSetting(SX1272_t *Radio, 
            FSK_ModulationType_t ModulationType,
            FSK_ModulationShaping_FSK_t ModulationShapingFsk,
            FSK_ModulationShaping_OOK_t ModulationShapingOok,
            uint16_t bitrate,
            uint16_t fdev);
    
    void FskMode(SX1272_t *Radio, FSK_OpMode_t sel);

    void FskRestartRxOnCollision(SX1272_t *Radio, bool sel);
    void FskRestartRxWithougPllLock(SX1272_t *Radio );
    void FskRestartRxWithPllLock(SX1272_t *Radio );
    void FskAfcAutoOn(SX1272_t *Radio, bool sel);
    void FskAgcAutoOn(SX1272_t *Radio, bool sel);
    void FskRxTrigger(SX1272_t *Radio, FSK_RxTrigger_t rxTrigger);//needs table to be checked (SX1272_t *Radio, Table 23 for description on datasheet)
    void FskRssiConfig(SX1272_t *Radio, 
                    uint8_t rssiOffset,
                    FSK_RssiSmoothing_t rssiSmoothing,
                    uint8_t rssiCollision,
                    uint8_t rssiThres,
                    uint8_t rssiVal);
    void FskRxBw(SX1272_t *Radio, FSK_RxBwMant_t RxBwMant, uint8_t RxBwExp);
    void FskAfcBw(SX1272_t *Radio, uint8_t RxBwMantAfc, uint8_t RxBwExpAfc);
    void FskOok(SX1272_t *Radio, 
                    bool bitSyncOn,
                    FSK_OokThresType_t OokThresType,
                    FSK_OokPeakThresStep_t OokPeakTheshStep,
                    uint8_t OokFixedThreshold,
                    FSK_OokPeakThresDec_t OokPeakThreshDec,
                    FSK_OokAverageOffset_t OokAverageOffset,
                    FSK_OokAverageThreshFilt_t  OokAverageThreshFilt
                    );
    void FskAgcStart(SX1272_t *Radio );
    void FskAfcClear(SX1272_t *Radio );
    void FskAfcAutoClearOn(SX1272_t *Radio, bool sel);
    void FskAfcValue(SX1272_t *Radio, uint16_t val);
    void FskFeiValue(SX1272_t *Radio, uint16_t val);
    void FskPreambleDetector(SX1272_t *Radio, 
                    bool PreambleDetectorOn,
                    FSK_PreambleDetectorSize_t PreambleDetectorSize,
                    uint8_t PreambleDetectorTol
                    );
    void FskTimeout(SX1272_t *Radio, uint8_t TimeoutRxRssi, uint8_t TimeoutRxPreamble, uint8_t TimeoutSignalSync, uint8_t InterPacketRxDelay);

    void FskRcCalStart(SX1272_t *Radio );
    void FskClkOut(SX1272_t *Radio, FSK_ClkOut_t sel);

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
            uint8_t fifoThreshold );


    void FskSequenceStart(SX1272_t *Radio );
    void FskSequenceStop(SX1272_t *Radio );
    void FskSequencerConfig(SX1272_t *Radio, 
                    FSK_IdleMode_t IdleMode,
                    FSK_TransitionFromStart_t FromStart,
                    FSK_LowPowerSelection_t LowPowerSelection,
                    FSK_TransitionFromIdle_t FromIdle,
                    FSK_TransitionFromTransmit_t FromTransmit,
                    FSK_TransitionFromReceive_t FromReceive,
                    FSK_TransitionFromRxTimeout_t FromRxTimeout,
                    FSK_TransitionFromPacketReceived_t FromPacketReceived
                    );
    void FskTimer(SX1272_t *Radio, 
                    FSK_TimerResolution_t Timer1,
                    FSK_TimerResolution_t Timer2,
                    uint8_t Timer1Coeff,
                    uint8_t Timer2Coeff
                    );
    void FskAutoImageCAlOn(SX1272_t *Radio, bool sel);
    void FskImageCalStart(SX1272_t *Radio );
    bool FskImageCalRunning(SX1272_t *Radio );
    bool FskTempChange(SX1272_t *Radio );
    void FskTempThreshold(SX1272_t *Radio, FSK_TempThreshold_t sel);
    void FskTempMonitorOff(SX1272_t *Radio, bool sel);
    uint8_t FskTempValue(SX1272_t *Radio );
    void FskLowBat(SX1272_t *Radio, bool LowBatOn,FSK_LowBatTrim_t LowBatTrim);
    bool FskIrqFlags(SX1272_t *Radio, FSK_IrqFlags_t flag);
    void FskClearIrqFlag(SX1272_t *Radio, FSK_IrqFlags_t flag);



    void LoRaMode(SX1272_t *Radio, LoRa_OpMode_t mode);
    void LoRaWriteFifoAddrPtr(SX1272_t *Radio, uint8_t addr);
    uint8_t LoRaReadFifoAddrPtr(SX1272_t *Radio );
    void LoRaWriteFifoTxBaseAddr(SX1272_t *Radio, uint8_t addr);
    uint8_t LoRaReadFifoTxBaseAddr(SX1272_t *Radio );
    void LoRaWriteFifoRxBaseAddr(SX1272_t *Radio, uint8_t addr);
    uint8_t LoRaReadFifoRxBaseAddr(SX1272_t *Radio);
    uint8_t LoRaReadFifoRxCurrentAddr(SX1272_t *Radio );
    void LoRaIrqFlagsMask(SX1272_t *Radio, uint8_t mask);
    bool LoRaIrqFlags(SX1272_t *Radio, LoRa_IrqFlags_t flag);
    void LoRaClearIrqFlags(SX1272_t *Radio, LoRa_IrqFlags_t flag);
    uint8_t LoRaRxPayloadBytes(SX1272_t *Radio );
    uint16_t LoRaValidHeaderCount(SX1272_t *Radio );
    uint16_t LoRaValidPacketCount(SX1272_t *Radio );
    uint8_t LoRaModemStatus(SX1272_t *Radio, LoRa_ModemStatus_t status);
    uint8_t LoRaPacketSnr(SX1272_t *Radio );
    uint8_t LoRaPacketRssi(SX1272_t *Radio );
    uint8_t LoRaRssi(SX1272_t *Radio );
    bool LoRaPllTimeout(SX1272_t *Radio );
    bool LoRaCrcOnPayload(SX1272_t *Radio );
    uint8_t LoRaFhssPresentChannel(SX1272_t *Radio );
    void LoRaModemConfig(SX1272_t *Radio, 
            LoRa_ModemBw_t bandwidth,
            LoRa_ModemCodingRate_t codingRate,
            bool implicitHeaderModeOn,
            bool RxPayloadCrcOn,
            bool LowDataRateOptimize,
            LoRa_ModemSpreadingFactor_t spreadingfactor,
            bool txContinuousMode,
            bool AgcAutoOn
            );
    void LoRaSymbTimeout(SX1272_t *Radio, uint16_t timeout);
    void LoRaPreambleLength(SX1272_t *Radio, uint16_t len);
    void LoRaPayloadLength(SX1272_t *Radio, uint8_t len);
    void LoRaPayloadMaxLength(SX1272_t *Radio, uint8_t len);
    void LoRaFreqHopPeriod(SX1272_t *Radio, uint8_t period);
    uint8_t LoRaFifoRxByteAddrPtr(SX1272_t *Radio );
    uint32_t LoRaFreqError(SX1272_t *Radio );
    uint8_t LoRaRssiWideband(SX1272_t *Radio );
    void LoRaDetectOptimize(SX1272_t *Radio, LoRa_DetectOptimize_t sel);
    void LoRaInvertIQSignal(SX1272_t *Radio, bool sel);
    void LoRaDetectThreshold(SX1272_t *Radio, LoRa_DetectThreshold_t sel);
    void LoRaSyncWord(SX1272_t *Radio, uint8_t SyncWord);
    
    
    
    
    
    
    
    
    
    
#ifdef __cplusplus
}
#endif

#endif /* SX1272_H */

