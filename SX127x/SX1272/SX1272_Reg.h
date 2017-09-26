/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SX1272_Reg.h
 * Author: emon
 *
 * Created on September 26, 2017, 8:03 PM
 */

#ifndef SX1272_REG_H
#define SX1272_REG_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{

	REG_COMMON_FIFO          					=0x00,
	// Common settings
	REG_COMMON_OPMODE        					=0x01,
	REG_FSK_BITRATEMSB         					=0x02,
	REG_FSK_BITRATELSB     						=0x03,
	REG_FSK_FDEVMSB      						=0x04,
	REG_FSK_FDEVLSB       						=0x05,

	REG_COMMON_FRFMSB     						=0x06,
	REG_COMMON_FRFMID          					=0x07,
	REG_COMMON_FRFLSB                                               =0x08,
	// Tx settings
	REG_COMMON_PACONFIG                                             =0x09,
	REG_COMMON_PARAMP                                               =0x0A,
	REG_COMMON_OCP                                                  =0x0B,
	// Rx settings,
	REG_COMMON_LNA                              =0x0C,
	REG_FSK_RXCONFIG                            =0x0D,
	REG_FSK_RSSICONFIG                          =0x0E,
	REG_FSK_RSSICOLLISION                       =0x0F,
	REG_FSK_RSSITHRESH                          =0x10,
	REG_FSK_RSSIVALUE                           =0x11,
	REG_FSK_RXBW                                =0x12,
	REG_FSK_AFCBW                               =0x13,
	REG_FSK_OOKPEAK                             =0x14,
	REG_FSK_OOKFIX                              =0x15,
	REG_FSK_OOKAVG                              =0x16,
	REG_FSK_RES17                               =0x17,
	REG_FSK_RES18                               =0x18,
	REG_FSK_RES19                               =0x19,
	REG_FSK_AFCFEI                              =0x1A,
	REG_FSK_AFCMSB                              =0x1B,
	REG_FSK_AFCLSB                              =0x1C,
	REG_FSK_FEIMSB                              =0x1D,
	REG_FSK_FEILSB                              =0x1E,
	REG_FSK_PREAMBLEDETECT                      =0x1F,
	REG_FSK_RXTIMEOUT1                          =0x20,
	REG_FSK_RXTIMEOUT2                          =0x21,
	REG_FSK_RXTIMEOUT3                          =0x22,
	REG_FSK_RXDELAY                             =0x23,
	// Oscillator settings
	REG_FSK_OSC                                 =0x24,
	// Packet handler settings
	REG_FSK_PREAMBLEMSB                         =0x25,
	REG_FSK_PREAMBLELSB                         =0x26,
	REG_FSK_SYNCCONFIG                          =0x27,
	REG_FSK_SYNCVALUE1                          =0x28,
	REG_FSK_SYNCVALUE2                          =0x29,
	REG_FSK_SYNCVALUE3                          =0x2A,
	REG_FSK_SYNCVALUE4                          =0x2B,
	REG_FSK_SYNCVALUE5                          =0x2C,
	REG_FSK_SYNCVALUE6                          =0x2D,
	REG_FSK_SYNCVALUE7                          =0x2E,
	REG_FSK_SYNCVALUE8                          =0x2F,
	REG_FSK_PACKETCONFIG1                       =0x30,
	REG_FSK_PACKETCONFIG2                       =0x31,
	REG_FSK_PAYLOADLENGTH                       =0x32,
	REG_FSK_NODEADRS                            =0x33,
	REG_FSK_BROADCASTADRS                       =0x34,
	REG_FSK_FIFOTHRESH                          =0x35,
	// SM settings
	REG_FSK_SEQCONFIG1                          =0x36,
	REG_FSK_SEQCONFIG2                          =0x37,
	REG_FSK_TIMERRESOL                          =0x38,
	REG_FSK_TIMER1COEF                          =0x39,
	REG_FSK_TIMER2COEF                          =0x3A,
	// Service settings
	REG_FSK_IMAGECAL                            =0x3B,
	REG_FSK_TEMP                                =0x3C,
	REG_FSK_LOWBAT                              =0x3D,
	// Status
	REG_FSK_IRQFLAGS1                           =0x3E,
	REG_FSK_IRQFLAGS2                           =0x3F,

	// LoRa registers
	REG_LORA_FIFOADDRPTR                        =0x0D,
	REG_LORA_FIFOTXBASEADDR                     =0x0E,
	REG_LORA_FIFORXBASEADDR                     =0x0F,
	REG_LORA_FIFORXCURRENTADDR                  =0x10,
	REG_LORA_IRQFLAGSMASK                       =0x11,
	REG_LORA_IRQFLAGS                           =0x12,
	REG_LORA_RXNBBYTES                          =0x13,
	REG_LORA_RXHEADERCNTVALUEMSB                =0x14,
	REG_LORA_RXHEADERCNTVALUELSB                =0x15,
	REG_LORA_RXPACKETCNTVALUEMSB                =0x16,
	REG_LORA_RXPACKETCNTVALUELSB                =0x17,
	REG_LORA_MODEMSTAT                          =0x18,
	REG_LORA_PKTSNRVALUE                        =0x19,
	REG_LORA_PKTRSSIVALUE                       =0x1A,
	REG_LORA_RSSIVALUE                          =0x1B,
	REG_LORA_HOPCHANNEL                         =0x1C,
	REG_LORA_MODEMCONFIG1                       =0x1D,
	REG_LORA_MODEMCONFIG2                       =0x1E,
	REG_LORA_SYMBTIMEOUTLSB                     =0x1F,
	REG_LORA_PREAMBLEMSB                        =0x20,
	REG_LORA_PREAMBLELSB                        =0x21,
	REG_LORA_PAYLOADLENGTH                      =0x22,
	REG_LORA_PAYLOADMAXLENGTH                   =0x23,
	REG_LORA_HOPPERIOD                          =0x24,
	REG_LORA_FIFORXBYTEADDR                     =0x25,
	REG_LORA_FEIMSB                             =0x28,
	REG_LORA_FEIMID                             =0x29,
	REG_LORA_FEILSB                             =0x2A,
	REG_LORA_RSSIWIDEBAND                       =0x2C,
	REG_LORA_DETECTOPTIMIZE                     =0x31,
	REG_LORA_INVERTIQ                           =0x33,
	REG_LORA_DETECTIONTHRESHOLD                 =0x37,
	REG_LORA_SYNCWORD                           =0x39,
	REG_LORA_INVERTIQ2                          =0x3B,

	// I/O settings
	REG_COMMON_DIOMAPPING1                      =0x40,
	REG_COMMON_DIOMAPPING2                      =0x41,
	// Version
	REG_COMMON_VERSION                          =0x42,
	// Additional settings
	REG_COMMON_AGCREF                           =0x43,
	REG_COMMON_AGCTHRESH1                       =0x44,
	REG_COMMON_AGCTHRESH2                       =0x45,
	REG_COMMON_AGCTHRESH3                       =0x46,
	REG_COMMON_PLLHOP                           =0x4B,
	REG_COMMON_TCXO                             =0x58,
	REG_COMMON_PADAC                            =0x5A,
	REG_COMMON_PLL                              =0x5C,
	REG_COMMON_PLLLOWPN                         =0x5E,
	REG_COMMON_FORMERTEMP                       =0x6C,
	REG_COMMON_BITRATEFRAC                      =0x70,
}SX1272_REG_t;


#define SX1272_OpMode_LongRangeMode_bp		7
#define SX1272_OpMode_LongRangeMode_bm		(1<<SX1272_OpMode_LongRangeMode_bp)

#define SX1272_PaConfig_PaSelect_bp			7
#define SX1272_PaConfig_PaSelect_bm			(1<<SX1272_PaConfig_PaSelect_bp)

#define SX1272_PaConfig_OutputPower_gp		0
#define SX1272_PaConfig_OutputPower_gm		(0x0F <<SX1272_PaConfig_OutputPower_gp)

#define SX1272_PaRamp_LowPnTxPllOff_bp		4
#define SX1272_PaRamp_LowPnTxPllOff_bm		(1 <<SX1272_PaRamp_LowPnTxPllOff_bp)

#define SX1272_PaRamp_PaRamp_gp				0
#define SX1272_PaRamp_PaRamp_gm				(0x0F <<SX1272_PaRamp_PaRamp_gp)

#define SX1272_Ocp_OcpOn_bp					5
#define SX1272_Ocp_OcpOn_bm					(1 <<SX1272_Ocp_OcpOn_bp)

#define SX1272_Ocp_OcpTrim_gp				0
#define SX1272_Ocp_OcpTrim_gm				(0x1F <<SX1272_Ocp_OcpTrim_gp)


#define SX1272_Lna_LnaGain_gp				7
#define SX1272_Lna_LnaGain_gm				(0x03 <<SX1272_Lna_LnaGain_gp)

#define SX1272_Lna_LnaBoost_gp				0
#define SX1272_Lna_LnaBoost_gm				(0x03 <<SX1272_Lna_LnaBoost_gp)

#define SX1272_Dio0Mapping_gp				6
#define SX1272_Dio0Mapping_gm				(0x03 <<SX1272_Dio0Mapping_gp)
#define SX1272_Dio1Mapping_gp				4
#define SX1272_Dio1Mapping_gm				(0x03 <<SX1272_Dio1Mapping_gp)
#define SX1272_Dio2Mapping_gp				2
#define SX1272_Dio2Mapping_gm				(0x03 <<SX1272_Dio2Mapping_gp)
#define SX1272_Dio3Mapping_gp				0
#define SX1272_Dio3Mapping_gm				(0x03 <<SX1272_Dio3Mapping_gp)
#define SX1272_Dio4Mapping_gp				6
#define SX1272_Dio4Mapping_gm				(0x03 <<SX1272_Dio4Mapping_gp)
#define SX1272_Dio5Mapping_gp				4
#define SX1272_Dio5Mapping_gm				(0x03 <<SX1272_Dio5Mapping_gp)

#define SX1272_DioMapPreambleDetect_bp		0
#define SX1272_DioMapPreambleDetect_bm		(1 <<SX1272_DioMapPreambleDetect_bp)

#define SX1272_ArgRefLevel_gp				0
#define SX1272_ArgRefLevel_gm				(0x1F <<SX1272_ArgRefLevel_gp)


#define SX1272_ArgStep1Thresl_gp			0
#define SX1272_ArgStep1Thresl_gm			(0x1F <<SX1272_ArgStep1Thresl_gp)
#define SX1272_ArgStep1Thres2_gp			4
#define SX1272_ArgStep1Thres2_gm			(0x0F <<SX1272_ArgStep1Thres2_gp)
#define SX1272_ArgStep1Thres3_gp			0
#define SX1272_ArgStep1Thres3_gm			(0x0F <<SX1272_ArgStep1Thres3_gp)
#define SX1272_ArgStep1Thres4_gp			4
#define SX1272_ArgStep1Thres4_gm			(0x0F <<SX1272_ArgStep1Thres4_gp)
#define SX1272_ArgStep1Thres5_gp			0
#define SX1272_ArgStep1Thres5_gm			(0x0F <<SX1272_ArgStep1Thres5_gp)


#define SX1272_PllHop_FastHopOn_bp			7
#define SX1272_PllHop_FastHopOn_bm			(1 <<SX1272_PllHop_FastHopOn_bp)


#define SX1272_Txco_FastHopOn_bp			4
#define SX1272_Txco_FastHopOn_bm			(1 <<SX1272_Txco_FastHopOn_bp)


#define SX1272_PaDac_gp						0
#define SX1272_PaDac_gm						(0x03 <<SX1272_PaDac_gp)


#define SX1272_PllBandwidth_gp				6
#define SX1272_PllBandwidth_gm				(0x03 <<SX1272_PllBandwidth_gp)


#define SX1272_PllBandwidthLowPhaseNoise_gp			6
#define SX1272_PllBandwidthLowPhaseNoise_gm			(0x03 <<SX1272_PllBandwidthLowPhaseNoise_gp)


#define SX1272_BitrateFrac_gp				0
#define SX1272_BitrateFrac_gm				(0x0F <<SX1272_BitrateFrac_gp)



#define LoRa_OpMode_AccessSharedReg_bp		6
#define LoRa_OpMode_AccessSharedReg_bm		(1<<LoRa_OpMode_AccessSharedReg_bp)

#define LoRa_OpMode_mode_gp					0
#define LoRa_OpMode_mode_gm					(0x07<<LoRa_OpMode_mode_gp)

#define LoRa_ModemStat_RxCodingRate_gp			5
#define LoRa_ModemStat_RxCodingRate_gm			(0x07<<LoRa_ModemStat_RxCodingRate_gp)

#define LoRa_ModemStat_ModemClear_bp			4
#define LoRa_ModemStat_ModemClear_bm			(1<<LoRa_ModemStat_ModemClear_bp)

#define LoRa_ModemStat_HeaderInfoValid_bp		3
#define LoRa_ModemStat_HeaderInfoValid_bm		(1<<LoRa_ModemStat_HeaderInfoValid_bp)

#define LoRa_ModemStat_RxOnGoing_bp				2
#define LoRa_ModemStat_RxOnGoing_bm				(1<<LoRa_ModemStat_RxOnGoing_bp)

#define LoRa_ModemStat_SignalSynchronized_bp	1
#define LoRa_ModemStat_SignalSynchronized_bm	(1<<LoRa_ModemStat_SignalSynchronized_bp)

#define LoRa_ModemStat_SignalDetected_bp		0
#define LoRa_ModemStat_SignalDetected_bm		(1<<LoRa_ModemStat_SignalDetected_bp)

#define LoRa_HopChannel_PllTimeout_bp					7
#define LoRa_HopChannel_PllTimeout_bm					(1<<LoRa_HopChannel_PllTimeout_bp)

#define LoRa_HopChannel_CrcOnPayload_bp					6
#define LoRa_HopChannel_CrcOnPayload_bm					(1<<LoRa_HopChannel_CrcOnPayload_bp)

#define LoRa_HopChannel_FhssPresentChannel_gp			0
#define LoRa_HopChannel_FhssPresentChannel_gm			(0x1F<<LoRa_HopChannel_FhssPresentChannel_gp)


#define LoRa_ModemConfig1_Bw_gp							6
#define LoRa_ModemConfig1_Bw_gm							(0x03<<LoRa_ModemConfig1_Bw_gp)

#define LoRa_ModemConfig1_CodingRate_gp					3
#define LoRa_ModemConfig1_CodingRate_gp_gm				(0x07<<LoRa_ModemConfig1_CodingRate_gp_gp)

#define LoRa_ModemConfig1_ImplicitHeaderModeOn_bp		2
#define LoRa_ModemConfig1_ImplicitHeaderModeOn_bm		(1<<LoRa_ModemConfig1_ImplicitHeaderModeOn_bp)

#define LoRa_ModemConfig1_RxPayloadCrcOn_bp				1
#define LoRa_ModemConfig1_RxPayloadCrcOn_bm				(1<<LoRa_ModemConfig1_RxPayloadCrcOn_bp)

#define LoRa_ModemConfig1_LowDataRateOptimize_bp		0
#define LoRa_ModemConfig1_LowDataRateOptimize_bm		(1<<LoRa_ModemConfig1_LowDataRateOptimize_bp)

#define LoRa_ModemConfig2_SpreadingFactor_gp			4
#define LoRa_ModemConfig2_SpreadingFactor_gm			(0x0F<<LoRa_ModemConfig2_SpreadingFactor_gp)

#define LoRa_ModemConfig2_TxContMode_bp					3
#define LoRa_ModemConfig2_TxContMode_bm					(1<<LoRa_ModemConfig2_TxContMode_bp)

#define LoRa_ModemConfig2_ArcAutoOn_bp					2
#define LoRa_ModemConfig2_ArcAutoOn_bm					(1<<LoRa_ModemConfig2_ArcAutoOn_bp)

#define LoRa_ModemConfig2_SymbTimeoutMSB_gp				0
#define LoRa_ModemConfig2_SymbTimeoutMSB_gm				(0x03<<LoRa_ModemConfig2_SymbTimeoutMSB_gp)

#define LoRa_InvertIQ_bp				6
#define LoRa_InvertIQ_bm				(1<<LoRa_InvertIQ_bp)




#define FSK_OpMode_ModulationType_gp					5
#define FSK_OpMode_ModulationType_gm					(0x03<<FSK_OpMode_ModulationType_gp)

#define FSK_OpMode_ModulationShaping_gp					3
#define FSK_OpMode_ModulationShaping_gm					(0x03<<FSK_OpMode_ModulationShaping_gp)

#define FSK_OpMode_Mode_gp								0
#define FSK_OpMode_Mode_gm								(0x07<<FSK_OpMode_Mode_gp)

#define FSK_RegRxConfig_RestartRxOnCollision_bp			7
#define FSK_RegRxConfig_RestartRxOnCollision_bm			(1<<FSK_RegRxConfig_RestartRxOnCollision_bp)

#define FSK_RegRxConfig_RestartRxWithoutPllLock_bp		6
#define FSK_RegRxConfig_RestartRxWithoutPllLock_bm		(1<<FSK_RegRxConfig_RestartRxWithoutPllLock_bp)

#define FSK_RegRxConfig_RestartRxWithPllLock_bp			5
#define FSK_RegRxConfig_RestartRxWithPllLock_bm			(1<<FSK_RegRxConfig_RestartRxWithPllLock_bp)

#define FSK_RegRxConfig_AfcAutoOn_bp					4
#define FSK_RegRxConfig_AfcAutoOn_bm					(1<<FSK_RegRxConfig_AfcAutoOn_bp)

#define FSK_RegRxConfig_AgcAutoOn_bp					3
#define FSK_RegRxConfig_AgcAutoOn_bm					(1<<FSK_RegRxConfig_AgcAutoOn_bp)

#define FSK_RegRxConfig_RxTrigger_gp					0
#define FSK_RegRxConfig_RxTrigger_gm					(0x07<<FSK_RegRxConfig_RxTrigger_gp)


#define FSK_RegRssiConfig_RssiOffset_gp					3
#define FSK_RegRssiConfig_RssiOffset_gm					(0x1F<<FSK_RegRssiConfig_RssiOffset_gp)

#define FSK_RegRssiConfig_RssiSmoothing_gp				0
#define FSK_RegRssiConfig_RssiSmoothing_gm				(0x07<<FSK_RegRssiConfig_RssiSmoothing_gp)


#define FSK_RegRxBw_RxBwMant_gp							3
#define FSK_RegRxBw_RxBwMant_gm							(0x03<<FSK_RegRxBw_RxBwMant_gp)

#define FSK_RegRxBw_RxBwExp_gp							0
#define FSK_RegRxBw_RxBwExp_gm							(0x07<<FSK_RegRxBw_RxBwExp_gp)


#define FSK_RegAfcBw_RxBwMantAfc_gp						3
#define FSK_RegAfcBw_RxBwMantAfc_gm						(0x03<<FSK_RegAfcBw_RxBwMantAfc_gp)

#define FSK_RegAfcBw_RxBwExpAfc_gp						0
#define FSK_RegAfcBw_RxBwExpAfc_gm						(0x07<<FSK_RegAfcBw_RxBwExpAfc_gp)


#define FSK_RegOokPeak_BitSyncOn_bp						5
#define FSK_RegOokPeak_BitSyncOn_bm						(1<<FSK_RegOokPeak_BitSyncOn_bp)

#define FSK_RegOokPeak_OokThreshType_gp					3
#define FSK_RegOokPeak_OokThreshType_gm					(0x04<<FSK_RegOokPeak_OokThreshType_gp)

#define FSK_RegOokPeak_OokPeakTheshStep_gp				0
#define FSK_RegOokPeak_OokPeakTheshStep_gm				(0x07<<FSK_RegOokPeak_OokPeakTheshStep_gp)


#define FSK_RegOokAvg_OokPeakThreshDec_gp				5
#define FSK_RegOokAvg_OokPeakThreshDec_gm				(0x07<<FSK_RegOokAvg_OokPeakThreshDec_gp)

#define FSK_RegOokAvg_OokAverageOffset_gp				2
#define FSK_RegOokAvg_OokAverageOffset_gm				(0x03<<FSK_RegOokAvg_OokAverageOffset_gp)

#define FSK_RegOokAvg_OokAverageThreshFilt_gp			0
#define FSK_RegOokAvg_OokAverageThreshFilt_gm			(0x03<<FSK_RegOokAvg_OokAverageThreshFilt_gp)


#define FSK_RegAfcFei_AgcStart_bp						4
#define FSK_RegAfcFei_AgcStart_bm						(1<<FSK_RegAfcFei_AgcStart_bp)

#define FSK_RegAfcFei_AfcClear_bp						1
#define FSK_RegAfcFei_AfcClear_bm						(1<<FSK_RegAfcFei_AfcClear_bp)

#define FSK_RegAfcFei_AfcAutoClearOn_bp					0
#define FSK_RegAfcFei_AfcAutoClearOn_bm					(1<<FSK_RegAfcFei_AfcAutoClearOn_bp)


#define FSK_RegPreambleDetect_PreambleDetectorOn_bp		7
#define FSK_RegPreambleDetect_PreambleDetectorOn_bm		(1<<FSK_RegPreambleDetect_PreambleDetectorOn_bp)

#define FSK_RegPreambleDetect_PreambleDetectorSize_gp	5
#define FSK_RegPreambleDetect_PreambleDetectorSize_gm	(0x03<<FSK_RegPreambleDetect_PreambleDetectorSize_gp)

#define FSK_RegPreambleDetect_PreambleDetectorTol_gp	0
#define FSK_RegPreambleDetect_PreambleDetectorTol_gm	(0x1F<<FSK_RegPreambleDetect_PreambleDetectorTol_gp)


#define FSK_RegOsc_RcCalStart_bp						3
#define FSK_RegOsc_RcCalStart_bm						(1<<FSK_RegOsc_RcCalStart_bp)

#define FSK_RegOsc_ClkOut_gp							0
#define FSK_RegOsc_ClkOut_gm							(0x07<<FSK_RegOsc_ClkOut_gp)


#define FSK_RegSyncConfig_AutoRestartRxMode_gp			6
#define FSK_RegSyncConfig_AutoRestartRxMode_gm			(0x03<<FSK_RegSyncConfig_AutoRestartRxMode_gp)

#define FSK_RegSyncConfig_PreamblePolarity_bp			5
#define FSK_RegSyncConfig_PreamblePolarity_bm			(1<<FSK_RegSyncConfig_PreamblePolarity_bp)

#define FSK_RegSyncConfig_SyncOn_bp						4
#define FSK_RegSyncConfig_SyncOn_bm						(1<<FSK_RegSyncConfig_SyncOn_bp)

#define FSK_RegSyncConfig_FifoFillCondition_bp			3
#define FSK_RegSyncConfig_FifoFillCondition_bm			(1<<FSK_RegSyncConfig_FifoFillCondition_bp)

#define FSK_RegSyncConfig_SyncSize_gp					0
#define FSK_RegSyncConfig_SyncSize_gm					(0x07<<FSK_RegSyncConfig_SyncSize_gp)



#define FSK_RegPacketConfig1_PacketFormat_bp			7
#define FSK_RegPacketConfig1_PacketFormat_bm			(1<<FSK_RegSyncConfig_FifoFillCondition_bp)

#define FSK_RegPacketConfig1_DcFree_gp					5
#define FSK_RegPacketConfig1_DcFree_gm					(0x03<<FSK_RegPacketConfig1_DcFree_gp)

#define FSK_RegPacketConfig1_CrcOn_bp					4
#define FSK_RegPacketConfig1_CrcOn_bm					(1<<FSK_RegPacketConfig1_CrcOn_bp)


#define FSK_RegPacketConfig1_CrcAutoClearOff_bp			4
#define FSK_RegPacketConfig1_CrcAutoClearOff_bm			(1<<FSK_RegPacketConfig1_CrcAutoClearOff_bp)

#define FSK_RegPacketConfig1_AddressFiltering_gp		1
#define FSK_RegPacketConfig1_AddressFiltering_gm		(0x03<<FSK_RegPacketConfig1_AddressFiltering_gp)


#define FSK_RegPacketConfig1_CrcWhiteningType_bp		0
#define FSK_RegPacketConfig1_CrcWhiteningType_bm		(1<<FSK_RegPacketConfig1_CrcWhiteningType_bp)

#define FSK_RegPacketConfig2_DataMode_bp				6
#define FSK_RegPacketConfig2_DataMode_bm				(1<<FSK_RegPacketConfig2_DataMode_bp)

#define FSK_RegPacketConfig2_IoHomeOn_bp				5
#define FSK_RegPacketConfig2_IoHomeOn_bm				(1<<FSK_RegPacketConfig2_IoHomeOn_bp)

#define FSK_RegPacketConfig2_IoHomePowerFrame_bp		4
#define FSK_RegPacketConfig2_IoHomePowerFrame_bm		(1<<FSK_RegPacketConfig2_IoHomePowerFrame_bp)

#define FSK_RegPacketConfig2_BeaconOn_bp				3
#define FSK_RegPacketConfig2_BeaconOn_bm				(1<<FSK_RegPacketConfig2_BeaconOn_bp)

#define FSK_RegFifoThresh_TxStartCondition_bp			7
#define FSK_RegFifoThresh_TxStartCondition_bm			(1<<FSK_RegFifoThresh_TxStartCondition_bp)

#define FSK_RegFifoThresh_FifoThreshold_gp				0
#define FSK_RegFifoThresh_FifoThreshold_gm				(0x3F<<FSK_RegFifoThresh_FifoThreshold_gp)



#define FSK_RegSeqConfig1_SequencerStart_bp			7
#define FSK_RegSeqConfig1_SequencerStart_bm			(1<<FSK_RegSeqConfig1_SequencerStart_bp)

#define FSK_RegSeqConfig1_SequencerStop_bp			6
#define FSK_RegSeqConfig1_SequencerStop_bm			(1<<FSK_RegSeqConfig1_SequencerStop_bp)

#define FSK_RegSeqConfig1_IdleMode_bp				5
#define FSK_RegSeqConfig1_IdleMode_bm				(1<<FSK_RegSeqConfig1_IdleMode_bp)

#define FSK_RegSeqConfig1_FromStart_gp				3
#define FSK_RegSeqConfig1_FromStart_gm				(0x03<<FSK_RegSeqConfig1_FromStart_gp)

#define FSK_RegSeqConfig1_LowPowerSelection_bp		2
#define FSK_RegSeqConfig1_LowPowerSelection_bm		(1<<FSK_RegSeqConfig1_LowPowerSelection_bp)

#define FSK_RegSeqConfig1_FromIdle_bp				1
#define FSK_RegSeqConfig1_FromIdle_bm				(1<<FSK_RegSeqConfig1_FromIdle_bp)

#define FSK_RegSeqConfig1_FromTransmit_bp			0
#define FSK_RegSeqConfig1_FromTransmit_bm			(1<<FSK_RegSeqConfig1_FromTransmit_bp)


#define FSK_RegSeqConfig2_FromReceive_gp				5
#define FSK_RegSeqConfig2_FromReceive_gm				(0x07<<FSK_RegSeqConfig1_FromStart_gp)

#define FSK_RegSeqConfig2_FromRxTimeout_gp				3
#define FSK_RegSeqConfig2_FromRxTimeout_gm				(0x03<<FSK_RegSeqConfig2_FromRxTimeout_gp)

#define FSK_RegSeqConfig2_FromPacketReceived_gp			0
#define FSK_RegSeqConfig2_FromPacketReceived_gm			(0x07<<FSK_RegSeqConfig2_FromPacketReceived_gp)


#define FSK_RegTimerResol_Timer1Resolution_gp			2
#define FSK_RegTimerResol_Timer1Resolution_gm			(0x03<<FSK_RegTimerResol_Timer1Resolution_gp)

#define FSK_RegTimerResol_Timer2Resolution_gp			0
#define FSK_RegTimerResol_Timer2Resolution_gm			(0x03<<FSK_RegTimerResol_Timer2Resolution_gp)


#define FSK_RegImageCal_AutoImageCalOn_bp			7
#define FSK_RegImageCal_AutoImageCalOn_bm			(1<<FSK_RegImageCal_AutoImageCalOn_bp)

#define FSK_RegImageCal_ImageCalStart_bp			6
#define FSK_RegImageCal_ImageCalStart_bm			(1<<FSK_RegImageCal_ImageCalStart_bp)

#define FSK_RegImageCal_ImageCalRunning_bp			5
#define FSK_RegImageCal_ImageCalRunning_bm			(1<<FSK_RegImageCal_ImageCalRunning_bp)

#define FSK_RegImageCal_TempChange_bp				3
#define FSK_RegImageCal_TempChange_bm				(1<<FSK_RegImageCal_TempChange_bp)

#define FSK_RegImageCal_TempThreshold_gp				1
#define FSK_RegImageCal_TempThreshold_gm				(0x03<<FSK_RegImageCal_TempThreshold_gp)

#define FSK_RegImageCal_TempMonitorOff_bp				0
#define FSK_RegImageCal_TempMonitorOff_bm				(1<<FSK_RegImageCal_TempMonitorOff_bp)


#define FSK_RegLowBat_LowBatOn_bp				3
#define FSK_RegLowBat_LowBatOn_bm				(1<<FSK_RegLowBat_LowBatOn_bp)

#define FSK_RegLowBat_LowBatTrim_gp				0
#define FSK_RegLowBat_LowBatTrim_gm				(0x07<<FSK_RegLowBat_LowBatTrim_gp)


#define FSK_RegIrqFlags1_ModeReady_bp				7
#define FSK_RegIrqFlags1_ModeReady_bm				(1<<FSK_RegIrqFlags1_ModeReady_bp)

#define FSK_RegIrqFlags1_RxReady_bp					6
#define FSK_RegIrqFlags1_RxReady_bm					(1<<FSK_RegIrqFlags1_RxReady_bp)

#define FSK_RegIrqFlags1_TxReady_bp					5
#define FSK_RegIrqFlags1_TxReady_bm					(1<<FSK_RegIrqFlags1_TxReady_bp)

#define FSK_RegIrqFlags1_PllLock_bp					4
#define FSK_RegIrqFlags1_PllLock_bm					(1<<FSK_RegIrqFlags1_PllLock_bp)

#define FSK_RegIrqFlags1_Rssi_bp					3
#define FSK_RegIrqFlags1_Rssi_bm					(1<<FSK_RegIrqFlags1_Rssi_bp)

#define FSK_RegIrqFlags1_Timeout_bp					2
#define FSK_RegIrqFlags1_Timeout_bm					(1<<FSK_RegIrqFlags1_Timeout_bp)

#define FSK_RegIrqFlags1_PreambleDetect_bp			1
#define FSK_RegIrqFlags1_PreambleDetect_bm			(1<<FSK_RegIrqFlags1_PreambleDetect_bp)

#define FSK_RegIrqFlags1_SyncAddressMatch_bp		0
#define FSK_RegIrqFlags1_SyncAddressMatch_bm		(1<<FSK_RegIrqFlags1_SyncAddressMatch_bp)





#define FSK_RegIrqFlags2_FifoFull_bp					7
#define FSK_RegIrqFlags2_FifoFull_bm					(1<<FSK_RegIrqFlags2_FifoFull_bp)

#define FSK_RegIrqFlags2_FifoEmpty_bp					6
#define FSK_RegIrqFlags2_FifoEmpty_bm					(1<<FSK_RegIrqFlags2_FifoEmpty_bp)

#define FSK_RegIrqFlags2_FifoLevel_bp					5
#define FSK_RegIrqFlags2_FifoLevel_bm					(1<<FSK_RegIrqFlags2_FifoLevel_bp)

#define FSK_RegIrqFlags2_FifoOverrun_bp					4
#define FSK_RegIrqFlags2_FifoOverrun_bm					(1<<FSK_RegIrqFlags2_FifoOverrun_bp)

#define FSK_RegIrqFlags2_PacketSent_bp					3
#define FSK_RegIrqFlags2_PacketSent_bm					(1<<FSK_RegIrqFlags2_PacketSent_bp)

#define FSK_RegIrqFlags2_PayloadReady_bp				2
#define FSK_RegIrqFlags2_PayloadReady_bm				(1<<FSK_RegIrqFlags2_PayloadReady_bp)

#define FSK_RegIrqFlags2_CrcOk_bp					1
#define FSK_RegIrqFlags2_CrcOk_bm					(1<<FSK_RegIrqFlags2_CrcOk_bp)

#define FSK_RegIrqFlags2_LowBat_bp					0
#define FSK_RegIrqFlags2_LowBat_bm					(1<<FSK_RegIrqFlags2_LowBat_bp)


typedef enum{
	OpMode_SLEEP                   = 0x00,
	OpMode_STDBY                   = 0x01,
	OpMode_FSTX                    = 0x02,
	OpMode_TX                      = 0x03,
	OpMode_FSRX                    = 0x04,
	OpMode_RX                      = 0x05,
        OpMode_RX_Single               = 0x06,
        OpMode_CAD                     = 0x07,
}OpMode_t;


typedef enum{
	FSK_RxTriggerNONE					= 0,
	FSK_RxTrigger_RssiInterrupt				= 1,
	FSK_RxTrigger_PreambleDetect				= 6,
	FSK_RxTrigger_RssiInterruptAndPreambleDetect		= 7,

}FSK_RxTrigger_t;


typedef enum{
	SX1272_LongRangeMode_FSK  = 0x00,
	SX1272_LongRangeMode_LoRa = 0x80,
}SX1272_LongRangeMode_t;


typedef enum{
	SX1272_PaSelect_RFIO_pin      = 0,
	SX1272_PaSelect_PA_BOOST_pin  = 1
}SX1272_PaSelect_t;


typedef enum{
	SX1272_PaRamp_3400us,
	SX1272_PaRamp_2000us,
	SX1272_PaRamp_1000us,
	SX1272_PaRamp_500us,
	SX1272_PaRamp_250us,
	SX1272_PaRamp_125us,
	SX1272_PaRamp_100us,
	SX1272_PaRamp_62us,
	SX1272_PaRamp_50us,
	SX1272_PaRamp_40us,
	SX1272_PaRamp_31us,
	SX1272_PaRamp_25us,
	SX1272_PaRamp_20us,
	SX1272_PaRamp_15us,
	SX1272_PaRamp_12us,
	SX1272_PaRamp_10us,
}SX1272_PaRamp_t;



typedef enum{
	SX1272_LnaGain_G1 = (1<<5),
	SX1272_LnaGain_G2 = (2<<5),
	SX1272_LnaGain_G3 = (3<<5),
	SX1272_LnaGain_G4 = (4<<5),
	SX1272_LnaGain_G5 = (5<<5),
	SX1272_LnaGain_G6 = (6<<5),
}SX1272_LnaGain_t;

typedef enum{
	SX1272_DioMapping_0,
	SX1272_DioMapping_1,
	SX1272_DioMapping_2,
	SX1272_DioMapping_3
}SX1272_DioMapping_t;


typedef enum{
	SX1272_DioMapPreambleDetect_RSSI = 0,
	SX1272_DioMapPreambleDetect_PreambleDetect = 1,
}SX1272_DioMapPreambleDetect_t;


typedef enum{
	SX1272_LnaBoost_Default = 0b00,
	SX1272_LnaBoost_BoostOn = 0b11,
}SX1272_LnaBoost_t;

typedef enum{
	SX1272_AgcStep1 = 1,
	SX1272_AgcStep2 = 2,
	SX1272_AgcStep3 = 3,
	SX1272_AgcStep4 = 4,
	SX1272_AgcStep5 = 5,
}SX1272_AgcStep_t;

typedef enum{
	SX1272_PaDac_Default = 0x04,
	SX1272_PaDac_plus20dBm = 0x07
}SX1272_PaDac_t;


typedef enum{
	SX1272_PllBandwidth_75kHz,
	SX1272_PllBandwidth_150kHz,
	SX1272_PllBandwidth_225kHz,
	SX1272_PllBandwidth_300kHz
}SX1272_PllBandwidth_t;



typedef enum{
	FSK_ModulationType_FSK ,
	FSK_ModulationType_OOK
}FSK_ModulationType_t;



typedef enum{
	FSK_ModulationShaping_FSK_NoShaping,
	FSK_ModulationShaping_FSK_BT_1_0,
	FSK_ModulationShaping_FSK_BT_0_5,
	FSK_ModulationShaping_FSK_BT_0_3
}FSK_ModulationShaping_FSK_t;


typedef enum{
	FSK_ModulationShaping_OOK_NoShaping,
	FSK_ModulationShaping_OOK_CutAtBitRate,
	FSK_ModulationShaping_OOK_CutAt2BitRate,
}FSK_ModulationShaping_OOK_t;


typedef enum{
	FSK_OpMode_Sleep 	= 0x00,
	FSK_OpMode_Stdby 	= 0x01,
	FSK_OpMode_FSTx 	= 0x02,
	FSK_OpMode_Tx 		= 0x03,
	FSK_OpMode_FSRx 	= 0x04,
	FSK_OpMode_Rx 		= 0x05,
}FSK_OpMode_t;



typedef enum{
	FSK_RssiSmoothing_2samples,
	FSK_RssiSmoothing_4samples,
	FSK_RssiSmoothing_8samples,
	FSK_RssiSmoothing_16samples,
	FSK_RssiSmoothing_32samples,
	FSK_RssiSmoothing_64samples,
	FSK_RssiSmoothing_128samples,
	FSK_RssiSmoothing_256samples,
}FSK_RssiSmoothing_t;

typedef enum{
	FSK_RxBwMant_16,
	FSK_RxBwMant_20,
	FSK_RxBwMant_24,
}FSK_RxBwMant_t;


typedef enum{
	FSK_OokThresType_fixed,
	FSK_OokThresType_peak,
	FSK_OokThresType_average,
}FSK_OokThresType_t;

typedef enum{
	FSK_OokPeakThresStep_0_5dB,
	FSK_OokPeakThresStep_1_0dB,
	FSK_OokPeakThresStep_1_5dB,
	FSK_OokPeakThresStep_2_0dB,
	FSK_OokPeakThresStep_3_0dB,
	FSK_OokPeakThresStep_4_0dB,
	FSK_OokPeakThresStep_5_0dB,
	FSK_OokPeakThresStep_6_0dB,
}FSK_OokPeakThresStep_t;


typedef enum{
	FSK_OokPeakThresDec_oncePerChip,
	FSK_OokPeakThresDec_onceEvery2Chips,
	FSK_OokPeakThresDec_onceEvery4Chips,
	FSK_OokPeakThresDec_onceEvery8Chips,
	FSK_OokPeakThresDec_twiceInEachChip,
	FSK_OokPeakThresDec_4timesInEachChip,
	FSK_OokPeakThresDec_8timesInEachChip,
	FSK_OokPeakThresDec_16timesInEachChip,
}FSK_OokPeakThresDec_t;


typedef enum{
	FSK_OokAverageOffset_0dB,
	FSK_OokAverageOffset_2dB,
	FSK_OokAverageOffset_4B,
	FSK_OokAverageOffset_6dB,
}FSK_OokAverageOffset_t;


typedef enum{
	FSK_OokAverageThreshFilt_chipRate_Div_32pi,
	FSK_OokAverageThreshFilt_chipRate_Div_8pi,
	FSK_OokAverageThreshFilt_chipRate_Div_4pi,
	FSK_OokAverageThreshFilt_chipRate_Div_2pi,
}FSK_OokAverageThreshFilt_t;


typedef enum{
	FSK_PreambleDetectorSize_1byte,
	FSK_PreambleDetectorSize_2byte,
	FSK_PreambleDetectorSize_3byte,
}FSK_PreambleDetectorSize_t;

typedef enum{
	FSK_ClkOut_FXOSC_Div_1,
	FSK_ClkOut_FXOSC_Div_2,
	FSK_ClkOut_FXOSC_Div_4,
	FSK_ClkOut_FXOSC_Div_8,
	FSK_ClkOut_FXOSC_Div_16,
	FSK_ClkOut_FXOSC_Div_32,
	FSK_ClkOut_RC,
	FSK_ClkOut_OFF,
}FSK_ClkOut_t;

typedef enum{
	FSK_AutoRestartRxMode_Off,
	FSK_AutoRestartRxMode_On,
	FSK_AutoRestartRxMode_On_waitForPLLLock
}FSK_AutoRestartRxMode_t;


typedef enum{
	FSK_PreamblePolarity_0xAA,
	FSK_PreamblePolarity_0x55
}FSK_PreamblePolarity_t;

typedef enum{
	FSK_PacketFormat_FixedLength,
	FSK_PacketFormat_VariableLength
}FSK_PacketFormat_t;


typedef enum{
	FSK_DCFree_None,
	FSK_DCFree_Manchester,
	FSK_DCFree_Whitening,
}FSK_DCFree_t;


typedef enum{
	FSK_AddressFiltering_None,
	FSK_AddressFiltering_Matches_NodeAddress,
	FSK_AddressFiltering_Matches_NodeAddressOrBroadCastAddress
}FSK_AddressFiltering_t;

typedef enum{
	FSK_CrcWhitening_CCITT,
	FSK_CrcWhitening_IBM,
}FSK_CrcWhitening_t;

typedef enum{
	FSK_DataMode_Continuous,
	FSK_DataMode_Packet
}FSK_DataMode_t;


typedef enum{
	FSK_TxStartCondition_FifoLevel,
	FSK_TxStartCondition_FifoNotEmpty,
}FSK_TxStartCondition_t;


typedef enum{
	FSKIdleMode_Standby,
	FSKIdleMode_Sleep,
}FSK_IdleMode_t;


typedef enum{
	FSKTransitionFromStart_toLowPowerSelection,
	FSKTransitionFromStart_toReceiveState,
	FSKTransitionFromStart_toTransmitState,
	FSKTransitionFromStart_toTransmitStateOnFifoLevelInterrupt,
}FSK_TransitionFromStart_t;


typedef enum{
	FSKLowPowerSelection_SEL1,
        FSKLowPowerSelection_SEL2,
}FSK_LowPowerSelection_t;

typedef enum{
	FSKTransitionFromIdle_toTransmitState,
	FSKTransitionFromIdle_toReceiveState,
}FSK_TransitionFromIdle_t;

typedef enum{
	FSK_TransitionFromTransmit_toLowPowerSelectionOnPacketSentInterrupt,
	FSK_TransitionFromTransmit_toReceiveStateOnPacketSentInterrupt,
}FSK_TransitionFromTransmit_t;

typedef enum{
	FSK_TransitionFromReceive_toPacketReceivedState,
	FSK_TransitionFromReceive_toLowPowerSelection,
	FSK_TransitionFromReceive_toPacketReceived,
	FSK_TransitionFromReceive_toSequencrOffonRssiInterrupt,
	FSK_TransitionFromReceive_toSequencrOffonSyncAddrInterrupt,
	FSK_TransitionFromReceive_toSequencrOffonPreambleDetectInterrupt,
}FSK_TransitionFromReceive_t;


typedef enum{
	FSK_TransitionFromRxTimeout_toReceiveState,
	FSK_TransitionFromRxTimeout_toTransmitState,
	FSK_TransitionFromRxTimeout_LowPowerSelection,
	FSK_TransitionFromRxTimeout_toSequencerOffState,
}FSK_TransitionFromRxTimeout_t;

typedef enum{
	FSK_TransitionFromPacketReceived_toSeuencerOff,
	FSK_TransitionFromPacketReceived_toTransmitStateOnFifoEmptyInterupt,
	FSK_TransitionFromPacketReceived_toLowPowerSelection,
	FSK_TransitionFromPacketReceived_toReceiveState
}FSK_TransitionFromPacketReceived_t;


typedef enum{
	FSK_TimerResolution_disabled,
	FSK_TimerResolution_64us,
	FSK_TimerResolution_4100us,
	FSK_TimerResolution_262000us
}FSK_TimerResolution_t;


typedef enum{
	FSK_TempThreshold_5degreeCelcius,
	FSK_TempThreshold_10degreeCelcius,
	FSK_TempThreshold_15degreeCelcius,
	FSK_TempThreshold_20degreeCelcius,
}FSK_TempThreshold_t;


typedef enum{
	FSK_LowBatTrim_1695mV,
	FSK_LowBatTrim_1764mV,
	FSK_LowBatTrim_1835mV,
	FSK_LowBatTrim_1905mV,
	FSK_LowBatTrim_1976mV,
	FSK_LowBatTrim_2045mV,
	FSK_LowBatTrim_2166mV,
	FSK_LowBatTrim_2185mV,
}FSK_LowBatTrim_t;


typedef enum{
	FSK_IrqFlags_SyncAddressMatch = 0,
	FSK_IrqFlags_PreambleDetect,
	FSK_IrqFlags_Timeout,
	FSK_IrqFlags_Rssi,
	FSK_IrqFlags_PllLock,
	FSK_IrqFlags_TxReady,
	FSK_IrqFlags_RxReady,
	FSK_IrqFlags_ModeReady,
	FSK_IrqFlags_LowBat,
	FSK_IrqFlags_CrcOk,
	FSK_IrqFlags_PayloadReady,
	FSK_IrqFlags_PacketSent,
	FSK_IrqFlags_FIfoOverrun,
	FSK_IrqFlags_FifoLevel,
	FSK_IrqFlags_FifoEmpty,
	FSK_IrqFlags_FIfoFull,
}FSK_IrqFlags_t;



typedef enum{
	LoRa_OpMode_SLEEP                   = (1<<7) | (0<<6) | OpMode_SLEEP,
	LoRa_OpMode_STDBY                   = (1<<7) | (0<<6) | OpMode_STDBY,
        LoRa_OpMode_STDBY_FskRegAccess      = (1<<7) | (1<<6) | OpMode_STDBY,
	LoRa_OpMode_FSTX                    = (1<<7) | (0<<6) | OpMode_FSTX,
	LoRa_OpMode_TX                      = (1<<7) | (0<<6) | OpMode_TX,
	LoRa_OpMode_FSRX                    = (1<<7) | (0<<6) | OpMode_FSRX,
	LoRa_OpMode_RXContinuous            = (1<<7) | (0<<6) | OpMode_RX,
	LoRa_OpMode_RXSingle                = (1<<7) | (0<<6) | (6<<0),
	LoRa_OpMode_CAD                     = (1<<7) | (0<<6) | (7<<0),
}LoRa_OpMode_t;





typedef enum{
	LoRa_IrqFlags_RxTimeout 		= (1<<7),
	LoRa_IrqFlags_RxDone 			= (1<<6),
	LoRa_IrqFlags_PldCrcError 		= (1<<5),
	LoRa_IrqFlags_ValidHeader 		= (1<<4),
	LoRa_IrqFlags_TxDone 			= (1<<3),
	LoRa_IrqFlags_CadDone 			= (1<<2),
	LoRa_IrqFlags_FhssChangeChannel         = (1<<1),
	LoRa_IrqFlags_CadDetected 		= (1<<0),
        LoRa_IrqFlags_All                       = 0xFF,
}LoRa_IrqFlags_t;


typedef enum{
	LoRa_ModemStatus_RxCodingRate,
	LoRa_ModemStatus_ModemClear,
	LoRa_ModemStatus_HeaderInfoValid,
	LoRa_ModemStatus_RxOnGoing ,
	LoRa_ModemStatus_SignalSynchronized,
	LoRa_ModemStatus_SignalDetected,
}LoRa_ModemStatus_t;



typedef enum{
	LoRa_ModemBw_125kHz = (0<<6),
	LoRa_ModemBw_250kHz = (1<<6),
	LoRa_ModemBw_500kHz = (2<<6),
}LoRa_ModemBw_t;

typedef enum{
	LoRa_ModemCodingRate_4Div5 = 1<<3,
	LoRa_ModemCodingRate_4Div6 = 2<<3,
	LoRa_ModemCodingRate_4Div7 = 3<<3,
	LoRa_ModemCodingRate_4Div8 = 4<<3
}LoRa_ModemCodingRate_t;


typedef enum{
	LoRa_ModemSpreadingFactor_6    	= (6<<4),
	LoRa_ModemSpreadingFactor_7    	= (7<<4),
	LoRa_ModemSpreadingFactor_8    	= (8<<4),
	LoRa_ModemSpreadingFactor_9    	= (9<<4),
	LoRa_ModemSpreadingFactor_10    = (10<<4),
	LoRa_ModemSpreadingFactor_11    = (11<<4),
	LoRa_ModemSpreadingFactor_12    = (12<<4),
}LoRa_ModemSpreadingFactor_t;

typedef enum{
	DetectOptimize_SF7_SF12 = 0x03,
	DetectOptimize_SF6 = 0x05,
}LoRa_DetectOptimize_t;

typedef enum{
	DetectThreshold_SF7_SF12 = 0x0A,
	DetectThreshold_SF6 = 0x0C,
}LoRa_DetectThreshold_t;


#ifdef __cplusplus
}
#endif

#endif /* SX1272_REG_H */

