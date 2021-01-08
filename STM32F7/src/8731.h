#ifndef __CODEC_H
#define __CODEC_H

void     Codec_SwitchTxRxMode(uint8_t txrx_mode);
void     Codec_MCUInterfaceInit(uint32_t AudioFreq);
void 	 Codec_VolumeLineOut(uint8_t txrx_mode);
void     Codec_VolumeSpkr(uint8_t vol);
void 	 Codec_LineInGainAdj(uint8_t gain);
void     Codec_IQInGainAdj(uint8_t gain);
void 	 Codec_MuteDAC(bool state);

uint32_t Codec_Reset(uint32_t AudioFreq);
void     Codec_RestartI2S(void);
void     Codec_TxSidetoneSetgain(uint8_t mode);
void     Codec_SwitchMicTxRxMode(uint8_t mode);
void     Codec_PrepareTx(uint8_t current_txrx_mode);

bool     Codec_ReadyForIrqCall(void);
#endif
