/*  -*-  mode: c; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4; coding: utf-8  -*-  */
/************************************************************************************
**                                                                                 **
**                               mcHF QRP Transceiver                              **
**                             K Atanassov - M0NKA 2014                            **
**                                                                                 **
**---------------------------------------------------------------------------------**
**                                                                                 **
**  File name:                                                                     **
**  Description:                                                                   **
**  Last Modified:                                                                 **
**  Licence:		GNU GPLv3                                                      **
************************************************************************************/


#include "sai.h"
#include "main.h"
#include <string.h>
#include "wavetable.h"
#include "process.h"

#define __UHSDR_DMAMEM

/*
typedef struct {
    __packed audio_data_t l;
    __packed audio_data_t r;
} AudioSample_t;
*/

typedef struct
{
    AudioSample_t out[2*AUDIO_BLOCK_SIZE]; // 2 x 32 samples =64 left and 64 right
    AudioSample_t in[2*AUDIO_BLOCK_SIZE];
} dma_audio_buffer_t;

static __UHSDR_DMAMEM dma_audio_buffer_t dma;

extern Wavetable wavtable;

static inline void floot_to_int(int16_t* outbuffer, float* inbuffer,u16 howmany){
  int16_t tmp;

  for (int n = 0; n < howmany; n++) {
    tmp = inbuffer[n] * 32768.0f;
    tmp = (tmp <= -32768) ? -32768 : (tmp >= 32767) ? 32767 : tmp;
    outbuffer[n] = (int16_t)tmp;
		}
}

static inline void int_to_floot(int16_t* inbuffer, float* outbuffer, u16 howmany){
  for (int n = 0; n < howmany; n++) {
    outbuffer[n]=(float)(inbuffer[n])/32768.0f;
  }
}

/* void UhsdrHwI2s_Codec_ClearTxDmaBuffer() */
/* { */
/*     memset((void*)&dma.iq_buf.out, 0, sizeof(dma.iq_buf.out)); */
/* } */

// #define PROFILE_APP
static void MchfHw_Codec_HandleBlock(uint16_t which)
{
	// with callback to audio processing handle to check out

	// Transfer complete interrupt
    // Point to 2nd half of buffers
	const size_t sz = IQ_BLOCK_SIZE;
    const uint16_t offset = which == 0?sz:0;
	uint16_t x;
	float fbuffer[32],flinbuffer[32],frinbuffer[32];
	int16_t pbuf[32], linbuf[32],rinbuf[32];
	
    AudioSample_t *audio;

	audio = &dma.in[offset];

    AudioSample_t *audioDst = &dma.out[offset];

    // Handle
	//    AudioDriver_I2SCallback(audio, iq, audioDst, sz);
	// void AudioDriver_I2SCallback(AudioSample_t *audio, IqSample_t *iq, AudioSample_t *audioDst, int16_t blockSize)
	// in audio_driver.c - test first with samples out or/???
	// question of stereo -> l and r (int16_t) so is audio->l, audio->r? for in and audioDst->l for out
	// 32 samples
	//int_to_floot

	// split left and right incoming
	for (x=0;x<sz;x++){
		linbuf[x]=audio[x].l;
		rinbuf[x]=audio[x].r;
	}
	int_to_floot(linbuf,flinbuffer,sz);
	int_to_floot(rinbuf,frinbuffer,sz);
	process(flinbuffer,frinbuffer,fbuffer,sz); // in process.c
	//	dowavetable(fbuffer, &wavtable, 440.0f, sz); 
	floot_to_int(pbuf,fbuffer,sz);
	//	audio_comb_stereo(sz, audioDst, pbuf, pbuf);
	for (x=0;x<sz;x++){
		audioDst[x].l=pbuf[x];
		audioDst[x].r=pbuf[x]; // both l and r for the moment
	}
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hi2s)
{
	// these are the callbacks which call into handleblock and another handler...
	// but in the original they only handle A2 but this is maybe to sync it!
    MchfHw_Codec_HandleBlock(0);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hi2s)
{
    MchfHw_Codec_HandleBlock(1);
}

static void UhsdrHWI2s_Sai32Bits(SAI_HandleTypeDef* hsai)
{
    hsai->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hsai->hdmarx->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    HAL_DMA_Init(hsai->hdmarx);

    HAL_SAI_InitProtocol(hsai, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_32BIT, 2);
}

static void UhsdrHwI2s_SetBitWidth()
{
	//    UhsdrHWI2s_Sai32Bits(&hsai_BlockA2);
	//    UhsdrHWI2s_Sai32Bits(&hsai_BlockB2);
}



void UhsdrHwI2s_Codec_StartDMA()
{
    UhsdrHwI2s_SetBitWidth();

    // we clean the buffers since we don't know if we are in a "cleaned" memory segement
    memset((void*)&dma,0,sizeof(dma));
	//    memset((void*)&dma.iq_buf,0,sizeof(dma.iq_buf));

    HAL_SAI_Receive_DMA(&hsai_BlockA1,(uint8_t*)dma.in,sizeof(dma.in)/sizeof(dma.in[0].l));
    HAL_SAI_Transmit_DMA(&hsai_BlockB1,(uint8_t*)dma.out,sizeof(dma.out)/sizeof(dma.out[0].l));
}


void UhsdrHwI2s_Codec_StopDMA(void)
{
    HAL_SAI_DMAStop(&hsai_BlockA1);
    HAL_SAI_DMAStop(&hsai_BlockB1);
}
