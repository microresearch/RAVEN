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
#include <main.h>
#include <string.h>

#define __UHSDR_DMAMEM

typedef struct
{
    AudioSample_t out[2*AUDIO_BLOCK_SIZE];
    AudioSample_t in[2*AUDIO_BLOCK_SIZE];
} dma_audio_buffer_t;

static __UHSDR_DMAMEM dma_audio_buffer_t dma;


/* void UhsdrHwI2s_Codec_ClearTxDmaBuffer() */
/* { */
/*     memset((void*)&dma.iq_buf.out, 0, sizeof(dma.iq_buf.out)); */
/* } */

// #define PROFILE_APP
static void MchfHw_Codec_HandleBlock(uint16_t which)
{
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hi2s)
{
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hi2s)
{
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
