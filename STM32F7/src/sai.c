/**
  ******************************************************************************
  * File Name          : SAI.c
  * Description        : This file provides code for the configuration
  *                      of the SAI instances.
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "sai.h"
#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
//SAI_HandleTypeDef hsai_BlockA2;
//SAI_HandleTypeDef hsai_BlockB2;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;
//DMA_HandleTypeDef hdma_sai2_a;
//DMA_HandleTypeDef hdma_sai2_b;

/* SAI1 init function */
void MX_SAI1_Init(void)
{

  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockA1.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2); // slots was 2 = should it be 1?

  hsai_BlockB1.Instance = SAI1_Block_B;
  hsai_BlockB1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockB1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2);
}

static uint32_t SAI1_client =0;
//static uint32_t SAI2_client =0;

void HAL_SAI_MspInit(SAI_HandleTypeDef* hsai)
{

  GPIO_InitTypeDef GPIO_InitStruct;
/* SAI1 */
    if(hsai->Instance==SAI1_Block_A)
    {
    /* Peripheral clock enable */
    if (SAI1_client == 0)
    {
       __HAL_RCC_SAI1_CLK_ENABLE();
    }
    SAI1_client ++;
    
    /**SAI1_A_Block_A GPIO Configuration    
    PD6     ------> SAI1_SD_A 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Peripheral DMA init*/
    
    hdma_sai1_a.Instance = DMA2_Stream1;
    hdma_sai1_a.Init.Channel = DMA_CHANNEL_0;
    hdma_sai1_a.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_sai1_a.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sai1_a.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai1_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai1_a.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_sai1_a.Init.Mode = DMA_CIRCULAR;
    hdma_sai1_a.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_sai1_a.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_sai1_a.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_sai1_a.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_sai1_a.Init.PeriphBurst = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma_sai1_a);

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one stream to perform all the requested DMAs. */
    __HAL_LINKDMA(hsai,hdmarx,hdma_sai1_a);
    __HAL_LINKDMA(hsai,hdmatx,hdma_sai1_a);
    }
    if(hsai->Instance==SAI1_Block_B)
    {
      /* Peripheral clock enable */
      if (SAI1_client == 0)
      {
       __HAL_RCC_SAI1_CLK_ENABLE();
      }
    SAI1_client ++;
    
    /**SAI1_B_Block_B GPIO Configuration    
    PF6     ------> SAI1_SD_B
    PF7     ------> SAI1_MCLK_B
    PF8     ------> SAI1_SCK_B
    PF9     ------> SAI1_FS_B 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* Peripheral DMA init*/
    
    hdma_sai1_b.Instance = DMA2_Stream0;
    hdma_sai1_b.Init.Channel = DMA_CHANNEL_10;
    hdma_sai1_b.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_sai1_b.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sai1_b.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai1_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai1_b.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_sai1_b.Init.Mode = DMA_CIRCULAR;
    hdma_sai1_b.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_sai1_b.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_sai1_b.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_sai1_b.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_sai1_b.Init.PeriphBurst = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma_sai1_b);

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one stream to perform all the requested DMAs. */
    __HAL_LINKDMA(hsai,hdmarx,hdma_sai1_b);
    __HAL_LINKDMA(hsai,hdmatx,hdma_sai1_b);
    }
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef* hsai)
{

/* SAI1 */
    if(hsai->Instance==SAI1_Block_A)
    {
    SAI1_client --;
    if (SAI1_client == 0)
      {
      /* Peripheral clock disable */ 
       __HAL_RCC_SAI1_CLK_DISABLE();
      }
    
    /**SAI1_A_Block_A GPIO Configuration    
    PD6     ------> SAI1_SD_A 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_6);

    HAL_DMA_DeInit(hsai->hdmarx);
    HAL_DMA_DeInit(hsai->hdmatx);
    }
    if(hsai->Instance==SAI1_Block_B)
    {
    SAI1_client --;
      if (SAI1_client == 0)
      {
      /* Peripheral clock disable */
      __HAL_RCC_SAI1_CLK_DISABLE();
      }
    
    /**SAI1_B_Block_B GPIO Configuration    
    PF6     ------> SAI1_SD_B
    PF7     ------> SAI1_MCLK_B
    PF8     ------> SAI1_SCK_B
    PF9     ------> SAI1_FS_B 
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);

    HAL_DMA_DeInit(hsai->hdmarx);
    HAL_DMA_DeInit(hsai->hdmatx);
    }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
