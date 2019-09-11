// license:GPL-2.0+
// copyright-holders: Martin Howse

/*
 PATH=~/sat/bin:$PATH
 PATH=~/stm32f4/stlink/flash:$PATH
 make stlink_flash
*/

#ifdef TESTING
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include "stm32f4xx.h"
#include "codec.h"
#include "i2s.h"
#include "adc.h"
#include "audio.h"
#include "wavetable.h"
#include "wavetables.h"
#else
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include "stm32f4xx.h"
#include "codec.h"
#include "i2s.h"
#include "adc.h"
#include "audio.h"
#include "wvocoder.h"
#include "wavetable.h"
#include "wavetables.h"
#endif

void LPCAnalysisinit(int _windowsize);
void LPCAnalyzer_init4();


/* DMA buffers for I2S */
__IO int16_t tx_buffer[BUFF_LEN], rx_buffer[BUFF_LEN];

/* DMA buffer for ADC  & copy */
__IO uint16_t adc_buffer[5];

extern int errno;

extern Wavetable wavtable;

extern int16_t audio_buffer[AUDIO_BUFSZ] __attribute__ ((section (".ccmdata")));

void main(void)
{
  int16_t x;

  wavetable_init(&wavtable, plaguetable_simplesir, 328); // now last arg as length of table=less than 512 

  // fill audio buffer

  for (u16 xx=0;xx<AUDIO_BUFSZ;xx++) {
    audio_buffer[xx]=0;
  }
  
  Vocoder_Init(32000);
  //  samplerate_init();

 ////////
  ADC1_Init((uint16_t *)adc_buffer);
  Codec_Init(32000); 
  I2S_Block_Init();
  I2S_Block_PlayRec((uint32_t)&tx_buffer, (uint32_t)&rx_buffer, BUFF_LEN);
  //  Audio_Init(); not needed
  LPCAnalysisinit(32); // windowsize?
  LPCAnalyzer_init4();
  
  while(1)
    {
    }
}

#ifdef  USE_FULL_ASSERT

#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))

void assert_failed(uint8_t* file, uint32_t line)
{ 
  while (1)
    {
    }
}
#endif

#if 1
void NMI_Handler(void)
{ 
  while(1){};
}

void HardFault_Handler(void)
{ 
  while(1){};
  }


void MemManage_Handler(void)
{ 
  while(1){};
}

void BusFault_Handler(void)
{ 
  while(1){};
}

void UsageFault_Handler(void)
{ 
  while(1){};
}

void SVC_Handler(void)
{ 
  while(1){};
}

void DebugMon_Handler(void)
{ 
  while(1){};
}

void PendSV_Handler(void)
{ 
  while(1){};
}
#endif
