#ifndef __WFILT_H
#define __WFILT_H

#include "svf.h"

typedef struct PooledDelayLine{
float* delay_line_;
int32_t size_;
int32_t head_;
} PooledDelayLine;

typedef struct Band {
  //  int32_t group;
  float sample_rate;
  float post_gain;
  SVF svf[2];
  //  int32_t decimation_factor;
  float* samples;
  //  PooledDelayLine delay_line;
  //  int32_t delay;
} Band;


typedef struct Filterbank{  
  //  float tmp_[2][96];
  float samples_[16*AUDIO_BLOCK_SIZE]; // 32samples*16bands=512
  //  float delay_buffer_[6144];
  Band band_[17];
} Filterbank;

// functions

void FilterBank_Init(Filterbank* Filterbankk, float sample_rate);
void FilterBank_Synthesize(Filterbank *Filterbankk, float* out, u8 size);
void FilterBank_Analyze(Filterbank *Filterbankk, float* in, u8 size);
#endif


