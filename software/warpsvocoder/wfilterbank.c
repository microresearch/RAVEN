// porting of warps filter banks

// Copyright 2014 Olivier Gillet.
//
// Author: Olivier Gillet (ol.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Filter bank.

//#include "warps/dsp/filter_bank.h"
//#include <algorithm>
//#include "warps/resources.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
//#include "main.h"
#include "wfilterbank.h"
#include "filtertable.h"


void SVF_Reset(SVF* svf) {
  svf->lp_[0] = svf->bp_[0] = svf->lp_[1] = svf->bp_[1] = 0.0f;
  svf->x_[0] = 0.0f;
  svf->x_[1] = 0.0f;
}
  
void SVF_Init(SVF* svf) {
    SVF_Reset(svf);
  }


static inline void set_f_fq(SVF* svf, float f, float fq) {
    svf->f_ = f;
    svf->fq_ = fq;
  }
  
static inline void SVF_Process(SVF* svf, float* in, float* out, u8 size, u8 mode) {
    float lp_1 = svf->lp_[0];
    float bp_1 = svf->bp_[0];
    float lp_2 = svf->lp_[1];
    float bp_2 = svf->bp_[1];
    float x_1 = svf->x_[0];
    float x_2 = svf->x_[1];
    float fq = svf->fq_;
    float f = svf->f_;

    while (size--) {
      lp_1 += f * bp_1;
      bp_1 += -fq * bp_1 -f * lp_1 + *in;
      if (mode == FILTER_MODE_BAND_PASS ||
          mode == FILTER_MODE_BAND_PASS_NORMALIZED) {
        bp_1 += x_1;
      }
      x_1 = *in++;
      
      float y;
      if (mode == FILTER_MODE_LOW_PASS) {
        y = lp_1 * f;
      } else if (mode == FILTER_MODE_BAND_PASS) {
        y = bp_1 * f;
      } else if (mode == FILTER_MODE_BAND_PASS_NORMALIZED) {
        y = bp_1 * fq;
      } else if (mode == FILTER_MODE_HIGH_PASS) {
        y = x_1 - lp_1 * f - bp_1 * fq;
      }
      
      lp_2 += f * bp_2;
      bp_2 += -fq * bp_2 -f * lp_2 + y;
      if (mode == FILTER_MODE_BAND_PASS ||
          mode == FILTER_MODE_BAND_PASS_NORMALIZED) {
        bp_2 += x_2;
      }
      x_2 = y;
      
      if (mode == FILTER_MODE_LOW_PASS) {
        *out++ = lp_2 * f;
      } else if (mode == FILTER_MODE_BAND_PASS) {
        *out++ = bp_2 * f;
      } else if (mode == FILTER_MODE_BAND_PASS_NORMALIZED) {
        *out++ = bp_2 * fq;
      } else if (mode == FILTER_MODE_HIGH_PASS) {
        *out++ = x_2 - lp_2 * f - bp_2 * fq;
      }
    }
    svf->lp_[0] = lp_1;
    svf->bp_[0] = bp_1;
    svf->lp_[1] = lp_2;
    svf->bp_[1] = bp_2;
    svf->x_[0] = x_1;
    svf->x_[1] = x_2;
  }
  



const int32_t kLowFactor = 4;
const int32_t kMidFactor = 3;
const int32_t kDelayLineSize = 6144*2;
const int32_t kMaxFilterBankBlockSize = 48;
//const int32_t kSampleMemorySize = 480;

extern const u8 kNumBands;

#define max(a, b)	(((a) > (b)) ? (a) : (b))
#define min(a, b)	(((a) > (b)) ? (b) : (a))
   
void Pooled_Init(PooledDelayLine* delayline, float* ptr, int32_t delay) {
delayline->delay_line_ = ptr;
delayline->size_ = delay + 1;
delayline->head_ = 0;
for (int32_t x=0;x<delayline->size_;x++){
ptr[x]=0.0f;
}
    //    std::fill(&ptr[0], &ptr[size_], 0.0f);
}
  
inline int32_t Pooled_size(PooledDelayLine* delayline) { return delayline->size_; }
  
float Pooled_ReadWrite(PooledDelayLine* delayline, float value) {
    delayline->delay_line_[delayline->head_] = value;
    delayline->head_ = (delayline->head_ + 1) % delayline->size_;
    return delayline->delay_line_[delayline->head_];
  }
  
//////////////////////////////////////////////////////////////////

// TODO: from here ON CHECK storage and inits of structs,SRC_UP etc...

/// filterbank defs:

//  const Band& band(int32_t index) {
//    return band_[index];
//  }
  

//const int32_t kLowFactor = 4;
//const int32_t kMidFactor = 3;

//  SampleRateConverter<SRC_DOWN, kMidFactor, 36> mid_src_down_;
//  SampleRateConverter<SRC_UP, kMidFactor, 36> mid_src_up_;
//  SampleRateConverter<SRC_DOWN, kLowFactor, 48> low_src_down_;
//  SampleRateConverter<SRC_UP, kLowFactor, 48> low_src_up_;

// TODO process for each ///

/*

UP:

  inline void Process(const float* in, float* out, u8 input_size) {
    SRC_FIR<SRC_UP, ratio, filter_size> ir;
    FilterState<N> x;
    x.Load(x_);
    while (input_size--) {
      x.Push(*in++);
      PolyphaseStage<K, filter_size> polyphase_stage;
      polyphase_stage(out, x, ir);
    }
    x.Save(x_);
  }

  inline void Process(const float* in, float* out, u8 input_size) {
    // When downsampling, the number of input samples must be a multiple
    // of the downsampling ratio.
    if ((input_size % ratio) != 0) {
      return;
    }

    SRC_FIR<SRC_DOWN, ratio, filter_size> ir;
    if (input_size >= 8 * filter_size) {
      std::copy(&in[0], &in[N], &x_[N - 1]);
      
      // Generate the samples which require access to the history buffer.
      for (int32_t i = 0; i < N; i += ratio) {
        Accumulator<N, -1, 1, filter_size> accumulator;
        *out++ = accumulator(&x_[N - 1 + i], ir);
        in += ratio;
        input_size -= ratio;
      }
        
      // From now on, all the samples we need to access are located inside
      // the input buffer passed as an argument, and since the filter
      // is small, we can unroll the summation loop.
      if ((input_size / ratio) & 1) {
        while (input_size) {
          Accumulator<N, -1, 1, filter_size> accumulator;
          *out++ = accumulator(in, ir);
          input_size -= ratio;
          in += ratio;
        }
      } else {
        while (input_size) {
          Accumulator<N, -1, 1, filter_size> accumulator;
          *out++ = accumulator(in, ir);
          *out++ = accumulator(in + ratio, ir);
          input_size -= 2 * ratio;
          in += 2 * ratio;
        }
      }

      // Copy last input samples to history buffer.
      std::copy(&in[-N + 1], &in[0], &x_[0]);
    } else {
      // Variant which uses a circular buffer to store history.
      while (input_size) {
        for (int32_t i = 0; i < ratio; ++i) {
          x_ptr_[0] = x_ptr_[N] = *in++;
          --x_ptr_;
          if (x_ptr_ < x_) {
            x_ptr_ += N;
          }
        }
        input_size -= ratio;

        Accumulator<N, 1, 1, filter_size> accumulator;
        *out++ = accumulator(&x_ptr_[1], ir);
      }
    }
  }
 
 */

typedef struct SampleRate{
  float* x_ptr_;
  float x_[48];
  int32_t delay;
} SampleRate;

inline int32_t sr_delay(SampleRate* sr) { //return sr->filter_size / sr->ratio / 2; }
 return sr->delay;
}

void low_src_up_Init(SampleRate* sr){ // //  SampleRateConverter<SRC_UP, kLowFactor, 48> low_src_up_;
  
  //  u8 N = filter_size / ratio; , thus N= 48/ kLow=4 == 12
  //    K = ratio
  //    std::fill(&x_[0], &x_[N], 0);
  for (u8 x=0;x<12;x++){
    sr->x_[x]=0;
  }
  sr->delay=6;
}

void low_src_down_Init(SampleRate* sr){ // //  SampleRateConverter<SRC_UP, kLowFactor, 48> low_src_up_;
  //      N = filter_size / ratio, thus N= 48/ kLow=4 == 12
  //    K = ratio
  //    std::fill(&x_[0], &x_[N], 0);
  for (u8 x=0;x<12;x++){
    sr->x_[x]=0;
  }
  sr->delay=6;
}


void mid_src_up_Init(SampleRate* sr){ // //  SampleRateConverter<SRC_UP, kLowFactor, 48> low_src_up_;
  //      N = filter_size / ratio, thus N= 48/ kLow=3 == 16
  //    K = ratio
  //    std::fill(&x_[0], &x_[N], 0);
  for (u8 x=0;x<16;x++){
    sr->x_[x]=0;
  }
  sr->delay=8;
}

void mid_src_down_Init(SampleRate* sr){ // //  SampleRateConverter<SRC_UP, kLowFactor, 48> low_src_up_;
  //      N = filter_size / ratio, thus N= 48/ kLow=3 == 16
  //    K = ratio
  //    std::fill(&x_[0], &x_[N], 0);
  for (u8 x=0;x<16;x++){
    sr->x_[x]=0;
  }
  sr->delay=8;
}

//SampleRate low_src_up_;
//SampleRate low_src_down_;
//SampleRate mid_src_down_;
//SampleRate mid_src_up_;

///////////////////////////////

//float coefficients[]={-0.0282793973573, 0.00959297446756, -0.0163167758833, 0.0230064882424, -0.0393651013703, 0.00641386905172, -0.0334160506508, 0.00544619053794, -0.0574054032856, 0.00934663014085, -0.0487363845568, 0.00793888326107, -0.0779545431939, 0.0126823781741, -0.0661927558613, 0.0107763298717, -0.0942366298986, 0.015321870747, -0.0800287301336, 0.013023431221, -0.114286604369, 0.0185679468016, -0.097072311225, 0.0157895175849, -0.138288264423, 0.0224478486855, -0.117484032941, 0.0190998615243, -0.167253501531, 0.0271218210932, -0.14213102646, 0.023094469639, -0.202167149801, 0.0327441793954, -0.171861964199, 0.027910744163, -0.244389527147, 0.0395277773228, -0.207852602085, 0.0337403624366, -0.29500629759, 0.0476391521105, -0.251056229345, 0.0407420466369, -0.355824872964, 0.0573583693292, -0.303060665772, 0.0491839233527, -0.428878649093, 0.0689993382983, -0.365679453184, 0.0593844544377, -0.55499261433, 0.0890290943739, -0.474238488247, 0.0772216105788, -0.800710156926, 0.127936456871, -0.687889474666, 0.113347474541, -0.883201326334, 0.283602362272, -0.98933441725, 0.859117052889}; // total 64 is 16*4 yes

//extern float coefficients[];

void FilterBank_Init(Filterbank *Filterbankk, float sample_rate) {
  //  low_src_down_Init(&low_src_up_);
  //  low_src_up_Init(&low_src_down_);
  //  mid_src_down_Init(&mid_src_up_);
  //  mid_src_up_Init(&mid_src_down_);
  
  int32_t max_delay = 0;
  float* samples = &(Filterbankk->samples_[0]);
  
  int32_t group = -1;
  int32_t decimation_factor = -1;

  for (int32_t i = 0; i < kNumBands; ++i) {
    float* coefficients = filter_bank_table[i];

    Band* b = &(Filterbankk->band_[i]); 

/*
 b.decimation_factor = (int32_t)(coefficients[0]);
    
    if (b.decimation_factor != decimation_factor) {
      decimation_factor = b.decimation_factor;
      ++group;
    }
*/   
//    b.group = group;
    b->sample_rate = sample_rate;
    b->samples = samples;
    samples += kMaxFilterBankBlockSize;// / b.decimation_factor; +32
  
    //    b.delay = (int32_t)(coefficients[1]);
    //    b.delay *= b.decimation_factor;
        b->post_gain = 2.0f*coefficients[0];
    //b->post_gain=0.2f;
    //    max_delay = max(max_delay, b.delay);
    for (int32_t pass = 0; pass < 2; ++pass) {
      SVF_Init(&(b->svf[pass]));
      set_f_fq(&(b->svf[pass]),coefficients[(pass * 2) + 1],coefficients[(pass * 2) + 2]);
      //  set_f_fq(&b->svf[pass],coefficients[(i*4)+(pass * 2)],coefficients[(i*4)+(pass * 2)+1]);
 
    }
  }
  //  Filterbankk->band_[kNumBands]->group = Filterbankk->band_[kNumBands - 1]->group + 1;

  /*  max_delay = min(max_delay, (int32_t)(256));
  float* delay_ptr = &Filterbankk->delay_buffer_[0];
  for (int32_t i = 0; i < kNumBands; ++i) {
    Band b = Filterbankk->band_[i];
    int32_t compensation = max_delay - b.delay;
    if (b.group == 0) {
      compensation -= kLowFactor * \
	(low_src_down_.delay + low_src_up_.delay);
	      compensation -= mid_src_down_.delay;
	      compensation -= mid_src_up_.delay;
    } else if (b.group == 1) {
            compensation -= mid_src_down_.delay;
            compensation -= mid_src_up_.delay;
    }
    compensation = max(compensation - b.decimation_factor / 2, (int32_t)(0));
    //void Pooled_Init(PooledDelayLine* delayline, float* ptr, int32_t delay) {
    Pooled_Init(&b.delay_line, delay_ptr, compensation / b.decimation_factor);
    delay_ptr += b.delay_line.size_;*/
}


void FilterBank_Analyze(Filterbank *Filterbankk, float* in, u8 size) {
  //  mid_src_down_.Process(in, Filterbankk->tmp_[0], size);
  //  low_src_down_.Process(Filterbankk->tmp_[0], Filterbankk->tmp_[1], size / kMidFactor);
  
  //  const float* sources[3] = {Filterbankk->tmp_[1], Filterbankk->tmp_[0], in };
    //    const float* input = sources[b.group];
    float* input = in;
    u8 band_size = size;// / b.decimation_factor;

  for (int i = 0; i < 16; ++i) {
    Band* b = &(Filterbankk->band_[i]);
    
    for (u8 pass = 0; pass < 2; ++pass) {
      float* source = pass == 0 ? input : b->samples;
      
      float* destination = b->samples;
      if (i == 0) {
	SVF_Process(&(b->svf[pass]),source, destination, band_size, FILTER_MODE_LOW_PASS);
      } else if (i == kNumBands - 1) {
	SVF_Process(&(b->svf[pass]),source, destination, band_size, FILTER_MODE_HIGH_PASS);
      } else {
	SVF_Process(&(b->svf[pass]),source, destination, band_size, FILTER_MODE_BAND_PASS_NORMALIZED);
      }
    }
    // Apply post-gain for each band
    float gain = b->post_gain;
    float* output = b->samples;
    for (u8 ii = 0; ii < band_size; ++ii) {
        output[ii] *= gain;
    }
  }
}

void FilterBank_Synthesize(Filterbank *Filterbankk, float* out, u8 size) {

  for (u8 i = 0; i < kNumBands; ++i) {
    Band* b = &Filterbankk->band_[i];
    
    float* s = out;
    for (u8 j = 0; j < size; ++j) {
                 s[j] += b->samples[j];
      //                                    s[j]=(float)(rand()%32768)/32768.0f;
    }
  }

  /*  
  float* buffers[3] = { Filterbankk->tmp_[1], Filterbankk->tmp_[0], out };

    fill(&buffers[0][0], &buffers[0][size / band_[0].decimation_factor], 0.0f); // TODO
  for (int16_t i=0; i<size / Filterbankk->band_[0].decimation_factor; i++){
    buffers[0][i]=0.0f;
  }

  for (int32_t i = 0; i < kNumBands; ++i) {
    Band b = Filterbankk->band_[i];
    
    u8 band_size = size / b.decimation_factor;
    float* s = buffers[b.group];
    for (u8 j = 0; j < band_size; ++j) {
      s[j] += Pooled_ReadWrite(&b.delay_line, b.samples[j]);
    }
    
    if (Filterbankk->band_[i + 1].group != b.group) {
      if (b.group == 0) {
	//        low_src_up_.Process(Filterbankk->tmp_[1], Filterbankk->tmp_[0], band_size);
      } else if (b.group == 1) {
	//        mid_src_up_.Process(Filterbankk->tmp_[0], out, band_size);
      }
    }
    }*/
}
