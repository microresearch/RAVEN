#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

const float crowtable[142] ={-0.283905, -0.055756, 0.206726, 0.495911, 0.636169, 0.630768, 0.330597, 0.458313, 0.337769, 0.321716, 0.154480, 0.151001, 0.140137, 0.057526, 0.132965, 0.294678, -0.039459, 0.053833, 0.190521, 0.194092, 0.362946, 0.368439, 0.407898, 0.434875, 0.309113, 0.305450, 0.217468, 0.098816, 0.233612, 0.127594, 0.140167, 0.386353, 0.420502, 0.355804, 0.382782, 0.267731, 0.079102, 0.194031, 0.197723, 0.165314, 0.167114, -0.055725, 0.073730, 0.129303, 0.195984, 0.100525, 0.095306, -0.145569, -0.215668, -0.043091, 0.001770, -0.073669, -0.307281, -0.402496, -0.672150, -0.557007, -0.722443, -0.857147, -0.867981, -0.882294, -0.880554, -0.803284, -0.724121, -0.632690, -0.438293, -0.330811, -0.224487, -0.201385, -0.258698, -0.249817, -0.246155, -0.226471, -0.422241, -0.217529, -0.096924, -0.093536, 0.079132, 0.141907, 0.165344, 0.019806, 0.159882, 0.079102, 0.267761, 0.170654, 0.203156, -0.097137, 0.070160, 0.273102, 0.061127, 0.265930, 0.328857, 0.546295, 0.864349, 0.828430, 0.654144, 0.569550, 0.469147, 0.510223, 0.413422, 0.567810, 0.539062, 0.776398, 0.600067, 0.573395, 0.468903, 0.316345, 0.098785, 0.077271, 0.077332, -0.050385, -0.312592, -0.479919, -0.334137, -0.016266, 0.131287, -0.143829, -0.366577, -0.637909, -0.513977, -0.497772, -0.368317, -0.334381, -0.499390, -0.469208, -0.303558, -0.323547, -0.023285, -0.160004, 0.079132, 0.053802, -0.044830, -0.172546, -0.161713, -0.123993, -0.346863, -0.244354, -0.447479, -0.587585, -0.460052, -0.598450, -0.321564, -0.025269};

typedef uint16_t u16;
typedef unsigned char u8;


typedef struct _Wavetable {
    const float *wavetable;
    float basicIncrement;
    float currentPosition;
  int16_t length;
} Wavetable;

void wavetable_init(Wavetable* wavtable, const float *tableitself, int16_t length); // need to declare wavetable struct and and ourtable we use
//void dowavetable(float* outgoing, Wavetable *wavetable, float frequency, u8 length);

Wavetable wavtable;
//Wavetable *wavetable;



#define TWO_PI 6.28318530717958647693
#define OVERSAMPLING_OSCILLATOR 0

inline int iincrement(int pointer, int modulus)
{
    if (++pointer >= modulus)
	return 0;

    return pointer;
}

inline int ddecrement(int pointer, int modulus)
{
    if (--pointer < 0)
return modulus - 1;

    return pointer;
}

inline static float mod0(float value, int16_t length)
{
    while (value > length-1)
        value -= length;
    return value;
}

// Increments the position in the wavetable according to the desired frequency. WORMY THIS
inline static void WavetableIncrementPosition(Wavetable *wavetable, float frequency)
{
wavetable->currentPosition = mod0(wavetable->currentPosition + (frequency * wavetable->basicIncrement), wavetable->length);
}

void dowavetable(float* outgoing, Wavetable *wavetable, float frequency, u8 length)  //  Plain oscillator
{
    int lowerPosition, upperPosition;
    for (u8 ii = 0; ii < length; ii++) {

    //  First increment the table position, depending on frequency
    WavetableIncrementPosition(wavetable, frequency);

    //  Find surrounding integer table positions
    lowerPosition = (int)wavetable->currentPosition;
    upperPosition = mod0(lowerPosition + 1, wavetable->length);

    //  Return interpolated table value
    float sample= (wavetable->wavetable[lowerPosition] +
            ((wavetable->currentPosition - lowerPosition) *
             (wavetable->wavetable[upperPosition] - wavetable->wavetable[lowerPosition])));

    outgoing[ii]=sample;
    }
}

void wavetable_init(Wavetable* wavtable, const float *tableitself, int16_t length){ // need to declare wavetable struct and ourtable we use
  wavtable->wavetable=tableitself;
  wavtable->basicIncrement=(float)length/32000.0f; // 32000 for real thing - test lap is 8000
  wavtable->currentPosition=0.0;
  wavtable->length=length;
}



#define AUDIO_SAMPLE_RATE (32000)

// a lot of code pieces assume that this frequency
// is 1500 Hz, so don't change
#define IQ_INTERRUPT_FREQ (1000) // what should this be? 32 samples? what did we have in WORM? 32 samples for mono buffer

#define IQ_BLOCK_SIZE (AUDIO_SAMPLE_RATE/IQ_INTERRUPT_FREQ)

// we process one dma block of samples at once
// block sizes should be a power of two
// a lot of code process information in these blocks
//#define IQ_BLOCK_SIZE (IQ_SAMPLE_RATE/IQ_INTERRUPT_FREQ)
#define AUDIO_BLOCK_SIZE (AUDIO_SAMPLE_RATE/IQ_INTERRUPT_FREQ)

// use for clocking based on DMA IRQ
#define SAMPLES_PER_DMA_CYCLE   (IQ_BLOCK_SIZE)
#define SAMPLES_PER_CENTISECOND (IQ_SAMPLE_RATE/100)


u16 kNumBands=16;

// for EMS2000 frequencies

const float fb__140_32000[]={1.0, -0.02827939735734698, 0.0095929744675562478, -0.016316775883298176, 0.023006488242404699};
const float fb__185_32000[]={0.25, -0.039365101370256728, 0.0064138690517188124, -0.033416050650801672, 0.0054461905379422637};
const float fb__270_32000[]={0.25, -0.057405403285554102, 0.0093466301408474672, -0.048736384556789927, 0.0079388832610738369};
const float fb__367_32000[]={0.25, -0.07795454319389078, 0.012682378174050157, -0.066192755861286437, 0.01077632987173216};
const float fb__444_32000[]={0.25, -0.094236629898622026, 0.015321870747048494, -0.080028730133604134, 0.013023431221036774};
const float fb__539_32000[]={0.25, -0.11428660436946862, 0.018567946801589463, -0.097072311224966887, 0.015789517584910095};
const float fb__653_32000[]={0.25, -0.1382882644230701, 0.022447848685485217, -0.11748403294060142, 0.019099861524316974};
const float fb__791_32000[]={0.25, -0.16725350153137555, 0.027121821093174181, -0.14213102645951653, 0.023094469638971549};
const float fb__958_32000[]={0.25, -0.20216714980134362, 0.032744179395433504, -0.17186196419862124, 0.027910744162963241};
const float fb__1161_32000[]={0.25, -0.24438952714724374, 0.039527777322790114, -0.20785260208461731, 0.033740362436566529};
const float fb__1406_32000[]={0.25, -0.29500629758978919, 0.04763915211053027, -0.25105622934481459, 0.04074204663687564};
const float fb__1703_32000[]={0.25, -0.35582487296402066, 0.057358369329194581, -0.303060665771827, 0.049183923352743575};
const float fb__2064_32000[]={0.25, -0.42887864909301165, 0.068999338298302737, -0.36567945318394357, 0.059384454437704015};
const float fb__2700_32000[]={0.25, -0.55499261433024361, 0.089029094373943574, -0.47423848824694409, 0.077221610578843358};
const float fb__4000_32000[]={0.25, -0.80071015692573044, 0.12793645687124022, -0.68788947466615036, 0.11334747454129357};
const float fb__5388_32000[]={7.07175, -0.8832013263336429, 0.28360236227213853, -0.98933441724968019, 0.85911705288897544};


const float* filter_bank_table[] = {
fb__140_32000,
fb__185_32000,
fb__270_32000,
fb__367_32000,
fb__444_32000,
fb__539_32000,
fb__653_32000,
fb__791_32000,
fb__958_32000,
fb__1161_32000,
fb__1406_32000,
fb__1703_32000,
fb__2064_32000,
fb__2700_32000,
fb__4000_32000,
fb__5388_32000
};

typedef struct SVF{
  float f_;
  float fq_;
  float x_[2];
  float lp_[2];
  float bp_[2];
} SVF;



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
  float samples_[512]; // 32samples*16bands=512
  //  float delay_buffer_[6144];
  Band band_[17];
} Filterbank;

// functions

void FilterBank_Init(Filterbank* Filterbankk, float sample_rate);
void FilterBank_Synthesize(Filterbank *Filterbankk, float* out, u8 size);
void FilterBank_Analyze(Filterbank *Filterbankk, float* in, u8 size);

enum FilterMode {
  FILTER_MODE_LOW_PASS,
  FILTER_MODE_BAND_PASS,
  FILTER_MODE_BAND_PASS_NORMALIZED,
  FILTER_MODE_HIGH_PASS
};


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
const int32_t kDelayLineSize = 6144;
const int32_t kMaxFilterBankBlockSize = 32;
//const int32_t kSampleMemorySize = 480;

//extern const u8 kNumBands;

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
  float x_[32];
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
        b->post_gain = coefficients[0];
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
      //                              s[j]=(float)(rand()%32768)/32768.0f;
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

#define CONSTRAIN(var, min, max) \
  if (var < (min)) { \
    var = (min); \
  } else if (var > (max)) { \
    var = (max); \
  }

#define false 0
//typedef u8 bool;

static float kFollowerGain; 
//const u8 kNumBands=16;

typedef struct EnvelopeFollower {  
float attack_;
float decay_;
float envelope_;
float peak_;
float freeze_;
} EnvelopeFollower;

void EnvF_Init(EnvelopeFollower* env) {
  kFollowerGain = sqrtf(kNumBands);
  env->envelope_ = 0.0f;
  env->freeze_ = false;
  env->attack_ = env->decay_ = 0.1f;
  env->peak_ = 0.0f;
}
  
void EnvF_set_attack(EnvelopeFollower* env, float attack) {
  env->attack_ = attack;
}
  
void EnvF_set_decay(EnvelopeFollower* env, float decay) {
  env->decay_ = decay;
}
  
void EnvF_set_freeze(EnvelopeFollower* env, bool freeze) {
  env->freeze_ = freeze;
}
  
void EnvF_Process(EnvelopeFollower* env, float* in, float* out, size_t size) {
  float envelope = env->envelope_;
  float attack = env->freeze_ ? 0.0f : env->attack_;
  float decay = env->freeze_ ? 0.0f : env->decay_;
  float peak = 0.0f;
  while (size--) {
    float error = fabsf(*in++ * kFollowerGain) - envelope;
    envelope += (error > 0.0f ? attack : decay) * error;
    if (envelope > peak) {
      peak = envelope;
    }
    *out++ = envelope;
  }
  env->envelope_ = envelope;
  float error = peak - env->peak_;
  env->peak_ += (error > 0.0f ? 0.5f : 0.1f) * error;
}
  
static inline float EnvF_peak(EnvelopeFollower* env) { return env->peak_; }

typedef struct BandGain {
  float carrier;
  float vocoder;
} BandGain;

static float release_time_;
static   float formant_shift_;
  
static BandGain previous_gain_[16];
static BandGain gain_[16];

static float tmp_[32]; // for some reason can't be const variables

static Filterbank modulator_filter_bank_;
static Filterbank carrier_filter_bank_;
//static Limiter limiter_;

EnvelopeFollower follower_[16];
  
void Vocoder_set_release_time(float release_time) {
  release_time_ = release_time;
}

void Vocoder_set_formant_shift(float formant_shift) {
  formant_shift_ = formant_shift;
}

//// limiter

/*
class Limiter {
 public:
  Limiter() { }
  ~Limiter() { }

  void Init() {
    peak_ = 0.5f;
  }

  void Process(
      float* in_out,
      float pre_gain,
      size_t size) {
    while (size--) {
      float s = *in_out * pre_gain;
      SLOPE(peak_, fabs(s), 0.05f, 0.00002f);
      float gain = (peak_ <= 1.0f ? 1.0f : 1.0f / peak_);
      *in_out++ = stmlib::SoftLimit(s * gain * 0.8f);
    }
  }

 private:
  float peak_;
*/

/// vocoder.c break down:

// TODO - *******filter banks*, limiter?for later? -> all compiles but neeed finish wfilterbank and add limiter if necessary
// left all decimation out for now... but means need calc new coeffs
// these have been done I think

void Vocoder_Init(float sample_rate) {
  FilterBank_Init(&carrier_filter_bank_,sample_rate);
  FilterBank_Init(&modulator_filter_bank_, sample_rate);
//  limiter_.Init();
  u8 x;
  release_time_ = 0.5f;
  formant_shift_ = 0.5f;
  
  for (x=0;x<kNumBands;x++){
    previous_gain_[x].carrier=0.0f;
    previous_gain_[x].vocoder=0.0f;
    gain_[x].carrier=0.0f;
    gain_[x].vocoder=0.0f;
  }
//  fill(&previous_gain_[0], &previous_gain_[kNumBands], zero);
//  fill(&gain_[0], &gain_[kNumBands], zero);
  
  for (u8 i = 0; i < kNumBands; ++i) {
    EnvF_Init(&follower_[i]);
  }
}

void Vocoder_Process(
    float* modulator,
    float* carrier,
    float* out,
    u8 size) {
  // Run through filter banks.
      FilterBank_Analyze(&modulator_filter_bank_, modulator, size);
      FilterBank_Analyze(&carrier_filter_bank_, carrier, size);
           
  // Set the attack/release release_time of envelope followers.
  //float f = 80.0f * SemitonesToRatio(-72.0f * release_time_); // what kind of figures come out here?
    float f=80.0f;
    for (int32_t i = 0; i < kNumBands; ++i) {
    float decay = f / modulator_filter_bank_.band_[i].sample_rate;
    EnvF_set_attack(&follower_[i],decay * 2.0f);
    EnvF_set_decay(&follower_[i],decay * 0.5f);
    EnvF_set_freeze(&follower_[i],release_time_ > 0.995f);
    //    f *= 1.2599f;  // 2 ** (4/12.0), a third octave.
  }
    
  // Compute the amplitude (or modulation amount) in all bands.
  //  formant_shift_=adc_buffer[SELY]/4096.0f;
  formant_shift_=0.0f;
  float formant_shift_amount = 2.0f * fabsf(formant_shift_ - 0.5f);
  formant_shift_amount *= (2.0f - formant_shift_amount);
  formant_shift_amount *= (2.0f - formant_shift_amount);
//  float envelope_increment = 4.0f * SemitonesToRatio(-48.0f * formant_shift_);
  float envelope_increment = 0.1f;
  float envelope = 0.0f;
  float kLastBand = kNumBands - 1.0001f;
  
  for (u8 i = 0; i < kNumBands; ++i) {
    float source_band = envelope;
    CONSTRAIN(source_band, 0.0f, kLastBand);
    //    MAKE_INTEGRAL_FRACTIONAL(source_band); // just the INTEGER part! as _integral below!
    //  int32_t x ## _integral = static_cast<int32_t>(x);		
    //  float x ## _fractional = x - static_cast<float>(x ## _integral);
    int32_t source_band_integral= (int32_t) source_band;
    float source_band_fractional= source_band - (float)source_band_integral;
    //    source_band_integral=1;
    float a = EnvF_peak(&follower_[source_band_integral]);
    float b = EnvF_peak(&follower_[source_band_integral + 1]);
    //    float a=1.0f; float b=1.0f;
    float band_gain = (a + (b - a) * source_band_fractional); // fractional part
    float attenuation = envelope - kLastBand;
    if (attenuation >= 0.0f) {
      band_gain *= 1.0f / (1.0f + 1.0f * attenuation);
    }
    envelope += envelope_increment;

    gain_[i].carrier = band_gain * formant_shift_amount;
    gain_[i].vocoder = 1.0f - formant_shift_amount;
  }
  
  for (u8 x=0;x<32;x++) out[x]=0.0f;

      
  for (u8 i = 0; i < kNumBands; ++i) {
    u8 band_size = size;// / modulator_filter_bank_.band_[i].decimation_factor;
    float step = 1.0f / (float)(band_size);

    float* carrierx = carrier_filter_bank_.band_[i].samples;
    float* modulatorx = modulator_filter_bank_.band_[i].samples;
    float* envelopex = tmp_;  

    EnvF_Process(&follower_[i], modulatorx, envelopex, band_size);
    
    float vocoder_gain = previous_gain_[i].vocoder;
    float vocoder_gain_increment = (gain_[i].vocoder - vocoder_gain) * step;
    float carrier_gain = previous_gain_[i].carrier;
    float carrier_gain_increment = (gain_[i].carrier - carrier_gain) * step;
    for (u8 j = 0; j < band_size; ++j) {
        carrierx[j] *= (carrier_gain + vocoder_gain * envelopex[j]);
      vocoder_gain += vocoder_gain_increment;
      carrier_gain += carrier_gain_increment;
      //      if (i==0) out[j]=carrierx[j];
      //      out[j]+=carrierx[j]; // ????
  }
        previous_gain_[i] = gain_[i];
  }
    
  //  carrier_filter_bank_.Synthesize(out, size);
  FilterBank_Synthesize(&carrier_filter_bank_, out, size); // same as above with out[j] ???

  //  limiter_.Process(out, 1.4f, size);

}  




static Filterbank modulator_filter_bank_;
static Filterbank carrier_filter_bank_;

void main(){
  float sample_rate=32000.0f;
  Wavetable wavtable;
  FilterBank_Init(&carrier_filter_bank_,sample_rate);
  FilterBank_Init(&modulator_filter_bank_, sample_rate);
  wavetable_init(&wavtable, crowtable, 142); // now last arg as length of table=less than 512 

  float modulator[32];
  float out[32];
  float carrier[32];
  u8 size=32;  
  while(1){
    //    FilterBank_Analyze(&modulator_filter_bank_, modulator, size);
    //    FilterBank_Analyze(&carrier_filter_bank_, carrier, size);
    dowavetable(modulator, &wavtable, 200.0f, 32); 
    Vocoder_Process(modulator, carrier, out, 32);
    
  }
  
}
