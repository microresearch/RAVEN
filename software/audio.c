// license:GPL-2.0+
// copyright-holders: Martin Howse

#define STEREO_BUFSZ (BUFF_LEN/2) // 64
#define MONO_BUFSZ (STEREO_BUFSZ/2) // 32

#include "audio.h"
#include "wavetable.h"
#include "wvocoder.h"
#include "resources.h"

// for vocoder tests
float modulator[32];
float carrier[32];
float vout[32];
int16_t incoming[32];

extern __IO uint16_t adc_buffer[10];
int16_t audio_buffer[AUDIO_BUFSZ] __attribute__ ((section (".ccmdata")));

int16_t	left_buffer[MONO_BUFSZ], mono_buffer[MONO_BUFSZ];

float smoothed_adc_value[5]={0.0f, 1.0f, 0.0f, 0.0f, 0.0f}; // as mode is inverted and we don't want to start with COMPOSTF?

static u16 cc=0;

extern float exy[240];
float _mode, _speed, _selx, _sely, _selz;
static float oldselx=0.5f, oldsely=0.5f, oldselz=1.0f;

#define float float32_t

extern Wavetable wavtable;

static inline void audio_comb_stereo(int16_t sz, int16_t *dst, int16_t *lsrc, int16_t *rsrc)
{
	while(sz)
	{
		*dst++ = *lsrc++;
		sz--;
		*dst++ = (*rsrc++);
		sz--;
	}
}

static inline void doadc(){
  float value;
  
  value =(float)adc_buffer[SELX]/65536.0f; 
  smoothed_adc_value[2] += 0.1f * (value - smoothed_adc_value[2]);
  _selx=smoothed_adc_value[2];
  CONSTRAIN(_selx,0.0f,1.0f);

  value =(float)adc_buffer[SELY]/65536.0f; 
  smoothed_adc_value[3] += 0.1f * (value - smoothed_adc_value[3]);
  _sely=smoothed_adc_value[3];
  CONSTRAIN(_sely,0.0f,1.0f);

  value =(float)adc_buffer[SELZ]/65536.0f; 
  smoothed_adc_value[4] += 0.01f * (value - smoothed_adc_value[4]); // smoother
  _selz=smoothed_adc_value[4];
  CONSTRAIN(_selz,0.0f,1.0f);
}

static inline void floot_to_int(int16_t* outbuffer, float* inbuffer,u16 howmany){
  int32_t tmp;

  for (int n = 0; n < howmany; n++) {
    tmp = inbuffer[n] * 32768.0f;
    tmp = (tmp <= -32768) ? -32768 : (tmp >= 32767) ? 32767 : tmp;
    outbuffer[n] = (int16_t)tmp;
		}
}

static inline void int_to_floot(int16_t* inbuffer, float* outbuffer, u16 howmany){
  for (int n = 0; n < howmany; n++) {
    outbuffer[n]=(float32_t)(inbuffer[n])/32768.0f;
  }
}



////////////////////////////////////////////// TESTING!

#ifdef TESTING


void test_wave(int16_t* incoming,  int16_t* outgoing, float samplespeed, u8 size){ // how we choose the wavetable - table of tables?
  float lastbuffer[32];
  uint16_t val=0;
  doadc();
    val=(1.0f-_selx)*1027.0f; // how can we test all others???? - add and then mod
    val+=(1.0f-_sely)*1027.0f; 
    val+=(1.0f-_selz)*1027.0f;

  float value =(float)adc_buffer[SPEED]/65536.0f; 
    smoothed_adc_value[0] += 0.1f * (value - smoothed_adc_value[0]);
    _speed=smoothed_adc_value[0];
    CONSTRAIN(_speed,0.0f,1.0f);
        val+=(1.0f-_speed)*1027.0f;


    value =(float)adc_buffer[MODE]/65536.0f; 
    smoothed_adc_value[1] += 0.01f * (value - smoothed_adc_value[1]); // 0.01f for SMOOTHER mode locking
    _mode=smoothed_adc_value[1];
    CONSTRAIN(_mode,0.0f,1.0f);
    val+=(1.0f-_mode)*1027.0f;

  //  val=val%1024;

  MAXED(val,1023);
    dowavetable(lastbuffer, &wavtable, 2.0f+(logspeed[val]*440.0f), size); // for exp/1v/oct test
  //    dowavetable(lastbuffer, &wavtable, 2.0f+(1.038673f*440.0f), size); // for exp/1v/oct test
  //    dowavetable(lastbuffer, &wavtable, 440.0f, size); // for exp/1v/oct test
  floot_to_int(outgoing,lastbuffer,size);
}  

void I2S_RX_CallBack(int16_t *src, int16_t *dst, int16_t sz)
{
  static u8 triggered=0;

 float  samplespeed=1.0f;
  void (*generators[])(int16_t* incoming,  int16_t* outgoing, float samplespeed, u8 size)={test_wave}; 

  // splitting input
    for (u8 x=0;x<sz/2;x++){
  // we want to test trigger here??? void wave_newsay(void)
    sample_buffer[x]=*(src++); // right is input on LACH, LEFT ON EURO!
    src++;
    if (sample_buffer[x]>=THRESH && !triggered) {
      doadc();
      wave_newsay();
      triggered=1;
  }
  else if (sample_buffer[x]<THRESHLOW && triggered) triggered=0;
    //  mono_buffer[x]=rand()%32768;
  }
    
    
    generators[0](sample_buffer,mono_buffer,samplespeed,sz/2); 
  
  audio_comb_stereo(sz, dst, mono_buffer,left_buffer);
}

#else

////////--->>> list of modes


int16_t none_get_sample(){
  return 0;
}

void none_newsay(){
}

typedef struct wormer_ {
  u8 maxextent;
  float sampleratio;
  int16_t(*getsample)(void);
  void(*newsay)(void);
  u8 xy;
  u8 TTS;
} wormer;

// static const wormer compostfrer={0, 1.0f, compost_get_sample, compost_newsay_frozen, 0, 0};

//static const wormer *wormlist[]={&tmser, &tmslowbiter, &tmssinger, &tmsbendlengther, &tmsphoner, &tmsphonsinger, &tmsttser, &tmsraw5100er, &tmsraw5200er, &tmsraw5220er, &tmsbend5100er, &tmsbend5200er, &tmsbend5200erx, &tms5100pitchtablebender, &tms5200pitchtablebender, &tms5200pitchtablebenderx, &tms5100ktablebender, &tms5200ktablebender, &tms5100kandpitchtablebender, &tms5200kandpitchtablebender, &tms5200kandpitchtablebenderx, &sp0256er, &sp0256singer, &sp0256TTSer, &sp0256vocaboneer, &sp0256vocabtwoer, &sp02561219er, &sp0256bender, &votraxer, &votraxTTSer, &votraxgorfer, &votraxwower, &votraxwowfilterbender, &votraxbender, &votraxparamer, &votraxsinger, &sambanks0er, &sambanks1er, &samTTSer, &samTTSser, &samphoner, &samphonser, &samphonsinger, &samxyer, &samparamer, &sambender, &digitalker, &digitalker_sing, &digitalker_bendpitchvals, &rsynthy, &rsynthelm, &rsynthsingle, &rsynthysing, &klatter, &klattsingle, &klattsinglesing, &klattvocab, &klattvocabsing, &simpleklatter, &nvper, &nvpvocaber, &nvpvocabsing, &composter, &compostfrer};

#define MODEF 65.0f // float 68.0f
#define MODEFC 64.0f // float 68.0f
#define MODET 63 // mode top 63 // 61 for no COMPOST

/////////////////---------------------------------------------------------

void I2S_RX_CallBack(int16_t *src, int16_t *dst, int16_t sz)
{
  static u8 _intmode=0;
  u8 trigger=0;
  int16_t sample;
  float samplespeed;
  float value;
  u8 oldmode; 
  //  value =(float)adc_buffer[SPEED]/65536.0f; 
  //  smoothed_adc_value[0] += 0.1f * (value - smoothed_adc_value[0]); // smooth
  //  _speed=smoothed_adc_value[0];
  //  CONSTRAIN(_speed,0.0f,1.0f);
  //  _speed=1.0f-_speed;

  value =(float)adc_buffer[MODE]/65536.0f; 
  smoothed_adc_value[1] += 0.01f * (value - smoothed_adc_value[1]); // 0.01f for SMOOTHER mode locking
  _mode=smoothed_adc_value[1];
  CONSTRAIN(_mode,0.0f,1.0f);
  _mode=1.0f-_mode; // invert
    oldmode=_intmode;
  _intmode=_mode*MODEF;
  
  MAXED(_intmode, MODET); 

      if (oldmode!=_intmode) {// IF there is a modechange!
	//      trigger=1; 
    // and if we are not leaving compost - if we are entering compost ???
    }
  
    

    for (u8 x=0;x<sz/2;x++){ /// sz/2=128/2-64 = /2=32 
    incoming[x]=*(src++);
    src++;
  }
    //    samplerate_simple_exy(mono_buffer, samplespeed, sz/2, wormlist[_intmode]->getsample, wormlist[_intmode]->sampleratio, wormlist[_intmode]->maxextent, triggered); // trigger toggle only on threshold

    // wvocoder testing now - generate carrier-wavetable as in testing, modulator is sample, convert this to floats, convert back from floats below
    dowavetable(carrier, &wavtable, 1.0f, sz/2); // not sure what speed 1.0f should be - test?
    // modulator
    int_to_floot(incoming, modulator, sz/2);

    Vocoder_Process(modulator, carrier, vout, sz/2);
    floot_to_int(mono_buffer, vout, sz/2);
     
      
    for (u8 x=0;x<sz/2;x++) {
      audio_buffer[cc++]=mono_buffer[x];
    if (cc>AUDIO_BUFSZ-1) cc=0;
    }
  

  audio_comb_stereo(sz, dst, mono_buffer,left_buffer);
}
#endif
