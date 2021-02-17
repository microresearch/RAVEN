#include <math.h>
#include <stdlib.h>
#include <sndfile.h>
#include "wfilterbank.h"
#include "wvocoder.h"

// change now for testing of vocoder and other process - we need makefile

// so we read in soundfile, call process in chunks of 48 samples (48k samplerate) and output file


//////////////////////////////////////////////////////

//  gcc -std=c99 laptest.c -o test -lm -lsndfile -g   

// 44100 so 0.005 seconds would be 220 samples - close to 256 

/* fix on parameters-below, check coeff calc and filter calc (delay), printfs in lpcana, fitlpc !!!

 1- all coeffs from 3 methods are now the same (in praat list of coeffs starts with 1)
 1.5- same results with double and float. so coeffs seems okay
 1.6- why are there different results in praat (crossover? pre-emph and windowing might all be different(

 2- delay and filter is the issue - return to SC code:

but filter of source using coeffs always works but NOT inverse filter on source?

 3- check filter and pre-emph - only works with pre-emph and predict from fitlpc - blocksize and chunksize issue to fix
///////TODO from here
 4- porting

 5- look at other LPC code

*/

#define BLOCK_SIZE 128
#define CHUNKSIZE 48 // chunksize less than blocksize ???
#define SAMPLERATE 48000
// what is samplerate

typedef float LPCfloat;
static const int windowsize=BLOCK_SIZE; // say up to 1024

///source= Impulse.ar(delaytime.reciprocal); 
// 
void do_impulse(float *out, int numSamples, uint16_t freq){ //- so for 256 samples we have freq 125 for impulse
    // from Impulse->LFUGens.cpp
  int i;
  static float phase =0.0f;
  float z, freqinc;
  freqinc=0.00003125 * freq;

  for (i=0; i<numSamples;++i) {
    if (phase >= 1.f) {
      phase -= 1.f;
      z = 1.f;
    } else {
      z = 0.f;
    }
    phase += freqinc; // punch in freq is freqmul=1/32000 = 0.00003125 * 1000 (32000/32) = 0.03125
    out[i]=z;
  }
}

static float delay[1280];


//(DelayN.ar(input,delaytime, delaytime)- LPCAnalyzer.ar(input,source,1024,MouseX.kr(1,256))).poll(10000)
void do_delay(float *in, float *out, uint16_t delaytime, int numSamples){
  // delay up to buffersize =512
  static uint16_t dpointer=0;
  for (int i=0;i<numSamples;i++){
    out[i]=delay[dpointer];
  // place sample in buffer
    delay[dpointer++]=in[i];
    if (dpointer==delaytime) dpointer=0;
}
}

void main(int argc, char * argv []){

  // read in wav file(?) into float and output coefficients as array
	char 		*progname, *infilename, *outfilename ;
	SNDFILE	 	*infile = NULL ;
	SNDFILE		*outfile = NULL ;
	SF_INFO	 	sfinfo ;
	
	float pout[1024];

	infilename = argv [1] ;

	if ((infile = sf_open (infilename, SFM_READ, &sfinfo)) == NULL)
	{	printf ("Not able to open input file %s.\n", infilename) ;
		puts (sf_strerror (NULL)) ;
		} ;

	if (! (outfile = sf_open("output.wav", SFM_WRITE, &sfinfo)))
	  {   printf ("Not able to open output file %s.\n", "output.wav") ;
	    sf_perror (NULL) ;
	    //	    return  1 ;
	  } ;

	printf ("# Channels %d, Sample rate %d\n", sfinfo.channels, sfinfo.samplerate) ;

	////// 	ANY_INIT
	Vocoder_Init(SAMPLERATE);
	// read in 32 samples and print coeffs to test
	int k, m, readcount,count=0;
	static float input[1280]; 
	static float output[1280];

	while ((readcount = sf_readf_float (infile, input,CHUNKSIZE)) > 0)
	//			while ((readcount = sf_readf_double (infile, input, BLOCK_SIZE)) > 0)
	{	
	  // do_process for 48 samples
	  // if we need exc source additionally call
	  do_impulse(pout, CHUNKSIZE, 220);

	  Vocoder_Process(input, pout, output, CHUNKSIZE); // output is very quiet!

	  sf_writef_float (outfile, output, readcount) ;
	  count++;
	} ;

	sf_close (infile) ;
	sf_close (outfile) ;
}


