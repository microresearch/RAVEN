// for pc version

#include <math.h>
#include <stdlib.h>
#include <sndfile.h>
#include "forlap.h"
#include "svf.h"
#include "pitch.h"
//#include "tms5110r.inc" // tables
#include "CodingTable.h"
//#include "tms5220x.h"
#include "lpc_lap.h"

//  gcc -std=c99 tmslive.c lpc_lap.c svf.c PitchEstimator.c -o tms -lm -lsndfile -g   

// start with simple inout of framesize x from wav (what is default) at 8k samplerate and output values
// then check values and playback - how values match with python

// questions about: RMS, normalizing, voiced/unvoiced, repeat, bends->k values in arrays, pitches, freezes, add other encodings together, very raw bends

// TODO: does playback table from talkie match the one we use to encode, different chips and playbalcks, pre-emph and above 

// frameRate default is 25, windowWidth in frames is 2

/*
    pitchValue = 0
    unvoicedThreshold = 0.3
    windowWidth = 2
    normalizeUnvoicedRMS = False
    normalizeVoicedRMS = False
    includeExplicitStopFrame = True
    preEmphasis = True
    preEmphasisAlpha = -0.9373
    overridePitch = False
    pitchOffset = 0
    maximumPitchInHZ = 500
    minimumPitchInHZ = 50
    frameRate = 25
    subMultipleThreshold = 0.9
    outputFormat = "arduino"
    rmsLimit = 14
    tablesVariant = "tms5100"
 */

#define CHUNKSIZE 400 // not sure how long =self.sizeForWindow = sizeof 200 x windowWidth x2 frames = 400

// from segmenter.py:         self.size = int(sp.ceil(buf.sampleRate / 1e3 * framerate)) 

//8000/1000 *25= 8*25 =200 

float coeffs[4]={-0.04213060834239553, 0.05158579880405556, -0.32595702702170076, 0.36023835078244226};
//float coeffs[4]={-0.0282793973573, 0.00959297446756, -0.0163167758833, 0.0230064882424};
SVF svf[2]; // 2 passes

float samplerate=8000.0f;
uint16_t minimumpitch=50;
uint16_t maximumpitch=500;
float unvoicedThreshold=0.3f; // default threshold is: 0.3
extern double _ks[11];

uint16_t _k1, _k2, _k3, _k4, _k5, _k6, _k7, _k8, _k9, _k10, gainindex,pitchindex;

////////////////////////////////////

void filterer(float *inbuffer, float *outbuffer, int16_t size){

  // this doesn't seem to work!
  
  /// computes butterworth coeffs IIR filtfilt I think - order=5, bandpass
  // lowpass as 50, high as 500 so can pre-compute any coeffs

  // try svf.c
  // void SVF_Process(SVF* svf, const float* in, float* out, u8 size, u8 mode) {

    for (u8 pass = 0; pass < 2; ++pass) {
      const float* source = pass == 0 ? inbuffer : outbuffer;
      float* destination = outbuffer;
      SVF_Process(&svf[pass],source, destination, size, FILTER_MODE_BAND_PASS_NORMALIZED);
    }  
    //  printf("sample: %f \n", outbuffer[0]);

}

/*
class Filterer(object):
    def __init__(self, buf, lowPassCutoffInHZ, highPassCutoffInHZ, gain, order=5, ):
        self.gain = gain
        self.buf = buf
        nyq = 0.5 * buf.sampleRate
        low = lowPassCutoffInHZ / nyq
        # Avoid highpass frequency above nyqist-freq, this leads to
        # weird behavior
        high = min( (highPassCutoffInHZ/ nyq, 1) )
        self.b, self.a = signal.butter(order, [low, high], btype='band')

    def process(self):
        #a linear filter that achieves zero phase delay by applying an IIR
        #filter to a signal twice, once forwards and once backwards. The order
        #of the filter is twice the original filter order. The function also
        #computes the initial filter parameters in order to provide a more
        #stable response
        
        myFilter = lambda x: signal.filtfilt(self.b, self.a, x)
        return Buffer.copy(self.buf, applyFilter=myFilter)
*/


double pitchtable(float *buffer, uint16_t size){
  float tmpbuffer[1280]; 
  double estimater;
  uint16_t minimumPeriod=(int)samplerate/maximumpitch;
  uint16_t maximumPeriod=(int)samplerate/minimumpitch;
  // filters from filterer
  filterer(buffer, tmpbuffer, size);
  estimater = pitchForPeriod(buffer, size, minimumPeriod, maximumPeriod); // min is 8000/500, max is 8000/50
  //             pitchTable[index] = PitchEstimator.pitchForPeriod(buf)
  //        return cls(buf).estimate()
  //  printf("estimate pitch: %d %d EST %f \n", minimumPeriod, maximumPeriod, estimater);
  return estimater;
}

/*
    def pitchTableForBuffer(self, pitchBuffer):
        filterer = Filterer(pitchBuffer, lowPassCutoffInHZ=settings.minimumPitchInHZ, highPassCutoffInHZ=settings.maximumPitchInHZ, gain=1)
        buf = filterer.process()

        segmenter = Segmenter(buf, windowWidth=2)
        pitchTable = sp.zeros(segmenter.numberOfSegments())

        for (buf, index) in segmenter.eachSegment():
            pitchTable[index] = PitchEstimator.pitchForPeriod(buf)
*/


////////////////////////////////////

void main(int argc, char * argv []){

  // read in wav file(?) into float and output coefficients as array
	char 		*progname, *infilename, *outfilename ;
	SNDFILE	 	*infile = NULL ;
	SNDFILE		*outfile = NULL ;
	SF_INFO	 	sfinfo ;
	
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

	int k, m, readcount,count=0;
	static float input[1280]; 
	static float output[1280];

	lpcinit();
	
	for (u8 pass = 0; pass < 2; ++pass) {
	  SVF_Init(&svf[pass]);
	  set_f_fq(&svf[pass],coeffs[(pass * 2)],coeffs[(pass * 2)+1]); // calculated 4 coefficients using filterfortm
    }

	int counter=0;
	uint16_t offset=0;
	double rms;
	readcount = sf_readf_float (infile, input+offset,CHUNKSIZE/2); // read first chunk
	offset=CHUNKSIZE/2;
	while ((readcount = sf_readf_float (infile, input+offset,CHUNKSIZE/2)) > 0)
	{
	  double pitch = pitchtable(input, CHUNKSIZE);
	  //	  counter++; printf("%d  ",counter);
	  Hamming(input, CHUNKSIZE);
	  getCoefficientsFor(input, CHUNKSIZE);
	  rms=translateCoefficients(CHUNKSIZE);

	  // so we have to deal with:
	  //voiced/unvoiced
	  if (_ks[1]>=unvoicedThreshold){ // default threshold is: 0.3
	    printf("unvoiced "); // and/??
	    pitch=0;
	  }
	  
	  // and repeat seems to be always 0/false

	  // stopframe? but can we have any stopframe as never stops?
	  gainindex=ClosestValueFinder(rms, rmstable, sizeof(rmstable)); // still need to work a bit on rms
	  float gain= rmstable[gainindex]; // samples seem a bit quiet
	  printf("gain %f ", gain);
	  float pitchh;
	  if (pitch==0) {
	    pitchh=0.0;
	    pitchindex=0;
	      }
	  else
	    {
	      pitchindex=ClosestValueFinder(pitch, pitchtablet, sizeof(pitchtablet)/sizeof(float)); // index !
	      pitchh=pitchtablet[pitchindex];

	    }
	      printf("pitch %f ", pitchh);
	  // ks? k1-k10
	  //	  printf(" %f %f %f", _ks[1], _ks[2], _ks[3]);
	  //	  float _k1=k1[ClosestValueFinder(_ks[1], k1, sizeof(k1))]; // but what we want is the index
	  _k1=ClosestValueFinder(_ks[1], k1, sizeof(k1)/sizeof(float));
	  printf("k1 %d ", _k1);
	  _k2=ClosestValueFinder(_ks[2], k2, sizeof(k2)/sizeof(float));
	  printf("k2 %d ", _k2);
	  _k3=ClosestValueFinder(_ks[3], k3, sizeof(k3)/sizeof(float));
	  printf("k3 %d ", _k3);
	  _k4=ClosestValueFinder(_ks[4], k4, sizeof(k4)/sizeof(float));
	  printf("k4 %d ", _k4);
	  _k5=ClosestValueFinder(_ks[5], k5, sizeof(k5)/sizeof(float));
	  printf("k5 %d ", _k5);
	  _k6=ClosestValueFinder(_ks[6], k6, sizeof(k6)/sizeof(float));
	  printf("k6 %d ", _k6);
	  _k7=ClosestValueFinder(_ks[7], k7, sizeof(k7)/sizeof(float));
	  printf("k7 %d ", _k7);
	  _k8=ClosestValueFinder(_ks[8], k8, sizeof(k8)/sizeof(float));
	  printf("k8 %d ", _k8);
	  _k9=ClosestValueFinder(_ks[9], k9, sizeof(k9)/sizeof(float));
	  printf("k9 %d ", _k9);
	  _k10=ClosestValueFinder(_ks[10], k10, sizeof(k10)/sizeof(float));
	  printf("k10 %d\n", _k10);

	  // also how playback works as we overlap frames? playback every 200 samples each frame...

	  // for bends we can shift around in the tables!!
	  
	  // tms playback for each/half frame TODO ????
	  // let's try to write 200 samples to buffer
	  for (uint16_t j=0; j<CHUNKSIZE/2; j++){
	    //	    uint16_t lpc_get_sample(void)
	    uint16_t sample =(lpc_get_sample()<<6)-32768; //     int16_t samplel=(lpc_get_sample()<<6)-32768; // TODO or scale samples/speed???

	    // conv to float and write
	    output[j]=(float)(sample)/32768.0f; 
	    //	    printf("%f ", output[j]);
	  }
	  sf_writef_float (outfile, output, CHUNKSIZE/2);  
	  // copy last second chunk to first
	  for (uint16_t i=0; i<CHUNKSIZE/2; i++){
	    input[i]=input[i+(CHUNKSIZE/2)];
	      }
	    
	  /*
        if settings.includeExplicitStopFrame:
            frames.append(frameData.stopFrame())
	  */
	  
	  
	  //	  sf_writef_double (outfile, output, readcount) ;
		count++;
	} ;

	sf_close (infile) ;
	sf_close (outfile) ;
}


