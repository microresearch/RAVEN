// for pc version

#include <math.h>
#include <stdlib.h>
#include <sndfile.h>
#include "forlap.h"
#include "svf.h"
#include "pitch.h"
#include "tms5110r.inc" // tables

//  gcc -std=c99 tmslive.c svf.c PitchEstimator.c -o tms -lm -lsndfile -g   

// start with simple inout of framesize x from wav (what is default) at 8k samplerate and output values

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
	/*
	if (! (outfile = sf_open("output.wav", SFM_WRITE, &sfinfo)))
	  {   printf ("Not able to open output file %s.\n", "output.wav") ;
	    sf_perror (NULL) ;
	    //	    return  1 ;
	  } ;
	*/
	printf ("# Channels %d, Sample rate %d\n", sfinfo.channels, sfinfo.samplerate) ;

	int k, m, readcount,count=0;
	static float input[1280]; 
	static float output[1280];

	for (u8 pass = 0; pass < 2; ++pass) {
	  SVF_Init(&svf[pass]);
	  set_f_fq(&svf[pass],coeffs[(pass * 2)],coeffs[(pass * 2)+1]); // calculated 4 coefficients using filterfortm
    }

	int counter=0;
	uint16_t offset=0;
	readcount = sf_readf_float (infile, input+offset,CHUNKSIZE/2); // read first chunk
	offset=CHUNKSIZE/2;
	while ((readcount = sf_readf_float (infile, input+offset,CHUNKSIZE/2)) > 0)
	{
	  // how can we do the overlap...

	  double pitch = pitchtable(input, CHUNKSIZE);
	  //	  counter++; printf("%d  ",counter);
	  Hamming(input, CHUNKSIZE);
	  getCoefficientsFor(input, CHUNKSIZE);
	  translateCoefficients(CHUNKSIZE);
	  /*
            HammingWindow.processBuffer(cur_buf)
            coefficients = cur_buf.getCoefficientsFor()
            reflector = Reflector.translateCoefficients(self.codingTable, coefficients, cur_buf.size)
	    frameData = FrameData(reflector, pitch, repeat=False)
	    PRINT the frame which is: gain, repat, pitch, set of 10 k coefficients
	  */


	  // copy last second chunk to first
	  for (uint16_t i=0; i<CHUNKSIZE/2; i++){
	    input[i]=input[i+(CHUNKSIZE/2)];
	      }
	    
	  
	  /*
	  b=Buffer.fromWave(args.filename) // this does downsampling to 8k
	  x=Processor(b, args.tablesVariant)
	  result = BitPacker.pack(x)
	  */

	  /* processor.py

	     self.mainBuffer = buf
	     self.pitchTable = None
	     self.pitchBuffer = Buffer.copy(buf)
our tms5110r.inc:        self.codingTable = CodingTable(model)

        if settings.preEmphasis: // leave this for now!
            PreEmphasizer.processBuffer(buf)

        self.pitchTable = {}
        wrappedPitch = False
        if settings.overridePitch:
            wrappedPitch = settings.pitchValue
        else:
DONE:            self.pitchTable = self.pitchTableForBuffer(self.pitchBuffer)

        coefficients = sp.zeros(11)

input:        segmenter = Segmenter(buf=self.mainBuffer, windowWidth=settings.windowWidth)

        frames = []

TODO:        for (cur_buf, i) in segmenter.eachSegment():
            HammingWindow.processBuffer(cur_buf)
            coefficients = cur_buf.getCoefficientsFor()
            reflector = Reflector.translateCoefficients(self.codingTable, coefficients, cur_buf.size)

            if wrappedPitch:
                pitch = int(wrappedPitch)
            else:
                pitch = self.pitchTable[i]

translations from tables and print this i guess:            frameData = FrameData(reflector, pitch, repeat=False)

            frames.append(frameData)

        if settings.includeExplicitStopFrame:
            frames.append(frameData.stopFrame())
	  */
	  
	  //	  sf_writef_float (outfile, output, readcount) ;
	  //	  sf_writef_double (outfile, output, readcount) ;
		count++;
	} ;

	sf_close (infile) ;
	sf_close (outfile) ;
}


