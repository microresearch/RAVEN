#include <math.h>
#include <stdlib.h>
#include <sndfile.h>
#include "forlap.h"

//  gcc -std=c99 lpcforlap.c -o sclpc -lm -lsndfile -g   

/* 2021 RAVEN:

- revisiting and LPC_samecross now reconstructs signal - find residual(=inverse filter) from incoming and filter this with coeffs from incoming should be same signal more or less
(added delay and re-ordering.

- TODO: try faster coeff calcs, work with different window sizes

- what are parameters/versions: P_MAX, bending of coefficients, merge
  or no merge, gain/envelope or none, hold LPC coeffs, slow envelope
  of coeffs/interpolate, lagging of the gain/envelope, storage and
  recall of coeffs, pre-computed coeffs as tables, different sizes of
  windows, delays and lags

- how can we do any kind of pitch shift or other changes?

- versions across->

excitation/source: inverse filtering with different lengths of coeffs, how would freeze or slow hold of coeffs for this work

transform: most of above 

- merged windowing/fade of params in SC code...

 */


// 44100 so 0.005 seconds would be 220 samples - close to 256 

/* fix on parameters-below, check coeff calc and filter calc (delay), printfs in lpcana, fitlpc !!!

 1- all coeffs from 3 methods are now the same (in praat list of coeffs starts with 1)
 1.5- same results with double and float. so coeffs seems okay
 1.6- why are there different results in praat (crossover? pre-emph and windowing might all be different)

 2- delay and filter is the issue - return to SC code:

but filter of source using coeffs always works but NOT inverse filter on source?

 3- check filter and pre-emph - only works with pre-emph and predict from fitlpc - blocksize and chunksize issue to fix
///////TODO from here
 4- porting

 5- look at other LPC code

*/

#define BLOCK_SIZE 32
#define CHUNKSIZE 32 // chunksize less than blocksize ???
#define	P_MAX	31	/* order p of LPC analysis, typically 8..14 */
#define SAMPLERATE 48000

// what is samplerate

typedef float LPCfloat;
static const int windowsize=BLOCK_SIZE; // say up to 1024
static LPCfloat inputty[129];

static LPCfloat window128[128]={0.000003, 0.000007, 0.000012, 0.000020, 0.000031, 0.000045, 0.000066, 0.000094, 0.000132, 0.000184, 0.000254, 0.000346, 0.000470, 0.000633, 0.000846, 0.001125, 0.001485, 0.001950, 0.002544, 0.003300, 0.004256, 0.005455, 0.006953, 0.008810, 0.011098, 0.013900, 0.017308, 0.021428, 0.026375, 0.032277, 0.039273, 0.047510, 0.057144, 0.068335, 0.081248, 0.096044, 0.112883, 0.131909, 0.153256, 0.177034, 0.203323, 0.232174, 0.263593, 0.297542, 0.333931, 0.372615, 0.413388, 0.455984, 0.500077, 0.545278, 0.591145, 0.637184, 0.682857, 0.727594, 0.770803, 0.811880, 0.850228, 0.885265, 0.916443, 0.943263, 0.965282, 0.982134, 0.993531, 0.999279, 0.999279, 0.993531, 0.982134, 0.965282, 0.943263, 0.916443, 0.885265, 0.850228, 0.811880, 0.770803, 0.727594, 0.682857, 0.637184, 0.591145, 0.545278, 0.500077, 0.455984, 0.413388, 0.372615, 0.333931, 0.297542, 0.263593, 0.232174, 0.203323, 0.177034, 0.153256, 0.131909, 0.112883, 0.096044, 0.081248, 0.068335, 0.057144, 0.047510, 0.039273, 0.032277, 0.026375, 0.021428, 0.017308, 0.013900, 0.011098, 0.008810, 0.006953, 0.005455, 0.004256, 0.003300, 0.002544, 0.001950, 0.001485, 0.001125, 0.000846, 0.000633, 0.000470, 0.000346, 0.000254, 0.000184, 0.000132, 0.000094, 0.000066, 0.000045, 0.000031, 0.000020, 0.000012, 0.000007, 0.000003};

static LPCfloat window32[32]={0.000019, 0.000088, 0.000318, 0.001015, 0.002934, 0.007748, 0.018718, 0.041390, 0.083793, 0.155316, 0.263593, 0.409601, 0.582778, 0.759205, 0.905585, 0.989041, 0.989041, 0.905585, 0.759205, 0.582778, 0.409601, 0.263593, 0.155316, 0.083793, 0.041390, 0.018718, 0.007748, 0.002934, 0.001015, 0.000318, 0.000088, 0.000019};

// gaussian window of varying sizes are generated below

static LPCfloat lasted[128];
static LPCfloat lasteded[128];
static LPCfloat delay[128];
static LPCfloat tmp[128];
static LPCfloat coeff[32];
static LPCfloat G; //gain;

static LPCfloat R[32];
static int pos=0;

///source= Impulse.ar(delaytime.reciprocal); 
//

/*
	unit->mPhaseOffset = 0.f;
	unit->mFreqMul = unit->mRate->mSampleDur; // sample dur is seconds per sample so for 48k we have 1/48000 
	if (unit->mPhase == 0.f) unit->mPhase = 1.f;

	float freq = ZIN0(0) * unit->mFreqMul;


	LOOP1(inNumSamples,
		float z;
		if (phase >= 1.f) {
			phase -= 1.f;
			z = 1.f;
		} else {
			z = 0.f;
		}
		phase += freq;
		ZXP(out) = z;
	);
 */

void do_noise(float *out, int numSamples){
  int i;
  for (i=0; i<numSamples;++i) {
    out[i]=1.0f-((float)(rand()%32768)/16355.0f);
  }
}

void do_impulse(float *out, int numSamples, float freq){ //- so for 256 samples we have freq 125 for impulse
    // from Impulse->LFUGens.cpp
  int i;
  float recip=1.0f/SAMPLERATE;
  static float phase =0.0f;
  float z, freqinc;
  freqinc=recip * freq; 

  for (i=0; i<numSamples;++i) {
    if (phase >= 1.f) {
      phase -= 1.f;
      z = 1.f;
    } else {
      z = 0.f;
    }
    phase += freqinc; // punch in freq is freqmul=1/32000 = 0.00003125 * 1000 (32000/32) = 0.03125 ????
    out[i]=z;
  }
}

//(DelayN.ar(input,delaytime, delaytime)- LPCAnalyzer.ar(input,source,1024,MouseX.kr(1,256))).poll(10000) // here delaytime is 1024
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

void lpc_preemphasis(LPCfloat * x, int len, LPCfloat alpha )
{
    for( int i = len - 1; i > 0; i-- )
      x[i] = x[i] - alpha * x[i-1];////y[k]=x[k]-0.95x[k-1]
}

//recalculate poles based on recent window of input
void SCcalculatePoles() {

	//can test for convergence by looking for 1-((Ei+1)/Ei)<d

	int i, j;
	LPCfloat sum;

	//safety
	int numpoles=windowsize;

	//printf("p? %d",p);

	//calculate new LPC filter coefficients following (Makhoul 1975) autocorrelation, deterministic signal, Durbin iterative matrix solver

	//float R[21];//autocorrelation coeffs;
	float preva[21];
	float a[21];
	LPCfloat E, k;

	//faster calculation of autocorrelation possible?

	for(i=0; i<=numpoles; ++i) {
		sum=0.0;

		for (j=0; j<= windowsize-1-i; ++j)
			sum+= inputty[j]*inputty[j+i];

		R[i]=sum;
	}

	E= R[0];
	k=0;

	if(E<0.00000000001) {

		//zero power, so zero all coeff
		for (i=0; i<numpoles;++i)
			coeff[i]=0.0;
//
//		latesterror= E;
		G=0.0;
		//printf("zero power %f\n", E);
		return;

	};

	//rescaling may help with numerical instability issues?
//	float mult= 1.0/E;
//	for(i=1; i<=numpoles; ++i)
//		R[i]= R[i]*mult;
//
	for(i=0; i<=(numpoles+1); ++i) {
		a[i]=0.0;
		preva[i]=0.0; //CORRECTION preva[j]=0.0;
	}


//	for(i=0; i<numpoles; ++i) {
//		printf("sanity check a %f R %1.15f ",a[i+1], R[i]);
//	}
//	printf("\n");


	LPCfloat prevE= E;

	for(i=1; i<=numpoles; ++i) {

		sum=0.0;

		for(j=1;j<i;++j)
			sum+= a[j]*R[i-j];

		k=(-1.0*(R[i]+sum))/E;

		a[i]=k;

		for(j=1;j<=(i-1);++j)
			a[j]=preva[j]+(k*preva[i-j]);

		for(j=1;j<=i;++j)
			preva[j]=a[j];

		E= (1-k*k)*E;

		//printf("E check %f %d k was %f\n", E,i,k);

		//check for instability; all E must be greater than zero

	}

       	G= sqrtf(E);

	//	latesterror= E;

	//solution is the final set of a
	for(i=0; i<numpoles; ++i) {
		//coeff[numpoles-1-i]=a[i+1];
		coeff[i]=a[i+1];
		//printf("a %f R %f",a[i+1], R[i]);
	}
}

void calculateDurbPoles(){ // into coeffs
  int i, j;  LPCfloat r,sum;

	// this is as autocorrelation

	for(i=0; i<=P_MAX; ++i) {
	  coeff[i]=0.0f;
		sum=0.0;
		for (j=0; j<= windowsize-1-i; ++j) sum+= inputty[j]*inputty[j+i]; // 
		R[i]=sum;
		//		printf("SUM: %f,,,,\n", R[i]);
	}


	/// NEW pitch work
	//	float max=0;
	//	for (i=0;i<=P_MAX;


    LPCfloat error = R[0];

    for (i = 0; i < P_MAX; i++) {
      r = -R[i + 1];
      for (j = 0; j < i; j++) r -= coeff[j] * R[i - j];
      r /= error;
			coeff[i] = r;
			for (j = 0; j < i/2; j++) {
				LPCfloat tmp  = coeff[j];
				coeff[j]     += r * coeff[i-1-j];
				coeff[i-1-j] += r * tmp;
				///				printf("iiii %d %d\n",j, i-1-j);
			}
			if (i % 2) coeff[j] += coeff[j] * r;

error *= 1.0 - r * r;
    }

    /*	for(i=0; i<=P_MAX; ++i) {
	  printf("%f ",coeff[i]); 
	}
    */
//	G= sqrtf(error); .// TODO!
}

void zeroAll() {
  int i;
  //  P_MAX=10;
  for (i=0; i<windowsize;++i) {
    inputty[i]= 0.0f;
    //    last[i]=0.0f;
    lasted[i]=0.0f;
    lasteded[i]=0.0f;
    //    lastnew[i]=0.0;
  }

  for (i=0; i<P_MAX;++i) {
    coeff[i]= 0.0f;
  }
  G=0.0f; //gain starts at zero;
}

void LPCAnalyzer_init() {
  zeroAll();
}

// residual= sum += source[i-j]*coeff[j];
// IIR= source[i] -= source[i-j]*coeff[j];  	//		y[i] -= a[j] * y[i - j];

float predict(long order,long length,float *data,float *coeffs, float * errur)
{
    long i,j;
    float power=0.0,error,tmp;
    static float Zs[P_MAX] = {0.0};
//    short shortError;       //  Use this if want error to be soundfile

    for (i=0;i<length;i++)     {         //  0 to hopsize??????????
        tmp = 0.0;
	for (j=0;j<order;j++)  {
	  tmp += Zs[j]*coeffs[j];
	}
	for (j=order-1;j>0;j--) Zs[j] = Zs[j-1];
	Zs[0] = data[i];
        error = data[i] - tmp;
	errur[i]=error;
	//	printf("error: %f - data %f xx",tmp, data[i]); 
	//        fwrite(&error,4,1,resFile);
//        shortError = error;      //  Use these if want error to be soundfile
//        fwrite(&shortError,2,1,resFile);
	power += error * error;
    }
    return sqrt(power) / length;  
}

void calculateOutput(LPCfloat * source, LPCfloat * target, int startpos, int num) {
  u8 j; int i;
	int basepos,posnow;
//G=1.0; // TESTY!

	for(i=0; i<num; ++i) {

		basepos= startpos+i+windowsize-1; //-1 since coefficients for previous values starts here
		LPCfloat sum=0.0;
		for(j=0; j<P_MAX; ++j) {
		  posnow= (basepos-j)%windowsize;
		  sum += lasted[posnow]*coeff[j]; 
		  //		  printf("%f ",coeff[j]); 
		}

		sum= G*source[i]-sum; //scale factor G calculated by squaring energy E below

		//		sum= source[i]-sum; //scale factor G calculated by squaring energy E below TODO - if we use this from coeffs
		lasted[startpos+i]=sum;
		target[i]= sum;//*window32[i];
		//target[i]= source[i];
	}
}

void calculateOutput2(LPCfloat * source, LPCfloat * target, int startpos, int num) {
  u8 j; int i;
	int basepos,posnow;
//G=1.0; // TESTY!

	for(i=0; i<num; ++i) {

		basepos= startpos+i+windowsize-1; //-1 since coefficients for previous values starts here
		LPCfloat sum=0.0;
		for(j=0; j<P_MAX; ++j) {
		  posnow= (basepos-j)%windowsize;
		  sum += lasteded[posnow]*coeff[j]; 
		  //		  printf("%f ",coeff[j]); 
		}

		sum= G*source[i]-sum; //scale factor G calculated by squaring energy E below

		//		sum= source[i]-sum; //scale factor G calculated by squaring energy E below TODO - if we use this from coeffs
		lasteded[startpos+i]=sum;
		target[i]= sum;//*window32[i];
		//target[i]= source[i];
	}
}


void LPC_cross(LPCfloat * newinput, LPCfloat *newsource, LPCfloat * output, int numSamples) {

  // cross newinput as LPC analysis with newsource as residual...
  int i;
  int left= windowsize-pos;

  // test with
   do_impulse(newsource, numSamples, 1000);

   
	if(numSamples>=left) {
		lpc_preemphasis(newinput,numSamples,0.47);

		for (i=0; i<left;++i) {
		  float temp= newinput[i]*window32[pos]; //where are we in window 
		  inputty[pos++]=temp;
		  newinput[i]=temp;
		  //		  output[i]=temp;
		}
		calculateDurbPoles(); // this calculates the coeffs so... - these all give same results
		pos=0;
		predict(P_MAX,left,newinput,coeff,newsource); // this gives the error signal into newsource
		//		zeroAll();
		int remainder= numSamples-left;

			for (i=0; i<remainder;++i) {
			  float temp= newinput[left+i]*window32[pos]; //where are we in window 
			  inputty[pos++]=temp;
			  newinput[i]=temp;
		}
			calculateOutput(newsource, output+left, pos-remainder, remainder);
			printf("remain %d pos %d \n",remainder, pos); 
	} else {
		lpc_preemphasis(newinput,numSamples,0.97);
		for (i=0; i<numSamples;++i) {
		  //			inputty[pos++]= newinput[i];
		  float temp= newinput[i]*window32[pos]; //where are we in window 
		  inputty[pos++]=temp;
		  newinput[i]=temp;
		  //		  output[i]=temp;
		}
		predict(P_MAX,left,newinput,coeff,newsource); // this gives the error signal into newsource
		calculateOutput(newsource, output, pos-numSamples, numSamples);
		printf("pos %d \n",pos); 
	}
}

// this one seems to work in reconstructing sample

void LPC_samecross(LPCfloat * newinput, LPCfloat *newsource, LPCfloat * output, int numSamples) {

  // calc residual for 32 samples and apply coeff filter from same...
  int i; pos=0;
  //    do_impulse(newsource, numSamples, 1500.0f); // source= Impulse.ar(delaytime.reciprocal); 1/1024 or 1/32 buffersize - this is working

  //  do_noise(newsource, numSamples);
  
     /*

var delaytime= 32.0/SampleRate.ir; 32/48000

source= Impulse.ar(delaytime.reciprocal);  = 1500 for 48k samplerare

    */
  
  //  lpc_preemphasis(newinput,numSamples,0.97);

  for (i=0; i<numSamples;++i) {
    //float temp= newinput[i]*window32[pos]; //where are we in window
    float temp= newinput[i];//*0.5f;//*window32[pos]; //where are we in window 
    inputty[pos++]=temp;
	//newinput[i]=temp;
    //    output[i]=temp;
  }
  do_delay(inputty, newinput, CHUNKSIZE, numSamples);
  predict(P_MAX,numSamples,newinput,coeff,newsource); // this gives the error signal into newsource
    // but we want the last newsource 
  calculateOutput(newsource, output, 0, numSamples); // apply LPC filter from coeffs to newsource and we reconstruct the signal

  /*
//(DelayN.ar(input,delaytime, delaytime)- LPCAnalyzer.ar(input,source,1024,MouseX.kr(1,256))).poll(10000) // residual is delayed input minus LPC filtered input 
// try this one
  for (i=0; i<numSamples;++i) {
    tmp[i]=inputty[i]-output[i]; // calculate residual = input - filtered
    //output[i]=inputty[i];
  }
  calculateOutput2(tmp, output, 0, numSamples); // apply LPC filter from coeffs
  //  calculateDurbPoles(); // this calculates the coeffs so... - these all give same results - in sc this is last
  */
  SCcalculatePoles();
}




void LPC_residual(LPCfloat * newinput, LPCfloat * output, int numSamples) { // error signal into out

  int i;
  int left= windowsize-pos;

	if(numSamples>=left) {
		lpc_preemphasis(newinput,numSamples,0.97);

		for (i=0; i<left;++i) {
		  float temp= newinput[i]*window128[pos]; //where are we in window 
		  inputty[pos++]=temp;
		  newinput[i]=temp;
		}
		calculateDurbPoles(); // this calculates the coeffs so... - these all give same results
		pos=0;
		predict(P_MAX,left,newinput,coeff,output); // this gives the error signal into newsource
		int remainder= numSamples-left;

			for (i=0; i<remainder;++i) {
			  //	inputty[pos++]= newinput[left+i];
			  float temp= newinput[left+i]*window128[pos]; //where are we in window 
		  inputty[pos++]=temp;
		  newinput[i]=temp;
		}
			//	calculateresOutput(newinput, output+left, pos-remainder, remainder);
			//			calculateOutput(tt, newsource+left, pos-remainder, remainder);
			predict(P_MAX,remainder,newinput,coeff,output+left); // this gives the error signal into output
	} else {
		lpc_preemphasis(newinput,numSamples,0.97);
		for (i=0; i<numSamples;++i) {
		  //			inputty[pos++]= newinput[i];
		  float temp= newinput[i]*window128[pos]; //where are we in window 
		  inputty[pos++]=temp;
		  newinput[i]=temp;
		}
		predict(P_MAX,numSamples,newinput,coeff,output); // this gives the error signal into newsource
		//		calculateOutput(tt, newsource+left, pos-remainder, remainder);
		//		calculateresOutput(newinput, output+left, pos-remainder, remainder);
	}
}

void LPCAnalysis_update(LPCfloat * newinput, LPCfloat *newsource, LPCfloat * output, int numSamples, int p) {

  int i; float tt[128];
	int left= windowsize-pos;
	do_impulse(tt, numSamples, 200);

	if(numSamples>=left) {
	  //		lpc_preemphasis(newinput,numSamples,0.97);

		for (i=0; i<left;++i) {
		  float temp= newinput[i];//*window128[pos]; //where are we in window 
		  inputty[pos++]=temp;
		  newinput[i]=temp;
		}
		calculateDurbPoles(); // this calculates the coeffs so... - these all give same results
//		calculatePraatPoles(); // this calculates the coeffs so...
//		calculatePoles(); // this calculates the coeffs so... - which one is fastest? timing?
		pos=0;
		predict(P_MAX,left,newinput,coeff,newsource); // this gives the error signal into newsource
		int remainder= numSamples-left;

			for (i=0; i<remainder;++i) {
			  //	inputty[pos++]= newinput[left+i];
			  float temp= newinput[left+i]*window128[pos]; //where are we in window 
		  inputty[pos++]=temp;
		  newinput[i]=temp;
		}
			//		calculateresOutput(newinput, output+left, pos-remainder, remainder);
			calculateOutput(tt, newsource+left, pos-remainder, remainder);
			predict(P_MAX,remainder,newinput,coeff,newsource+left); // this gives the error signal into newsource
	} else {
		lpc_preemphasis(newinput,numSamples,0.97);
		for (i=0; i<numSamples;++i) {
		  //			inputty[pos++]= newinput[i];
		  float temp= newinput[i]*window128[pos]; //where are we in window 
		  inputty[pos++]=temp;
		  newinput[i]=temp;
		}
		///		calculateresOutput(newinput, output, pos-numSamples, numSamples);
		calculateOutput(tt, newsource, pos-numSamples, numSamples);
		predict(P_MAX,numSamples,newinput,coeff,newsource); // this gives the error signal into newsource
	}

}

void lpctimer(LPCfloat *in){
  int i;
  for (i=0; i<128;++i) {
    inputty[i]=in[i];
  }

  for (i=0;i<10000;i++){
    //		calculateDurbPoles(); // this calculates the coeffs so... - these all give same results
    //    calculatePraatPoles(); // this calculates the coeffs so...
//		calculatePoles(); // this calculates the coeffs so... - which one is fastest? timing?
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

	// for guassian window!

	double imid = 0.5 * (32 + 1), edge = exp (-12.0);
	for (long i = 1; i <= 32; i++) {
	  float xx = (exp (-48.0 * (i - imid) * (i - imid) / (32 + 1) / (32 + 1)) - edge) / (1 - edge);
	  //	  printf("%f, ",xx);
	}


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

	LPCAnalyzer_init();

	// read in 32 samples and print coeffs to test
	//	float buf [BLOCK_SIZE] ;
	int k, m, readcount,count=0;
	static float input[1280]; 
	static float output[1280];

	while ((readcount = sf_readf_float (infile, input,CHUNKSIZE)) > 0)
	//			while ((readcount = sf_readf_double (infile, input, BLOCK_SIZE)) > 0)
	{	
	  // 	  LPCAnalysis_update(input, pout, output, CHUNKSIZE, P_MAX);//
	  LPC_samecross(input, pout, output, CHUNKSIZE);
	  //	  printf("SAMPLE %f", input[16]);

	  //	  LPC_residual(input,output, CHUNKSIZE);
	  sf_writef_float (outfile, output, readcount) ;
	  //		sf_writef_double (outfile, output, readcount) ;
		count++;
		} ;

	sf_close (infile) ;
	sf_close (outfile) ;
}


