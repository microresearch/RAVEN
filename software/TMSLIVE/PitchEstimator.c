#include "forlap.h"
#include "math.h"
#include "CodingTable.h"

#define YES 1
#define NO 0
#define false 0
#define M_PI 3.14159265358979323846264338327950288



double coefficientss[11]={0.0f};
double normalizedCoefficients[400]={0.0f};
double subMultipleThreshold=0.9f;
static uint16_t _bestPeriod=255;
double sumOfSquaresFor(float *buffer, int16_t size);
double aForLag(float *buffer, int16_t size, int16_t lag);
uint16_t  bestPeriodd(uint16_t minimumPeriod, uint16_t maximumPeriod);
double estimatee(int16_t minimumPeriod, int16_t maximumPeriod);
static const uint16_t kNumberOfKParameters = 11;
double _ks[11];
double _rms;

uint16_t shouldLimitRMS=0;
uint16_t kStopFrameIndex=15;

double formattedRMS(double rms, uint16_t size){
    return sqrtf(rms / size) * (1 << 15);
}

void translateCoefficients(uint16_t size){

    // Leroux Guegen algorithm for finding K's

    double k[11] = {0};
    double b[11] = {0};
    double d[12] = {0};
    
    
    k[1] = -normalizedCoefficients[1] / normalizedCoefficients[0];
    d[1] = normalizedCoefficients[1];
    d[2] = normalizedCoefficients[0] + (k[1] * normalizedCoefficients[1]);
    
    int i = 2;
    while (i <= 10) {
        double y = normalizedCoefficients[i];
        b[1] = y;
    
        int j = 1;
        while (j <= i - 1) {
            b[j + 1] = d[j] + (k[j] * y);
            y = y + (k[j] * d[j]);
            d[j] = b[j];
            j += 1;
        }
    
        k[i] = -y / d[i];
        d[i + 1] = d[i] + (k[i] * y);
        d[i] = b[i];
        i += 1;
    }
    
    double rms = formattedRMS(d[11], size);
    //    return [[Reflector alloc] initWithKs:k rms:rms limitRMS:YES];
      for (uint16_t i=0;i<11;i++){
	_ks[i]=k[i];
      }
}


double rmss (void) {
    if (shouldLimitRMS && _rms >= rms[kStopFrameIndex - 1]) {
        return rms[kStopFrameIndex - 1];
    } else {
        return _rms;
    }
}

void Hamming(float *buffer, int16_t size) {
    for (uint16_t i=0; i < size; i++) {
      double window = 0.54f - 0.46f * (double long)cos(2 * M_PI * i / (size - 1)); // TODO: precompute window for our size!
        buffer[i] *= window;
    }
}

double energy(float *buffer, int16_t size){
  return sumOfSquaresFor(buffer, size);  
	  }

double sumOfSquaresFor(float *buffer, int16_t size){
  //        return sp.square(self.samples[self.start:self.end]).sum()
  double sum=0.0f;
  for (uint16_t i=0;i<size;i++){
    sum+=buffer[i]*buffer[i];
  }
  return sum;
}

void getCoefficientsFor(float *buffer, int16_t size){
  //        coefficients = [0]*11
  for (uint16_t i=0;i<11;i++){
    coefficientss[i] = aForLag(buffer,size,i);
      }
}


double aForLag(float *buffer, int16_t size, int16_t lag){
  int16_t samples = size - lag;
  double sum=0.0f;

	  //        return sum(self.samples[0:samples] * self.samples[lag:samples+lag])
  for (uint16_t i=0;i<samples;i++){
    sum+=(buffer[i]*buffer[i+lag]);	      
  }
      return sum;
	  }
	  
  /*  double rms(self, x){
        return sp.sqrt(x.dot(x)/x.size)
  */

  void getNormalizedCoefficientsFor(float *buffer, int16_t size, int16_t minimumPeriod, int16_t maximumPeriod){ // min is 8000/500, max is 8000/50

        for (uint16_t lag = 0; lag <= maximumPeriod; lag++) {
        if (lag < minimumPeriod) {
            normalizedCoefficients[lag] = 0.0f;
            continue;
        }

        double sumOfSquaresBeginning = 0.0;
        double sumOfSquaresEnding = 0.0;

        double sum = 0.0;
        int16_t samples = size - lag; // what if maxperiod=160 is bigger than size - check this - chunksize is 400
        for (int16_t i = 0; i < samples; i++) {
            sum += buffer[i] * buffer[i + lag];
            sumOfSquaresBeginning += buffer[i] * buffer[i];
            sumOfSquaresEnding    += buffer[i + lag] * buffer[i + lag];
        }
        
        normalizedCoefficients[lag] = sum / sqrtf(sumOfSquaresBeginning * sumOfSquaresEnding);
    }
}

  // pitch est
  
  
double pitchForPeriod(float *buffer, int16_t size, int16_t minimumPeriod, int16_t maximumPeriod){ // min is 8000/500, max is 8000/50
      //    return [[[self alloc] initWithBuffer:buffer] estimate];
  _bestPeriod=255;
  getNormalizedCoefficientsFor(buffer, size, minimumPeriod, maximumPeriod);     
  return estimatee(minimumPeriod, maximumPeriod);
      //      return 0.0;
}

/*
-(instancetype)initWithBuffer:(Buffer *)buffer {
    if (self = [super init]) {
        _buffer = buffer;
        _normalizedCoefficients = [self getNormalizedCoefficients];
    }
    return self;
}
 
  
-(void)dealloc {
    free(self.normalizedCoefficients);
}
  */

  u8 isOutOfRange(int16_t minimumPeriod, int16_t maximumPeriod) {
    uint16_t bestPeriod = bestPeriodd(minimumPeriod, maximumPeriod);
    return normalizedCoefficients[bestPeriod] < normalizedCoefficients[bestPeriod - 1] &&
           normalizedCoefficients[bestPeriod] < normalizedCoefficients[bestPeriod + 1];
}

  double interpolated(int16_t minimumPeriod, int16_t maximumPeriod) {
  uint16_t bestPeriod = bestPeriodd(minimumPeriod, maximumPeriod);
    double middle = normalizedCoefficients[bestPeriod];
    double left   = normalizedCoefficients[bestPeriod - 1];
    double right  = normalizedCoefficients[bestPeriod + 1];
    
    if (!(2 * middle - left - right))
        return bestPeriod;
    else
        return bestPeriod + 0.5 * (right - left) / (2 * middle - left - right);
}

  double estimatee(int16_t minimumPeriod, int16_t maximumPeriod) {
    uint16_t bestPeriod = bestPeriodd(minimumPeriod, maximumPeriod);
    uint16_t maximumMultiple = bestPeriod / minimumPeriod;
    
    u8 found = false;
    
    double estimate = interpolated(minimumPeriod, maximumPeriod);
    if (estimate != estimate) return 0.0;
        
    while (!found && maximumMultiple >= 1) {
        u8 subMultiplesAreStrong = YES;
        
        for (int i = 0; i < maximumMultiple; i++) {
            uint16_t subMultiplePeriod = floor((i + 1) * estimate / maximumMultiple + 0.5);
                
            if (normalizedCoefficients[subMultiplePeriod] &&
                normalizedCoefficients[subMultiplePeriod] < subMultipleThreshold * normalizedCoefficients[bestPeriod]) {
                subMultiplesAreStrong = NO;
            }
        }
                    
        if (subMultiplesAreStrong) {
            estimate /= maximumMultiple;
            found = YES;
        }
                
        maximumMultiple -= 1;
    }
                
    return estimate;
}
  /*
  getNormalizedCoefficients {
    uint16_t minimumPeriod = [self minimumPeriod] - 1;
    uint16_t maximumPeriod = [self maximumPeriod] + 1;
    
    double *normalizedCoefficients = malloc(sizeof(double) * maximumPeriod + 1);

    [Autocorrelator getNormalizedCoefficientsFor:normalizedCoefficients
                                       forBuffer:buffer
                                   minimumPeriod:minimumPeriod
                                   maximumPeriod:maximumPeriod];
    return normalizedCoefficients;
}
  */
  
  uint16_t  bestPeriodd(uint16_t minimumPeriod, uint16_t maximumPeriod ) {
        uint16_t bestPeriod = minimumPeriod;
        uint16_t startPeriod = bestPeriod + 1;
	//        uint16_t maximumPeriod = maximumPeriod;
	if (_bestPeriod==255){
        for (uint16_t period = startPeriod; period < maximumPeriod; period++) {
            if (normalizedCoefficients[period] > normalizedCoefficients[bestPeriod]) {
                bestPeriod = period;
            }
	    if (bestPeriod < minimumPeriod)                bestPeriod = minimumPeriod;
            if (bestPeriod > maximumPeriod)                bestPeriod = maximumPeriod;
	    
        }


	_bestPeriod = bestPeriod;
	}
	
	//	printf("best %d \n", _bestPeriod);

	return _bestPeriod;
    }

