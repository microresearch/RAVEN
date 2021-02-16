#include "forlap.h"
#include "math.h"
#include "CodingTable.h"

#define YES 1
#define NO 0
#define false 0
#define M_PI 3.14159265358979323846264338327950288

const double window[200]={0.08000000000000002, 0.08022926900196004, 0.08091684746751304, 0.08206205000349126, 0.08366373504710145, 0.0857203060038591, 0.08822971283910902, 0.09118945412154417, 0.09459657951668671, 0.09844769272784581, 0.10273895488161988, 0.10746608835456922, 0.11262438103724398, 0.11820869103131715, 0.1242134517751412, 0.13063267759261704, 0.13745996965984691, 0.1446885223836205, 0.15231113018537867, 0.16032019468389097, 0.16870773226948677, 0.1774653820622915, 0.18658441424653344, 0.19605573877261379, 0.2058699144182659, 0.21601715819977085, 0.2264873551238485, 0.23727006827050268, 0.24835454919677025, 0.25972974865100235, 0.27138432758699915, 0.2833066684670176, 0.2954848868423854, 0.30790684320017836, 0.3205601550641508, 0.3334322093378568, 0.3465101748776592, 0.35978101528309314, 0.37323150189183274, 0.3868482269663098, 0.4006176170588386, 0.41452594654192254, 0.4285593512902598, 0.44270384250080325, 0.45694532063710497, 0.47126958948404, 0.48566237029890236, 0.5001093160447667, 0.5145960256919263, 0.5291080585731529, 0.5436309487784682, 0.5581502195750763, 0.5726513978380874, 0.5871200284776421, 0.6015416888480604, 0.6159020031246477, 0.6301866566338299, 0.6443814101223314, 0.6584721139511726, 0.6724447222003366, 0.6862853066700496, 0.6999800707647119, 0.713515363245645, 0.726877691838943, 0.7400537366848648, 0.75303036361536, 0.7657946372464931, 0.7783338338727156, 0.7906354541501325, 0.8026872355561196, 0.8144771646128738, 0.8259934888627078, 0.8372247285831564, 0.8481596882302134, 0.8587874675982927, 0.8690974726857907, 0.8790794262554182, 0.8887233780787729, 0.8980197148549437, 0.9069591697932589, 0.9155328318506236, 0.9237321546142421, 0.9315489648208675, 0.9389754705040889, 0.9460042687615317, 0.9526283531342332, 0.9588411205908327, 0.9646363781096159, 0.9700083488518552, 0.9749516779202869, 0.9794614376969897, 0.9835331327553414, 0.9871627043411583, 0.9903465344185487, 0.9930814492764508, 0.9953647226922564, 0.9971940786493685, 0.9985676936059849, 0.9994841983128441, 0.9999426791781224, 0.9999426791781224, 0.9994841983128441, 0.9985676936059849, 0.9971940786493685, 0.9953647226922564, 0.9930814492764508, 0.9903465344185487, 0.9871627043411583, 0.9835331327553415, 0.9794614376969897, 0.9749516779202869, 0.9700083488518553, 0.964636378109616, 0.9588411205908327, 0.9526283531342333, 0.9460042687615315, 0.9389754705040888, 0.9315489648208676, 0.9237321546142421, 0.9155328318506237, 0.9069591697932591, 0.8980197148549438, 0.8887233780787729, 0.8790794262554185, 0.869097472685791, 0.8587874675982927, 0.8481596882302134, 0.8372247285831567, 0.8259934888627078, 0.814477164612874, 0.8026872355561196, 0.7906354541501324, 0.7783338338727158, 0.7657946372464932, 0.7530303636153602, 0.740053736684865, 0.7268776918389429, 0.7135153632456451, 0.6999800707647119, 0.6862853066700498, 0.6724447222003367, 0.6584721139511729, 0.6443814101223316, 0.6301866566338299, 0.6159020031246479, 0.6015416888480605, 0.5871200284776424, 0.5726513978380875, 0.5581502195750763, 0.5436309487784684, 0.529108058573153, 0.5145960256919266, 0.5001093160447668, 0.48566237029890236, 0.4712695894840402, 0.4569453206371051, 0.4427038425008036, 0.4285593512902599, 0.41452594654192254, 0.4006176170588388, 0.38684822696630994, 0.3732315018918331, 0.35978101528309325, 0.34651017487765956, 0.3334322093378569, 0.3205601550641512, 0.30790684320017825, 0.2954848868423855, 0.2833066684670175, 0.2713843275869993, 0.25972974865100235, 0.24835454919677047, 0.2372700682705028, 0.2264873551238487, 0.2160171581997709, 0.2058699144182662, 0.196055738772614, 0.18658441424653355, 0.17746538206229145, 0.16870773226948688, 0.16032019468389097, 0.15231113018537884, 0.14468852238362057, 0.13745996965984708, 0.13063267759261715, 0.12421345177514137, 0.11820869103131726, 0.11262438103724404, 0.10746608835456922, 0.10273895488161994, 0.09844769272784581, 0.09459657951668676, 0.09118945412154417, 0.08822971283910908, 0.08572030600385916, 0.08366373504710145, 0.08206205000349132, 0.0809168474675131, 0.08022926900196004, 0.08000000000000002};

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

uint16_t ClosestValueFinder(float actual, float* table, uint16_t size){

  if (actual < table[0]) return 0;
    
    for (int i = 1; i < size; i++) {
        if (table[i] > actual) {
            float previous = table[i - 1];
	    //	    printf("p: %f ", previous);
            if (table[i] - actual < actual - previous) {
                return i;
            } else {
                return i - 1;
            }
        }
    }
    return size - 1;
}


double formattedRMS(double rms, uint16_t size){
    return sqrtf(rms / size) * (1 << 15);
}

double translateCoefficients(uint16_t size){

    // Leroux Guegen algorithm for finding K's

    double k[11] = {0};
    double b[11] = {0};
    double d[12] = {0};
    
    
    k[1] = -coefficientss[1] / coefficientss[0];
    d[1] = coefficientss[1];
    d[2] = coefficientss[0] + (k[1] * coefficientss[1]);
    
    int i = 2;
    while (i <= 10) {
        double y = coefficientss[i];
        b[1] = y;
    
        int j = 1;
        while (j <= i - 1) {
            b[j + 1] = d[j] + (k[j] * y);
            y = y + (k[j] * d[j]);
            d[j] = b[j];
            j += 1;
        }
    
        k[i] = -y / d[i];
	//	printf("%f ", k[i]);
        d[i + 1] = d[i] + (k[i] * y);
        d[i] = b[i];
        i += 1;
    }
    
    double rms = formattedRMS(d[11], size);
    //    return [[Reflector alloc] initWithKs:k rms:rms limitRMS:YES];
      for (uint16_t i=0;i<11;i++){
	_ks[i]=k[i];
      }
      _rms=rms; // question of RMS and rmss function?
      return rms;
}


double rmss (void) {
    if (shouldLimitRMS && _rms >= rmstable[kStopFrameIndex - 1]) {
        return rmstable[kStopFrameIndex - 1];
    } else {
        return _rms;
    }
}

void Hamming(float *buffer, int16_t size) {
    for (uint16_t i=0; i < size; i++) {
      //      double window = 0.54f - 0.46f * (double long)cos(2 * M_PI * i / (size - 1)); //  precomputed window for our size ==200
        buffer[i] *= window[i];
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

void lpc_preemphasis(float* x, int len, float alpha)
{
  double pre_energy=sumOfSquaresFor(x,len);
  for( int i = len - 1; i > 0; i-- )      x[i] = x[i] - alpha * x[i-1];////y[k]=x[k]-0.95x[k-1] - just as but alpha is negative in  pywiz and co.
  double post_energy=sumOfSquaresFor(x,len);
  double scale=sqrt(pre_energy / post_energy);
  for( int i = len - 1; i > 0; i-- )      x[i] *= scale;
  // in wiz there is also scaling:
  /*
  +(void)scaleBuffer:(Buffer *)buffer preEnergy:(double)preEnergy postEnergy:(double)postEnergy {
    double scale = sqrt(preEnergy / postEnergy);
    
    for (int i = 0; i < buffer.size; i++) {
        buffer.samples[i] *= scale;
    }
 
 // postEnergy is sumofsquaresfor - what is pre is before
  */ 
}


void getCoefficientsFor(float *buffer, int16_t size){
  //        coefficients = [0]*11
  for (uint16_t i=0;i<11;i++){
    coefficientss[i] = aForLag(buffer,size,i);
    //    printf("%f ", coefficientss[i]);
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

