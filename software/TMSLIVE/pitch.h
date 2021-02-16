double pitchForPeriod(float *buffer, int16_t size, int16_t minimumPeriod, int16_t maximumPeriod);
void Hamming(float *buffer, int16_t size);
void getCoefficientsFor(float *buffer, int16_t size);
double translateCoefficients(uint16_t size);
uint16_t ClosestValueFinder(float actual, float* table, uint16_t size);
