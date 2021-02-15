double pitchForPeriod(float *buffer, int16_t size, int16_t minimumPeriod, int16_t maximumPeriod);
void Hamming(float *buffer, int16_t size);
void getCoefficientsFor(float *buffer, int16_t size);
void translateCoefficients(uint16_t size);
