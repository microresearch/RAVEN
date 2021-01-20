void Vocoder_Init(float sample_rate);

void Vocoder_Process(
    float* modulator,
    float* carrier,
    float* out,
    u8 size);
