enum FilterMode {
  FILTER_MODE_LOW_PASS,
  FILTER_MODE_BAND_PASS,
  FILTER_MODE_BAND_PASS_NORMALIZED,
  FILTER_MODE_HIGH_PASS
};

typedef struct SVF{
  float f_;
  float fq_;
  float x_[2];
  float lp_[2];
  float bp_[2];
} SVF;


// functions also
void SVF_Reset(SVF* svf);
void SVF_Init(SVF* svf);
void SVF_Init_();
void BANDS_Init_();
void runBANDStest_(float* incoming, float* outgoing, u8 band_size);
void runSVFtest_(float* incoming, float* outgoing, u8 band_size);
void set_f_fq(SVF* svf, float f, float fq);
void SVF_Process(SVF* svf, const float* in, float* out, u8 size, u8 mode);
