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


