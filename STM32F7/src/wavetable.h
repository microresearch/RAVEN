// license:GPL-2.0+
// copyright-holders: Martin Howse

#include "main.h"

typedef struct _Wavetable {
    const float *wavetable;
    float basicIncrement;
    float currentPosition;
  int16_t length;
} Wavetable;

void wavetable_init(Wavetable* wavtable, const float *tableitself, int16_t length); // need to declare wavetable struct and and ourtable we use
void dowavetable(float* outgoing, Wavetable *wavetable, float frequency, u8 length);
