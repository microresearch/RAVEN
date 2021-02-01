// license:GPL-2.0+
// copyright-holders: Martin Howse

#include "stdio.h"
#include <math.h>
#include "main.h"
#include "wavetable.h"

Wavetable wavtable;
//Wavetable *wavetable; we have it


#define TWO_PI 6.28318530717958647693

#define OVERSAMPLING_OSCILLATOR 0

inline int iincrement(int pointer, int modulus)
{
    if (++pointer >= modulus)
	return 0;

    return pointer;
}

inline int ddecrement(int pointer, int modulus)
{
    if (--pointer < 0)
return modulus - 1;

    return pointer;
}

inline static float mod0(float value, int16_t length)
{
    while (value > length-1)
        value -= length;
    return value;
}

// Increments the position in the wavetable according to the desired frequency. WORMY THIS
inline static void WavetableIncrementPosition(Wavetable *wavetable, float frequency)
{
wavetable->currentPosition = mod0(wavetable->currentPosition + (frequency * wavetable->basicIncrement), wavetable->length);
}

void dowavetable(float* outgoing, Wavetable *wavetable, float frequency, u8 length)  //  Plain oscillator
{
    int lowerPosition, upperPosition;
    for (u8 ii = 0; ii < length; ii++) {

    //  First increment the table position, depending on frequency
    WavetableIncrementPosition(wavetable, frequency);

    //  Find surrounding integer table positions
    lowerPosition = (int)wavetable->currentPosition;
    upperPosition = mod0(lowerPosition + 1, wavetable->length);

    //  Return interpolated table value
    float sample= (wavetable->wavetable[lowerPosition] +
            ((wavetable->currentPosition - lowerPosition) *
             (wavetable->wavetable[upperPosition] - wavetable->wavetable[lowerPosition])));

    outgoing[ii]=sample;
    }
}



void wavetable_init(Wavetable* wavtable, const float *tableitself, int16_t length){ // need to declare wavetable struct and ourtable we use
  wavtable->wavetable=tableitself;
  wavtable->basicIncrement=(float)length/32000.0f; // 32000 for real thing - test lap is 8000
  wavtable->currentPosition=0.0;
  wavtable->length=length;
}

