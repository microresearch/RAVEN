#ifndef MAIN_H
#define MAIN_H

//#define IQ_SAMPLE_RATE (48000)
#define AUDIO_SAMPLE_RATE (48000)

// a lot of code pieces assume that this frequency
// is 1500 Hz, so don't change
#define IQ_INTERRUPT_FREQ (1500)

// we process one dma block of samples at once
// block sizes should be a power of two
// a lot of code process information in these blocks
//#define IQ_BLOCK_SIZE (IQ_SAMPLE_RATE/IQ_INTERRUPT_FREQ)
#define AUDIO_BLOCK_SIZE (AUDIO_SAMPLE_RATE/IQ_INTERRUPT_FREQ)

// use for clocking based on DMA IRQ
#define SAMPLES_PER_DMA_CYCLE   (IQ_BLOCK_SIZE)
#define SAMPLES_PER_CENTISECOND (IQ_SAMPLE_RATE/100)

typedef int16_t audio_data_t;

typedef struct {
    __packed audio_data_t l;
    __packed audio_data_t r;
} AudioSample_t;

#endif  /* _MAIN */
