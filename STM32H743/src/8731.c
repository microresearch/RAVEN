#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32h7xx_hal.h"
//#include "stm32h7xx_nucleo_144.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_pwr.h"
#include "stm32h7xx_hal_pwr_ex.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_rcc_ex.h"
#include "stm32h7xx_hal_i2s.h"
#include "stm32h7xx_hal_i2c.h"


#include <main.h>
#include "hw_i2s.h"
#include "hw_i2c.h"
#include "8731.h"
#include "arm_math.h"

// from https://github.com/df8oe/UHSDR/tree/active-devel/mchf-eclipse/drivers/audio/codec - for 8731 and f767 - ovi40 board

// set up controller (in main maybe maybe) to right speeds/PLL

// set up correct pins for 8731: from sai.c DONE

// again from OVI40 in sai.c:

    /**SAI1_B_Block_B GPIO Configuration    
    PF6     ------> SAI1_SD_B -> DACDAT = MOSI 
    PF7     ------> SAI1_MCLK_B -> MCLK on 8731 = 12.288MHz on board
    PF8     ------> SAI1_SCK_B -> BCLK = SCK on board
    PF9     ------> SAI1_FS_B -> DACLRC and ADCLRC = marked
    */

    /**SAI1_A_Block_A GPIO Configuration    
    PD6     ------> SAI1_SD_A -> ADCDAT = MISO
    */

/* 12c.c
  PB10     ------> I2C2_SCL -> SCLK 47R = SCL
  PB11     ------> I2C2_SDA -> SDIN = SDA
*/

// set up SAI and DMA - DONE

// i2s/i2c setup - BASIC DONE but see also: uhsdr_hw_i2s.c -> also for callbacks for input (two channels) and output

// set up of 8371 codec itself

// test throughput in audio.c // where do we init everything and what calls are these?

/*

        // Reg 04: Analog Audio Path Control (DAC sel, ADC line, Mute Mic)
        Codec_WriteRegister(CODEC_ANA_I2C, W8731_ANLG_AU_PATH_CNTR,
                W8731_ANLG_AU_PATH_CNTR_DACSEL|
                W8731_ANLG_AU_PATH_CNTR_INSEL_LINE|
                W8731_ANLG_AU_PATH_CNTR_MUTEMIC);

        // Reg 06: Power Down Control (Clk off, Osc off, Mic Off)
        // COMMENT:  It would be tempting to set bit 1 "MICPD" of "W8731_POWER_DOWN_CTR" to zero to disable mic power down
        // and maintain microphone bias during receive, but this seems to cause problems on receive (e.g. deafness) even
        // if the microphone is muted and "mic boost" is disabled.  (KA7OEI 20151030)

        Codec_WriteRegister(CODEC_ANA_I2C, W8731_POWER_DOWN_CNTR,W8731_POWER_DOWN_CNTR_MCHF_MIC_OFF);	// turn off mic bias

 */

/* /////////////////////////////////////////////////////////////////////////////////

Below from codec.c .h

 */

#define CODEC_ANA_I2C               (&hi2c2)
#define CODEC_ANA_SAI               SAI1


typedef enum
{
    WORD_SIZE_16 =	0,
    WORD_SIZE_24 =  2,
    WORD_SIZE_32 =  3,
} CodecSampleWidth_t;

#define CODEC_SPEAKER_MAX_VOLUME AUDIO_GAIN_MAX

// --------------------------------------------------



// I2C addresses
#define W8731_ADDR_0                    0x1A        // CS = 0, MODE to GND
#define W8731_ADDR_1                    0x1B        // CS = 1, MODE to GND

// The 7 bits Codec address (sent through I2C interface)
#define CODEC_ADDRESS                   (W8731_ADDR_0<<1) // this should be so

// Registers
#define W8731_LEFT_LINE_IN              0x00        // 0000000
#define W8731_RIGHT_LINE_IN             0x01        // 0000001
#define W8731_LEFT_HEADPH_OUT           0x02        // 0000010
#define W8731_RIGHT_HEADPH_OUT          0x03        // 0000011
#define W8731_ANLG_AU_PATH_CNTR         0x04        // 0000100
#define W8731_DIGI_AU_PATH_CNTR         0x05        // 0000101
#define W8731_POWER_DOWN_CNTR           0x06        // 0000110
#define W8731_DIGI_AU_INTF_FORMAT       0x07        // 0000111
#define W8731_SAMPLING_CNTR             0x08        // 0001000
#define W8731_ACTIVE_CNTR               0x09        // 0001001
#define W8731_RESET                     0x0F        // 0001111

// -------------------------------------------------

//#define W8731_DEEMPH_CNTR                 0x06        // WM8731 codec De-emphasis enabled
#define W8731_DEEMPH_CNTR               0x00        // WM8731 codec De-emphasis disabled


#define W8731_HEADPH_OUT_ZCEN     0x0080      // bit 7 W8731_LEFT_HEADPH_OUT / W8731_RIGHT_HEADPH_OUT
#define W8731_HEADPH_OUT_HPBOTH   0x0100      // bit 8 W8731_LEFT_HEADPH_OUT / W8731_RIGHT_HEADPH_OUT
#define W8731_LINE_IN_LRBOTH   0x0100      // bit 8 W8731_LEFT_LINE_IN_OUT / W8731_RIGHT_LINE_IN

#define W8731_ANLG_AU_PATH_CNTR_DACSEL      (0x10)
#define W8731_ANLG_AU_PATH_CNTR_INSEL_MIC       (0x04)
#define W8731_ANLG_AU_PATH_CNTR_INSEL_LINE       (0x00)
#define W8731_ANLG_AU_PATH_CNTR_MUTEMIC     (0x02)
#define W8731_ANLG_AU_PATH_CNTR_MICBBOOST   (0x01)
#define W8731_DIGI_AU_INTF_FORMAT_PHILIPS 0x02
#define W8731_DIGI_AU_INTF_FORMAT_PCM     0x00
#define W8731_DIGI_AU_INTF_FORMAT_16B     (0x0 << 2)
#define W8731_DIGI_AU_INTF_FORMAT_20B     (0x1 << 2)
#define W8731_DIGI_AU_INTF_FORMAT_24B     (0x2 << 2)
#define W8731_DIGI_AU_INTF_FORMAT_32B     (0x3 << 2)

#define W8731_DIGI_AU_INTF_FORMAT_I2S_PROTO W8731_DIGI_AU_INTF_FORMAT_PHILIPS

#define W8731_POWER_DOWN_CNTR_POWEROFF  (0x80)
#define W8731_POWER_DOWN_CNTR_CLKOUTPD  (0x40)
#define W8731_POWER_DOWN_CNTR_OSCPD     (0x20)
#define W8731_POWER_DOWN_CNTR_OUTPD     (0x10)
#define W8731_POWER_DOWN_CNTR_DACPD     (0x08)
#define W8731_POWER_DOWN_CNTR_ADCPD     (0x04)
#define W8731_POWER_DOWN_CNTR_MICPD     (0x02)
#define W8731_POWER_DOWN_CNTR_LINEPD    (0x01)

#define W8731_SAMPLING_CNTR_BOSR        (0x0002)

#define W8731_SAMPLING_CNTR_96K         (0x0007 << 2)
#define W8731_SAMPLING_CNTR_48K         (0x0000 << 2)
#define W8731_SAMPLING_CNTR_32K         (0x0006 << 2)
#define W8731_SAMPLING_CNTR_8K          (0x0003 << 2)


#define W8731_VOL_MAX 0x50

#define W8731_POWER_DOWN_CNTR_MCHF_ALL_ON    (W8731_POWER_DOWN_CNTR_CLKOUTPD|W8731_POWER_DOWN_CNTR_OSCPD)
// all on but osc and out, since we don't need it, clock comes from STM

//#define W8731_POWER_DOWN_CNTR_MCHF_MIC_OFF    (W8731_POWER_DOWN_CNTR_CLKOUTPD|W8731_POWER_DOWN_CNTR_OSCPD|W8731_POWER_DOWN_CNTR_MICPD
#define W8731_POWER_DOWN_CNTR_MCHF_MIC_OFF    (W8731_POWER_DOWN_CNTR_CLKOUTPD|W8731_POWER_DOWN_CNTR_OSCPD)
typedef struct
{
    bool present;
} mchf_codec_t;


__IO mchf_codec_t mchf_codecs[1];

// FIXME: for now we use 32bits transfer size, does not change the ADC/DAC resolution
// which is 24 bits in any case. We should reduce finally to 24bits (which requires also the I2S/SAI peripheral to
// use 24bits)
//#define USE_32_AUDIO_BITS


#if defined(USE_32_AUDIO_BITS)
    #define AUDIO_WORD_SIZE WORD_SIZE_32
#else
    #define AUDIO_WORD_SIZE WORD_SIZE_16
#endif

/**
 * @brief writes 16 bit data word to codec register
 * @returns I2C error code
 */
static uint32_t Codec_WriteRegister(I2C_HandleTypeDef* hi2c, uint8_t RegisterAddr, uint16_t RegisterValue)
{
    // Assemble 2-byte data in WM8731 format
    uint8_t Byte1 = ((RegisterAddr<<1)&0xFE) | ((RegisterValue>>8)&0x01);
    uint8_t Byte2 = RegisterValue&0xFF;
    return UhsdrHw_I2C_WriteRegister(hi2c, CODEC_ADDRESS, Byte1, 1, Byte2);
}

static uint32_t Codec_ResetCodec(I2C_HandleTypeDef* hi2c, uint32_t AudioFreq, CodecSampleWidth_t word_size)
{
    uint32_t retval = HAL_OK;

    retval = Codec_WriteRegister(hi2c, W8731_RESET, 0);
    // Reset register
    if( retval == HAL_OK)
    {
        // Reg 00: Left Line In (0dB, mute off)
      Codec_WriteRegister(hi2c, W8731_LEFT_LINE_IN,0x001F);

        // Reg 01: Right Line In (0dB, mute off)
      Codec_WriteRegister(hi2c, W8731_RIGHT_LINE_IN,0x001F);

        // Reg 02: Left Headphone out (0dB)
        //Codec_WriteRegister(0x02,0x0079);
        // Reg 03: Right Headphone out (0dB)
        //Codec_WriteRegister(0x03,0x0079);


        // Reg 04: Analog Audio Path Control (DAC sel, ADC line, Mute Mic)
	        Codec_WriteRegister(hi2c, W8731_ANLG_AU_PATH_CNTR,
				    W8731_ANLG_AU_PATH_CNTR_DACSEL |
				    //				    W8731_ANLG_AU_PATH_CNTR_INSEL_LINE) |
				    W8731_ANLG_AU_PATH_CNTR_INSEL_MIC |
				    W8731_ANLG_AU_PATH_CNTR_MICBBOOST); // so now we have mic in from board

		//Codec_WriteRegister(hi2c, W8731_ANLG_AU_PATH_CNTR,0x08); // try bypass
	
        // Reg 05: Digital Audio Path Control(all filters disabled)
        // De-emphasis control, bx11x - 48kHz
        //                      bx00x - off
        // DAC soft mute        b1xxx - mute on
        //                      b0xxx - mute off
        //
		Codec_WriteRegister(hi2c, W8731_DIGI_AU_PATH_CNTR,W8731_DEEMPH_CNTR);

        // Reg 06: Power Down Control (Clk off, Osc off, Mic off)) 
		Codec_WriteRegister(hi2c, W8731_POWER_DOWN_CNTR,W8731_POWER_DOWN_CNTR_MCHF_MIC_OFF);


        // Reg 07: Digital Audio Interface Format (i2s, 16/32 bit, slave)
        uint16_t size_reg_val;

        switch(word_size)
        {
            case WORD_SIZE_32:
                size_reg_val = W8731_DIGI_AU_INTF_FORMAT_32B;
                break;
            case WORD_SIZE_24:
                size_reg_val = W8731_DIGI_AU_INTF_FORMAT_24B;
                break;
            case WORD_SIZE_16:
            default:
                size_reg_val = W8731_DIGI_AU_INTF_FORMAT_16B;
                break;
        }

	Codec_WriteRegister(hi2c, W8731_DIGI_AU_INTF_FORMAT,W8731_DIGI_AU_INTF_FORMAT_I2S_PROTO|size_reg_val); // reg 07 MASTER 0x40
	//      Codec_WriteRegister(hi2c, W8731_DIGI_AU_INTF_FORMAT,0x43);

        // Reg 08: Sampling Control (Normal, 256x, 48k ADC/DAC)
        // master clock: 12.288 Mhz
        uint16_t samp_reg_val;
	//	AudioFreq=96000;
        switch (AudioFreq)
        {
        case 32000:
            samp_reg_val = W8731_SAMPLING_CNTR_32K;
            break;
        case 8000:
            samp_reg_val = W8731_SAMPLING_CNTR_8K;
            break;
        case 96000:
            samp_reg_val = W8731_SAMPLING_CNTR_96K;
            break;
        case 48000:
        default:
            samp_reg_val = W8731_SAMPLING_CNTR_48K;
            break;
        }
	//	samp_reg_val=0x10 << 2;
	Codec_WriteRegister(hi2c, W8731_SAMPLING_CNTR,samp_reg_val); // doesn't seem to work for any vals so we need do clock-> DONE
	// from WARPS:
	// According to the WM8731 datasheet, the 32kHz and 96kHz modes require the
	// master clock to be at 12.288 MHz (384 fs / 128 fs). The STM32F4 I2S clock
	// is always at 256 fs. So the 32kHz and 96kHz modes are achieved by
	// pretending that we are doing 48kHz, but with a slower or faster master
	// clock. //2/3 slower for 32K?
	// 8.192 MHz?
	// now fixed in main.c

	
	//	Codec_WriteRegister(hi2c, 5, 0b00100);      // DAC unmuted - from other code
	//	Codec_WriteRegister(hi2c, 4, 0b00010000);    // DAC selected

	
        // Reg 09: Active Control
        // and now we start the Codec Digital Interface
		Codec_WriteRegister(hi2c, W8731_ACTIVE_CNTR,0x0001);
    }
    return retval;

}

/**
 * @brief initializes codec
 * @param AudioFreq sample rate in Hertz
 * @param word_size should be set to WORD_SIZE_16, since we have not yet implemented any other word_size
 */
uint32_t Codec_Reset(uint32_t AudioFreq)
{

    uint32_t retval;
    retval = Codec_ResetCodec(CODEC_ANA_I2C, AudioFreq, AUDIO_WORD_SIZE);

    if (retval == 0)
    {
      //        mchf_codecs[0].present = true;

	//        AudioPA_Enable(true);
        //Codec_VolumeSpkr(16); // mute speakerNOT
	//        Codec_VolumeLineOut(ts.txrx_mode); // configure lineout according to mode
	//	Codec_WriteRegister(CODEC_ANA_I2C, W8731_RIGHT_HEADPH_OUT, 100 | W8731_HEADPH_OUT_ZCEN | W8731_HEADPH_OUT_HPBOTH );   // value selected for 0.5VRMS at AGC setting
	//	Codec_WriteRegister(CODEC_ANA_I2C, W8731_LEFT_HEADPH_OUT, 100 | W8731_HEADPH_OUT_ZCEN | W8731_HEADPH_OUT_HPBOTH );   // value selected for 0.5VRMS at AGC setting
    }
    return retval;
}

/**
 * @brief Call this if the twin peaks happen, this restarts the I2S audio stream and it may fix the issue
 */

/**
 * @brief audio volume control in TX and RX modes for speaker [left headphone]
 * @param vol speaker / headphone volume in range  [0 - CODEC_SPEAKER_MAX_VOLUME], unit is dB, 0 represents muting, one increment represents 5db
 */

void Codec_VolumeSpkr(uint8_t vol)
{
    uint32_t lv = vol*5>W8731_VOL_MAX?W8731_VOL_MAX:vol*5;
    // limit max value to 80

    lv += 0x2F; // volume offset, all lower values including 0x2F represent muting
    // Reg 02: Speaker - variable volume, change at zero crossing in order to prevent audible clicks
    //    Codec_WriteRegister(W8731_LEFT_HEADPH_OUT,lv); // (lv | W8731_HEADPH_OUT_ZCEN));
    Codec_WriteRegister(CODEC_ANA_I2C, W8731_LEFT_HEADPH_OUT,(lv | W8731_HEADPH_OUT_ZCEN));
}
/**
 * @brief audio volume control in TX and RX modes for lineout [right headphone]
 *
 * At RX Lineout is always on with constant level (control via ts.lineout_gain)
 * At TX only if no frequency translation is active AND TX lineout mute is not set OR in CW
 * (because we send always without frequency translation in CW)
 *
 * @param txrx_mode txrx for which volume is to be set
 */

void Codec_VolumeLineOut(uint8_t txrx_mode)
{

  //    uint16_t lov =  ts.lineout_gain + 0x2F;
    // Selectively mute "Right Headphone" output (LINE OUT) depending on transceiver configuration

  //    UNUSED(txrx_mode);
    // we have a special shared lineout/headphone on the OVI40.
    // And since we have a dedidacted IQ codec, there is no need to switch of the lineout or headphones here
  //     Codec_WriteRegister(CODEC_ANA_I2C, W8731_RIGHT_HEADPH_OUT, lov | W8731_HEADPH_OUT_ZCEN | W8731_HEADPH_OUT_HPBOTH );   // value selected for 0.5VRMS at AGC setting
}

/**
 * @brief mute the Codecs Digital to Analog Converter Output
 * @param state true -> mute, false -> unmute
 */
void Codec_MuteDAC(bool state)
{
    if(state)
    {
        Codec_WriteRegister(CODEC_ANA_I2C, W8731_DIGI_AU_PATH_CNTR,(W8731_DEEMPH_CNTR|0x08));	// mute
    }
    else
    {
        Codec_WriteRegister(CODEC_ANA_I2C, W8731_DIGI_AU_PATH_CNTR,(W8731_DEEMPH_CNTR));		// mute off
    }
}

/**
 * @brief Sets the Codec WM8371 line input gain for both channels
 * @param gain in range of [0-255]
 */
static void Codec_InGainAdj(I2C_HandleTypeDef* hi2c, uint16_t gain)
{
    // Use Reg 00: Left Line In, set flag to adjust gain of both channels simultaneously
    Codec_WriteRegister(hi2c, W8731_LEFT_LINE_IN, gain | W8731_LINE_IN_LRBOTH);
}

/**
 * @brief Sets the Codec WM8371 line input gain for both audio in channels
 * @param gain in range of [0-255]
 */
void Codec_LineInGainAdj(uint8_t gain)
{
    Codec_InGainAdj(CODEC_ANA_I2C, gain);
}

/**
 * @brief Checks if all codec resources are available for switching
 * It basically checks if the I2C is currently in use
 * This function must be called before changing the oscillator in interrupts
 * otherwise deadlocks may happen
 * @return true if it is safe to call codec functions in an interrupt
 */
bool Codec_ReadyForIrqCall()
{
  return (CODEC_ANA_I2C->Lock == HAL_UNLOCKED);// && (CODEC_IQ_I2C->Lock == HAL_UNLOCKED);
}

