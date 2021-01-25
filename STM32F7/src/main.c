#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <main.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_nucleo_144.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_pwr.h"
#include "stm32f7xx_hal_pwr_ex.h"
#include "stm32f7xx_hal_rcc.h"
#include "stm32f7xx_hal_rcc_ex.h"
#include "stm32f7xx_hal_i2s.h"
#include "stm32f7xx_nucleo_144.h"
#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_hal_gpio.h"

#include "hw_i2s.h"
#include "dma.h"
#include "i2c.h"
#include "sai.h"
#include "8731.h"
#include "wavetable.h"
#include "wavetables.h"
#include "wvocoder.h"

#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG


/*

This is test code for Nucleo board from: https://github.com/dpiegdon/STM32F767ZI-Nucleo-144  with expanded Makefile and includes here

TODO: up and running with 8731 audio codec:  set up i2s and dma - using SAI?, set up 8731 (generic), callbacks, pins?

// to test audio codec what are callbacks?

*in hw_i2s.c -> look back to original code!*

    PF6     ------> SAI1_SD_B -> DACDAT = MOSI 
    PF7     ------> SAI1_MCLK_B -> MCLK on 8731 = 12.288MHz on board
    PF8     ------> SAI1_SCK_B -> BCLK = SCK on board
    PF9     ------> SAI1_FS_B -> DACLRC and ADCLRC = marked

    PD6     ------> SAI1_SD_A -> ADCDAT = MISO

    PB10     ------> I2C2_SCL -> SCLK 47R = SCL    PB11     ------> I2C2_SDA -> SDIN = SDA

 11/1/2021 failure and no signals on bus... causes:

- we had wrong pins and setup for I2C2 which is now corrected, to test again
- we didn't have it.c for interrupts which included DMA so we added that now

TO TEST!

- problem with clock setup esp. for SAI. Here clock is setup for HSE: external high speed oscillator
- other wrong init or code: slots in sai.c

12/1/2021: still no luck - added as much of clock code as works

- now with hw_i2s included action on pins but no sound
- but now not working

- trying different clocks - now using STM32CubeMX to reverse engineer settings here for clock and pll -> SAI runs at 12.288 MHz which is 48k sample rate x256

13/1 2021: can see now bits on i2c but we can't set bypass mode and still no sound... was missing systick interrupt and also usb pin

- all clocks seem to be right...
- now it is working, removed clock and maybe a few other alterations but sound is not quite right... packing?

*/

typedef uint8_t uchar;

#define GPIO_SetBits(PORT,PINS) { (PORT)->BSRR = (PINS); }
#define GPIO_ResetBits(PORT,PINS) { (PORT)->BSRR = (PINS) << 16U; }

static void SystemClock_Config(void);
void Error_Handler(void);
static void MPU_Config(void);

uint16_t Hw_I2C_WriteRegister(I2C_HandleTypeDef* hi2c, uchar I2CAddr,uint16_t addr,uint16_t addr_size, uchar RegisterValue)
{
	//    return UhsdrHw_I2C_WriteRegister(hi2c, CODEC_ADDRESS, Byte1, 1, Byte2);

	HAL_StatusTypeDef i2cRet = HAL_I2C_Mem_Write(hi2c,I2CAddr,addr,addr_size,&RegisterValue,1,100);

    return  i2cRet != HAL_OK?0xFF00:0;
}

static uint32_t Codec_WriteRegister(I2C_HandleTypeDef* hi2c, uint8_t RegisterAddr, uint16_t RegisterValue)
{
    // Assemble 2-byte data in WM8731 format
    uint8_t Byte1 = ((RegisterAddr<<1)&0xFE) | ((RegisterValue>>8)&0x01);
    uint8_t Byte2 = RegisterValue&0xFF;
    return Hw_I2C_WriteRegister(hi2c, (0x1A)<<1, Byte1, 1, Byte2);
    //    HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
    //    HAL_I2C_Master_Transmit_IT(hi2c, (0x1A)<<1, Byte1, 1);
    //    HAL_I2C_Master_Transmit_IT(hi2c, (0x1A)<<1, Byte2, 1);
    
    //return 1;
}


int main(void)
{

extern Wavetable wavtable;

  
// set up controller (in main maybe) to right speeds/PLL/system clock? do we need to?

  /* MPU Configuration----------------------------------------------------------*/
 MPU_Config();

  /* Enable I-Cache-------------------------------------------------------------*/
 SCB_EnableICache();

  /* Enable D-Cache-------------------------------------------------------------*/
 SCB_EnableDCache();
    
  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   __HAL_RCC_PWR_CLK_ENABLE();
   __HAL_RCC_SYSCFG_CLK_ENABLE();


 HAL_Init();


  /* Ensure we have a clock configuration as in reset state, i.e. PLL clock is off and we are using HSI */
  //HAL_RCC_DeInit(); // crashes too

  /* Configure the system clock */
  SystemClock_Config();


  //  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  //    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  //  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  
  // how do we init all/audio codec
  // do we use SAI or i2c? or is it both as codec references i2c (setup?)

    GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  
  wavetable_init(&wavtable, crowtable, 142); // now last arg as length of table=less than 512 
  Vocoder_Init(32000.0f);
  //  BANDS_Init_();
  
    MX_DMA_Init();
    MX_I2C2_Init();


    MX_SAI1_Init();    

  UhsdrHwI2s_Codec_StartDMA(); // this was missing and now we have action on some pins

  Codec_Reset(AUDIO_SAMPLE_RATE);
  
/// LED basic test
  
//  BSP_LED_Init(LED1);
//  BSP_LED_Init(LED2);
//  BSP_LED_Init(LED3);
  
  for(;;)
    {
      //      Codec_WriteRegister(&hi2c2,0x04,0x08); // bypass - can see bits but still no sound in any case
      //Codec_Reset(48000);
      	}

	return 0;
}

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disables the MPU */
  HAL_MPU_Disable();
    /**Initializes and configures the Region and the memory to be protected 
    */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x60000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 128;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 5;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler()
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}
