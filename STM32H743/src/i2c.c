/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

#include "gpio.h"

//I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2; // audio is I2C2
//I2C_HandleTypeDef hi2c4;

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  //    hi2c2.Init.Timing = 0x20404768; //we want /* I2C clock speed configuration (in Hz)  */#define I2C_SPEED                        100000 = 100k
  //      hi2c2.Init.Timing = 0x20303E5D;
  //hi2c2.Init.Timing = 0x00506682; // this from cube but no luck
  //  hi2c2.Init.Timing = 0x00506682;

  //  hi2c2.Init.Timing = 0x20404768; // also from cube?
  //  hi2c2.Init.Timing = 0x00202DBC; // slower
  //  hi2c2.Init.Timing = 0x0010006F;
  //  hi2c2.Init.Timing = 0x0060A3FE;

  //  hi2c2.Init.Timing = 0x00303D5B; 
  //    hi2c2.Init.Timing = 0x0060A3FE;// >>>??? latest one
  //    hi2c2.Init.Timing = 0x10707DBC; // from https://www.stm32duino.com/viewtopic.php?t=640
  //  hi2c2.Init.Timing = 0x10C0ECFF; // from ovi and from stm modded ovi
  //      hi2c2.Init.Timing = 0x10C0ECFF;
  //    hi2c2.Init.Timing = 0x00000107; // for HSI config?
  //  hi2c2.Init.Timing = 0x00303D5B; // for HSi without high speed 0 standard mode
  //    hi2c2.Init.Timing = 0x0060A3FE;// >>>??? latest one

  //  hi2c2.Init.Timing = 0x0000011F;

  //hi2c2.Init.Timing = 0x102023B4; // half above
  //  0x10707DBC;
    hi2c2.Init.Timing = 0x10707DBC; // from other
  //hi2c2.Init.Timing =  0x00D00E28;  /* (Rise time = 120ns, Fall time = 25ns)
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
 
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    //   Error_Handler();
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    //    Error_Handler();
  }

  //     HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C2);

  
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    __HAL_RCC_I2C2_CLK_ENABLE();

    //    __HAL_RCC_I2C2_FORCE_RESET();
    //    HAL_Delay(2);
    //__HAL_RCC_I2C2_RELEASE_RESET();
  
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();
  
    /**I2C2 GPIO Configuration    
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA 
    */
    //    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
} 

