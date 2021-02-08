// from https://www.stm32duino.com/viewtopic.php?t=640 though for H7 and without SAI! but maybe clock?

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;
I2C_HandleTypeDef hi2c2;

#define I2C_TX_handle   &hi2c2
#define I2C_RX_handle   &hi2c2

#define AUDIO_INPUT_LINEIN  0
#define AUDIO_INPUT_MIC     1

#define I2S_USE_DMA 1

#include "stm32_def_build.h"

void setup() {

  Serial.begin(115200);
  Serial.println();
  Serial.println("--------------------------------------------");
  Serial.println(__DATE__);
  Serial.println(__TIME__);
  Serial.println("--------------------------------------------");

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_I2C2_Init();

  WM8731_init();
  WM8731_inputSelect(AUDIO_INPUT_MIC);
  WM8731_volumeInteger(110);

#if I2S_USE_DMA
  StartAudioBuffers(&hi2s2); // - `I2S2_SDO` :: `PC3_C` is not triggered … why?!?!
#endif
}

void loop() {
#if I2S_USE_DMA
  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
  //    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  //    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
  HAL_Delay(1000);
#else
  static uint16_t foo[2] = {0, 0};
  foo[0] += 129;
  foo[1] += 255;
  HAL_I2S_Transmit(&hi2s2, foo, 2, 1000000);
#endif
}

bool I2C_TX_write(uint8_t device_address, uint8_t *data, uint8_t length) {
  HAL_StatusTypeDef mResult = HAL_I2C_Master_Transmit(I2C_TX_handle,
                              device_address, data, length, 10);
  if (mResult != HAL_OK) {
    Serial.println("--- ERROR @ I2C_TX_write");
    return false;
  }
  return true;
}

bool I2C_RX_write(uint8_t device_address, uint8_t *data, uint8_t length) {
  HAL_StatusTypeDef mResult = HAL_I2C_Master_Transmit(I2C_RX_handle,
                              device_address, data, length, 10);
  if (mResult != HAL_OK) {
    Serial.println("--- ERROR @ I2C_RX_write");
    return false;
  }
  return true;
}

/* -------------------------------------------------------------------------------- */

#define   I2S_BUFFER_SIZE    64
uint32_t dma_TX_buffer[I2S_BUFFER_SIZE];
uint32_t dma_RX_buffer[I2S_BUFFER_SIZE];
float osc_phi = 0;
float osc_phi_inc = 440.0f / 44100.0f; // generating 440HZ

uint8_t mCounterHAL_I2S_RxCpltCallback = 0;
uint8_t mCounterHAL_I2S_TxCpltCallback = 0;

void FillBuffer(uint32_t *buffer, uint16_t len) {
  float a;
  int16_t y;
  uint16_t c;

  for (c = 0; c < len; c++) {
    a = (float) sin(osc_phi * 6.2832f) * 0.20f;
    osc_phi += osc_phi_inc;
    osc_phi -= (float) ((uint16_t) osc_phi);
    
     y = (int16_t) (a * 32767.0f);
    buffer[c] = ((uint32_t) (uint16_t) y) << 0 | ((uint32_t) (uint16_t) y) << 16;
  }
}

void StartAudioBuffers(I2S_HandleTypeDef *hi2s) {
  memset(dma_TX_buffer, 0, sizeof(dma_TX_buffer));
  memset(dma_RX_buffer, 0, sizeof(dma_RX_buffer));

  if (HAL_I2S_Transmit_DMA(hi2s, (uint16_t*) dma_TX_buffer, I2S_BUFFER_SIZE << 1) != HAL_OK) {
    Error_Handler();
  }
  hi2s2.State = HAL_I2S_STATE_READY; // state flag needs to be cleared manually
  if (HAL_I2S_Receive_DMA(hi2s, (uint16_t*) dma_RX_buffer, I2S_BUFFER_SIZE << 1) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
  FillBuffer(&(dma_TX_buffer[I2S_BUFFER_SIZE >> 1]), I2S_BUFFER_SIZE >> 1);
  mCounterHAL_I2S_TxCpltCallback++;
  if (mCounterHAL_I2S_TxCpltCallback == 0) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  FillBuffer(&(dma_TX_buffer[0]), I2S_BUFFER_SIZE >> 1);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
  mCounterHAL_I2S_RxCpltCallback++;
  if (mCounterHAL_I2S_RxCpltCallback == 0) {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
  }
}

void _Error_Handler(const char * c, int i) {
  Serial.print("ERROR in line: "); Serial.println(i);
}

/* -------------------------------------------------------------------------------- */
/* copied from `main.c`, `stm32h7xx_hal_msp.c`, `stm32h7xx_it.c` */
/* -------------------------------------------------------------------------------- */

extern "C" {

  /**
      @brief System Clock Configuration
      @retval None
  */
  void SystemClock_Config(void)
  {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
    /** Initializes the RCC Oscillators according to the specified parameters
      in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 24;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                  | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
      Error_Handler();
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_SPI2
        | RCC_PERIPHCLK_I2C2 | RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
    /** Enable USB Voltage detector
    */
    HAL_PWREx_EnableUSBVoltageDetector();
  }

  /**
      @brief I2C2 Initialization Function
      @param None
      @retval None
  */
  void MX_I2C2_Init(void)
  {

    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x10707DBC;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
      Error_Handler();
    }
    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
      Error_Handler();
    }
    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
    {
      Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */

    /* USER CODE END I2C2_Init 2 */

  }

  /**
      @brief I2S2 Initialization Function
      @param None
      @retval None
  */
  void MX_I2S2_Init(void)
  {

    /* USER CODE BEGIN I2S2_Init 0 */

    /* USER CODE END I2S2_Init 0 */

    /* USER CODE BEGIN I2S2_Init 1 */

    /* USER CODE END I2S2_Init 1 */
    hi2s2.Instance = SPI2;
    hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
    hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
    hi2s2.Init.CPOL = I2S_CPOL_LOW;
    hi2s2.Init.FirstBit = I2S_FIRSTBIT_MSB;
    hi2s2.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
    hi2s2.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
    hi2s2.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
    if (HAL_I2S_Init(&hi2s2) != HAL_OK)
    {
      Error_Handler();
    }
    /* USER CODE BEGIN I2S2_Init 2 */
    hi2s2.Init.Mode = I2S_MODE_MASTER_FULLDUPLEX; /* fixes bug in CubeMX */
    if (HAL_I2S_Init(&hi2s2) != HAL_OK)
    {
      Error_Handler();
    }
    /* USER CODE END I2S2_Init 2 */

  }

  /**
      Enable DMA controller clock
  */
  void MX_DMA_Init(void)
  {

    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    /* DMA2_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

  }

  /**
      @brief GPIO Initialization Function
      @param None
      @retval None
  */
  void MX_GPIO_Init(void)
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PC1 PC4 PC5 */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA1 PA2 PA7 */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : LD1_Pin LD3_Pin */
    GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PB13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PG11 PG13 */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  }

  /**
      Initializes the Global MSP.
  */
  void HAL_MspInit(void)
  {
    /* USER CODE BEGIN MspInit 0 */

    /* USER CODE END MspInit 0 */

    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* System interrupt init*/

    /* USER CODE BEGIN MspInit 1 */

    /* USER CODE END MspInit 1 */
  }

  /**
    @brief I2C MSP Initialization
    This function configures the hardware resources used in this example
    @param hi2c: I2C handle pointer
    @retval None
  */
  void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hi2c->Instance == I2C2)
    {
      /* USER CODE BEGIN I2C2_MspInit 0 */

      /* USER CODE END I2C2_MspInit 0 */

      __HAL_RCC_GPIOF_CLK_ENABLE();
      /**I2C2 GPIO Configuration
        PF0     ------> I2C2_SDA
        PF1     ------> I2C2_SCL
      */
      GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
      HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

      /* Peripheral clock enable */
      __HAL_RCC_I2C2_CLK_ENABLE();
      /* USER CODE BEGIN I2C2_MspInit 1 */

      /* USER CODE END I2C2_MspInit 1 */
    }

  }

  /**
    @brief I2S MSP Initialization
    This function configures the hardware resources used in this example
    @param hi2s: I2S handle pointer
    @retval None
  */
  void HAL_I2S_MspInit(I2S_HandleTypeDef* hi2s)
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hi2s->Instance == SPI2)
    {
      /* USER CODE BEGIN SPI2_MspInit 0 */

      /* USER CODE END SPI2_MspInit 0 */
      /* Peripheral clock enable */
      __HAL_RCC_SPI2_CLK_ENABLE();

      __HAL_RCC_GPIOC_CLK_ENABLE();
      __HAL_RCC_GPIOB_CLK_ENABLE();
      /**I2S2 GPIO Configuration
        PC2_C     ------> I2S2_SDI
        PC3_C     ------> I2S2_SDO
        PB10     ------> I2S2_CK
        PB12     ------> I2S2_WS
        PC6     ------> I2S2_MCK
      */
      GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

      GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_12;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      /* I2S2 DMA Init */
      /* SPI2_RX Init */
      hdma_spi2_rx.Instance = DMA2_Stream1;
      hdma_spi2_rx.Init.Request = DMA_REQUEST_SPI2_RX;
      hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
      hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
      hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
      hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
      hdma_spi2_rx.Init.Mode = DMA_CIRCULAR;
      hdma_spi2_rx.Init.Priority = DMA_PRIORITY_LOW;
      hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
      if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
      {
        Error_Handler();
      }

      __HAL_LINKDMA(hi2s, hdmarx, hdma_spi2_rx);

      /* SPI2_TX Init */
      hdma_spi2_tx.Instance = DMA2_Stream0;
      hdma_spi2_tx.Init.Request = DMA_REQUEST_SPI2_TX;
      hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
      hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
      hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
      hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
      hdma_spi2_tx.Init.Mode = DMA_CIRCULAR;
      hdma_spi2_tx.Init.Priority = DMA_PRIORITY_LOW;
      hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
      if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK)
      {
        Error_Handler();
      }

      __HAL_LINKDMA(hi2s, hdmatx, hdma_spi2_tx);

      /* USER CODE BEGIN SPI2_MspInit 1 */

      /* USER CODE END SPI2_MspInit 1 */
    }

  }

  /**
      @brief This function handles DMA2 stream0 global interrupt.
  */
  void DMA2_Stream0_IRQHandler(void)
  {
    /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

    /* USER CODE END DMA2_Stream0_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_spi2_tx);
    /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

    /* USER CODE END DMA2_Stream0_IRQn 1 */
  }

  /**
      @brief This function handles DMA2 stream1 global interrupt.
  */
  void DMA2_Stream1_IRQHandler(void)
  {
    /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

    /* USER CODE END DMA2_Stream1_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_spi2_rx);
    /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

    /* USER CODE END DMA2_Stream1_IRQn 1 */
  }

} /* extern "C" */

/* -------------------------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>

#define WM8731_I2C_ADDR 0x34

#define WM8731_REG_LLINEIN  0
#define WM8731_REG_RLINEIN  1
#define WM8731_REG_LHEADOUT 2
#define WM8731_REG_RHEADOUT 3
#define WM8731_REG_ANALOG   4
#define WM8731_REG_DIGITAL  5
#define WM8731_REG_POWERDOWN    6
#define WM8731_REG_INTERFACE    7
#define WM8731_REG_SAMPLING 8
#define WM8731_REG_ACTIVE   9
#define WM8731_REG_RESET    15

bool WM8731_init() {
  /* I2S configuration */
  delay(5);
  WM8731_write(WM8731_REG_RESET, 0);

  WM8731_write(WM8731_REG_INTERFACE, 0b00000010); // 0x02=0b00000010 // I2S, 16 bit, MCLK slave
  WM8731_write(WM8731_REG_SAMPLING,  0b00100000); // 0x20=0b00100000  // 256*Fs, 44.1 kHz, MCLK/1

  WM8731_write(WM8731_REG_DIGITAL, 0x08);   // DAC soft mute
  WM8731_write(WM8731_REG_ANALOG, 0x00);    // disable all

  WM8731_write(WM8731_REG_POWERDOWN, 0x00);

  WM8731_write(WM8731_REG_LHEADOUT, 0x80);
  WM8731_write(WM8731_REG_RHEADOUT, 0x80);

  delay(5);
  WM8731_write(WM8731_REG_ACTIVE, 1);
  delay(5);

  WM8731_write(WM8731_REG_DIGITAL, 0b00100);      // DAC unmuted
  WM8731_write(WM8731_REG_ANALOG, 0b00010000);    // DAC selected

  WM8731_volume(0.5);
  return true;
}

bool WM8731_write(uint8_t reg, uint16_t val) {
  /*
    sub procedure WM8731_CMD(dim address as byte, dim cmd as word)
    dim addr as byte
    ' B[15:9] Are Control Address Bits
    ' B[8:0]  Are Control Data Bits
    addr = address << 1                 ' Shift left for one positions
    addr = addr or (hi(cmd) and 1)

    I2C2_Start()                           ' Start I2C2 module
    I2C2_wm8731_write(WM8731_ADDRESS)             ' Write Adress of WM8731 chip
    I2C2_wm8731_write(addr)                       ' Write register address
    I2C2_wm8731_write(lo(cmd))                    ' Write command
    I2C2_Stop()                            ' Stop I2C2 module
    end sub
  */

  const static uint8_t TRANSMIT_LENGTH = 2;
  uint8_t data[TRANSMIT_LENGTH];

  data[0] = (reg << 1) | ((val >> 8) & 1);
  data[1] = val & 0xFF;

  bool mResult = I2C_TX_write(WM8731_I2C_ADDR, data, TRANSMIT_LENGTH);
  return mResult;
}

bool WM8731_volumeInteger(unsigned int n) {
  // n = 127 for max volume (+6 dB)
  // n = 48 for min volume (-73 dB)
  // n = 0 to 47 for mute
  if (n > 127)
    n = 127;
  WM8731_write(WM8731_REG_LHEADOUT, n | 0x180);
  WM8731_write(WM8731_REG_RHEADOUT, n | 0x80);
  return true;
}

bool WM8731_inputLevel(float n) {
  // range is 0x00 (min) - 0x1F (max)

  int _level = (int) (n * 31.f);

  _level = _level > 0x1F ? 0x1F : _level;
  WM8731_write(WM8731_REG_LLINEIN, _level);
  WM8731_write(WM8731_REG_RLINEIN, _level);
  return true;
}

bool WM8731_inputSelect(int n) {
  if (n == AUDIO_INPUT_LINEIN) {
    WM8731_write(WM8731_REG_ANALOG, 0x12);
  } else if (n == AUDIO_INPUT_MIC) {
    WM8731_write(WM8731_REG_ANALOG, 0x15);
  } else {
    return false;
  }
  return true;
}

bool WM8731_volume(float n) {
  return WM8731_volumeInteger(n * 80.0 + 47.499);
}
