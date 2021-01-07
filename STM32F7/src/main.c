#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_nucleo_144.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_pwr.h"
#include "stm32f7xx_hal_pwr_ex.h"
#include "stm32f7xx_hal_rcc.h"
#include "stm32f7xx_hal_rcc_ex.h"
#include "stm32f7xx_hal_i2s.h"
#include "stm32f7xx_nucleo_144.h"

/*

This is test code for Nucleo board from: https://github.com/dpiegdon/STM32F767ZI-Nucleo-144  with expanded Makefile and includes here

TODO: up and running with 8731 audio codec:  set up i2s and dma - using SAI?, set up 8731 (generic), callbacks, pins?

*/


int main(void)
{
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED3);

	for(;;)
	{
		BSP_LED_Toggle(LED1);
		for (int j = 0; j < 100000; j++)
			;

		BSP_LED_Toggle(LED2);
		for (int j = 0; j < 100000; j++)
			;

		BSP_LED_Toggle(LED3);
		for (int j = 0; j < 100000; j++)
			;
	}

	return 0;
}
