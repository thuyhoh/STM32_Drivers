#ifndef STM32F407XX_WWDG_DRIVER_H
#define STM32F407XX_WWDG_DRIVER_H

#include "stm32f407xx.h"

typedef struct
{
	uint8_t Mode; // base it
	uint8_t W;
	uint8_t T;
}WWDG_Config_t;

#define WWDG_CR_T				0
#define WWDG_CR_WDGA		7

#define WWDG_CFR_W			0
#define WWDG_CFR_WDGTB	7
#define WWDG_CFR_EWI		9

#define WWDG_SR_EWIF		0

void WWDG_EnableWatchdog(void);
void WWDG_ClearFlag(void);
void WWDG_Refresh(WWDG_Config_t * sConfig);
void WWDG_IRQHandling(void);
__weak void WWDG_Callback(void);

void WWDG_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void WWDG_IRQPrioriryConfig(uint8_t IRQNumber, uint8_t IRQPriority);

#endif