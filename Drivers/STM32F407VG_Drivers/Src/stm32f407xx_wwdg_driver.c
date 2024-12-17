#include "stm32f407xx_wwdg_driver.h"

void WWDG_EnableWatchdog(void)
{
	WWDG->CR |= (0x1 << WWDG_CR_WDGA);
}

void WWDG_ClearFlag(void)
{
	WWDG->SR |= 1 << WWDG_SR_EWIF;
}

void WWDG_Refresh(WWDG_Config_t * sConfig)
{
	if(sConfig->T > 0x40)
	{
		return;
	}
	RCC_WWDG_PCLK_EN();
	RCC_WWDG_REG_RESET();
	
	if(sConfig->Mode == WWDG_MODE_IT)
	{
		WWDG->CFR |= 0x1 << WWDG_CFR_EWI; 	
		WWDG->CFR |= sConfig->W;
	}
	WWDG->CR |= sConfig->T << WWDG_CR_T;
	WWDG_EnableWatchdog();
}

void WWDG_IRQHandle(void)
{
	WWDG_Callback();
	
	WWDG_ClearFlag();
}

__weak void WWDG_Callback(void)
{
	
}

void WWDG_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{}
void WWDG_IRQPrioriryConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{}


