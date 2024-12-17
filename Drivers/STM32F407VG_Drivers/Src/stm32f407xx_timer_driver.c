#include "stm32f407xx_timer_driver.h"


static void TIM_PeriControl(TIM_RegDef_t *pTIM, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		pTIM->CR1 |= (1 << TIM_CR1_CEN);
	}else
	{
		pTIM->CR1 &= ~(1 << TIM_CR1_CEN);
	}
}

static void TIM_ITControl(TIM_RegDef_t *pTIM, uint32_t Interrupt, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		pTIM->DIER |= 1<<Interrupt;
	}else
	{
		pTIM->DIER &= ~(1<<Interrupt);
	}
}

void TIM_Base_Init(TIM_Handler_t *pTIMBase)
{
	pTIMBase->pTIMx->CR1 = pTIMBase->Init.CounterMode |
												 pTIMBase->Init.AutoReloadPreload|
												 pTIMBase->Init.ClockDivision;
	pTIMBase->pTIMx->PSC = pTIMBase->Init.Prescaler;
	pTIMBase->pTIMx->ARR = pTIMBase->Init.Period;
}

void TIM_Base_Start(TIM_RegDef_t *pTIMx)
{
	TIM_PeriControl(pTIMx, ENABLE);
}

void TIM_Base_Stop(TIM_RegDef_t *pTIMx)
{
	TIM_PeriControl(pTIMx, DISABLE);
}

uint8_t TIM_GetFlag(TIM_RegDef_t *pTIMx ,uint8_t TIM_Flag)
{
	return ((pTIMx->SR >> TIM_Flag) & 1);
}

