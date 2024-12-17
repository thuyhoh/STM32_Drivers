#include "stm32f407xx_pwr_driver.h"

/*
 * PWR_PVD_Mode_Mask PWR PVD Mode Mask
 */
#define PVD_MODE_EV													0x00020000U
#define PVD_MODE_IT													0x00010000U 
#define PVD_RISING_EDGE											0x1
#define PVD_FALING_EDGE											0x2


/* ===============================================================================
                 ##### Peripheral Control functions #####
 =============================================================================== */



void PWR_EnableBkUpAccess(void)
{
	PWR->CR |= (1 << PWR_CR_DBP);
}

void PWR_DisableBkUpAccess(void)
{
	PWR->CR &= ~(1 << PWR_CR_DBP);
}

void PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD)
{
	/* Set PLS[7:5] bits according to PVDLevel value */
	PWR->CR &= (0x7 << PWR_CR_LPDS);
	PWR->CR |= sConfigPVD->Mode;
	/* Clear any previous config */
	EXTI->IMR &= ~(0x1 << EXTI_IMR_MR16); // Clear Initerrupt
	EXTI->EMR &= ~(0x1 << EXTI_EMR_MR16); // Clear Event
	EXTI->RTSR &= ~(0x1 << EXTI_RTSR_TR16); // Claer Rising edge selection
	EXTI->FTSR &= ~(0x1 << EXTI_RTSR_TR16); // Claer Rising edge selection
	
	if((sConfigPVD->Mode & PVD_MODE_IT) == PVD_MODE_IT)
	{
		EXTI->IMR |= (0x1 << EXTI_IMR_MR16); // enable Initerrupt
	}
	if((sConfigPVD->Mode & PVD_MODE_EV) == PVD_MODE_EV)
	{
		EXTI->EMR &= ~(0x1 << EXTI_EMR_MR16); // enable Event
	}
	if((sConfigPVD->Mode & PVD_RISING_EDGE) == PVD_RISING_EDGE)
	{
		EXTI->RTSR |= (0x1 << EXTI_RTSR_TR16); // enable Rising edge selection
	}
	if((sConfigPVD->Mode & PVD_FALING_EDGE) == PVD_FALING_EDGE)
	{
		EXTI->FTSR |= (0x1 << EXTI_RTSR_TR16); // enable Rising edge selection
	}	
}

void PWR_EnablePVD(void)
{
	PWR->CR |= (1 << PWR_CR_PVDE);
}

void PWR_DisablePVD(void)
{
	PWR->CR &= ~(1 << PWR_CR_PVDE);
}

void PWR_EnableWakeUppin(void)
{
	PWR->CSR |= (0x1 << WAKEUP_PIN);
}

void PWR_DisableWakeUppin(void)
{
	PWR->CSR &= ~(0x1 << WAKEUP_PIN);
}
/*********************************************************************
 * @fn      		  		- PWR_EnterSLEEPMode
 *
 * @brief             -
 *
 * @param[1]          - @arg PWR_MAINREGULATOR_ON: SLEEP mode with regulator ON
												@arg PWR_LOWPOWERREGULATOR_ON: SLEEP mode with low power regulator ON 
					
 * @param[2]          - @arg PWR_SLEEPENTRY_WFI              : Enter SLEEP mode with WFI instruction
												@arg PWR_SLEEPENTRY_WFE              : Enter SLEEP mode with WFE instruction and clear of pending events before.
												@arg PWR_SLEEPENTRY_WFE_NO_EVT_CLEAR : Enter SLEEP mode with WFE instruction and no clear of pending event before
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry)
{
	// clear sleepdeep 
	SCB->SCR &= ~(0x1 << SCB_SCR_SLEEPDEEP);
	// sleep mode entry
	if(SLEEPEntry == PWR_SLEEPENTRY_WFI)
	{
		WFI();
	}else
	{
		if(SLEEPEntry != PWR_SLEEPENTRY_WFE_NO_EVT_CLEAR)
		{
			SEV(); // clear all pending event
		}
		WFE(); // wait for event
	}
}



void PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry)
{
	PWR->CR &= ~(1 << PWR_CR_PDDS);
	if(Regulator == PWR_LOWPOWERREGULATOR_ON)
	{
		PWR->CR |= 1 << PWR_CR_LPDS;
	}else // PWR_MAINREGULATOR_ON
	{
		PWR->CR &= ~(1 << PWR_CR_LPDS);
	}
	// set SLEEPDEEP in SCB_SCR
	SCB->SCR |= (1 << SCB_SCR_SLEEPDEEP);
	
	if(STOPEntry == PWR_STOPENTRY_WFI)
	{
		WFI();
	}else
	{
		if(STOPEntry != PWR_STOPENTRY_WFE_NO_EVT_CLEAR)
    {
      SEV(); // clear all pending event
    }
    WFE(); // wait for event
	}
	// reset SLEEPDEEP bit of SCB
	SCB->SCR &= ~(1 << SCB_SCR_SLEEPDEEP);
}

void PWR_EnableBkUpRegulator(void)
{
	// enable 
	PWR->CSR |= (0x1 << PWR_CSR_BRE);
	// waiting for ready
	while((PWR->CSR & (0x1 << PWR_CSR_BRR)) != (0x1 << PWR_CSR_BRR));
}

void PWR_EnterSTANDBYMode(void)
{
	/* Select Standby mode */	
	PWR->CR |= (1<<PWR_CR_PDDS);
	
	/* Set SLEEPDEEP bit of Cortex System Control Register */
	SCB->SCR |= (1 << SCB_SCR_SLEEPDEEP);
	
	/* Request Wait For Interrupt */
	WFI();
}

void PWR_PVD_IRQHandler(void)
{
	if((EXTI->PR & EXTI_PR_PR16) == EXTI_PR_PR16)
	{
		PWR_PVD_Callback();
		EXTI->PR |= 0x1 << EXTI_PR_PR16;
	}
}

__weak void PWR_PVD_Callback(void)
{
	
}

void PWR_EnableSleepOnExit(void)
{
	// Set Sleep On Exit
	SLEEPONEXIT();  
}

void PWR_DisableSleepOnExit(void)
{
	// Clear Sleep On Exit
	SCB->SCR &= ~(1<<SCB_SCR_SLEEPONEXIT);
}

void PWR_EnableSEVOnPend(void)
{ 
	// Set SEVONPEND
	SCB->SCR |= (1 << SCB_SCR_SEVONPEND);
}

void PWR_DisableSEVOnPend(void)
{
	// Clear SEVONPEND
	SCB->SCR &= ~(1 << SCB_SCR_SEVONPEND);
}
